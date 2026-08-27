function P = run_parameter_studies(varargin)
%RUN_PARAMETER_STUDIES  Controlled sweeps over any pipeline parameter.
%
%   Four modes, all of which execute the real pipeline rather than a surrogate:
%
%     'sweep'      vary one configuration field over a list of values and
%                  report the full metric set for each, with everything else
%                  held fixed and the scene seeds held paired across values
%     'grid'       vary two fields jointly and return the response surface
%     'stagewise'  cache the physical scenes once, then replay the downstream
%                  chain under many detector and tracker candidates, and
%                  validate the winner on independently seeded scenes
%     'sensitivity' perturb every tunable field one at a time by a fixed
%                  relative step and rank them by how much the operating
%                  point moves, which identifies what is worth tuning before
%                  any tuning is attempted
%
%   Parameters are addressed by dotted path, so any nested field is reachable:
%   'cfar.Pfa', 'detector.min_amf_db', 'detector.gs.angle_step_deg',
%   'tbd.coherent.path_score_threshold', 'n_tx'.
%
%   Examples
%     P = run_parameter_studies('mode','sweep', ...
%             'parameter','detector.min_amf_db','values',6:2:16);
%     P = run_parameter_studies('mode','grid', ...
%             'parameter','cfar.Pfa','values',[1e-4 1e-5 1e-6], ...
%             'parameter2','detector.min_amf_db','values2',[7 10 13]);
%     P = run_parameter_studies('mode','sensitivity','trials',3);

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('mode','sweep','parameter','','values',[], ...
    'parameter2','','values2',[],'trials',4,'frames',10, ...
    'snr_values',[0 10],'master_seed',20260822,'config',struct(), ...
    'output_dir',fullfile(pwd,'docs','results'),'verbose',true, ...
    'relative_step',0.20,'validation_trials',4);
opts = parse_opts(opts,varargin{:});

outdir = fullfile(opts.output_dir,'parameter_studies');
if ~exist(outdir,'dir'), mkdir(outdir); end

switch lower(char(opts.mode))
    case 'sweep',       P = mode_sweep(opts,outdir);
    case 'grid',        P = mode_grid(opts,outdir);
    case 'stagewise',   P = mode_stagewise(opts,outdir);
    case 'sensitivity', P = mode_sensitivity(opts,outdir);
    otherwise
        error('run_parameter_studies:Mode', ...
            'mode must be sweep, grid, stagewise or sensitivity.');
end
P.options = opts;
P.output_dir = outdir;
save(fullfile(outdir,[lower(char(opts.mode)) '_study.mat']),'P','-v7.3');
end

% =========================================================================
function P = mode_sweep(opts,outdir)
if isempty(opts.parameter) || isempty(opts.values)
    error('run_parameter_studies:Sweep','A parameter path and a value list are required.');
end
rows = repmat(study_row(),0,1);
if opts.verbose
    fprintf('\n[SWEEP] %s over %d values | %d trials x %d frames\n', ...
        opts.parameter,numel(opts.values),opts.trials,opts.frames);
end
for i = 1:numel(opts.values)
    v = value_at(opts.values,i);
    ov = set_path(opts.config,opts.parameter,v);
    m = evaluate_setting(ov,opts);
    r = pack_row(m);
r.parameter = opts.parameter;
    r.value = num_or_nan(v);
    r.label = sprintf('%s = %s',opts.parameter,format_value(v));
    rows(end+1) = r;
    if opts.verbose, print_row(r); end
end
write_rows(rows,fullfile(outdir,'sweep.csv'));
P = struct('mode','sweep','rows',rows,'best',pick_best(rows));
end

function P = mode_grid(opts,outdir)
if isempty(opts.parameter) || isempty(opts.parameter2)
    error('run_parameter_studies:Grid','Two parameter paths are required.');
end
rows = repmat(study_row(),0,1);
for i = 1:numel(opts.values)
    for j = 1:numel(opts.values2)
        vi = value_at(opts.values,i); vj = value_at(opts.values2,j);
        ov = set_path(opts.config,opts.parameter,vi);
        ov = set_path(ov,opts.parameter2,vj);
        m = evaluate_setting(ov,opts);
        r = pack_row(m);
        r.parameter = opts.parameter;  r.value  = num_or_nan(vi);
        r.parameter2 = opts.parameter2; r.value2 = num_or_nan(vj);
        r.label = sprintf('%s=%s, %s=%s',opts.parameter,format_value(vi), ...
            opts.parameter2,format_value(vj));
        rows(end+1) = r;
        if opts.verbose, print_row(r); end
    end
end
write_rows(rows,fullfile(outdir,'grid.csv'));
P = struct('mode','grid','rows',rows,'best',pick_best(rows));
end

function P = mode_stagewise(opts,outdir)
%MODE_STAGEWISE  Cache the physics once, then explore the downstream chain.
%
%   The physical simulator dominates the cost of a sweep. Caching the receive
%   cubes and replaying only the detection, estimation, weak-target and
%   tracking stages makes a candidate roughly two orders of magnitude cheaper,
%   which is what makes a grid of any useful size affordable. The winner is
%   then validated on freshly generated, independently seeded scenes, because
%   a candidate selected on the cache has by construction been fitted to it.
cache = build_cache(opts);
candidates = candidate_grid();
rows = repmat(study_row(),0,1);
if opts.verbose
    fprintf('\n[STAGEWISE] %d cached scenes | %d candidates\n',numel(cache),numel(candidates));
end
for c = 1:numel(candidates)
    m = replay_cache(cache,candidates(c));
    r = pack_row(m);
    r.label = candidates(c).label;
r.candidate_index = c;
    rows(end+1) = r;
    if opts.verbose, print_row(r); end
end
best = pick_best(rows);
bestCfg = candidates(best.candidate_index).config;

if opts.verbose
    fprintf('\n[STAGEWISE] Validating "%s" on independent scenes.\n',best.label);
end
V = run_radar_publication_monte_carlo('split','validation', ...
    'snr_values',opts.snr_values,'trials',opts.validation_trials, ...
    'frames',opts.frames,'config',bestCfg,'save_figures',false,'verbose',opts.verbose);

write_rows(rows,fullfile(outdir,'stagewise.csv'));
P = struct('mode','stagewise','rows',rows,'best',best, ...
    'best_config',bestCfg,'validation',V,'cache_size',numel(cache));
end

function P = mode_sensitivity(opts,outdir)
%MODE_SENSITIVITY  Rank parameters by how much they move the operating point.
%
%   Tuning a parameter that does not control the observed failure is wasted
%   effort. This mode perturbs each tunable field by a fixed relative step,
%   in both directions, and reports the resulting change in detection
%   probability and false objects per frame. Fields that move neither are not
%   worth a controller.
fields = sensitivity_fields();
baseM = evaluate_setting(opts.config,opts);
rows = repmat(sens_row(),0,1);
if opts.verbose
    fprintf('\n[SENSITIVITY] baseline Pd=%.4f false/frame=%.4f | %d fields at %+.0f%%\n', ...
        baseM.pd,baseM.false_per_frame,numel(fields),100*opts.relative_step);
end
for i = 1:numel(fields)
    path = fields{i};
    v0 = get_path(defaults_struct(),path);
    if isempty(v0) || ~isnumeric(v0) || ~isscalar(v0), continue; end
    r = sens_row(); r.parameter = path; r.baseline = v0;
    up = evaluate_setting(set_path(opts.config,path,v0*(1+opts.relative_step)),opts);
    dn = evaluate_setting(set_path(opts.config,path,v0*(1-opts.relative_step)),opts);
r.d_pd_up = up.pd - baseM.pd;
r.d_pd_down = dn.pd - baseM.pd;
r.d_false_up = up.false_per_frame - baseM.false_per_frame;
r.d_false_down = dn.false_per_frame - baseM.false_per_frame;
    r.influence = max(abs([r.d_pd_up r.d_pd_down])) + max(abs([r.d_false_up r.d_false_down]));
    rows(end+1) = r;
    if opts.verbose
        fprintf('  %-46s influence=%.4f  dPd=[%+.3f %+.3f]  dFalse=[%+.3f %+.3f]\n', ...
            path,r.influence,r.d_pd_up,r.d_pd_down,r.d_false_up,r.d_false_down);
    end
end
if ~isempty(rows)
    [~,ord] = sort([rows.influence],'descend');
    rows = rows(ord);
end
write_rows(rows,fullfile(outdir,'sensitivity.csv'));
P = struct('mode','sensitivity','rows',rows,'baseline',baseM);
end

% =========================================================================
function m = evaluate_setting(override,opts)
%EVALUATE_SETTING  Pooled metrics for one configuration over paired scenes.
base = radar_experiment_common('base_config');
base = radar_experiment_common('merge',base,override);
sceneOpts = struct('min_targets',4,'max_targets',10,'stationary_fraction',0.25);
acc = repmat(struct(),0,1);
for si = 1:numel(opts.snr_values)
    for tr = 1:opts.trials
        seed = opts.master_seed + radar_experiment_common('split_offset','dev') + 100000*tr;
        cfg = radar_experiment_common('scene',base,seed,sceneOpts);
        cfg = radar_experiment_common('merge',cfg,override);
cfg.random_seed = seed;
cfg.noise_enabled = true;
        cfg.noise_model = 'SNR-controlled AWGN';
        cfg.noise_level = opts.snr_values(si);
cfg.snr_override_enabled = true;
        ro = struct('Nframes',opts.frames,'show_figures',false,'save_diagnostics',false, ...
            'save_results',false,'notify_gui',false,'snr_override',opts.snr_values(si));
        try
            out = run_radar_project(cfg,ro);
            [~,mm] = radar_experiment_common('score',out.objects,out.truth,out.info,out.params);
            acc(end+1) = mm;
        catch
        end
    end
end
m = pool(acc);
end

function cache = build_cache(opts)
base = radar_experiment_common('base_config');
sceneOpts = struct('min_targets',4,'max_targets',10,'stationary_fraction',0.25);
cache = repmat(struct('clean_frames',{{}},'params',struct(),'truth',[]),0,1);
for si = 1:numel(opts.snr_values)
    for tr = 1:opts.trials
        seed = opts.master_seed + radar_experiment_common('split_offset','dev') + 100000*tr;
        cfg = radar_experiment_common('scene',base,seed,sceneOpts);
cfg.random_seed = seed;
cfg.noise_enabled = true;
        cfg.noise_model = 'SNR-controlled AWGN';
        cfg.noise_level = opts.snr_values(si);
cfg.snr_override_enabled = true;
        ro = struct('Nframes',opts.frames,'show_figures',false,'save_diagnostics',false, ...
            'save_results',false,'notify_gui',false,'snr_override',opts.snr_values(si));
        out = run_radar_project(cfg,ro);
        cache(end+1) = struct('clean_frames',{out.clean_frames}, ...
            'params',out.params,'truth',out.truth);
    end
end
end

function m = replay_cache(cache,cand)
acc = repmat(struct(),0,1);
for k = 1:numel(cache)
    p = radar_experiment_common('apply_tuned',cache(k).params,cand.config);
    [objects,~,info] = radar_object_tracker(cache(k).clean_frames,{},p,struct('finalize',true));
    [~,mm] = radar_experiment_common('score',objects,cache(k).truth,info,p);
    acc(end+1) = mm;
end
m = pool(acc);
end

function c = candidate_grid()
%CANDIDATE_GRID  A small, deliberately readable grid over the decisive gates.
c = repmat(struct('label','','config',struct()),0,1);
for amf = [8 11 14]
    for gAmf = [7 10 13]
        for gHits = [2 3 4]
            for tbd = [12 18]
                cfg = struct();
                cfg.detector = struct('min_amf_db',amf);
                cfg.track = struct('group_final_mean_amf_db',gAmf, ...
                    'group_final_min_hits',gHits, ...
                    'stationary_group_final_mean_amf_db',gAmf+2);
                cfg.tbd = struct('path_promotion_score',tbd);
                c(end+1) = struct('label',sprintf('AMF %g | group %g | hits %d | TBD %g', ...
                    amf,gAmf,gHits,tbd),'config',cfg);
            end
        end
    end
end
end

function f = sensitivity_fields()
f = {'detector.min_amf_db','detector.amf_threshold_pfa','detector.angle_coarse_step_deg', ...
     'cfar.Pfa','cfar.weak_snr_db','cfar.os_fraction', ...
     'track.group_final_mean_amf_db','track.group_final_mean_cfar_db', ...
     'track.group_final_min_hits','track.group_final_min_support', ...
     'track.stationary_group_final_mean_amf_db','track.stationary_group_final_min_hits', ...
     'track.gate_range_sigma','track.gate_nis', ...
     'group.position_gate_m','group.angle_gate_deg', ...
     'tbd.path_promotion_score','tbd.min_path_support_fraction','tbd.path_min_amf_db', ...
     'tbd.coherent.path_score_threshold','tbd.coherent.coherent_score_threshold_db', ...
     'paper.stationary.Pfa'};
end

function d = defaults_struct()
persistent P
if isempty(P), P = radar_configuration(struct()); end
d = P;
end

% =========================================================================
function m = pool(acc)
m = struct('pd',NaN,'false_per_frame',NaN,'range_rmse',NaN,'velocity_rmse',NaN, ...
    'angle_rmse',NaN,'track_continuity',NaN,'trials',numel(acc), ...
    'miss_cfar',0,'miss_amf',0,'miss_group',0,'miss_track',0, ...
    'false_moving',0,'false_stationary',0,'false_tbd_dp',0,'false_tbd_coherent',0);
if isempty(acc), return; end
m.pd = sum(arrayfun(@(a) a.matched,acc))/max(sum(arrayfun(@(a) a.truth_count,acc)),1);
m.false_per_frame = sum(arrayfun(@(a) a.false_objects,acc))/max(sum(arrayfun(@(a) a.frames,acc)),1);
m.range_rmse    = mean(arrayfun(@(a) a.range_rmse,acc),'omitnan');
m.velocity_rmse = mean(arrayfun(@(a) a.velocity_rmse,acc),'omitnan');
m.angle_rmse    = mean(arrayfun(@(a) a.angle_rmse,acc),'omitnan');
m.track_continuity = mean(arrayfun(@(a) a.track_continuity,acc),'omitnan');
for f = {'miss_cfar','miss_amf','miss_group','miss_track', ...
         'false_moving','false_stationary','false_tbd_dp','false_tbd_coherent'}
    m.(f{1}) = sum(arrayfun(@(a) a.(f{1}),acc));
end
end

function r = pack_row(m)
r = study_row();
f = intersect(fieldnames(r),fieldnames(m));
for i = 1:numel(f), r.(f{i}) = m.(f{i}); end
% Utility is reported for convenience, but selection is lexicographic.
r.utility = m.pd - 0.20*log1p(max(m.false_per_frame,0));
end

function best = pick_best(rows)
%PICK_BEST  Lexicographic: fewest false objects, then highest Pd.
if isempty(rows), best = study_row(); return; end
fp = [rows.false_per_frame]; pd = [rows.pd];
fp(~isfinite(fp)) = Inf; pd(~isfinite(pd)) = -Inf;
minFp = min(fp);
cand = find(fp <= minFp + 1e-9);
[~,k] = max(pd(cand));
best = rows(cand(k));
end

function print_row(r)
fprintf('  %-52s Pd=%.4f  false/frame=%.4f  angRMSE=%.3f\n', ...
    r.label,r.pd,r.false_per_frame,r.angle_rmse);
end

% =========================================================================
% Dotted-path access
% =========================================================================
function out = set_path(s,path,v)
%SET_PATH  Write v at a dotted path, creating intermediate structs.
%   Writes exactly one leaf and nothing else.
parts = strsplit(char(path),'.');
out = s;
if numel(parts) == 1
    out.(parts{1}) = v;
return;
end
key = parts{1};
if isfield(out,key) && isstruct(out.(key)), child = out.(key); else, child = struct(); end
out.(key) = set_path(child,strjoin(parts(2:end),'.'),v);
end

function v = get_path(s,path)
parts = strsplit(char(path),'.');
v = [];
cur = s;
for i = 1:numel(parts)
    if ~isstruct(cur) || ~isfield(cur,parts{i}), return; end
    cur = cur.(parts{i});
end
v = cur;
end

function v = value_at(values,i)
if iscell(values), v = values{i}; else, v = values(i); end
end

function s = format_value(v)
if isnumeric(v) && isscalar(v), s = sprintf('%g',v);
elseif isnumeric(v), s = mat2str(v);
else, s = char(string(v)); end
end

function x = num_or_nan(v)
if isnumeric(v) && isscalar(v), x = double(v); else, x = NaN; end
end

function r = study_row()
r = struct('label','','parameter','','value',NaN,'parameter2','','value2',NaN, ...
    'candidate_index',0,'pd',NaN,'false_per_frame',NaN,'utility',NaN, ...
    'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN,'track_continuity',NaN, ...
    'trials',0,'miss_cfar',0,'miss_amf',0,'miss_group',0,'miss_track',0, ...
    'false_moving',0,'false_stationary',0,'false_tbd_dp',0,'false_tbd_coherent',0);
end

function r = sens_row()
r = struct('parameter','','baseline',NaN,'influence',NaN, ...
    'd_pd_up',NaN,'d_pd_down',NaN,'d_false_up',NaN,'d_false_down',NaN);
end

function o = parse_opts(o,varargin)
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(o,key)
        error('run_parameter_studies:Args','Unknown option "%s".',key);
    end
    o.(key) = varargin{k+1};
end
end

function write_rows(rows,path)
%WRITE_ROWS  Write a result table, tolerating an empty result set.
%   struct2table rejects a zero-element struct array, so a study that produced
%   no rows would fail at the point of reporting rather than reporting that it
%   found nothing.
if isempty(rows)
    fid = fopen(path,'w');
    if fid > 0
        fprintf(fid,'no rows produced\n');
        fclose(fid);
    end
    return;
end
writetable(struct2table(rows),path);
end
