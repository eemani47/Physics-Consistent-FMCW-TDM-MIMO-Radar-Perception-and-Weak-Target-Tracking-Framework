function B = run_radar_benchmarks(varargin)
%RUN_RADAR_BENCHMARKS  Suite-level performance characterisation.
%
%   Wraps the Monte Carlo engine in three sweeps that answer questions a
%   single SNR curve cannot:
%
%     'snr'   how performance degrades as the target signal weakens
%     'rcs'   how it degrades as the targets themselves get smaller, which is
%             a different question because cross-section changes the target
%             but leaves the noise floor alone
%     'arch'  how transmit and receive counts propagate into angular accuracy
%             and object quality, which is the hardware trade
%
%   Each condition runs the identical pipeline through the identical engine,
%   so suites are directly comparable to each other and to a plain campaign.
%
%   Options
%     'suite'    'snr' | 'rcs' | 'arch' | 'all'          ('snr')
%     'trials'   scenes per condition                     (10)
%     'frames'   frames per scene                         (12)
%     'config'   configuration override for every condition
%     'snr_values' / 'rcs_offsets_db' / 'architectures'
%
%   Example
%     B = run_radar_benchmarks('suite','arch','trials',8,'frames',12);

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('suite','snr','trials',10,'frames',12,'master_seed',20260822, ...
    'snr_values',-10:5:20,'rcs_offsets_db',-12:4:12, ...
    'architectures',[1 4; 2 4; 3 4; 2 8], ...
    'config',struct(),'output_dir',fullfile(pwd,'docs','results'), ...
    'alpha',0.05,'verbose',true);
opts = parse_opts(opts,varargin{:});

outdir = fullfile(opts.output_dir,'benchmarks');
if ~exist(outdir,'dir'), mkdir(outdir); end

suites = lower(char(opts.suite));
if strcmp(suites,'all'), list = {'snr','rcs','arch'}; else, list = {suites}; end

B = struct('suites',struct(),'options',opts,'output_dir',outdir);
for i = 1:numel(list)
    name = list{i};
    if opts.verbose
        fprintf('\n============================================================\n');
        fprintf(' BENCHMARK SUITE: %s\n',upper(name));
        fprintf('============================================================\n');
    end
    switch name
        case 'snr',  R = suite_snr(opts);
        case 'rcs',  R = suite_rcs(opts);
        case 'arch', R = suite_arch(opts);
        otherwise, error('run_radar_benchmarks:Suite','Unknown suite "%s".',name);
    end
    B.suites.(name) = R;
    if ~isempty(R.summary)
        writetable(struct2table(R.summary),fullfile(outdir,[name '_summary.csv']));
    end
    if opts.verbose, print_table(R.summary,name); end
end
save(fullfile(outdir,'benchmarks.mat'),'B','-v7.3');
end

% =========================================================================
function R = suite_snr(opts)
%SUITE_SNR  Performance against signal-to-noise ratio at fixed geometry.
S = run_radar_publication_monte_carlo('split','test','snr_values',opts.snr_values, ...
    'trials',opts.trials,'frames',opts.frames,'config',opts.config, ...
    'master_seed',opts.master_seed,'output_dir',opts.output_dir, ...
    'save_figures',false,'verbose',opts.verbose);
R = struct('rows',S.rows,'summary',relabel(S.summary,arrayfun(@(s) ...
    sprintf('SNR %+.1f dB',s.snr_db),S.summary,'UniformOutput',false)));
end

function R = suite_rcs(opts)
%SUITE_RCS  Performance against target cross-section at fixed noise.
%   The SNR sweep changes the noise floor; this sweep changes the targets.
%   The two produce different curves because cross-section also alters which
%   targets dominate the scene, and therefore which one the weakest-target
%   SNR reference is anchored to.
rows = repmat(bench_row(),0,1);
summary = repmat(bench_summary(),0,1);
for i = 1:numel(opts.rcs_offsets_db)
    off = opts.rcs_offsets_db(i);
    S = run_condition(opts,sprintf('RCS %+.1f dB',off),off,@(c) shift_rcs(c,off));
    rows = [rows S.rows];
    summary(end+1) = S.summary;
end
R = struct('rows',rows,'summary',summary);
end

function R = suite_arch(opts)
%SUITE_ARCH  Performance against array architecture.
%   Transmit count sets the TDM ambiguity and the virtual aperture length;
%   receive count sets the per-snapshot spatial degrees of freedom. Both
%   propagate into angular accuracy and therefore into object quality.
rows = repmat(bench_row(),0,1);
summary = repmat(bench_summary(),0,1);
for i = 1:size(opts.architectures,1)
    nTx = opts.architectures(i,1); nRx = opts.architectures(i,2);
    label = sprintf('%dTX x %dRX',nTx,nRx);
    S = run_condition(opts,label,nTx*nRx,@(c) set_array(c,nTx,nRx));
    rows = [rows S.rows];
    summary(end+1) = S.summary;
end
R = struct('rows',rows,'summary',summary);
end

% =========================================================================
function S = run_condition(opts,label,axisValue,shim)
%RUN_CONDITION  One benchmark condition at a fixed mid-band SNR.
base = radar_experiment_common('base_config');
if ~isempty(fieldnames(opts.config))
    base = radar_experiment_common('merge',base,opts.config);
end
snr = median(opts.snr_values);
sceneOpts = struct('min_targets',4,'max_targets',10,'stationary_fraction',0.25);
rows = repmat(bench_row(),0,1);

for tr = 1:opts.trials
    seed = opts.master_seed + radar_experiment_common('split_offset','test') + 100000*tr;
    cfg = radar_experiment_common('scene',base,seed,sceneOpts);
    cfg = shim(cfg);
cfg.random_seed = seed;
cfg.noise_enabled = true;
    cfg.noise_model = 'SNR-controlled AWGN';
cfg.noise_level = snr;
cfg.snr_override_enabled = true;
    ro = struct('Nframes',opts.frames,'show_figures',false,'save_diagnostics',false, ...
        'save_results',false,'notify_gui',false,'snr_override',snr);

    r = bench_row();
r.condition = label;
r.axis_value = axisValue;
r.trial = tr;
r.seed = seed;
r.frames = opts.frames;
t0 = tic;
    try
        out = run_radar_project(cfg,ro);
        [~,m] = radar_experiment_common('score',out.objects,out.truth,out.info,out.params);
        f = intersect(fieldnames(r),fieldnames(m));
        for k = 1:numel(f), r.(f{k}) = m.(f{k}); end
        r.wall_s = toc(t0);
    catch ME
        r.wall_s = toc(t0);
        r.error = sprintf('%s (%s)',ME.message,ME.identifier);
    end
    rows(end+1) = r;
    if opts.verbose
        fprintf('  %-14s trial %3d/%3d  Pd=%.3f  false/frame=%.3f\n', ...
            label,tr,opts.trials,r.pd,r.false_per_frame);
    end
end

good = rows(cellfun(@isempty,{rows.error}));
s = bench_summary();
s.condition = label;
s.axis_value = axisValue;
s.trials = numel(good); s.errors = numel(rows)-numel(good);
if ~isempty(good)
    K = sum([good.matched]); N = max(sum([good.truth_count]),1);
    [s.pd,s.pd_ci_low,s.pd_ci_high] = radar_experiment_common('wilson',K,N,opts.alpha);
    [s.false_per_frame,s.false_ci_low,s.false_ci_high] = ...
        radar_experiment_common('bootstrap',[good.false_per_frame],opts.alpha,1000);
    s.range_rmse    = mean([good.range_rmse],'omitnan');
    s.velocity_rmse = mean([good.velocity_rmse],'omitnan');
    s.angle_rmse    = mean([good.angle_rmse],'omitnan');
    s.track_continuity = mean([good.track_continuity],'omitnan');
    s.wall_s_per_trial = mean([good.wall_s],'omitnan');
end
S = struct('rows',rows,'summary',s);
end

function cfg = shift_rcs(cfg,offsetDb)
if isfield(cfg,'targets') && ~isempty(cfg.targets)
    cfg.targets(:,3) = cfg.targets(:,3) + offsetDb;
end
end

function cfg = set_array(cfg,nTx,nRx)
cfg.n_tx = nTx;
cfg.n_rx = nRx;
end

function s = relabel(s,labels)
for i = 1:numel(s), s(i).condition = labels{i}; end
end

function print_table(summary,name)
fprintf('\n%-16s %7s %18s %18s %9s %9s\n', ...
    upper(name),'trials','Pd [95%% CI]','false/frame','ang RMSE','cont.');
for i = 1:numel(summary)
    s = summary(i);
    fprintf('%-16s %7d  %.3f [%.3f,%.3f]  %.3f [%.3f,%.3f] %9.3f %9.3f\n', ...
        s.condition,s.trials,s.pd,s.pd_ci_low,s.pd_ci_high, ...
        s.false_per_frame,s.false_ci_low,s.false_ci_high, ...
        s.angle_rmse,s.track_continuity);
end
end

function r = bench_row()
r = struct('condition','','axis_value',NaN,'trial',0,'seed',0,'frames',0, ...
    'truth_count',0,'matched',0,'pd',NaN,'false_objects',0,'false_per_frame',NaN, ...
    'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN,'track_continuity',NaN, ...
    'miss_cfar',0,'miss_amf',0,'miss_group',0,'miss_track',0, ...
    'false_moving',0,'false_stationary',0,'false_tbd_dp',0,'false_tbd_coherent',0, ...
    'wall_s',NaN,'error','');
end

function s = bench_summary()
s = struct('condition','','axis_value',NaN,'trials',0,'errors',0, ...
    'pd',NaN,'pd_ci_low',NaN,'pd_ci_high',NaN, ...
    'false_per_frame',NaN,'false_ci_low',NaN,'false_ci_high',NaN, ...
    'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN, ...
    'track_continuity',NaN,'wall_s_per_trial',NaN);
end

function o = parse_opts(o,varargin)
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    switch key
        case {'seed','master_seed'},      o.master_seed = round(varargin{k+1});
        case {'snr','snr_values'},        o.snr_values = double(varargin{k+1});
        case {'rcs','rcs_offsets_db'},    o.rcs_offsets_db = double(varargin{k+1});
        case {'arch','architectures'},    o.architectures = round(varargin{k+1});
        otherwise
            if ~isfield(o,key)
                error('run_radar_benchmarks:Args','Unknown option "%s".',key);
            end
            o.(key) = varargin{k+1};
    end
end
end
