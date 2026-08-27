function S = run_radar_publication_monte_carlo(varargin)
%RUN_RADAR_PUBLICATION_MONTE_CARLO  Randomised-scene statistical campaign.
%
%   Executes the same pipeline RUN_RADAR_PROJECT executes, once per
%   (SNR condition, trial), and adds the experimental discipline that turns a
%   collection of runs into a measurement:
%
%     * physically meaningful nuisance variables are randomised per trial —
%       target count, range, velocity, cross-section, azimuth, clutter,
%       interference and receiver noise figure;
%     * scene seeds depend on the trial index but not on the condition, so the
%       same physical scene is reused across the SNR sweep and the comparison
%       is paired. Pairing removes scene-to-scene variance from the difference
%       between conditions, which is the variance that otherwise dominates at
%       small trial counts;
%     * train, validation and test splits occupy disjoint seed spaces, so a
%       parameter set tuned on one split has never seen the scenes it is
%       reported on;
%     * frame-level detection is reported separately from final-object
%       detection, because they fail for different reasons;
%     * every miss is attributed to the stage that lost it and every false
%       object to the stage that produced it.
%
%   INTERVALS
%   Proportions use the Wilson score interval, which stays inside [0,1] and
%   remains meaningful when the count is 0 or n — exactly where a development
%   run sits and exactly where the normal approximation fails. Continuous
%   metrics use a percentile bootstrap, imposing no distributional assumption
%   on quantities such as false objects per frame, whose distribution is
%   skewed and bounded below.
%
%   Options
%     'split'          'train' | 'validation' | 'test'        ('test')
%     'snr_values'     SNR grid in dB                    (-10:2:20)
%     'trials'         scenes per condition                     (100)
%     'frames'         frames per scene                          (16)
%     'config'         configuration override applied to every trial
%     'output_dir'     results root            (pwd/docs/results)
%     'save_figures'   write the curve suite                   (true)
%     'verbose'        progress reporting                      (true)
%
%   Example
%     S = run_radar_publication_monte_carlo('split','test', ...
%             'snr_values',-10:2:20,'trials',200,'frames',16);

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('split','test','trials',100,'frames',16,'master_seed',20260822, ...
    'snr_values',-10:2:20,'output_dir',fullfile(pwd,'docs','results'), ...
    'config',struct(),'min_targets',4,'max_targets',10, ...
    'stationary_fraction',0.25,'allow_alias_velocity',false, ...
    'randomize_clutter',true,'randomize_interference',true,'randomize_sensor',true, ...
    'alpha',0.05,'bootstrap',2000,'save_figures',true,'verbose',true, ...
    'append',true,'run_label','','store','campaign');
opts = parse_opts(opts,varargin{:});

outdir = fullfile(opts.output_dir,'publication_monte_carlo');
if ~exist(outdir,'dir'), mkdir(outdir); end
split = lower(char(opts.split));
offset = radar_experiment_common('split_offset',split);
base = radar_experiment_common('base_config');
if ~isempty(fieldnames(opts.config))
    base = radar_experiment_common('merge',base,opts.config);
end

conditions = double(opts.snr_values(:)).';
rows = repmat(empty_row(),0,1);
sceneOpts = struct('min_targets',opts.min_targets,'max_targets',opts.max_targets, ...
    'stationary_fraction',opts.stationary_fraction, ...
    'allow_alias_velocity',opts.allow_alias_velocity, ...
    'randomize_clutter',opts.randomize_clutter, ...
    'randomize_interference',opts.randomize_interference, ...
    'randomize_sensor',opts.randomize_sensor);

if opts.verbose
    fprintf('\n============================================================\n');
    fprintf(' MONTE CARLO CAMPAIGN | split=%s | %d conditions x %d trials x %d frames\n', ...
        split,numel(conditions),opts.trials,opts.frames);
    fprintf('============================================================\n');
end

tCampaign = tic;
for ic = 1:numel(conditions)
    snr = conditions(ic);
    for tr = 1:opts.trials
seed = opts.master_seed + offset + 100000*tr;
        cfg = radar_experiment_common('scene',base,seed,sceneOpts);
cfg.random_seed = seed;
cfg.noise_enabled = true;
        cfg.noise_model = 'SNR-controlled AWGN';
cfg.noise_level = snr;
cfg.snr_override_enabled = true;
        ro = struct('Nframes',opts.frames,'show_figures',false,'save_diagnostics',false, ...
            'save_results',false,'notify_gui',false,'snr_override',snr);

        r = empty_row();
        r.condition = sprintf('SNR %+.1f dB',snr);
r.snr_db = snr;
r.trial = tr;
r.seed = seed;
r.frames = opts.frames;
        r.truth_targets = size(cfg.targets,1);

tTrial = tic;
        try
            out = run_radar_project(cfg,ro);
            [~,m] = radar_experiment_common('score',out.objects,out.truth,out.info,out.params);
            r = copy_metrics(r,m);
            r.frame_pd = frame_level_pd(out,cfg);
            r.wall_s = toc(tTrial);
            r.realtime_factor = (opts.frames*out.params.track.dt)/max(r.wall_s,eps);
        catch ME
            r.wall_s = toc(tTrial);
            r.error = sprintf('%s (%s)',ME.message,ME.identifier);
        end
        rows(end+1) = r;

        if opts.verbose
            if isempty(r.error)
                fprintf('  SNR %+5.1f  trial %3d/%3d  Pd=%.3f  false/frame=%.3f  %.1fs\n', ...
                    snr,tr,opts.trials,r.pd,r.false_per_frame,r.wall_s);
            else
                fprintf('  SNR %+5.1f  trial %3d/%3d  ERROR: %s\n',snr,tr,opts.trials,r.error);
            end
        end
    end
end

summary = summarize(rows,opts);
S = struct('rows',rows,'summary',summary,'options',opts,'split',split, ...
    'wall_s',toc(tCampaign),'output_dir',outdir);

% ---- accumulate sections into one store ---------------------------------
% A long campaign is run in pieces. Each call appends its trials to a single
% store and the summary is recomputed over everything gathered so far, so
% partial runs combine into one result set rather than overwriting each other.
% The store survives closing MATLAB, so a campaign can be resumed at any time.
label = opts.run_label;
if isempty(label), label = datestr(now,'yyyymmdd_HHMMSS'); end
for i = 1:numel(rows), rows(i).run_label = label; end

storeMat = fullfile(outdir,sprintf('%s_%s_store.mat',opts.store,split));
allRows = rows;
if opts.append && exist(storeMat,'file')
    prev = load(storeMat,'rows');
    if isfield(prev,'rows') && ~isempty(prev.rows)
        allRows = merge_rows(prev.rows,rows);
    end
end

pooled = summarize(allRows,opts);
S.rows = rows;
S.all_rows = allRows;
S.summary = summarize(rows,opts);
S.pooled_summary = pooled;
S.run_label = label;
S.store = storeMat;

save_store(storeMat,allRows);
if ~isempty(allRows)
    writetable(struct2table(allRows),fullfile(outdir,sprintf('%s_%s_trials.csv',opts.store,split)));
end
if ~isempty(pooled)
    writetable(struct2table(pooled),fullfile(outdir,sprintf('%s_%s_summary.csv',opts.store,split)));
end
save(fullfile(outdir,sprintf('%s_%s_results.mat',opts.store,split)),'S','-v7.3');
summary = pooled;
if opts.save_figures, make_figures(summary,split,outdir); end
if opts.verbose
    print_summary(pooled,split,S.wall_s,outdir);
    fprintf(' This section: %d trials (%s). Store now holds %d trials across %d conditions.\n', ...
        numel(rows),label,numel(allRows),numel(pooled));
    fprintf(' Store: %s\n',storeMat);
end
end

% =========================================================================
function pd = frame_level_pd(out,cfg)
%FRAME_LEVEL_PD  Detection probability of the per-frame point cloud.
%   Frame-level and object-level detection fail for different reasons: the
%   first measures whether the measurement contains the target at all, the
%   second whether persistence and existence logic kept it. Reporting both
%   separates a detection problem from a tracking problem.
pd = NaN;
det = get_default_field(out.info,'frame_detections',{});
if isempty(det) || ~iscell(det), return; end
T = cfg.targets;
if isempty(T), return; end
hit = 0;
total = 0;
for f = 1:numel(det)
    d = det{f};
    for ti = 1:size(T,1)
total = total + 1;
        if isempty(d), continue; end
        near = abs([d.range]-T(ti,1)) <= 4 & abs([d.velocity]-T(ti,2)) <= 2;
        if any(near), hit = hit + 1; end
    end
end
if total > 0, pd = hit/total;
end
end

function summary = summarize(rows,opts)
%SUMMARIZE  One record per condition, pooled across trials.
conds = unique({rows.condition},'stable');
summary = repmat(empty_summary(),0,1);
for i = 1:numel(conds)
    R = rows(strcmp({rows.condition},conds{i}));
    good = R(cellfun(@isempty,{R.error}));
    s = empty_summary();
    s.condition = conds{i};
    s.snr_db = R(1).snr_db;
    s.trials = numel(good);
    s.errors = numel(R) - numel(good);
    if isempty(good), summary(end+1) = s; continue; end

    % Detection probability pooled over targets, with a Wilson interval.
    K = sum([good.matched]); N = max(sum([good.truth_count]),1);
    [s.pd,s.pd_ci_low,s.pd_ci_high] = radar_experiment_common('wilson',K,N,opts.alpha);

    [s.false_per_frame,s.false_ci_low,s.false_ci_high] = ...
        radar_experiment_common('bootstrap',[good.false_per_frame],opts.alpha,opts.bootstrap);
    [s.range_rmse,s.range_ci_low,s.range_ci_high] = ...
        radar_experiment_common('bootstrap',[good.range_rmse],opts.alpha,opts.bootstrap);
    [s.velocity_rmse,s.velocity_ci_low,s.velocity_ci_high] = ...
        radar_experiment_common('bootstrap',[good.velocity_rmse],opts.alpha,opts.bootstrap);
    [s.angle_rmse,s.angle_ci_low,s.angle_ci_high] = ...
        radar_experiment_common('bootstrap',[good.angle_rmse],opts.alpha,opts.bootstrap);

    s.frame_pd = mean([good.frame_pd],'omitnan');
    s.track_continuity = mean([good.track_continuity],'omitnan');
    s.realtime_factor = mean([good.realtime_factor],'omitnan');
    s.wall_s_per_trial = mean([good.wall_s],'omitnan');

    % Stage attribution as a fraction of all misses and all ghosts.
    misses = sum([good.miss_absent]) + sum([good.miss_pre_cfar]) + sum([good.miss_cfar]) + ...
             sum([good.miss_amf]) + sum([good.miss_group]) + sum([good.miss_track]);
    md = max(misses,1);
    s.miss_absent_frac   = sum([good.miss_absent])/md;
    s.miss_pre_cfar_frac = sum([good.miss_pre_cfar])/md;
    s.miss_cfar_frac     = sum([good.miss_cfar])/md;
    s.miss_amf_frac      = sum([good.miss_amf])/md;
    s.miss_group_frac    = sum([good.miss_group])/md;
    s.miss_track_frac    = sum([good.miss_track])/md;

    ghosts = sum([good.false_moving]) + sum([good.false_stationary]) + ...
             sum([good.false_tbd_dp]) + sum([good.false_tbd_coherent]);
    gd = max(ghosts,1);
    s.false_moving_frac     = sum([good.false_moving])/gd;
    s.false_stationary_frac = sum([good.false_stationary])/gd;
    s.false_tbd_dp_frac     = sum([good.false_tbd_dp])/gd;
    s.false_tbd_coh_frac    = sum([good.false_tbd_coherent])/gd;
s.total_misses = misses;
s.total_ghosts = ghosts;

    summary(end+1) = s;
end
end

function make_figures(summary,split,outdir)
%MAKE_FIGURES  Curve suite over every condition gathered so far.
%   A section in which every trial errored yields no summary rows; indexing an
%   empty struct array would fail at the point of reporting rather than
%   reporting that nothing succeeded.
if isempty(summary), return; end
x = [summary.snr_db];
curve(x,[summary.pd],[summary.pd_ci_low],[summary.pd_ci_high], ...
    'Final-object detection probability','P_d',[split '_final_pd.png'],outdir);
curve(x,[summary.frame_pd],[],[], ...
    'Frame-level detection probability','P_d',[split '_frame_pd.png'],outdir);
curve(x,[summary.false_per_frame],[summary.false_ci_low],[summary.false_ci_high], ...
    'False objects per frame','objects / frame',[split '_false_per_frame.png'],outdir);
curve(x,[summary.range_rmse],[summary.range_ci_low],[summary.range_ci_high], ...
    'Range RMSE','m',[split '_range_rmse.png'],outdir);
curve(x,[summary.velocity_rmse],[summary.velocity_ci_low],[summary.velocity_ci_high], ...
    'Velocity RMSE','m/s',[split '_velocity_rmse.png'],outdir);
curve(x,[summary.angle_rmse],[summary.angle_ci_low],[summary.angle_ci_high], ...
    'Angle RMSE','deg',[split '_angle_rmse.png'],outdir);
curve(x,[summary.track_continuity],[],[], ...
    'Track continuity','fraction',[split '_track_continuity.png'],outdir);
curve(x,[summary.realtime_factor],[],[], ...
    'Real-time factor','simulated / wall',[split '_realtime_factor.png'],outdir);
stacked(x,[[summary.miss_pre_cfar_frac];[summary.miss_cfar_frac]; ...
           [summary.miss_amf_frac];[summary.miss_group_frac];[summary.miss_track_frac]], ...
    {'pre-CFAR','CFAR','verification','grouping','tracking'}, ...
    'Where targets are lost',[split '_miss_attribution.png'],outdir);
stacked(x,[[summary.false_moving_frac];[summary.false_stationary_frac]; ...
           [summary.false_tbd_dp_frac];[summary.false_tbd_coh_frac]], ...
    {'moving grouping','stationary branch','TBD dynamic programming','TBD coherent'}, ...
    'Where false objects originate',[split '_false_attribution.png'],outdir);
end

function curve(x,y,lo,hi,ttl,ylab,name,outdir)
f = figure('Visible','off','Color','w','Position',[100 100 760 460]);
ax = axes(f); hold(ax,'on'); grid(ax,'on');
if ~isempty(lo) && ~isempty(hi) && all(isfinite(lo)) && all(isfinite(hi))
    fill(ax,[x fliplr(x)],[lo fliplr(hi)],[0.30 0.45 0.75], ...
        'FaceAlpha',0.18,'EdgeColor','none');
end
plot(ax,x,y,'-o','LineWidth',1.8,'MarkerSize',5,'Color',[0.15 0.30 0.60]);
xlabel(ax,'SNR (dB)'); ylabel(ax,ylab); title(ax,ttl);
saveas(f,fullfile(outdir,name)); close(f);
end

function stacked(x,Y,names,ttl,name,outdir)
f = figure('Visible','off','Color','w','Position',[100 100 760 460]);
ax = axes(f);
Y(~isfinite(Y)) = 0;
bar(ax,x,Y.','stacked');
grid(ax,'on'); ylim(ax,[0 1]);
xlabel(ax,'SNR (dB)'); ylabel(ax,'fraction'); title(ax,ttl);
legend(ax,names,'Location','eastoutside');
saveas(f,fullfile(outdir,name)); close(f);
end

function print_summary(summary,split,wall,outdir)
fprintf('\n------------------------------------------------------------\n');
fprintf(' CAMPAIGN SUMMARY | split=%s | %.1f s\n',split,wall);
fprintf('%8s %7s %7s %18s %18s\n','SNR','trials','errors','Pd [95%% CI]','false/frame [95%% CI]');
for i = 1:numel(summary)
    s = summary(i);
    fprintf('%+8.1f %7d %7d  %.3f [%.3f,%.3f]  %.3f [%.3f,%.3f]\n', ...
        s.snr_db,s.trials,s.errors,s.pd,s.pd_ci_low,s.pd_ci_high, ...
        s.false_per_frame,s.false_ci_low,s.false_ci_high);
end
fprintf('\n Results written to %s\n',outdir);
fprintf('------------------------------------------------------------\n');
end

% =========================================================================
function r = copy_metrics(r,m)
f = intersect(fieldnames(r),fieldnames(m));
for i = 1:numel(f), r.(f{i}) = m.(f{i}); end
end

function save_store(path,rows)
%SAVE_STORE  Persist the accumulated trial set under a stable variable name.
save(path,'rows','-v7.3');
end

function out = merge_rows(oldRows,newRows)
%MERGE_ROWS  Concatenate two trial sets that may carry different field sets.
%   A store written by an earlier revision can lack fields the current run
%   produces, and the reverse. Missing fields take their default rather than
%   the merge failing, so a campaign gathered across code changes still
%   combines into one result set.
proto = empty_row();
proto.run_label = '';
f = union(fieldnames(oldRows),fieldnames(newRows));
for i = 1:numel(f)
    if ~isfield(proto,f{i}), proto.(f{i}) = []; end
end
out = repmat(proto,0,1);
for src = {oldRows,newRows}
    R = src{1};
    for i = 1:numel(R)
        r = proto;
        g = fieldnames(R(i));
        for k = 1:numel(g), r.(g{k}) = R(i).(g{k}); end
        out(end+1) = r;
    end
end
end

function r = empty_row()
r = struct('condition','','snr_db',NaN,'trial',0,'seed',0,'frames',0, ...
    'truth_targets',0,'truth_count',0,'object_count',0,'matched',0, ...
    'pd',NaN,'frame_pd',NaN,'false_objects',0,'false_per_frame',NaN,'missed',0, ...
    'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN,'track_continuity',NaN, ...
    'miss_absent',0,'miss_pre_cfar',0,'miss_cfar',0,'miss_amf',0, ...
    'miss_group',0,'miss_track',0,'false_moving',0,'false_stationary',0, ...
    'false_tbd_dp',0,'false_tbd_coherent',0,'false_recovery',0,'false_persistent',0, ...
    'wall_s',NaN,'realtime_factor',NaN,'error','','run_label','');
end

function s = empty_summary()
s = struct('condition','','snr_db',NaN,'trials',0,'errors',0, ...
    'pd',NaN,'pd_ci_low',NaN,'pd_ci_high',NaN,'frame_pd',NaN, ...
    'false_per_frame',NaN,'false_ci_low',NaN,'false_ci_high',NaN, ...
    'range_rmse',NaN,'range_ci_low',NaN,'range_ci_high',NaN, ...
    'velocity_rmse',NaN,'velocity_ci_low',NaN,'velocity_ci_high',NaN, ...
    'angle_rmse',NaN,'angle_ci_low',NaN,'angle_ci_high',NaN, ...
    'track_continuity',NaN,'realtime_factor',NaN,'wall_s_per_trial',NaN, ...
    'miss_absent_frac',NaN,'miss_pre_cfar_frac',NaN,'miss_cfar_frac',NaN, ...
    'miss_amf_frac',NaN,'miss_group_frac',NaN,'miss_track_frac',NaN, ...
    'false_moving_frac',NaN,'false_stationary_frac',NaN, ...
    'false_tbd_dp_frac',NaN,'false_tbd_coh_frac',NaN, ...
    'total_misses',0,'total_ghosts',0);
end

function o = parse_opts(o,varargin)
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    switch key
        case {'seed','master_seed'}, o.master_seed = round(varargin{k+1});
        case {'snr','snr_values'},   o.snr_values = double(varargin{k+1});
        otherwise
            if ~isfield(o,key)
                error('run_radar_publication_monte_carlo:Args','Unknown option "%s".',key);
            end
            o.(key) = varargin{k+1};
    end
end
end
