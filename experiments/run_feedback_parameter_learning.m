function L = run_feedback_parameter_learning(varargin)
%RUN_FEEDBACK_PARAMETER_LEARNING  Closed-loop tuning for zero false objects.
%
%   Drives the detector, grouping, tracking and weak-target gates toward a
%   stated operating point: no false objects, then the highest detection
%   probability that operating point allows. The physical scenes are simulated
%   once and cached; every iteration replays the real downstream chain against
%   that cache, so the loop measures parameter effects and nothing else.
%
%   WHY THE OBJECTIVE IS LEXICOGRAPHIC
%   A weighted sum lets detection probability buy false objects: raise Pd by
%   0.1, accept two more ghosts, and the scalar improves. That is the wrong
%   trade when the requirement is zero false objects. Candidates are therefore
%   ordered lexicographically —
%
%       1. false objects per frame, until the budget is met
%       2. detection probability, maximised subject to that budget
%       3. position and velocity error
%
%   so no amount of Pd can purchase a ghost while the budget is violated, and
%   once it is met the loop spends everything on recall.
%
%   HOW THE LOOP MOVES
%   Eight controllers own disjoint parameter groups. Each forms an error from
%   the failure it is responsible for — misses attributed to its stage minus
%   false objects attributed to its stage — and applies a bounded PID update
%
%       u = Kp e + Ki I + Kd D
%
%   with clamped integral action, so a persistently unsatisfiable objective
%   cannot drive a gate to its limit and pin it there. The sign convention is
%   uniform: a positive error means the stage is losing targets and its gates
%   relax; a negative error means the stage is emitting ghosts and its gates
%   tighten.
%
%   Every bound brackets the shipped default, so the first update of a channel
%   is a genuine step rather than a jump to a preset limit.
%
%   BACKTRACKING
%   An iteration that worsens the lexicographic objective is rejected: the
%   parameter vector is restored, the step size is halved, and the loop
%   continues. This converts a fixed-gain controller into a descent method and
%   is what stops the loop oscillating between over- and under-suppression.
%
%   WHAT IS NOT TUNED
%   The calibrated CFAR false-alarm probability is locked and restored after
%   every update. It is a statistical guarantee; a learner that moves it is
%   optimising away the property that makes the detector interpretable. False
%   objects are suppressed by the stages that can be traded against recall —
%   verification, grouping, existence and weak-target promotion — never by
%   quietly desensitising the primary detector.
%
%   PERSISTENCE
%   The winning parameter set is written to core/config/learned_defaults.mat.
%   Every later call to RADAR_CONFIGURATION, RUN_RADAR_PROJECT and the
%   interface loads it automatically as a default layer beneath explicit
%   caller overrides, so the tuned operating point survives a MATLAB restart.
%
%   Options
%     'trials'                 scenes per SNR point                    (2)
%     'frames'                 frames per scene                        (8)
%     'snr_values'             SNR grid for the training cache  ([0 6 12 18])
%     'iterations'             feedback iterations                     (8)
%     'target_pd'              detection-probability goal              (0.95)
%     'target_false_per_frame' false-object budget                     (0)
%     'mode'                   'zero_false' | 'balanced' | 'recall'
%     'validation'             run a held-out campaign afterwards      (true)
%     'persist'                write learned defaults                  (true)
%     'seed_from_learned'      start from the last learned set        (false)
%
%   Examples
%     L = run_feedback_parameter_learning('mode','zero_false','iterations',10);
%     L = run_feedback_parameter_learning('trials',4,'snr_values',[0 10 20]);

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('trials',2,'frames',8,'snr_values',[0 6 12 18], ...
    'master_seed',20260822,'output_dir',fullfile(pwd,'docs','results','feedback_learning'), ...
    'iterations',8,'target_pd',0.95,'target_false_per_frame',0,'mode','zero_false', ...
    'verbose',true,'validation',true,'validation_trials',[],'validation_snr_values',[], ...
    'persist',true,'seed_from_learned',false,'min_step_scale',0.125, ...
    'min_targets',4,'max_targets',10,'stationary_fraction',0.25);
opts = parse_opts(opts,varargin{:});
switch lower(char(opts.mode))
    case 'zero_false', opts.target_false_per_frame = 0;    opts.recall_weight = 1.0;
    case 'balanced',   opts.recall_weight = 1.5;
    case 'recall',     opts.recall_weight = 3.0;
    otherwise, error('run_feedback_parameter_learning:Mode', ...
        'mode must be zero_false, balanced or recall.');
end
if ~exist(opts.output_dir,'dir'), mkdir(opts.output_dir); end

% ---- physical scene cache, simulated once --------------------------------
cache = build_scene_cache(opts);
if isempty(cache)
    error('run_feedback_parameter_learning:EmptyCache','No training scenes were generated.');
end

p0 = cache(1).params;
if opts.seed_from_learned
    lp = fullfile(root,'core','config','learned_defaults.mat');
    if exist(lp,'file')
        S = load(lp,'best');
        if isfield(S,'best'), p0 = radar_experiment_common('apply_tuned',p0,S.best); end
    end
end

state = initial_state(p0);
trace = repmat(empty_trace(),0,1);

metrics   = evaluate_cache(cache,state.params);
best      = state.params;
bestM     = metrics;
stepScale = 1.0;

if opts.verbose
    print_header(opts,numel(cache));
    print_iteration(0,opts.iterations,metrics,0,stepScale,'baseline');
end
trace(end+1) = make_trace(0,metrics,state.params,stepScale,'baseline');

for it = 1:opts.iterations
prevParams = state.params;
    state = feedback_update(state,metrics,opts,stepScale);
    candidateM = evaluate_cache(cache,state.params);

    if better_than(candidateM,bestM,opts)
best = state.params;
bestM = candidateM;
        metrics = candidateM; verdict = 'accepted';
        stepScale = min(1.0,stepScale*1.35);
    else
        % Reject and shrink. The controller keeps its integral memory so the
        % next, smaller step still moves in the direction the error demands.
state.params = prevParams;
metrics = candidateM;
        verdict = 'rejected, step halved';
        stepScale = max(opts.min_step_scale,stepScale*0.5);
    end

    trace(end+1) = make_trace(it,candidateM,state.params,stepScale,verdict);
    if opts.verbose
        print_iteration(it,opts.iterations,candidateM,getf(state,'max_param_delta',0),stepScale,verdict);
    end
    if bestM.false_per_frame <= opts.target_false_per_frame && bestM.pd >= opts.target_pd
        if opts.verbose, fprintf('\n[LEARNING] Operating point reached; stopping early.\n'); end
break;
    end
end

if opts.verbose, print_summary(bestM,opts); end

% ---- independent held-out validation -------------------------------------
tuned = extract_tuned(best);
if opts.validation
    vSNR = opts.validation_snr_values; if isempty(vSNR), vSNR = opts.snr_values; end
    vTrials = opts.validation_trials;  if isempty(vTrials), vTrials = max(2,opts.trials); end
    if opts.verbose
        fprintf('\n[LEARNING] Held-out validation: %d trials x %d frames over %d SNR points.\n', ...
            vTrials,opts.frames,numel(vSNR));
    end
    V = run_radar_publication_monte_carlo('split','validation','snr_values',vSNR, ...
        'trials',vTrials,'frames',opts.frames,'config',tuned,'verbose',opts.verbose);
else
    V = struct('enabled',false,'reason','validation disabled by caller');
end

% ---- persistence ---------------------------------------------------------
learnedPath = fullfile(root,'core','config','learned_defaults.mat');
if opts.persist
    save(learnedPath,'best','tuned','-v7.3');
    write_param_json(tuned,fullfile(root,'core','config','learned_parameters.json'));
    if opts.verbose
        fprintf('[LEARNING] Learned defaults written to %s\n',learnedPath);
        fprintf('[LEARNING] They apply automatically to every later run, including the interface.\n');
    end
end

L = struct('best_params',best,'best_tuned',tuned,'best_metrics',bestM, ...
    'trace',trace,'validation',V,'options',opts,'cache_size',numel(cache), ...
    'learned_default_path',learnedPath);
save(fullfile(opts.output_dir,'feedback_learning.mat'),'L','-v7.3');
write_learning_csv(trace,fullfile(opts.output_dir,'learning_trace.csv'));
write_param_json(tuned,fullfile(opts.output_dir,'learned_parameters.json'));
end

% =========================================================================
% Scene cache
% =========================================================================
function cache = build_scene_cache(opts)
%BUILD_SCENE_CACHE  Simulate every training scene once and keep the frames.
%
%   The physical simulator dominates the cost of a learning run. Caching the
%   receive cubes and replaying only the downstream chain makes an iteration
%   roughly two orders of magnitude cheaper, which is what makes closed-loop
%   tuning practical at all. The cache is the reason the loop can afford to
%   backtrack on a rejected step.
cache = repmat(empty_cache(),0,1);
base  = radar_experiment_common('base_config');
sceneOpts = struct('min_targets',opts.min_targets,'max_targets',opts.max_targets, ...
    'stationary_fraction',opts.stationary_fraction);
ci = 0;
for si = 1:numel(opts.snr_values)
    for tr = 1:opts.trials
ci = ci + 1;
        seed = opts.master_seed + radar_experiment_common('split_offset','train') + 100000*tr;
        cfg = radar_experiment_common('scene',base,seed,sceneOpts);
cfg.random_seed = seed;
cfg.noise_enabled = true;
        cfg.noise_model = 'SNR-controlled AWGN';
        cfg.noise_level = opts.snr_values(si);
cfg.snr_override_enabled = true;
        ro = struct('Nframes',opts.frames,'show_figures',false,'save_diagnostics',false, ...
            'save_results',false,'notify_gui',false,'snr_override',opts.snr_values(si));
        if opts.verbose
            fprintf('[CACHE] SNR %+5.1f dB  trial %d/%d  (%d targets)\n', ...
                opts.snr_values(si),tr,opts.trials,size(cfg.targets,1));
        end
        out = run_radar_project(cfg,ro);
        cache(ci).snr_db = opts.snr_values(si);
        cache(ci).trial = tr;
        cache(ci).seed = seed;
        cache(ci).clean_frames = out.clean_frames;
        cache(ci).params = out.params;
        cache(ci).truth = out.truth;
    end
end
end

% =========================================================================
% Evaluation of one parameter vector against the cache
% =========================================================================
function M = evaluate_cache(cache,params)
%EVALUATE_CACHE  Replay the downstream chain for every cached scene.
tuned = extract_tuned(params);
% A fieldless empty cannot receive a struct that has fields. The array has to
% be created already carrying the field set it will hold.
acc = repmat(empty_trial_metrics(),0,1);
for k = 1:numel(cache)
    p = radar_experiment_common('apply_tuned',cache(k).params,tuned);
    [objects,~,info] = radar_object_tracker(cache(k).clean_frames,{},p,struct('finalize',true));
    [~,m] = radar_experiment_common('score',objects,cache(k).truth,info,p);
    acc(end+1) = m;
end
M = pool_metrics(acc);
end

function M = pool_metrics(acc)
%POOL_METRICS  Pool per-trial metrics into one record.
%   Detection probability is pooled over targets rather than averaged over
%   trials, so a scene with ten targets carries ten times the weight of a
%   scene with one. Averaging trial-level rates would let a sparse scene
%   dominate the objective.
M = struct('pd',0,'false_per_frame',0,'range_rmse',NaN,'velocity_rmse',NaN, ...
    'angle_rmse',NaN,'track_continuity',NaN,'trials',numel(acc), ...
    'total_truth',0,'total_matched',0,'total_false',0,'total_frames',0, ...
    'miss_absent',0,'miss_pre_cfar',0,'miss_cfar',0,'miss_amf',0, ...
    'miss_group',0,'miss_track',0,'false_moving',0,'false_stationary',0, ...
    'false_tbd_dp',0,'false_tbd_coherent',0,'false_recovery',0,'false_persistent',0, ...
    'false_split',0,'false_spurious',0,'coasted_objects',0);
if isempty(acc), return; end
sumf = @(f) sum(arrayfun(@(a) a.(f),acc));
M.total_truth   = sumf('truth_count');
M.total_matched = sumf('matched');
M.total_false   = sumf('false_objects');
M.total_frames  = sumf('frames');
for f = {'miss_absent','miss_pre_cfar','miss_cfar','miss_amf','miss_group','miss_track', ...
         'false_moving','false_stationary','false_tbd_dp','false_tbd_coherent', ...
         'false_recovery','false_persistent','false_split','false_spurious','coasted_objects'}
    M.(f{1}) = sumf(f{1});
end
M.pd = M.total_matched/max(M.total_truth,1);
M.false_per_frame = M.total_false/max(M.total_frames,1);
M.range_rmse    = mean(arrayfun(@(a) a.range_rmse,acc),'omitnan');
M.velocity_rmse = mean(arrayfun(@(a) a.velocity_rmse,acc),'omitnan');
M.angle_rmse    = mean(arrayfun(@(a) a.angle_rmse,acc),'omitnan');
M.track_continuity = mean(arrayfun(@(a) a.track_continuity,acc),'omitnan');
end

% =========================================================================
% Lexicographic comparison
% =========================================================================
function tf = better_than(cand,ref,opts)
%BETTER_THAN  Lexicographic ordering: budget first, recall second, error third.
budget = opts.target_false_per_frame;
cOver = max(0,cand.false_per_frame - budget);
rOver = max(0,ref.false_per_frame  - budget);

if cOver > 0 || rOver > 0
    % While either candidate violates the budget, the only thing that counts
    % is how far outside it sits.
    if abs(cOver-rOver) > 1e-9, tf = cOver < rOver; return; end
end
if abs(cand.pd - ref.pd) > 1e-6, tf = cand.pd > ref.pd; return; end
cErr = err_scalar(cand); rErr = err_scalar(ref);
tf = cErr < rErr;
end

function e = err_scalar(m)
e = nan_to(m.range_rmse,10) + 2*nan_to(m.velocity_rmse,5) + 0.25*nan_to(m.angle_rmse,10);
end

function y = nan_to(x,d), if isfinite(x), y = x; else, y = d; end, end

% =========================================================================
% Controllers
% =========================================================================
function state = initial_state(p)
names = controller_names();
z = struct();
for i = 1:numel(names), z.(names{i}) = 0; end
state = struct('params',p,'integral',z,'prev',z,'max_param_delta',0);
state.Kp = struct('amf',0.45,'moving_group',0.55,'stationary_group',0.50, ...
    'existence',0.40,'association',0.30,'tbd_dp',0.45,'tbd_coherent',0.40,'quality',0.35);
state.Ki = struct('amf',0.06,'moving_group',0.08,'stationary_group',0.08, ...
    'existence',0.05,'association',0.04,'tbd_dp',0.06,'tbd_coherent',0.05,'quality',0.04);
state.Kd = struct('amf',0.05,'moving_group',0.06,'stationary_group',0.06, ...
    'existence',0.04,'association',0.03,'tbd_dp',0.05,'tbd_coherent',0.04,'quality',0.03);
end

function n = controller_names()
n = {'amf','moving_group','stationary_group','existence','association', ...
     'tbd_dp','tbd_coherent','quality'};
end

function state = feedback_update(state,m,opts,stepScale)
%FEEDBACK_UPDATE  One bounded PID step per controller.
%
%   Error sign convention, uniform across every controller:
%       e > 0  the stage is losing targets   -> relax its gates
%       e < 0  the stage is emitting ghosts  -> tighten its gates
%   Each controller is charged only with the failures it can actually fix, so
%   two controllers never fight over the same symptom.

T = max(1,m.total_truth);
F = max(1,m.total_false);
w = opts.recall_weight;
over = max(0,m.false_per_frame - opts.target_false_per_frame);
overN = min(1,over/max(opts.target_false_per_frame,0.25));

e = struct();
% Verification owns candidates that reached CFAR but failed the template test,
% and it is the first lever against ghosts of any origin.
e.amf = clip(w*m.miss_amf/T - 0.60*(m.false_moving + 0.5*m.false_stationary)/F - 0.30*overN,-1,1);
% Grouping and existence on the moving branch.
e.moving_group = clip(w*m.miss_group/T - 0.90*m.false_moving/F,-1,1);
% The stationary branch has no truth-side miss term of its own; clutter and
% zero-Doppler leakage are its only failure mode, so it is charged with those.
e.stationary_group = clip(0.35*w*m.miss_group/T - 1.00*m.false_stationary/F,-1,1);
% Object existence owns the promotion routes that survive a marginal frame.
e.existence = clip(w*m.miss_track/T - 0.80*(m.false_recovery + m.false_persistent)/F - 0.40*overN,-1,1);
% Association owns track continuity, not thresholds.
e.association = clip(w*m.miss_track/T - 0.40*m.false_moving/F,-1,1);
% Each weak-target branch answers for its own ghosts.
e.tbd_dp = clip(0.5*w*m.miss_track/T - 1.00*m.false_tbd_dp/F - 0.40*overN,-1,1);
e.tbd_coherent = clip(0.5*w*m.miss_track/T - 1.00*m.false_tbd_coherent/F - 0.40*overN,-1,1);
% Quality scoring is the soft lever, moved last and least.
e.quality = clip(w*(m.miss_amf + m.miss_group)/T - 0.50*(m.total_false/F) - 0.20*overN,-1,1);

names = controller_names();
state.max_param_delta = 0;
for i = 1:numel(names)
    nm = names{i};
    err = e.(nm);
    I = clip(state.integral.(nm) + err,-2.5,2.5);
    D = err - state.prev.(nm);
    state.integral.(nm) = I;
    state.prev.(nm) = err;
    u = stepScale*(state.Kp.(nm)*err + state.Ki.(nm)*I + state.Kd.(nm)*D);
before = state.params;
    state.params = apply_controller(state.params,nm,u,m,opts);
    state.max_param_delta = max(state.max_param_delta,param_delta(before,state.params));
end
% The locked statistical contract is restored after every update.
state.params.cfar.Pfa = state.params.cfar.Pfa;
end

function p = apply_controller(p,name,u,m,opts)
%APPLY_CONTROLLER  Move one controller's parameters. Bounds bracket defaults.
switch name
    case 'amf'
        p.detector.min_amf_db = clip(p.detector.min_amf_db - 1.20*u,5.0,18.0);
        p.detector.amf_threshold_pfa = 10^clip(log10(p.detector.amf_threshold_pfa) + 0.25*u,-6,-1.3);

    case 'moving_group'
        p.track.group_final_mean_amf_db  = clip(p.track.group_final_mean_amf_db  - 1.20*u,4.0,18.0);
        p.track.group_final_mean_cfar_db = clip(p.track.group_final_mean_cfar_db - 0.90*u,2.0,14.0);
        p.track.group_final_angle_std_deg = clip(p.track.group_final_angle_std_deg + 0.60*u,2.0,12.0);

    case 'stationary_group'
        p.track.stationary_group_final_mean_amf_db  = clip(p.track.stationary_group_final_mean_amf_db  - 1.20*u,4.0,20.0);
        p.track.stationary_group_final_mean_cfar_db = clip(p.track.stationary_group_final_mean_cfar_db - 0.90*u,2.0,14.0);
        p.track.stationary_group_final_min_hits     = round(clip(p.track.stationary_group_final_min_hits - 0.40*u,2,8));
        p.track.stationary_group_final_min_support  = clip(p.track.stationary_group_final_min_support - 0.06*u,0.35,0.98);
        p.paper.stationary.Pfa = 10^clip(log10(p.paper.stationary.Pfa) + 0.30*u,-8,-2);

    case 'existence'
        p.track.group_final_min_hits    = round(clip(p.track.group_final_min_hits    - 0.40*u,2,8));
        p.track.group_final_min_support = clip(p.track.group_final_min_support - 0.06*u,0.35,0.98);
        p.track.group_recovery_min_hits    = round(clip(p.track.group_recovery_min_hits - 0.40*u,3,10));
        p.track.group_recovery_min_support = clip(p.track.group_recovery_min_support - 0.05*u,0.50,0.99);
        p.track.persistent_recovery_min_hits    = round(clip(p.track.persistent_recovery_min_hits - 0.40*u,3,12));
        p.track.persistent_recovery_mean_amf_db = clip(p.track.persistent_recovery_mean_amf_db - 0.90*u,8.0,22.0);

    case 'association'
        p.track.gate_range_sigma    = clip(p.track.gate_range_sigma    + 0.35*u,1.5,5.0);
        p.track.gate_velocity_sigma = clip(p.track.gate_velocity_sigma + 0.30*u,1.2,4.5);
        p.track.gate_angle_sigma    = clip(p.track.gate_angle_sigma    + 0.20*u,0.8,3.5);
        p.track.group_confirm_hits  = round(clip(p.track.group_confirm_hits - 0.30*u,2,6));
        % The NIS gate is a chi-square quantile and stays fixed; the per-axis
        % sigma gates are the association tuning degrees of freedom.

    case 'tbd_dp'
        p.tbd.path_promotion_score       = clip(p.tbd.path_promotion_score - 1.50*u,6.0,32.0);
        p.tbd.min_path_support_fraction  = clip(p.tbd.min_path_support_fraction - 0.05*u,0.45,0.99);
        p.tbd.path_min_amf_db            = clip(p.tbd.path_min_amf_db - 1.00*u,1.0,20.0);
        p.tbd.path_promotion_min_amf_db  = clip(p.tbd.path_promotion_min_amf_db - 1.00*u,1.0,22.0);
        p.tbd.path_angle_max_std_deg     = clip(p.tbd.path_angle_max_std_deg + 0.50*u,2.0,12.0);

    case 'tbd_coherent'
        p.tbd.coherent.min_support_fraction        = clip(p.tbd.coherent.min_support_fraction - 0.05*u,0.45,0.99);
        p.tbd.coherent.path_score_threshold        = clip(p.tbd.coherent.path_score_threshold - 1.00*u,3.0,18.0);
        p.tbd.coherent.coherent_score_threshold_db = clip(p.tbd.coherent.coherent_score_threshold_db - 1.00*u,3.0,20.0);
        p.tbd.coherent.path_promotion_score_db     = clip(p.tbd.coherent.path_promotion_score_db - 1.00*u,4.0,22.0);
        p.tbd.coherent.Pfa = 10^clip(log10(p.tbd.coherent.Pfa) + 0.30*u,-8,-2);

    case 'quality'
        p.cfar.weak_snr_db = clip(p.cfar.weak_snr_db + 0.50*u,-12.0,0.0);
        p.cfar.min_snr_db  = clip(p.cfar.min_snr_db  + 0.60*u,-26.0,-6.0);
        p.group.angle_gate_deg    = clip(p.group.angle_gate_deg    + 0.20*u,1.5,8.0);
        p.group.position_gate_m   = clip(p.group.position_gate_m   + 0.15*u,0.8,6.0);
end
end

function d = param_delta(a,b)
%PARAM_DELTA  Largest relative move across the tuned sections.
d = 0;
sections = radar_experiment_common('tuned_sections');
for i = 1:numel(sections)
    s = sections{i};
    if ~isfield(a,s) || ~isfield(b,s), continue; end
    d = max(d,struct_delta(a.(s),b.(s)));
end
end

function d = struct_delta(a,b)
d = 0;
f = intersect(fieldnames(a),fieldnames(b));
for i = 1:numel(f)
    va = a.(f{i}); vb = b.(f{i});
    if isstruct(va) && isstruct(vb)
        d = max(d,struct_delta(va,vb));
    elseif isnumeric(va) && isnumeric(vb) && isscalar(va) && isscalar(vb)
        d = max(d,abs(vb-va)/max(abs(va),1));
    end
end
end

function tuned = extract_tuned(p)
tuned = struct();
sections = radar_experiment_common('tuned_sections');
for i = 1:numel(sections)
    s = sections{i};
    if isfield(p,s), tuned.(s) = p.(s); end
end
end

% =========================================================================
% Reporting
% =========================================================================
function print_header(opts,nCache)
fprintf('\n============================================================\n');
fprintf(' CLOSED-LOOP PARAMETER LEARNING\n');
fprintf(' mode=%s | target Pd>=%.3f | budget<=%.3f false/frame\n', ...
    opts.mode,opts.target_pd,opts.target_false_per_frame);
fprintf(' cache=%d scenes x %d frames | %d iterations\n',nCache,opts.frames,opts.iterations);
fprintf('============================================================\n');
end

function print_iteration(it,total,m,delta,step,verdict)
fprintf('\n[%02d/%02d] Pd=%.4f  false/frame=%.4f  RMSE(R,V,A)=[%.2f %.2f %.2f]  step=%.3f  %s\n', ...
    it,total,m.pd,m.false_per_frame,m.range_rmse,m.velocity_rmse,m.angle_rmse,step,verdict);
fprintf('        misses  absent=%d preCFAR=%d CFAR=%d AMF=%d group=%d track=%d\n', ...
    m.miss_absent,m.miss_pre_cfar,m.miss_cfar,m.miss_amf,m.miss_group,m.miss_track);
fprintf('        ghosts  moving=%d stationary=%d TBD-dp=%d TBD-coh=%d (recovery=%d persistent=%d)\n', ...
    m.false_moving,m.false_stationary,m.false_tbd_dp,m.false_tbd_coherent, ...
    m.false_recovery,m.false_persistent);
fprintf('        cause   split=%d (one target reported twice)  spurious=%d (no truth nearby)  coasted=%d  maxDelta=%.4g\n', ...
    m.false_split,m.false_spurious,m.coasted_objects,delta);
end

function print_summary(m,opts)
fprintf('\n------------------------------------------------------------\n');
fprintf(' BEST OPERATING POINT\n');
fprintf('   detection probability   %.4f   (target %.3f)  %s\n',m.pd,opts.target_pd, ...
    tern(m.pd >= opts.target_pd,'MET','not met'));
fprintf('   false objects per frame %.4f   (budget %.3f)  %s\n',m.false_per_frame, ...
    opts.target_false_per_frame,tern(m.false_per_frame <= opts.target_false_per_frame,'MET','not met'));
fprintf('   RMSE  range %.3f m | velocity %.3f m/s | angle %.3f deg\n', ...
    m.range_rmse,m.velocity_rmse,m.angle_rmse);
fprintf('   dominant ghost source: %s\n',dominant_ghost(m));
fprintf('   dominant mechanism:    %s\n',dominant_mechanism(m));
fprintf('------------------------------------------------------------\n');
end

function s = dominant_mechanism(m)
%DOMINANT_MECHANISM  Splitting and invention need opposite corrections.
tot = m.false_split + m.false_spurious;
if tot == 0, s = 'none'; return; end
if m.false_split > m.false_spurious
    s = sprintf(['splitting (%d of %d): one target reported more than once. ' ...
        'Widen track.duplicate_* or tbd.object_fusion_*; raising detection ' ...
        'thresholds will not help.'],m.false_split,tot);
else
    s = sprintf(['spurious (%d of %d): reports with no truth nearby. ' ...
        'Tighten the stage named above.'],m.false_spurious,tot);
end
end

function s = dominant_ghost(m)
v = [m.false_moving m.false_stationary m.false_tbd_dp m.false_tbd_coherent];
n = {'moving grouping','stationary branch','dynamic-programming TBD','coherent TBD'};
if all(v == 0), s = 'none'; return; end
[~,i] = max(v);
s = sprintf('%s (%d of %d)',n{i},v(i),sum(v));
end

% =========================================================================
% Small helpers
% =========================================================================
function o = parse_opts(o,varargin)
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(o,key)
        error('run_feedback_parameter_learning:Args','Unknown option "%s".',key);
    end
    o.(key) = varargin{k+1};
end
end

function t = make_trace(it,m,params,step,verdict)
t = empty_trace();
t.iteration = it;
t.metrics = m;
t.params = params;
t.step_scale = step;
t.verdict = verdict;
t.pd = m.pd;
t.false_per_frame = m.false_per_frame;
end

function t = empty_trace()
t = struct('iteration',0,'pd',NaN,'false_per_frame',NaN,'step_scale',NaN, ...
    'verdict','','params',struct(),'metrics',struct());
end

function c = empty_cache()
c = struct('snr_db',NaN,'trial',0,'seed',0,'clean_frames',{{}}, ...
    'params',struct(),'truth',[]);
end

function write_learning_csv(trace,path)
if isempty(trace)
    fid = fopen(path,'w');
    if fid > 0, fprintf(fid,'no iterations recorded\n'); fclose(fid); end
    return;
end
rows = [];
for i = 1:numel(trace)
    m = trace(i).metrics;
    rows(end+1,:) = [trace(i).iteration, m.pd, m.false_per_frame, trace(i).step_scale, ...
        m.range_rmse, m.velocity_rmse, m.angle_rmse, ...
        m.miss_pre_cfar, m.miss_cfar, m.miss_amf, m.miss_group, m.miss_track, ...
        m.false_moving, m.false_stationary, m.false_tbd_dp, m.false_tbd_coherent];
end
T = array2table(rows,'VariableNames',{'iteration','pd','false_per_frame','step_scale', ...
    'range_rmse','velocity_rmse','angle_rmse','miss_pre_cfar','miss_cfar','miss_amf', ...
    'miss_group','miss_track','false_moving','false_stationary','false_tbd_dp','false_tbd_coherent'});
writetable(T,path);
end

function write_param_json(tuned,path)
try
    txt = jsonencode(tuned);
    fid = fopen(path,'w'); fwrite(fid,txt,'char'); fclose(fid);
catch
    fid = fopen(path,'w');
    fprintf(fid,'JSON export failed; inspect feedback_learning.mat.');
    fclose(fid);
end
end

function y = clip(x,a,b), y = min(max(x,a),b); end
function v = getf(s,f,d), v = d; if isstruct(s) && isfield(s,f), v = s.(f); end, end
function s = tern(c,a,b), if c, s = a; else, s = b; end, end

function m = empty_trial_metrics()
%EMPTY_TRIAL_METRICS  Field set of one scored trial, for typed accumulation.
m = struct('truth_count',0,'object_count',0,'matched',0,'pd',0, ...
    'false_objects',0,'false_per_frame',0,'missed',0, ...
    'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN,'frames',0, ...
    'miss_absent',0,'miss_pre_cfar',0,'miss_cfar',0,'miss_amf',0, ...
    'miss_group',0,'miss_track',0, ...
    'false_moving',0,'false_stationary',0,'false_tbd_dp',0, ...
    'false_tbd_coherent',0,'false_recovery',0,'false_persistent',0, ...
    'false_split',0,'false_spurious',0,'coasted_objects',0, ...
    'track_continuity',NaN);
end
