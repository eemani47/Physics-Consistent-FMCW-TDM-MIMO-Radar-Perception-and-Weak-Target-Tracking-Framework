function varargout = radar_experiment_common(action,varargin)
%RADAR_EXPERIMENT_COMMON  Shared services for every experiment entry point.
%
%   The Monte Carlo engine, the benchmark suites, the parameter studies and
%   the feedback learner all need the same handful of operations: locate the
%   project, load the baseline configuration, draw a reproducible random
%   scene, merge nested overrides, score a completed run, and compute
%   confidence intervals. Implementing those once removes the possibility of
%   two experiments disagreeing about what a "trial" or a "false object" is.
%
%   Actions
%     'root'                       project root directory
%     'base_config'                stored GUI baseline configuration
%     'merge',base,override        recursive nested-struct merge
%     'scene',base,seed,opts       reproducible randomised scene
%     'split_offset',name          disjoint seed offset for train/val/test
%     'score',objects,truth,info,p scored result plus stage attribution
%     'wilson',k,n,alpha           Wilson score interval for a proportion
%     'bootstrap',x,alpha,B        percentile bootstrap interval for a mean
%     'tuned_sections'             configuration sections the learner may tune
%     'apply_tuned',base,tuned     overlay tuned sections onto a parameter set

switch lower(char(action))
    case 'root'
        varargout{1} = fileparts(fileparts(mfilename('fullpath')));
    case 'base_config'
        varargout{1} = load_base_config();
    case 'merge'
        varargout{1} = merge_struct(varargin{1},varargin{2});
    case 'scene'
        varargout{1} = randomize_scene(varargin{:});
    case 'split_offset'
        varargout{1} = split_offset(varargin{1});
    case 'score'
        [E,M] = score_run(varargin{:});
        varargout{1} = E;
        if nargout > 1, varargout{2} = M; end
    case 'wilson'
        [p,lo,hi] = wilson(varargin{:});
        varargout{1} = p;
        if nargout > 1, varargout{2} = lo; end
        if nargout > 2, varargout{3} = hi; end
    case 'bootstrap'
        [m,lo,hi] = bootstrap_mean(varargin{:});
        varargout{1} = m;
        if nargout > 1, varargout{2} = lo; end
        if nargout > 2, varargout{3} = hi; end
    case 'tuned_sections'
        varargout{1} = {'cfar','detector','track','group','tbd','paper'};
    case 'apply_tuned'
        varargout{1} = apply_tuned(varargin{1},varargin{2});
    otherwise
        error('radar_experiment_common:Action','Unknown action "%s".',char(action));
end
end

% =========================================================================
function base = load_base_config()
%LOAD_BASE_CONFIG  The stored interface baseline, or an empty struct.
base = struct();
f = fullfile(fileparts(fileparts(mfilename('fullpath'))),'gui','gui_config.mat');
if exist(f,'file')
    G = load(f,'config');
    if isfield(G,'config') && isstruct(G.config), base = to_double(G.config); end
end
end

function s = to_double(s)
%TO_DOUBLE  Normalize numeric configuration fields before experiment arithmetic.
if ~isstruct(s), return; end
f = fieldnames(s);
for i = 1:numel(f)
    v = s.(f{i});
    if isstruct(v)
        s.(f{i}) = to_double(v);
    elseif isnumeric(v) && ~isa(v,'double')
        s.(f{i}) = double(v);
    end
end
end

function out = merge_struct(base,ov)
%MERGE_STRUCT  Recursive override merge; nested sections merge field by field.
out = base;
if isempty(ov) || ~isstruct(ov), return; end
f = fieldnames(ov);
for k = 1:numel(f)
    key = f{k}; val = ov.(key);
    if isstruct(val) && isfield(out,key) && isstruct(out.(key))
        out.(key) = merge_struct(out.(key),val);
    else
        out.(key) = val;
    end
end
end

function off = split_offset(name)
%SPLIT_OFFSET  Disjoint seed spaces so splits can never share a scene.
switch lower(char(name))
    case 'train',      off = 0;
    case 'validation', off = 100000000;
    case 'test',       off = 200000000;
    case 'dev',        off = 300000000;
    otherwise, error('radar_experiment_common:Split','Unknown split "%s".',char(name));
end
end

function cfg = randomize_scene(base,seed,opts)
%RANDOMIZE_SCENE  Reproducible random scene under a controlled seed.
%
%   Nuisance variables are sampled from physically meaningful ranges rather
%   than arbitrary ones: target count, range within the processed band,
%   velocity inside the unambiguous interval unless aliasing is explicitly
%   permitted, radar cross-section over the span an automotive sensor sees,
%   and azimuth inside the field of view. Targets are separated so that two
%   of them never occupy the same resolution cell, which would make the truth
%   table itself ambiguous and corrupt the score.
if nargin < 3, opts = struct(); end
rng(seed,'twister');
cfg = base;

Rmax   = getf(cfg,'R_max',300);
vmax   = getf(cfg,'v_max',60);
azSpan = min(getf(cfg,'az_span',70),85);
fc     = getf(cfg,'fc',77e9);
c0     = 299792458;
lambda = c0/fc;
Tchirp = lambda/(4*max(vmax,eps));
nTx    = max(1,round(getf(cfg,'n_tx',2)));
vAmb   = lambda/(4*nTx*Tchirp);

nMin = round(getf(opts,'min_targets',4));
nMax = round(getf(opts,'max_targets',10));
nT   = randi([max(1,nMin) max(nMin,nMax)]);

allowAlias = logical(getf(opts,'allow_alias_velocity',false));
if allowAlias, vLimit = 0.85*vmax;
else, vLimit = 0.90*vAmb;
end
statFrac = min(max(getf(opts,'stationary_fraction',0),0),0.5);

Rlo = max(15,0.05*Rmax); Rhi = 0.90*Rmax;
R = zeros(nT,1); A = zeros(nT,1);
minRangeSep = max(2.0,getf(opts,'min_range_separation_m',2.0));
minAngleSep = max(3.0,getf(opts,'min_angle_separation_deg',3.0));
for i = 1:nT
    for guard = 1:200
        R(i) = Rlo + (Rhi-Rlo)*rand;
        A(i) = -0.9*azSpan + 1.8*azSpan*rand;
if i == 1, break;
end
        tooClose = abs(R(i)-R(1:i-1)) < minRangeSep & abs(A(i)-A(1:i-1)) < minAngleSep;
        if ~any(tooClose), break; end
    end
end
V = (2*rand(nT,1)-1)*vLimit;
nStat = round(statFrac*nT);
if nStat > 0, V(randperm(nT,nStat)) = 0; end
RCS = -10 + 30*rand(nT,1);
cfg.targets = [R V RCS A];

if logical(getf(opts,'randomize_clutter',true)) && isfield(cfg,'clutter') && isstruct(cfg.clutter)
    cfg.clutter.power_fraction_of_weakest = 10^(-3 + 2.5*rand);
cfg.clutter.slow_variation_std        = 0.02 + 0.08*rand;
cfg.clutter.range_decay_power         = 0.5 + 1.5*rand;
end
if logical(getf(opts,'randomize_interference',true)) && isfield(cfg,'interference') && isstruct(cfg.interference)
    cfg.interference.amplitude_vs_weakest = 10^(-1.0 + 2.0*rand);
cfg.interference.angle_deg            = -0.8*azSpan + 1.6*azSpan*rand;
    cfg.interference.bandwidth_Hz         = (0.2 + 0.8*rand)*4e6;
end
if logical(getf(opts,'randomize_sensor',true))
    cfg.NF_dB = min(max(getf(cfg,'NF_dB',10) + 1.5*randn,4),15);
end
end

function [E,M] = score_run(objects,truth,info,p)
%SCORE_RUN  One scored trial: metrics plus stage attribution.
%
%   Returns the evaluation record E and a flat metric struct M whose fields
%   are what every experiment aggregates. Miss attribution uses the exact
%   assignment to decide whether a target was missed, and the observability
%   trace to decide where it was lost — the two answer different questions
%   and both are needed.
[E,obs] = radar_object_evaluation(objects,truth,info.stage_data,p,info.frame_groups);

nFrames = max(numel(get_default_field(info,'frame_detections',{})),1);
M = empty_metrics();
M.truth_count     = E.truth;
M.object_count    = E.objects;
M.matched         = E.matched;
M.pd              = E.pd;
M.false_objects   = E.false_objects;
M.false_per_frame = E.false_objects/nFrames;
M.missed          = E.missed;
M.range_rmse      = E.range_rmse;
M.velocity_rmse   = E.velocity_rmse;
M.angle_rmse      = E.angle_rmse;
M.frames          = nFrames;

per = get_default_field(obs,'per_target',struct([]));
for i = 1:numel(per)
    id = get_default_field(per(i),'id',i);
    matched = id >= 1 && id <= numel(E.assignment) && E.assignment(id) > 0;
if matched, continue;
end
    switch upper(char(get_default_field(per(i),'deepest_stage','ABSENT')))
        case 'ABSENT',   M.miss_absent   = M.miss_absent + 1;
        case 'PRE_CFAR', M.miss_pre_cfar = M.miss_pre_cfar + 1;
        case 'CFAR',     M.miss_cfar     = M.miss_cfar + 1;
        case 'AMF',      M.miss_amf      = M.miss_amf + 1;
        case 'GROUP',    M.miss_group    = M.miss_group + 1;
otherwise,       M.miss_track    = M.miss_track + 1;
    end
end

fo = get_default_field(obs,'false_objects',struct([]));
for i = 1:numel(fo)
    src = lower(char(get_default_field(fo(i),'source','')));
    if contains(src,'tbd:dp'),            M.false_tbd_dp = M.false_tbd_dp + 1;
    elseif contains(src,'tbd:coherent'),  M.false_tbd_coherent = M.false_tbd_coherent + 1;
    elseif contains(src,'stationary'),    M.false_stationary = M.false_stationary + 1;
else,                                 M.false_moving = M.false_moving + 1;
    end
    if contains(src,'recovery'),   M.false_recovery = M.false_recovery + 1; end
    if contains(src,'persistent'), M.false_persistent = M.false_persistent + 1; end
    if strcmp(get_default_field(fo(i),'mechanism','spurious'),'split')
        M.false_split = M.false_split + 1;
    else
        M.false_spurious = M.false_spurious + 1;
    end
end
M.coasted_objects = get_default_field(info,'coasted_object_count',0);

M.track_continuity = track_continuity(info,truth);
end

function c = track_continuity(info,truth)
%TRACK_CONTINUITY  Mean per-target hit fraction over targets that were acquired.
c = NaN;
G = get_default_field(info,'frame_groups',{});
if isempty(G) || ~iscell(G), return; end
T = truth;
if ~isnumeric(T)
    if isempty(T), return; end
    % The simulator publishes initial range as range0. Reading range alone
    % returned NaN for every target and emptied the whole column.
    T = [arrayfun(@(t) get_default_field(t,'range', ...
                        get_default_field(t,'range0',NaN)),T(:)), ...
         arrayfun(@(t) get_default_field(t,'velocity',NaN),T(:)), ...
         zeros(numel(T),1), ...
         arrayfun(@(t) get_default_field(t,'angle_deg',NaN),T(:))];
    T = T(all(isfinite(T(:,[1 2])),2),:);
end
if isempty(T) || size(T,2) < 4, return; end
nF = numel(G); nT = size(T,1); hits = zeros(1,nT);
for ti = 1:nT
    for f = 1:nF
        g = G{f};
        if isempty(g), continue; end
        near = abs([g.range]-T(ti,1)) <= 4 & abs([g.velocity]-T(ti,2)) <= 2;
        if any(near), hits(ti) = hits(ti) + 1; end
    end
end
frac = hits/max(nF,1);
if any(frac > 0), c = mean(frac(frac > 0)); end
end

function [p,lo,hi] = wilson(k,n,alpha)
%WILSON  Score interval for a binomial proportion.
%   Stays inside [0,1] and remains meaningful when k is 0 or n, which is
%   where the normal approximation fails and where development runs live.
if nargin < 3 || isempty(alpha), alpha = 0.05; end
if n <= 0, p = NaN;
lo = NaN;
hi = NaN;
return;
end
z = norm_quantile(1-alpha/2);
p = k/n;
d = 1 + z^2/n;
c = p + z^2/(2*n);
s = z*sqrt(max(p*(1-p)/n + z^2/(4*n^2),0));
lo = max(0,(c-s)/d);
hi = min(1,(c+s)/d);
end

function [m,lo,hi] = bootstrap_mean(x,alpha,B)
%BOOTSTRAP_MEAN  Percentile bootstrap interval, no distributional assumption.
if nargin < 2 || isempty(alpha), alpha = 0.05; end
if nargin < 3 || isempty(B), B = 2000; end
x = x(isfinite(x));
if isempty(x), m = NaN; lo = NaN; hi = NaN; return; end
m = mean(x);
if numel(x) < 2, lo = m; hi = m; return; end
n = numel(x);
bs = zeros(1,B);
for b = 1:B
    bs(b) = mean(x(randi(n,1,n)));
end
bs = sort(bs);
lo = bs(max(1,floor(alpha/2*B)));
hi = bs(min(B,ceil((1-alpha/2)*B)));
end

function z = norm_quantile(q)
%NORM_QUANTILE  Standard normal inverse CDF by bisection on erf.
lo = -10;
hi = 10;
for i = 1:200
    mid = 0.5*(lo+hi);
    if 0.5*(1+erf(mid/sqrt(2))) < q, lo = mid; else, hi = mid; end
end
z = 0.5*(lo+hi);
end

function p = apply_tuned(base,tuned)
%APPLY_TUNED  Overlay tuned sections, preserving the locked statistical contract.
p = base;
sections = radar_experiment_common('tuned_sections');
for i = 1:numel(sections)
    s = sections{i};
    if isfield(tuned,s) && isstruct(tuned.(s))
        p.(s) = merge_struct(get_default_field(p,s,struct()),tuned.(s));
    end
end
% The calibrated false-alarm probability is a statistical guarantee, not a
% free parameter. It is restored after every overlay.
p.cfar.Pfa = base.cfar.Pfa;
end

function m = empty_metrics()
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

function v = getf(s,f,d)
v = d;
if isstruct(s) && isscalar(s) && isfield(s,f) && ~isempty(s.(f)), v = s.(f); end
end
