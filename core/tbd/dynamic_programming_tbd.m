function [paths,info] = dynamic_programming_tbd(frameEvidence,p)
%DYNAMIC_PROGRAMMING_TBD  Fixed-lag likelihood-ratio track-before-detect.
%
%   A target whose per-frame signal-to-noise ratio sits below the CFAR
%   threshold is invisible to any single-frame test, yet its energy is present
%   in every frame and lies on a physically admissible trajectory. This stage
%   recovers exactly that case by integrating evidence along trajectories
%   rather than thresholding cells.
%
%   CELL EVIDENCE
%   Let z = P / N be the normalised power of a candidate cell. Under H0 the
%   non-coherent sum of M receive channels is Gamma(M, 1/M) with unit mean.
%   Under a Swerling-I alternative of unknown strength gamma the same statistic
%   is Gamma(M, (1+gamma)/M). Maximising the likelihood ratio over gamma >= 0
%   gives the generalised log-likelihood ratio in closed form:
%
%       l(z) = M ( z - 1 - ln z )   for z > 1,   0 otherwise
%
%   The estimate gamma_hat = z - 1 is exactly the excess power, so l is zero
%   when the cell carries no more energy than the null predicts and grows
%   without a tunable knob when it does.
%
%   TRAJECTORY ACCUMULATION
%   Dynamic programming maximises the accumulated score
%
%       S(f,j) = max over predecessors [ S(f',i) + l(z_j) - C(i -> j) ]
%              + continuation bonus / gap
%
%   with the transition cost penalising departures from constant-velocity
%   motion in units of the configured process sigmas,
%
%       C = 0.5 (dr/sigma_r)^2 + 0.5 (dv/sigma_v)^2 + 0.25 (dtheta/sigma_th)^2
%
%   and dr measured against the predicted range r + v dt. Transitions that
%   exceed the kinematic limits are forbidden outright, so the search cannot
%   stitch unrelated cells into a trajectory. Missed frames are allowed up to
%   the configured gap, priced by a per-frame gap penalty plus the miss penalty
%   in decibels, which keeps a sparse path from outscoring a dense one.
%
%   PATH EXTRACTION
%   Paths are extracted greedily strongest first. After each acceptance the
%   neighbourhood of the winning path is suppressed so the next pass cannot
%   report the same target again. Paths that fail the length or support tests
%   are suppressed at half radius and the search continues, which prevents one
%   weak terminal region from consuming the extraction budget.
%
%   No truth and no target count enter this function.

if nargin < 2 || isempty(p)
    error('dynamic_programming_tbd:Input','Radar parameter struct is required.');
end
Nf = numel(frameEvidence);
paths = repmat(empty_path(),0,1);
info = struct('enabled',true,'frame_count',Nf,'candidate_counts',zeros(1,Nf), ...
    'total_candidates',0,'path_count',0,'accepted_path_count',0,'suppressed_candidate_count',0, ...
    'max_terminal_score',-Inf,'path_scores',zeros(1,0),'path_lengths',zeros(1,0), ...
    'path_support',zeros(1,0),'method','Gamma GLR dynamic-programming TBD');
if ~p.tbd.enabled || Nf == 0
info.enabled = false;
return;
end

M = max(1,round(get_default_field(p.cfar,'power_shape',p.n_rx)));
cand = cell(Nf,1);
for f = 1:Nf
    cand{f} = sanitize_candidates(frameEvidence{f},p,M);
    info.candidate_counts(f) = numel(cand{f});
end
info.total_candidates = sum(info.candidate_counts);
if info.total_candidates == 0, return;
end

maxPasses = max(1,round(get_default_field(p.tbd,'max_paths',32)));
for pass = 1:maxPasses
    [path,bestScore] = best_path(cand,p);
    info.max_terminal_score = max(info.max_terminal_score,bestScore);
    if isempty(path) || bestScore < p.tbd.path_min_score
break;
    end
    path = path_quality(path,cand,p,bestScore);
    if path.support_fraction < p.tbd.min_path_support_fraction || ...
       path.length < p.tbd.min_path_frames || ~path.fit_ok
        cand = suppress_path_neighborhood(cand,path,p,0.5);
info.suppressed_candidate_count = info.suppressed_candidate_count + path.length;
continue;
    end
    paths(end+1) = path;
    info.path_scores(end+1)  = path.score;
    info.path_lengths(end+1) = path.length;
    info.path_support(end+1) = path.support_fraction;
info.accepted_path_count = info.accepted_path_count + 1;
    cand = suppress_path_neighborhood(cand,path,p,1.0);
if info.accepted_path_count >= p.tbd.max_confirmed_paths, break;
end
end
info.path_count = numel(paths);
end

% =========================================================================
function c = sanitize_candidates(frameInput,p,M)
c = repmat(empty_candidate(),0,1);
if isempty(frameInput) || ~isstruct(frameInput), return; end
in = frameInput(:);
maxC = min(p.tbd.max_candidates_per_frame,numel(in));
birthFloorDb = get_default_field(p.tbd,'birth_min_cfar_db',-Inf);

for i = 1:numel(in)
    if numel(c) >= maxC, break; end
    if ~isfield(in(i),'range') || ~isfield(in(i),'velocity'), continue; end
    r = in(i).range; v = in(i).velocity;
    if ~isfinite(r) || ~isfinite(v), continue; end
    if r < 0 || r > max(p.R_max,1.05*max(p.range_axis)) || abs(v) > p.v_max + 1e-9, continue; end

    q = empty_candidate();
q.range = r;
q.velocity = v;
    q.r_bin = pick_bin(in(i),'r_bin',p.range_axis,r);
    q.d_bin = pick_bin(in(i),'d_bin',p.vel_axis,v);
    if isfield(in(i),'power') && isfinite(in(i).power), q.power = in(i).power; end
    if isfield(in(i),'noise_power') && isfinite(in(i).noise_power), q.noise_power = in(i).noise_power; end
    if q.noise_power <= 0 && isfield(in(i),'cfar_noise') && isfinite(in(i).cfar_noise)
        q.noise_power = in(i).cfar_noise;
    end
if q.noise_power <= 0, q.noise_power = 1;
end

    if isfield(in(i),'evidence_db') && isfinite(in(i).evidence_db)
        q.evidence_db = in(i).evidence_db;
    elseif isfield(in(i),'cfar_snr_db') && isfinite(in(i).cfar_snr_db)
        q.evidence_db = in(i).cfar_snr_db;
    elseif q.power > 0
        q.evidence_db = 10*log10(q.power/q.noise_power);
    else
q.evidence_db = 0;
    end
    if isfinite(birthFloorDb) && q.evidence_db < birthFloorDb, continue; end

    q.z = max(10^(q.evidence_db/10),realmin);
    q.llr = min(cell_glr_llr(q.z,M),get_default_field(p.tbd,'max_cell_llr',50.0));
    if isfield(in(i),'amf_db') && isfinite(in(i).amf_db), q.amf_db = in(i).amf_db; end
    if isfield(in(i),'angle_deg') && isfinite(in(i).angle_deg), q.angle_deg = in(i).angle_deg; end
    if isfield(in(i),'is_hard'), q.is_hard = logical(in(i).is_hard); end
    if isfield(in(i),'origin') && ~isempty(in(i).origin), q.origin = in(i).origin; end
    c(end+1) = q;
end
if numel(c) > 1
    [~,ord] = sort([c.llr],'descend');
    c = c(ord);
end
end

function b = pick_bin(s,field,axis,value)
if isfield(s,field) && ~isempty(s.(field)) && isfinite(s.(field))
    b = round(s.(field));
else
    [~,b] = min(abs(axis-value));
end
end

function [path,bestScore] = best_path(cand,p)
%BEST_PATH  Forward dynamic programming over the candidate lattice.
%
%   The recursion is unchanged: the best score reaching candidate j in frame f
%   is the best score reaching any admissible predecessor, plus this cell's
%   evidence, minus the transition cost. What changed is how it is evaluated.
%
%   Scoring every predecessor-successor pair one at a time costs
%   O(N_f * gap * K^2) scalar function calls, and with a few hundred weak
%   candidates per frame that is over a million calls per invocation. MATLAB
%   pays roughly thirty microseconds of call overhead on each, so the search
%   spent tens of seconds per frame on dispatch rather than on arithmetic.
%
%   The transition cost between two frames is a separable function of the
%   candidate state vectors, so the whole predecessor-by-successor cost matrix
%   is formed at once with array operations and reduced with a single MAX.
%   The result is identical; the dispatch is gone.

Nf = numel(cand);
path = repmat(empty_path(),0,1); bestScore = -Inf;
if Nf == 0, return; end

scores = cell(Nf,1); predF = cell(Nf,1); predI = cell(Nf,1);
endF = 0; endI = 0;
missPenalty = get_default_field(p.tbd,'miss_penalty_db',1.0);

% Flatten each frame's candidates into state vectors once.
R = cell(Nf,1); V = cell(Nf,1); A = cell(Nf,1); LLR = cell(Nf,1);
for f = 1:Nf
    c = cand{f};
    if isempty(c)
        R{f} = zeros(1,0); V{f} = zeros(1,0); A{f} = zeros(1,0); LLR{f} = zeros(1,0);
    else
        R{f} = [c.range]; V{f} = [c.velocity]; A{f} = [c.angle_deg]; LLR{f} = [c.llr];
    end
end

maxR = p.tbd.max_transition_range_m;
maxV = p.tbd.max_transition_velocity_mps;
sR0  = max(p.tbd.range_sigma_m,0.25);
sV0  = max(p.tbd.velocity_sigma_mps,0.10);
sA   = max(p.tbd.angle_sigma_deg,0.5);

for f = 1:Nf
    K = numel(R{f});
    scores{f} = -Inf(1,K); predF{f} = zeros(1,K); predI{f} = zeros(1,K);
    if K == 0, continue; end

    % Birth: a trajectory may start at this cell.
    scores{f} = p.tbd.birth_log_prior + LLR{f};
    if f == 1
        [s,j] = max(scores{f});
        if s > bestScore, bestScore = s; endF = f; endI = j; end
        continue;
    end

    best = -Inf(1,K); bf = zeros(1,K); bi = zeros(1,K);
    f0 = max(1,f-p.tbd.max_gap_frames-1);
    for pf = f0:f-1
        M = numel(R{pf});
        if M == 0, continue; end
        gap = f - pf;
        dt  = p.track.dt*gap;

        % Predicted range of every predecessor under constant velocity,
        % against the measured range of every successor: an M-by-K matrix.
        dr = R{f}(ones(M,1),:) - (R{pf}(:) + V{pf}(:)*dt);
        dv = V{f}(ones(M,1),:) - V{pf}(:);

        sR = sR0*sqrt(gap); sV = sV0*sqrt(gap);
        cost = 0.5*(dr/sR).^2 + 0.5*(dv/sV).^2;

        % Bearing agrees only where both endpoints have one.
        aP = A{pf}(:); aC = A{f}(:).';
        both = isfinite(aP) & isfinite(aC);
        if any(both(:))
            da = mod(aC(ones(M,1),:) - aP(:,ones(1,K)) + 180,360) - 180;
            cost(both) = cost(both) + 0.25*(da(both)/sA).^2;
        end

        % Kinematically impossible transitions are forbidden outright, so the
        % search cannot stitch unrelated cells into a trajectory.
        cost(abs(dr) > maxR*gap | abs(dv) > maxV*gap) = Inf;
        if gap > 1
            cost = cost + (p.tbd.gap_penalty + missPenalty)*(gap-1);
        end

        s = scores{pf}(:) + LLR{f}(ones(M,1),:) - cost + p.tbd.continuation_log_bonus/gap;
        s(~isfinite(cost)) = -Inf;
        [sBest,iBest] = max(s,[],1);
        take = sBest > best;
        best(take) = sBest(take);
        bf(take)   = pf;
        bi(take)   = iBest(take);
    end

    take = best > scores{f};
    scores{f}(take) = best(take);
    predF{f}(take)  = bf(take);
    predI{f}(take)  = bi(take);

    [s,j] = max(scores{f});
    if s > bestScore, bestScore = s; endF = f; endI = j; end
end

if ~isfinite(bestScore) || endF == 0, return; end

f = endF; i = endI; ff = zeros(1,Nf); ii = zeros(1,Nf); L = 0;
while f > 0 && i > 0
    L = L + 1; ff(L) = f; ii(L) = i;
    pf = predF{f}(i); pi = predI{f}(i);
if pf <= 0 || pi <= 0, break;
end
f = pf;
i = pi;
end
ff = ff(1:L); ii = ii(1:L);
[ff,ord] = sort(ff,'ascend'); ii = ii(ord);

path = empty_path();
path.frame_indices = ff;
path.candidate_indices = ii;
path.length = L;
path.score = bestScore;
end

function path = path_quality(path,cand,p,bestScore)
F = path.frame_indices; I = path.candidate_indices; L = numel(F);
path.score = bestScore;
path.length = L;
span = max(F) - min(F) + 1;
path.support_fraction = L/max(span,1);
path.window_coverage  = L/max(numel(cand),1);
R = zeros(1,L); V = zeros(1,L); A = nan(1,L); Z = zeros(1,L);
E = zeros(1,L); AMF = nan(1,L); hard = false(1,L);
for k = 1:L
    q = cand{F(k)}(I(k));
    R(k) = q.range; V(k) = q.velocity; A(k) = q.angle_deg;
    Z(k) = q.z; E(k) = q.llr; AMF(k) = q.amf_db; hard(k) = q.is_hard;
end
path.ranges = R;
path.velocities = V;
path.angles = A;
path.z = Z;
path.evidence = E;
path.amf_db = AMF;
path.hard_mask = hard;
path.mean_llr = mean(E,'omitnan'); path.sum_llr = sum(E,'omitnan');
path.mean_z = mean(Z,'omitnan');
path.angle_support = mean(isfinite(A));
path.angle_std = angle_spread(A);
path.hard_support = mean(hard);
path.mean_amf = mean(AMF(isfinite(AMF)),'omitnan');
path.final_range = R(end); path.final_velocity = V(end); path.final_angle = A(end);

% Kinematic consistency: fit a constant-velocity line to the range history and
% measure how far the accepted cells depart from it. A trajectory stitched from
% unrelated bright cells satisfies the per-step transition limits but does not
% lie on a single line, so the residual separates the two cases. The fitted
% slope is also compared against the reported Doppler, which must agree for a
% physical target.
tk = (0:L-1)*p.track.dt;
if L >= 3
    Amat = [ones(L,1) tk(:)];
    coef = Amat\R(:);
    resid = R(:) - Amat*coef;
    path.fit_residual_m = sqrt(mean(resid.^2));
    path.fit_velocity_mps = coef(2);
    path.fit_velocity_error_mps = abs(coef(2) - mean(V,'omitnan'));
else
path.fit_residual_m = 0;
    path.fit_velocity_mps = mean(V,'omitnan');
path.fit_velocity_error_mps = 0;
end
path.fit_ok = path.fit_residual_m <= get_default_field(p.tbd,'max_path_fit_residual_m',Inf) && ...
              path.fit_velocity_error_mps <= get_default_field(p.tbd,'max_path_fit_velocity_error_mps',Inf);

path.confirmation_score = path.score + p.tbd.path_mean_llr_bonus*path.mean_llr ...
+ p.tbd.path_angle_bonus*path.angle_support;
end

function out = suppress_path_neighborhood(cand,path,p,scale)
%SUPPRESS_PATH_NEIGHBORHOOD  Remove the accepted path's local support.
%   After a trajectory is taken, the cells that formed it are withdrawn so the
%   next extraction pass cannot report the same target again.
out = cand;
gR = scale*p.tbd.path_exclusion_range_m;
gV = scale*p.tbd.path_exclusion_velocity_mps;
for k = 1:path.length
    f = path.frame_indices(k);
    if f < 1 || f > numel(out) || isempty(out{f}), continue; end
    near = abs([out{f}.range] - path.ranges(k)) <= gR & ...
           abs([out{f}.velocity] - path.velocities(k)) <= gV;
    out{f} = out{f}(~near);
end
end

function ell = cell_glr_llr(z,M)
z = max(z,realmin);
if z <= 1, ell = 0; else, ell = M*(z - 1 - log(z)); end
end

function a = wrap_angle(a), a = mod(a+180,360)-180; end

function s = angle_spread(a)
a = a(isfinite(a));
if numel(a) < 2, s = 0; return; end
mu = atan2d(mean(sind(a)),mean(cosd(a)));
s = sqrt(mean(wrap_angle(a-mu).^2));
end

function q = empty_path()
q = struct('frame_indices',zeros(1,0),'candidate_indices',zeros(1,0),'length',0, ...
    'score',-Inf,'support_fraction',0,'ranges',zeros(1,0),'velocities',zeros(1,0), ...
    'angles',zeros(1,0),'z',zeros(1,0),'evidence',zeros(1,0),'amf_db',zeros(1,0), ...
    'hard_mask',false(1,0),'mean_llr',0,'sum_llr',0,'mean_z',0,'angle_support',0, ...
    'angle_std',NaN,'hard_support',0,'mean_amf',NaN,'final_range',NaN, ...
    'final_velocity',NaN,'final_angle',NaN,'confirmation_score',-Inf,'window_coverage',0, ...
    'fit_residual_m',0,'fit_velocity_mps',NaN,'fit_velocity_error_mps',0,'fit_ok',true);
end

function c = empty_candidate()
c = struct('range',0,'velocity',0,'r_bin',1,'d_bin',1,'power',0,'noise_power',1, ...
    'z',1,'llr',0,'evidence_db',0,'amf_db',NaN,'angle_deg',NaN,'is_hard',false,'origin','tbd');
end
