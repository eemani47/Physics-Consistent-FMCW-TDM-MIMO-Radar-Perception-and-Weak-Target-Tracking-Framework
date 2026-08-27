function [E,obs] = radar_object_evaluation(objects,truth,stageData,p,frameGroups)
%RADAR_OBJECT_EVALUATION  Offline scoring of radar objects against truth.
%
%   Scoring a multi-object radar is an assignment problem, not a nearest-
%   neighbour lookup. Greedy matching produces different answers depending on
%   the order objects happen to arrive, and it systematically over-reports
%   misses when two targets are close together. This module therefore solves
%   the assignment exactly.
%
%   Objects and truth are paired under a physical admissibility gate in range,
%   radial velocity and bearing. Within the gate the normalised cost is
%
%       C(i,j) = ( dR / gR )^2 + ( dV / gV )^2 + ( dTheta / gA )^2
%
%   and the objective is lexicographic: maximise the number of matched truth
%   objects first, and only among assignments of equal cardinality minimise
%   total normalised error. That ordering matters, because a solver that
%   minimises cost alone will happily leave a target unmatched to reduce the
%   error of the pairs it did make, which is precisely the wrong trade for a
%   detection metric.
%
%   The exact solution is a depth-first branch and bound over a bitmask of
%   used objects, with memoisation on (truth index, mask) for MATLAB releases
%   without MatchPairs.  When MATCHPAIRS is available it is used directly.
%   There is no artificial cap on the number of radar objects that may be
%   scored; every returned object participates in the assignment.
%
%   Reported quantities: detection probability, false-object and missed-object
%   counts, range, velocity and angle RMSE over matched pairs, the assignment
%   itself, and the achieved objective. Stage attribution for both misses and
%   false objects is delegated to TRUTH_OBSERVABILITY.

if nargin < 3, stageData = {}; end
if nargin < 4 || isempty(p)
    % The two-argument API is intentionally supported by live GUI, plotting,
    % Monte-Carlo, and analysis callers.  Supply only the evaluation section
    % here rather than constructing a complete radar configuration for every
    % frame; callers that own a full configuration still pass it explicitly.
    p = struct('eval',struct( ...
        'match_range_m',4.0, ...
        'match_velocity_mps',2.0, ...
        'match_angle_deg',8.0));
end
if nargin < 5, frameGroups = {}; end

truthN = normalize_truth(truth);
objN   = normalize_objects(objects);
nT = numel(truthN); nO = numel(objN);

gate = get_default_field(p,'eval',struct());
gR = get_default_field(gate,'match_range_m',4.0);
gV = get_default_field(gate,'match_velocity_mps',2.0);
gA = get_default_field(gate,'match_angle_deg',8.0);
E = struct('truth',nT,'objects',nO,'matched',0,'missed',nT,'false_objects',nO, ...
    'pd',0,'range_rmse',NaN,'velocity_rmse',NaN,'angle_rmse',NaN, ...
    'assignment',zeros(1,nT),'objective',0,'method','', ...
    'match_gate',struct('range_m',gR,'velocity_mps',gV,'angle_deg',gA), ...
    'errors',struct('range',zeros(1,0),'velocity',zeros(1,0),'angle',zeros(1,0)));

if nT == 0 || nO == 0
    E.method = 'trivial';
if nT == 0, E.pd = NaN;
end
    E = add_legacy_aliases(E);
    if nargout > 1
        obs = truth_observability(truthN,stageData,frameGroups,objN,p);
    end
return;
end

% ---- admissibility gate and normalised cost ------------------------------
C = inf(nT,nO);
for i = 1:nT
    for j = 1:nO
        dR = abs(truthN(i).range - objN(j).range);
        dV = abs(truthN(i).velocity - objN(j).velocity);
        dA = abs(wrap_angle(truthN(i).angle_deg - objN(j).angle_deg));
if dR > gR || dV > gV || dA > gA, continue;
end
        C(i,j) = (dR/gR)^2 + (dV/gV)^2 + (dA/gA)^2;
    end
end

if exist('matchpairs','file') == 2
unmatchedPenalty = 1e6;
    [pairs,~,~] = matchpairs(C,unmatchedPenalty);
    assign = zeros(1,nT);
    if ~isempty(pairs)
        for k = 1:size(pairs,1)
            i = pairs(k,1); j = pairs(k,2);
            if isfinite(C(i,j))
                assign(i) = j;
            end
        end
    end
    E.method = 'MATLAB matchpairs cardinality-first assignment';
else
    assign = exact_cardinality_first(C);
    E.method = 'exact cardinality-first assignment (shortest augmenting path)';
end

E.assignment = assign;
matched = assign > 0;
E.matched = nnz(matched);
E.missed = nT - E.matched;
E.false_objects = nO - E.matched;
E.pd = E.matched/max(nT,1);

dr = zeros(1,E.matched); dv = dr; da = dr; k = 0;
obj = 0;
for i = 1:nT
    if assign(i) == 0, continue; end
k = k + 1;
    j = assign(i);
    dr(k) = truthN(i).range - objN(j).range;
    dv(k) = truthN(i).velocity - objN(j).velocity;
    da(k) = wrap_angle(truthN(i).angle_deg - objN(j).angle_deg);
    obj = obj + C(i,j);
end
if k > 0
    E.range_rmse    = sqrt(mean(dr.^2));
    E.velocity_rmse = sqrt(mean(dv.^2));
    E.angle_rmse    = sqrt(mean(da.^2));
    E.errors = struct('range',dr,'velocity',dv,'angle',da);
end
E.objective = obj;

E = add_legacy_aliases(E);

if nargout > 1
    obs = truth_observability(truthN,stageData,frameGroups,objN,p);
E.truth_observability = obs;
E.false_object_observability = obs.false_objects;
end
end

function E = add_legacy_aliases(E)
%ADD_LEGACY_ALIASES  Names carried for existing consumers of this struct.
E.truth_count        = E.truth;
E.radar_object_count = E.objects;
E.matched_count      = E.matched;
E.missed_count       = E.missed;
E.false_object_count = E.false_objects;
E.object_pd          = 100*E.pd;
end

% =========================================================================
function assign = exact_cardinality_first(C)
%EXACT_CARDINALITY_FIRST  Lexicographic assignment via a shortest-path solver.
%
%   Cardinality-first optimisation is expressed as a single linear assignment
%   problem. Every admissible pair is given the cost
%
%       C'(i,j) = C(i,j) - B ,     B = (n+1) ( max C + 1 )
%
%   and every inadmissible pair, together with the padding rows and columns
%   that represent "left unmatched", is given zero cost. Because B exceeds the
%   largest total error any assignment can accumulate, adding one more matched
%   pair always improves the objective more than any redistribution of error
%   among the pairs already matched. The minimum-cost assignment is therefore
%   the one with the most matches, and among those the one with the least
%   normalised error - exactly the lexicographic objective, obtained in
%   polynomial time rather than by exponential search.

[nT,nO] = size(C);
n = max(nT,nO);
finiteC = C(isfinite(C));
if isempty(finiteC)
    assign = zeros(1,nT); return;
end
B = (n+1)*(max(finiteC)+1);

M = zeros(n,n);
for i = 1:nT
    for j = 1:nO
        if isfinite(C(i,j)), M(i,j) = C(i,j) - B; end
    end
end

col4row = hungarian(M);
assign = zeros(1,nT);
for i = 1:nT
    j = col4row(i);
    if j >= 1 && j <= nO && isfinite(C(i,j))
        assign(i) = j;
    end
end
end

function col4row = hungarian(M)
%HUNGARIAN  Minimum-cost assignment by successive shortest augmenting paths.
%   Standard O(n^3) dual-variable formulation. u and v are the row and column
%   potentials; each iteration grows a shortest-path tree in the reduced cost
%   until an unassigned column is reached, then flips the alternating path.
n = size(M,1);
u = zeros(1,n+1); v = zeros(1,n+1);
p = zeros(1,n+1); way = zeros(1,n+1);
for i = 1:n
    p(1) = i;
j0 = 1;
    minv = inf(1,n+1);
    used = false(1,n+1);
    while true
        used(j0) = true;
        i0 = p(j0); delta = Inf; j1 = 1;
        for j = 2:n+1
            if used(j), continue; end
            cur = M(i0,j-1) - u(i0) - v(j);
            if cur < minv(j), minv(j) = cur; way(j) = j0; end
            if minv(j) < delta, delta = minv(j); j1 = j; end
        end
        for j = 1:n+1
            if used(j)
                u(p(j)) = u(p(j)) + delta;
                v(j) = v(j) - delta;
            else
                minv(j) = minv(j) - delta;
            end
        end
j0 = j1;
        if p(j0) == 0, break; end
    end
    while j0 ~= 1
        j1 = way(j0);
        p(j0) = p(j1);
j0 = j1;
    end
end
col4row = zeros(1,n);
for j = 2:n+1
    if p(j) >= 1 && p(j) <= n
        col4row(p(j)) = j-1;
    end
end
end


function t = normalize_truth(truth)
t = repmat(struct('range',0,'velocity',0,'angle_deg',0),0,1);
if isempty(truth), return; end
if isnumeric(truth)
    for i = 1:size(truth,1)
        t(end+1) = struct('range',truth(i,1),'velocity',truth(i,2), ...
            'angle_deg',truth(i,min(4,size(truth,2))));
    end
return;
end
for i = 1:numel(truth)
    t(end+1) = struct( ...
        'range',get_default_field(truth(i),'range',get_default_field(truth(i),'range0',NaN)), ...
        'velocity',get_default_field(truth(i),'velocity',NaN), ...
        'angle_deg',get_default_field(truth(i),'angle_deg',NaN));
end
end

function o = normalize_objects(objects)
o = repmat(struct('range',0,'velocity',0,'angle_deg',0,'source','','score_db',-Inf),0,1);
for i = 1:numel(objects)
    o(end+1) = struct('range',get_default_field(objects(i),'range',NaN), ...
        'velocity',get_default_field(objects(i),'velocity',NaN), ...
        'angle_deg',get_default_field(objects(i),'angle_deg',NaN), ...
        'source',get_default_field(objects(i),'source',''), ...
        'score_db',get_default_field(objects(i),'score_db',-Inf));
end
end

function a = wrap_angle(a), a = mod(a+180,360)-180; end
