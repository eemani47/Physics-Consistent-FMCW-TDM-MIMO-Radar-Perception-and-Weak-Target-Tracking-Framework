function report = evaluation_selftest(verbose)
%EVALUATION_SELFTEST  Assignment optimality and stage attribution.
%
%   The scorer decides what every reported number means, so it is tested
%   against ground truth of its own: small assignment problems whose optimum
%   can be enumerated exhaustively, and hand-built pipeline states whose
%   correct stage attribution is known by construction.

if nargin < 1, verbose = true;
end
report = struct('name','evaluation','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());

% --- 1. cardinality-first optimality against exhaustive search ----------
rng(2024,'twister');
mismatch = 0;
trials = 60;
for t = 1:trials
    nT = randi(4); nO = randi(4);
    truth = [10+rand(nT,1)*200, (rand(nT,1)-0.5)*40, 10*ones(nT,1), (rand(nT,1)-0.5)*60];
    obj = repmat(struct('range',0,'velocity',0,'angle_deg',0),1,nO);
    for j = 1:nO
        src = randi(max(nT,1));
        if rand < 0.55 && nT > 0
            obj(j).range = truth(src,1)+randn*1.2;
            obj(j).velocity = truth(src,2)+randn*0.5;
            obj(j).angle_deg = truth(src,4)+randn*2;
        else
            obj(j).range = 10+rand*200;
            obj(j).velocity = (rand-0.5)*40;
            obj(j).angle_deg = (rand-0.5)*60;
        end
    end
    E = radar_object_evaluation(obj,truth,{},p);
    ref = brute_force(obj,truth,p);
    if E.matched ~= ref.count || abs(E.objective-ref.cost) > 1e-6
mismatch = mismatch + 1;
    end
end
report = add(report,'assignment matches exhaustive search on random scenes', ...
    mismatch == 0, sprintf('%d of %d trials disagreed',mismatch,trials));

% --- 2. cardinality dominates cost --------------------------------------
% Two truths and two objects, where the greedy nearest pairing would match one
% truth twice and leave the other unmatched.
truth = [100 0 10 0; 103 0 10 0];
obj = struct('range',{101,102},'velocity',{0,0},'angle_deg',{0,0});
E = radar_object_evaluation(obj,truth,{},p);
report = add(report,'both truths are matched when a valid pairing exists', ...
    E.matched == 2, sprintf('matched %d of 2',E.matched));

% --- 3. the gate is respected -------------------------------------------
far = struct('range',300,'velocity',0,'angle_deg',0);
E2 = radar_object_evaluation(far,[100 0 10 0],{},p);
report = add(report,'an object outside the gate is a false object', ...
    E2.matched == 0 && E2.false_objects == 1);

% --- 4. RMSE is computed over matched pairs only ------------------------
truth3 = [100 5 10 0];
obj3 = struct('range',101,'velocity',5.5,'angle_deg',2);
E3 = radar_object_evaluation(obj3,truth3,{},p);
E3default = radar_object_evaluation(obj3,truth3);
report = add(report,'two-argument evaluation API uses the canonical default gates', ...
    E3default.matched == 1 && E3default.false_objects == 0 && E3default.missed == 0);
report = add(report,'range RMSE reflects the matched pair', ...
    abs(E3.range_rmse-1) < 1e-9, sprintf('%.4f m',E3.range_rmse));
report = add(report,'angle RMSE reflects the matched pair', ...
    abs(E3.angle_rmse-2) < 1e-9, sprintf('%.4f deg',E3.angle_rmse));

% --- 5. legacy field aliases are present --------------------------------
report = add(report,'legacy result field names are still published', ...
    all(isfield(E3,{'truth_count','radar_object_count','matched_count', ...
                    'missed_count','false_object_count','object_pd'})));

% --- 6. stage attribution reports the deepest stage reached -------------
tr = [100 5 10 0];
mk = @(r,v,a) struct('range',r,'velocity',v,'angle_deg',a);
stage = cell(1,3);
for f = 1:3
    stage{f} = struct('rd_power_moving',[],'rd_power_reference',[], ...
        'cfar_moving',mk(100,5,0),'amf_moving',repmat(mk(0,0,0),0,1), ...
        'stationary_hits',repmat(mk(0,0,0),0,1),'amf_stationary',repmat(mk(0,0,0),0,1), ...
        'groups',repmat(mk(0,0,0),0,1));
end
obs = truth_observability(tr,stage,{},repmat(mk(0,0,0),0,1),p);
report = add(report,'a target seen only at CFAR is attributed to CFAR', ...
    strcmp(obs.per_target(1).deepest_stage,'CFAR'), ...
    sprintf('reported %s',obs.per_target(1).deepest_stage));

for f = 1:3
    stage{f}.amf_moving = mk(100,5,0);
    stage{f}.groups = mk(100,5,0);
end
obs2 = truth_observability(tr,stage,{},mk(100,5,0),p);
report = add(report,'a reported target is attributed to OBJECT', ...
    strcmp(obs2.per_target(1).deepest_stage,'OBJECT'), ...
    sprintf('reported %s',obs2.per_target(1).deepest_stage));

% --- 7. false-object origin attribution ---------------------------------
ghost = mk(200,-9,15);
for f = 1:3
    stage{f}.cfar_moving = [mk(100,5,0) ghost];
    stage{f}.amf_moving = mk(100,5,0);
    stage{f}.groups = mk(100,5,0);
end
obs3 = truth_observability(tr,stage,{},[mk(100,5,0) ghost],p);
fo = obs3.false_objects;
report = add(report,'a ghost first visible at CFAR is attributed there', ...
    numel(fo) == 1 && strcmp(fo(1).origin_stage,'moving_cfar'), ...
    sprintf('%d false objects, origin %s',numel(fo), ...
    ternary(isempty(fo),'none',fo(1).origin_stage)));

if verbose, print_report(report); end
end

function ref = brute_force(obj,truth,p)
%BRUTE_FORCE  Exhaustive cardinality-first optimum for small problems.
nT = size(truth,1); nO = numel(obj);
gR = p.eval.match_range_m;
gV = p.eval.match_velocity_mps;
gA = p.eval.match_angle_deg;
C = inf(nT,nO);
for i = 1:nT
    for j = 1:nO
        dR = abs(truth(i,1)-obj(j).range);
        dV = abs(truth(i,2)-obj(j).velocity);
        dA = abs(mod(truth(i,4)-obj(j).angle_deg+180,360)-180);
        if dR <= gR && dV <= gV && dA <= gA
            C(i,j) = (dR/gR)^2 + (dV/gV)^2 + (dA/gA)^2;
        end
    end
end
ref = struct('count',0,'cost',0);
best = [-1 Inf];
for r = 0:min(nT,nO)
    rowsets = nchoosek(1:nT,r);
    if r == 0, rowsets = zeros(1,0); end
    for a = 1:max(size(rowsets,1),1)
        rows = rowsets(min(a,size(rowsets,1)),:);
        cols = perms_of(nO,r);
        for b = 1:size(cols,1)
            cc = cols(b,:);
            if any(~isfinite(diag_pick(C,rows,cc))), continue; end
            cost = sum(diag_pick(C,rows,cc));
            if r > best(1) || (r == best(1) && cost < best(2)-1e-12)
                best = [r cost];
            end
        end
    end
end
ref.count = max(best(1),0);
ref.cost = ternary(isfinite(best(2)),best(2),0);
if ref.count == 0, ref.cost = 0;
end
end

function v = diag_pick(C,rows,cols)
v = zeros(1,numel(rows));
for k = 1:numel(rows), v(k) = C(rows(k),cols(k)); end
end

function P = perms_of(n,r)
if r == 0, P = zeros(1,0); return; end
base = nchoosek(1:n,r);
P = [];
for i = 1:size(base,1)
    P = [P; perms(base(i,:))];
end
end

function r = add(r,name,pass,detail)
if nargin < 4, detail = ''; end
r.checks(end+1) = struct('name',name,'pass',logical(pass),'detail',detail);
r.ok = r.ok && logical(pass);
end
function print_report(r)
fprintf('\n[%s] %s\n',upper(r.name),ternary(r.ok,'PASS','FAIL'));
for i = 1:numel(r.checks)
    c = r.checks(i);
    if isempty(c.detail), fprintf('   %-4s %s\n',ternary(c.pass,'ok','FAIL'),c.name);
    else, fprintf('   %-4s %s  (%s)\n',ternary(c.pass,'ok','FAIL'),c.name,c.detail); end
end
end
function y = ternary(c,a,b), if c, y = a; else, y = b; end, end
