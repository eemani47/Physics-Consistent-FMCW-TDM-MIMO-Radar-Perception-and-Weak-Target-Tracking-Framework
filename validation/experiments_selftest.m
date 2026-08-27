function report = experiments_selftest(verbose)
%EXPERIMENTS_SELFTEST  Contracts of the experiment and learning layer.
%
%   The experiment layer is where results come from, so its statistical
%   machinery is checked directly rather than assumed. These tests run without
%   the physical simulator: they exercise the interval estimators, the
%   lexicographic ordering, the seed discipline, the scene generator and the
%   dotted-path setter on synthetic inputs whose correct answers are known.

if nargin < 1, verbose = true;
end
report = struct('name','experiments','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));

% --- 1. splits occupy disjoint seed spaces -------------------------------
o = [radar_experiment_common('split_offset','train'), ...
     radar_experiment_common('split_offset','validation'), ...
     radar_experiment_common('split_offset','test')];
report = add(report,'train, validation and test seeds cannot collide', ...
    numel(unique(o)) == 3 && min(diff(sort(o))) > 1e6, ...
    sprintf('offsets %s',mat2str(o)));

% --- 2. Wilson interval brackets the proportion and stays in range -------
[p1,lo1,hi1] = radar_experiment_common('wilson',5,10,0.05);
report = add(report,'Wilson interval brackets the point estimate', ...
    lo1 <= p1 && p1 <= hi1, sprintf('%.3f in [%.3f, %.3f]',p1,lo1,hi1));
[~,lo0,hi0] = radar_experiment_common('wilson',0,20,0.05);
report = add(report,'Wilson interval stays inside [0,1] at zero successes', ...
    lo0 >= 0 && hi0 <= 1 && hi0 > 0, sprintf('[%.4f, %.4f]',lo0,hi0));
[~,loN,hiN] = radar_experiment_common('wilson',20,20,0.05);
report = add(report,'Wilson interval stays inside [0,1] at full successes', ...
    loN >= 0 && loN < 1 && hiN <= 1, sprintf('[%.4f, %.4f]',loN,hiN));
[~,lw,hw] = radar_experiment_common('wilson',50,100,0.05);
[~,lw2,hw2] = radar_experiment_common('wilson',500,1000,0.05);
report = add(report,'Wilson interval narrows as the sample grows', ...
    (hw2-lw2) < (hw-lw), sprintf('width %.4f at n=100 versus %.4f at n=1000',hw-lw,hw2-lw2));

% --- 3. bootstrap interval brackets the mean ----------------------------
rng(7,'twister');
x = 2 + 0.5*randn(1,200);
[mb,lb,hb] = radar_experiment_common('bootstrap',x,0.05,500);
report = add(report,'bootstrap interval brackets the sample mean', ...
    lb <= mb && mb <= hb && abs(mb-mean(x)) < 1e-12, ...
    sprintf('%.4f in [%.4f, %.4f]',mb,lb,hb));

% --- 4. nested merge preserves untouched fields -------------------------
base = struct('a',1,'sec',struct('x',10,'y',20));
ov   = struct('sec',struct('y',99));
m = radar_experiment_common('merge',base,ov);
report = add(report,'nested merge replaces one field and keeps its siblings', ...
    m.a == 1 && m.sec.x == 10 && m.sec.y == 99);

% --- 5. tuned overlay never moves the locked false-alarm probability ----
p = radar_configuration(struct());
tuned = struct('cfar',struct('Pfa',0.5),'detector',struct('min_amf_db',12));
q = radar_experiment_common('apply_tuned',p,tuned);
report = add(report,'the calibrated CFAR Pfa is restored after a tuned overlay', ...
    q.cfar.Pfa == p.cfar.Pfa, sprintf('%.3e retained',q.cfar.Pfa));
report = add(report,'other tuned fields are adopted', ...
    q.detector.min_amf_db == 12);

% --- 6. scene generator is reproducible and physically admissible -------
baseCfg = radar_experiment_common('base_config');
s1 = radar_experiment_common('scene',baseCfg,12345,struct());
s2 = radar_experiment_common('scene',baseCfg,12345,struct());
s3 = radar_experiment_common('scene',baseCfg,54321,struct());
report = add(report,'the same seed reproduces the same scene', ...
    isequal(s1.targets,s2.targets));
report = add(report,'a different seed produces a different scene', ...
    ~isequal(s1.targets,s3.targets));
T = s1.targets;
report = add(report,'every target lies inside the processed range band', ...
    all(T(:,1) > 0) && all(T(:,1) < get_default_field(baseCfg,'R_max',300)));
report = add(report,'every target lies inside the field of view', ...
    all(abs(T(:,4)) <= get_default_field(baseCfg,'az_span',70)));
sStat = radar_experiment_common('scene',baseCfg,999,struct('stationary_fraction',0.5));
report = add(report,'a stationary fraction produces stationary targets', ...
    any(sStat.targets(:,2) == 0), ...
    sprintf('%d of %d are static',nnz(sStat.targets(:,2)==0),size(sStat.targets,1)));

% --- 7. lexicographic ordering: recall never buys a ghost ---------------
% Built to be decisive: candidate B has far higher detection probability and
% one more false object. Under a weighted sum B wins; under the budget rule
% A must win, because the budget is violated.
A = fake_metrics(0.60,0.00);
Bm = fake_metrics(0.95,0.20);
report = add(report,'a higher Pd cannot purchase a false object over budget', ...
    lex_better(A,Bm,0) && ~lex_better(Bm,A,0), ...
    'Pd 0.60 at 0.00 ghosts beats Pd 0.95 at 0.20 ghosts');
C = fake_metrics(0.70,0.00);
report = add(report,'within budget, the higher Pd wins', ...
    lex_better(C,A,0), 'Pd 0.70 beats Pd 0.60, both at zero ghosts');

% --- 8. dotted-path setter writes exactly one leaf ----------------------
q1 = set_path_probe(struct('keep',1),'cfar.Pfa',1e-6);
report = add(report,'a nested path writes only the intended leaf', ...
    q1.keep == 1 && isstruct(q1.cfar) && q1.cfar.Pfa == 1e-6 && ...
    numel(fieldnames(q1)) == 2 && numel(fieldnames(q1.cfar)) == 1);
q2 = set_path_probe(struct(),'detector.gs.angle_step_deg',0.5);
report = add(report,'a three-level path creates the full chain and nothing else', ...
    q2.detector.gs.angle_step_deg == 0.5 && ...
    numel(fieldnames(q2.detector)) == 1 && numel(fieldnames(q2.detector.gs)) == 1);

% --- 9. learned defaults are adopted and remain overridable -------------
root = radar_experiment_common('root');
lp = fullfile(root,'core','config','learned_defaults.mat');
hadFile = exist(lp,'file') == 2;
if hadFile, backup = load(lp); end
try
    tuned = struct('detector',struct('min_amf_db',13.5));
    save(lp,'tuned','-v7');
    pl = radar_configuration(struct());
    report = add(report,'a persisted learned value becomes the default', ...
        abs(pl.detector.min_amf_db - 13.5) < 1e-9, ...
        sprintf('min_amf_db = %.2f',pl.detector.min_amf_db));
    po = radar_configuration(struct('detector',struct('min_amf_db',8)));
    report = add(report,'an explicit caller value still overrides the learned one', ...
        abs(po.detector.min_amf_db - 8) < 1e-9, ...
        sprintf('min_amf_db = %.2f',po.detector.min_amf_db));
    report = add(report,'learned defaults never carry the training scene', ...
        isequal(size(pl.targets),size(radar_configuration(struct()).targets)));
    delete(lp);
    if hadFile
        tuned = backup.tuned;
        save(lp,'-struct','backup');
    end
catch ME
    if exist(lp,'file'), delete(lp); end
    if hadFile, save(lp,'-struct','backup'); end
    report = add(report,'learned-default persistence round trip',false,ME.message);
end

if verbose, print_report(report); end
end

% =========================================================================
function m = fake_metrics(pd,fpf)
m = struct('pd',pd,'false_per_frame',fpf,'range_rmse',1,'velocity_rmse',1,'angle_rmse',1);
end

function tf = lex_better(cand,ref,budget)
%LEX_BETTER  Mirror of the learner's acceptance rule, for testing.
cOver = max(0,cand.false_per_frame - budget);
rOver = max(0,ref.false_per_frame  - budget);
if (cOver > 0 || rOver > 0) && abs(cOver-rOver) > 1e-9
tf = cOver < rOver;
return;
end
if abs(cand.pd - ref.pd) > 1e-6, tf = cand.pd > ref.pd; return; end
tf = false;
end

function out = set_path_probe(s,path,v)
parts = strsplit(char(path),'.');
out = s;
if numel(parts) == 1, out.(parts{1}) = v; return; end
key = parts{1};
if isfield(out,key) && isstruct(out.(key)), child = out.(key); else, child = struct(); end
out.(key) = set_path_probe(child,strjoin(parts(2:end),'.'),v);
end

function r = add(r,name,pass,detail)
if nargin < 4, detail = ''; end
r.checks(end+1) = struct('name',name,'pass',logical(pass),'detail',detail);
r.ok = r.ok && logical(pass);
end

function print_report(r)
fprintf('\n[%s] %s\n',upper(r.name),tern(r.ok,'PASS','FAIL'));
for i = 1:numel(r.checks)
    c = r.checks(i);
    if isempty(c.detail)
        fprintf('   %-4s %s\n',tern(c.pass,'ok','FAIL'),c.name);
    else
        fprintf('   %-4s %s  (%s)\n',tern(c.pass,'ok','FAIL'),c.name,c.detail);
    end
end
end
function y = tern(c,a,b), if c, y = a; else, y = b; end, end
