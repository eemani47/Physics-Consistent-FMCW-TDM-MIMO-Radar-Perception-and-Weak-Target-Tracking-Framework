function R = validate_cfar_pfa(varargin)
%VALIDATE_CFAR_PFA  Empirical false-alarm study of the calibrated detector.
%
%   Three tests are reported, each answering a different question.
%
%   1. ANALYTICAL NULL. Cells drawn directly from the law the calibration
%      assumes: Gamma(M, 1/M) for the cell under test and its references. This
%      isolates the numerical solution of
%
%          P_fa(alpha) = Integral Q(alpha x; M, 1/M) f_ref(x) dx
%
%      from every other stage. A failure here means the multiplier is wrong.
%
%   2. END-TO-END NULL. Complex white noise pushed through the real range and
%      Doppler transforms and then through CFAR. This answers what the
%      windowed, correlated front end actually delivers, which is not
%      identical to the idealised null: the analysis window correlates
%      neighbouring bins, so the effective number of independent reference
%      cells is smaller than the nominal count.
%
%   3. SPLIT-FIELD NULL. The cell under test taken from the coherent
%      moving-target map while the reference comes from the unsubtracted map,
%      which is the configuration the pipeline actually runs. This is the
%      regression test for normalisation drift between the two fields: if they
%      ever diverge in scale, the measured rate here departs from the target
%      by orders of magnitude even though tests 1 and 2 still pass.
%
%   Intervals are exact Clopper-Pearson bounds on the binomial proportion,
%   which is the right choice when the expected number of false alarms is
%   small. Note that resolving a rate of 1e-5 requires on the order of 1e7
%   independent cells; the default sample budget is far below that, so the
%   reported interval will be correspondingly wide. Raise 'null_cells' for a
%   real study.

root = fileparts(mfilename('fullpath'));
project_root = fileparts(root);
addpath(genpath(project_root));

opts = struct('trials',60,'seed',20260822,'alpha',0.05, ...
    'null_cells',200000,'e2e_frames',40,'config',struct(), ...
    'output_dir',fullfile(pwd,'docs','results'),'save',false, ...
    'pfa',1e-3,'modes',{{'ca','os','go','so','adaptive'}});
opts = parse_opts(opts,varargin{:});

base = opts.config;
guiCfg = fullfile(project_root,'gui','gui_config.mat');
if isempty(fieldnames(base)) && exist(guiCfg,'file')
    G = load(guiCfg,'config');
    if isfield(G,'config'), base = G.config; end
end
p = radar_configuration(base);
p.cfar.Pfa = opts.pfa;
M = p.cfar.power_shape;

rng(opts.seed,'twister');
modes = opts.modes;
rows = repmat(mode_row(),0,1);

for mi = 1:numel(modes)
    mode = modes{mi};
    q = small_map_config(p,mode);

    % --- 1. analytical null ---------------------------------------------
    [n1,k1] = run_null(q,@() gamma_map(q.Nr,q.Nd,M),opts.null_cells);

    % --- 2. end-to-end null ---------------------------------------------
    [n2,k2] = run_null(q,@() e2e_map(q,p),opts.null_cells);

    % --- 3. split-field null --------------------------------------------
    [n3,k3] = run_split_null(q,p,opts.null_cells);

    r = mode_row();
r.mode = mode;
r.pfa_target = q.cfar.Pfa;
    [r.pfa_empirical,r.pfa_ci_low,r.pfa_ci_high] = cp_interval(k1,n1,opts.alpha);
r.null_cut_count = n1;
r.null_false_count = k1;
    [r.e2e_pfa,r.e2e_ci_low,r.e2e_ci_high] = cp_interval(k2,n2,opts.alpha);
r.e2e_cut_count = n2;
r.e2e_false_count = k2;
    [r.split_pfa,r.split_ci_low,r.split_ci_high] = cp_interval(k3,n3,opts.alpha);
r.split_cut_count = n3;
r.split_false_count = k3;
    rows(end+1) = r;
end

R = struct();
R.modes = rows;
R.pfa_target = p.cfar.Pfa;
R.power_shape = M;
R.settings = opts;
R.note = ['Resolving a rate of 1e-5 needs on the order of 1e7 independent ' ...
          'cells; widen null_cells for a publication-grade estimate.'];

print_table(R);

if opts.save
    outdir = fullfile(opts.output_dir,'cfar_pfa');
    if ~exist(outdir,'dir'), mkdir(outdir); end
    writetable(struct2table(rows),fullfile(outdir,'cfar_pfa.csv'));
    save(fullfile(outdir,'cfar_pfa.mat'),'R');
end
end

% =========================================================================
function [n,k] = run_null(q,mapFcn,budget)
n = 0;
k = 0;
while n < budget
    P = mapFcn();
    [~,thr] = adaptive_cfar_2d(P,q);
valid = thr > 0;
    n = n + nnz(valid);
    k = k + nnz(P(valid) > thr(valid));
end
end

function [n,k] = run_split_null(q,p,budget)
%RUN_SPLIT_NULL  CUT and reference drawn from two independent noise maps.
%   Both maps carry the same absolute normalisation, so the achieved rate must
%   still match the target. A scale offset between them shows up here.
n = 0;
k = 0;
while n < budget
    A = gamma_map(q.Nr,q.Nd,p.cfar.power_shape);
    B = gamma_map(q.Nr,q.Nd,p.cfar.power_shape);
    [~,thr] = adaptive_cfar_2d(A,q,B);
valid = thr > 0;
    n = n + nnz(valid);
    k = k + nnz(A(valid) > thr(valid));
end
end

function P = gamma_map(nr,nd,M)
%GAMMA_MAP  Gamma(M,1/M) samples as a sum of M exponentials. No toolbox use.
P = zeros(nr,nd);
for i = 1:max(1,round(M))
    P = P - log(max(rand(nr,nd),realmin));
end
P = P/max(1,round(M));
end

function P = e2e_map(q,p)
%E2E_MAP  White complex noise through the real range and Doppler transforms.
x = (randn(p.Nr,p.Nd)+1j*randn(p.Nr,p.Nd))/sqrt(2);
[~,~,RD] = range_doppler_processor(x,p,struct('keystone',false,'clutter_method','off'));
valid = find(p.valid_range_mask);
r0 = valid(1)+2; r1 = min(r0+q.Nr-1,valid(end)-2);
if r1-r0+1 < q.Nr, r0 = valid(1)+2; r1 = r0+q.Nr-1; end
d0 = max(1,floor(p.Nd/2)-floor(q.Nd/2)); d1 = d0+q.Nd-1;
P = RD(r0:r1,d0:min(d1,p.Nd));
if size(P,2) < q.Nd, P = [P repmat(P(:,end),1,q.Nd-size(P,2))]; end
end

function q = small_map_config(p,mode)
%SMALL_MAP_CONFIG  A map just large enough to host the CFAR window.
q = p;
q.cfar.mode = mode;
H = q.cfar.Tr+q.cfar.Gr;
W = q.cfar.Td+q.cfar.Gd;
q.Nr = 2*H+40;
q.Nd = 2*W+40;
q.range_axis = linspace(1,q.R_max,q.Nr);
q.vel_axis = linspace(-q.v_max,q.v_max,q.Nd);
q.valid_range_mask = true(1,q.Nr);
end

function [phat,lo,hi] = cp_interval(k,n,alpha)
%CP_INTERVAL  Exact Clopper-Pearson bounds via the Beta quantile.
if n <= 0, phat = NaN;
lo = NaN;
hi = NaN;
return;
end
phat = k/n;
if k == 0, lo = 0; else, lo = beta_quantile(alpha/2,k,n-k+1); end
if k == n, hi = 1; else, hi = beta_quantile(1-alpha/2,k+1,n-k); end
end

function x = beta_quantile(pr,a,b)
lo = 0;
hi = 1;
for i = 1:200
    mid = 0.5*(lo+hi);
    if betainc(mid,a,b) < pr, lo = mid; else, hi = mid; end
end
x = 0.5*(lo+hi);
end

function print_table(R)
fprintf('\n================= CFAR FALSE-ALARM STUDY =================\n');
fprintf('target Pfa %.2e   Gamma shape M = %d\n\n',R.pfa_target,R.power_shape);
fprintf('%-10s %-28s %-28s %-28s\n','mode','analytical null','end-to-end null','split-field null');
for i = 1:numel(R.modes)
    m = R.modes(i);
    fprintf('%-10s %-28s %-28s %-28s\n',m.mode, ...
        fmt(m.pfa_empirical,m.pfa_ci_low,m.pfa_ci_high), ...
        fmt(m.e2e_pfa,m.e2e_ci_low,m.e2e_ci_high), ...
        fmt(m.split_pfa,m.split_ci_low,m.split_ci_high));
end
fprintf('\n%s\n',R.note);
fprintf('==========================================================\n\n');
end

function s = fmt(p,lo,hi)
s = sprintf('%.2e [%.1e,%.1e]',p,lo,hi);
end

function r = mode_row()
r = struct('mode','','pfa_target',NaN, ...
    'pfa_empirical',NaN,'pfa_ci_low',NaN,'pfa_ci_high',NaN, ...
    'null_cut_count',0,'null_false_count',0, ...
    'e2e_pfa',NaN,'e2e_ci_low',NaN,'e2e_ci_high',NaN, ...
    'e2e_cut_count',0,'e2e_false_count',0, ...
    'split_pfa',NaN,'split_ci_low',NaN,'split_ci_high',NaN, ...
    'split_cut_count',0,'split_false_count',0);
end

function o = parse_opts(o,varargin)
for i = 1:2:numel(varargin)
    key = lower(char(varargin{i}));
    if isfield(o,key), o.(key) = varargin{i+1}; end
end
end
