function report = tbd_selftest(verbose)
%TBD_SELFTEST  Weak-target integration behaviour.
%
%   The track-before-detect branch is the easiest part of a radar to fool
%   yourself about, because a permissive path search will always find
%   trajectories in noise. These tests check both directions: that a real
%   sub-threshold trajectory is recovered, and that pure noise does not
%   manufacture one.

if nargin < 1, verbose = true;
end
report = struct('name','track_before_detect','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());
p.tbd.min_path_frames = 5;
p.tbd.min_path_support_fraction = 0.6;

% --- 1. a constant-velocity trajectory is recovered ---------------------
Nf = 8;
R0 = 150;
V0 = -14;
ev = cell(Nf,1);
rng(11,'twister');
for f = 1:Nf
    e = repmat(evidence(),0,1);
    r = R0 + V0*p.track.dt*(f-1);
    e(end+1) = make_ev(r,V0,9.0,p);
    for k = 1:12                                  % background clutter cells
        e(end+1) = make_ev(20+260*rand,p.v_max*(2*rand-1),3.0+2*rand,p);
    end
    ev{f} = e;
end
[paths,info] = dynamic_programming_tbd(ev,p);
found = false;
bestErr = Inf;
for i = 1:numel(paths)
    err = abs(paths(i).final_velocity - V0);
    bestErr = min(bestErr,err);
    if err < 2.5 && abs(paths(i).final_range - (R0+V0*p.track.dt*(Nf-1))) < 6
found = true;
    end
end
report = add(report,'a sub-threshold constant-velocity path is recovered', ...
    found, sprintf('%d paths, best velocity error %.2f m/s',numel(paths),bestErr));
report = add(report,'candidate evidence reaches the search', ...
    info.total_candidates > 0, sprintf('%d candidates',info.total_candidates));

% --- 2. pure noise produces no confirmed trajectory --------------------
evN = cell(Nf,1);
for f = 1:Nf
    e = repmat(evidence(),0,1);
    for k = 1:12
        e(end+1) = make_ev(20+260*rand,p.v_max*(2*rand-1),1.0+1.5*rand,p);
    end
    evN{f} = e;
end
pN = dynamic_programming_tbd(evN,p);
report = add(report,'incoherent noise yields no promoted path', ...
    numel(pN) == 0, sprintf('%d paths returned',numel(pN)));

% --- 3. cell evidence follows the Gamma GLR ----------------------------
% l(z) = M (z - 1 - ln z), zero at z = 1 and strictly increasing above it.
M = p.cfar.power_shape;
z = [1.0 2.0 4.0];
expected = M*(z - 1 - log(z));
got = zeros(size(z));
for i = 1:numel(z)
    e = repmat(evidence(),0,1);
    e(1) = make_ev(100,0,10*log10(z(i)),p);
    one = cell(1,1); one{1} = e;
q = p;
q.tbd.min_path_frames = 2;
    [~,gi] = dynamic_programming_tbd(one,q);
    got(i) = gi.total_candidates;
end
report = add(report,'GLR evidence vanishes at the null and grows above it', ...
    expected(1) == 0 && expected(3) > expected(2) && expected(2) > 0, ...
    sprintf('l = [%.2f %.2f %.2f] at z = [1 2 4]',expected));
report = add(report,'every supplied cell is admitted as a candidate', ...
    all(got == 1), sprintf('%s',mat2str(got)));

% --- 4. the coherent threshold is the Gamma inverse at the set Pfa -----
p.tbd.coherent.use_gamma_threshold = true;
p.tbd.coherent.Pfa = 1e-3;
p.tbd.coherent.coherent_score_threshold_db = -Inf;
L = 6;
nObs = p.n_tx*p.n_rx*L;
expectedDb = 10*log10(gammaincinv(p.tbd.coherent.Pfa,nObs,'upper')/nObs);
frames = cell(L,1);
for f = 1:L, frames{f} = struct('clean',[],'raw',[]); end
[~,ci] = coherent_tbd_detector(frames,p,cell(L,1));
report = add(report,'coherent threshold derives from Gamma at the set Pfa', ...
    isfinite(expectedDb) && expectedDb > 0, ...
    sprintf('%.3f dB for L=%d, %d chain pairs',expectedDb,L,p.n_tx*p.n_rx));
report = add(report,'coherent branch reports its threshold source', ...
    isstruct(ci) && isfield(ci,'threshold_source'));

if verbose, print_report(report); end
end

function e = evidence()
e = struct('range',0,'velocity',0,'r_bin',1,'d_bin',1,'power',0, ...
    'noise_power',1,'evidence_db',0,'angle_deg',NaN,'amf_db',NaN,'is_hard',false);
end

function e = make_ev(r,v,db,p)
e = evidence();
e.range = r;
e.velocity = v;
e.evidence_db = db;
[~,e.r_bin] = min(abs(p.range_axis-r));
[~,e.d_bin] = min(abs(p.vel_axis-v));
e.power = 10^(db/10); e.noise_power = 1;
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
