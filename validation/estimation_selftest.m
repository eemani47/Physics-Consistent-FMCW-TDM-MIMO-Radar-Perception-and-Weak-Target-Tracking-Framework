function report = estimation_selftest(verbose)
%ESTIMATION_SELFTEST  Angle estimation and TDM ambiguity resolution.
%
%   Constructs measurements whose answer is known analytically and checks that
%   the estimators recover it. The TDM tests are the important ones: they
%   confirm that transmit-slot Doppler phase is actually being removed, which
%   is the difference between a real TDM-MIMO model and a plain increase in
%   the antenna count.

if nargin < 1, verbose = true;
end
report = struct('name','estimation','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());

% --- 1. beamformer and MUSIC recover a known bearing --------------------
theta = 17.5;
L = p.n_virt;
a = exp(1j*2*pi*(p.virtual_x(:)/p.lambda)*sind(theta));
va = a*ones(1,32) + 0.02*(randn(L,32)+1j*randn(L,32));
[~,~,~,~,info] = music_aoa_estimator(va,p,Inf);
report = add(report,'MUSIC recovers a plane-wave bearing', ...
    abs(info.music_peak_angle_deg-theta) < 1.5, ...
    sprintf('%.2f deg estimated against %.2f deg true',info.music_peak_angle_deg,theta));
report = add(report,'conventional beamformer agrees with MUSIC', ...
    abs(info.beamformer_peak_angle_deg-theta) < 4.0, ...
    sprintf('%.2f deg',info.beamformer_peak_angle_deg));
report = add(report,'model order is at least one for a single source', ...
    info.n_src >= 1, sprintf('MDL selected %d',info.n_src));
report = add(report,'Capon spectrum is populated and finite', ...
    ~isempty(info.capon_spectrum_db) && all(isfinite(info.capon_spectrum_db)));

% --- 2. two sources are resolved beyond the Rayleigh limit --------------
th1 = -8;
th2 = 8;
a1 = exp(1j*2*pi*(p.virtual_x(:)/p.lambda)*sind(th1));
a2 = exp(1j*2*pi*(p.virtual_x(:)/p.lambda)*sind(th2));
s1 = (randn(1,64)+1j*randn(1,64))/sqrt(2);
s2 = (randn(1,64)+1j*randn(1,64))/sqrt(2);
va2 = a1*s1 + a2*s2 + 0.05*(randn(L,64)+1j*randn(L,64));
[~,~,~,~,i2] = music_aoa_estimator(va2,p,Inf);
peaks = i2.music_peak_angles_deg;
got1 = any(abs(peaks-th1) < 4); got2 = any(abs(peaks-th2) < 4);
report = add(report,'MUSIC separates two closely spaced sources', ...
    got1 && got2, sprintf('peaks at %s deg',mat2str(round(peaks,1))));

% --- 3. virtual aperture removes the TDM Doppler phase ------------------
% Build a cube whose only content is a target at a known range and velocity,
% then confirm the de-rotated aperture points at the right bearing while the
% un-de-rotated one does not.
q = compact(p);
tgt = [80, 12, 12, -22];
q.noise.enabled = false;
q.clutter.enabled = false;
q.interference.enabled = false;
cube = simulate_mimo_rx(q,tgt,Inf);
rb = nearest(q.range_axis,tgt(1));
vaGood = tdm_virtual_aperture(cube,q,rb,tgt(2),false);
vaBad  = tdm_virtual_aperture(cube,q,rb,0,false);
[~,~,~,~,gInfo] = music_aoa_estimator(vaGood,q,tgt(1));
[~,~,~,~,bInfo] = music_aoa_estimator(vaBad ,q,tgt(1));
errGood = abs(gInfo.beamformer_peak_angle_deg-tgt(4));
errBad  = abs(bInfo.beamformer_peak_angle_deg-tgt(4));
report = add(report,'correct velocity hypothesis yields the true bearing', ...
    errGood < 5, sprintf('%.2f deg error',errGood));
report = add(report,'omitting Doppler de-rotation biases the bearing', ...
    errBad > errGood, sprintf('%.2f deg error without de-rotation',errBad));

% --- 4. alias spacing is the physical one -------------------------------
det = struct('range',tgt(1),'velocity',tgt(2),'r_bin',rb,'d_bin',1,'angle_deg',NaN);
[~,tinfo] = tdm_velocity_resolver(det,cube,q);
report = add(report,'alias spacing equals lambda/(2 n_tx T_chirp)', ...
    abs(tinfo.alias_spacing_mps - q.lambda/(2*q.n_tx*q.Tchirp)) < 1e-9, ...
    sprintf('%.3f m/s',tinfo.alias_spacing_mps));
report = add(report,'configured alias span reaches the resolver', ...
    tinfo.alias_span == q.detector.tdm_alias_span, ...
    sprintf('span %d',tinfo.alias_span));

% --- 5. the resolver keeps an unambiguous velocity unchanged ------------
slow = [60, 3, 12, 10];
cubeSlow = simulate_mimo_rx(q,slow,Inf);
rbS = nearest(q.range_axis,slow(1));
dS = struct('range',slow(1),'velocity',slow(2),'r_bin',rbS,'d_bin',1,'angle_deg',NaN);
[outS,~] = tdm_velocity_resolver(dS,cubeSlow,q);
report = add(report,'an unambiguous velocity survives resolution', ...
    abs(outS.velocity-slow(2)) < q.tdm_alias_spacing_mps/2, ...
    sprintf('%.2f m/s reported for %.2f m/s truth',outS.velocity,slow(2)));

if verbose, print_report(report); end
end

function q = compact(p)
q = p;
q.Nr = 512;
q.Nd = 32;
q.f_range_axis = (0:q.Nr-1)*(q.fs_ADC/q.Nr);
q.range_axis = q.c*q.f_range_axis/(2*q.slope);
q.vel_axis = linspace(-q.v_max,q.v_max,q.Nd);
q.valid_range_mask = q.range_axis <= q.R_max;
q.theta_axis = linspace(-q.az_span,q.az_span,256);
q.est.music_grid = 512;
end

function k = nearest(x,v), [~,k] = min(abs(x-v)); end

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
