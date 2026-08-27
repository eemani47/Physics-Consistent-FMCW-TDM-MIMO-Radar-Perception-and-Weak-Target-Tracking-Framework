function report = radar_integrity_selftest(verbose)
%RADAR_INTEGRITY_SELFTEST  Physical and numerical invariants of the core chain.
%
%   These are behavioural tests. Each one computes a quantity from the live
%   implementation and compares it against a value derived independently from
%   radar theory, so a regression is caught by the numbers rather than by a
%   string appearing somewhere in a source file.

if nargin < 1, verbose = true;
end
report = struct('name','radar_integrity','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());

% --- 1. coupled waveform identities --------------------------------------
report = add(report,'bandwidth follows range resolution', ...
    rel_err(p.B, p.c/(2*p.R_res)) < 1e-12);
report = add(report,'chirp duration follows velocity design point', ...
    rel_err(p.Tchirp, p.lambda/(4*p.v_max)) < 1e-12);
report = add(report,'chirp slope equals B/Tchirp', ...
    rel_err(p.slope, p.B/p.Tchirp) < 1e-12);
report = add(report,'TDM ambiguity narrows by exactly n_tx', ...
    rel_err(p.tdm_unambiguous_velocity, p.unambiguous_velocity_nominal/p.n_tx) < 1e-12);
report = add(report,'ADC rate clears the real-ADC Nyquist requirement', ...
    p.fs_ADC > 2*p.f_beat_max);

% --- 2. FMCW delay maps to the predicted beat frequency ------------------
% A point scatterer at range R must appear at f_b = 2 S R / c.
Rtest = 120;
tau = 2*Rtest/p.c;
t = (0:p.Nr-1).'/p.fs_ADC;
sIF = exp(1j*2*pi*p.fc*tau).*exp(1j*pi*p.slope*(2*t*tau - tau^2));
S = abs(fft(sIF,p.Nr)).^2;
[~,pk] = max(S(1:floor(p.Nr/2)));
fbMeasured = (pk-1)*p.fs_ADC/p.Nr;
fbPredicted = 2*p.slope*Rtest/p.c;
report = add(report,'beat frequency matches 2 S R / c to within one bin', ...
    abs(fbMeasured-fbPredicted) <= 1.5*p.fs_ADC/p.Nr, ...
    sprintf('measured %.4f MHz, predicted %.4f MHz',fbMeasured/1e6,fbPredicted/1e6));
rangeFromBin = p.c*fbMeasured/(2*p.slope);
report = add(report,'range axis inverts the beat-frequency relation', ...
    abs(rangeFromBin-Rtest) <= 2*p.range_resolution_actual, ...
    sprintf('recovered %.3f m for a %.1f m scatterer',rangeFromBin,Rtest));

% --- 3. thermal noise obeys kTBF -----------------------------------------
expected = p.k_B*p.temp*p.noise_bandwidth*10^(p.NF_dB/10);
report = add(report,'per-chain noise power equals k T B F', ...
    rel_err(p.rx_noise_power_W(1),expected) < 1e-12, ...
    sprintf('%.4e W',p.rx_noise_power_W(1)));

% --- 4. band-limited noise injects exactly the declared power ------------
q = p; q.Nr = 512; q.Nd = 8; q.targets = p.targets(1,:);
q.f_range_axis = (0:q.Nr-1)*(q.fs_ADC/q.Nr);
q.range_axis = q.c*q.f_range_axis/(2*q.slope);
q.valid_range_mask = true(1,q.Nr);
q.vel_axis = p.vel_axis(1:min(q.Nd,numel(p.vel_axis)));
q.fd_axis = q.vel_axis;
q.noise.mode = 'fixed_awgn'; q.noise.fixed_power_W = 1e-9;
q.noise.enabled = true;
q.clutter.enabled = false;
q.interference.enabled = false;
q.P_tx_W = 0;                       % noise only
[cube,~,meta] = simulate_mimo_rx(q,q.targets,Inf);
measured = mean(abs(cube(:)).^2);
report = add(report,'band-limited noise carries the declared total power', ...
    rel_err(measured,1e-9) < 0.15, ...
    sprintf('measured %.4e W against %.4e W declared',measured,1e-9));
report = add(report,'noise passband is narrower than the sampled band', ...
    meta.noise_passband_bins < q.Nr);

% --- 5. shared FFT normalisation is unitary in the noise sense -----------
[w,g] = rd_window(4096,'hann');
x = (randn(4096,1)+1j*randn(4096,1))/sqrt(2);       % unit variance
X = fft(x.*w)/(sqrt(4096)*g);
report = add(report,'windowed unitary FFT preserves noise variance', ...
    abs(mean(abs(X).^2)-1) < 0.10, ...
    sprintf('bin variance %.4f against 1.0 expected',mean(abs(X).^2)));

% --- 6. the separator and the RD front end share one power scale --------
% This is the regression test for the defect in which the moving-target map
% and the CFAR reference map differed by tens of decibels.
r = p;
r.Nr = 512;
r.Nd = 16;
r.n_rx = 2;
r.n_tx = 1;
r.n_virt = 2;
r.rx_x = (0:1)*r.d_rx; r.tx_x = 0; r.virtual_x = r.rx_x;
r.f_range_axis = (0:r.Nr-1)*(r.fs_ADC/r.Nr);
r.range_axis = r.c*r.f_range_axis/(2*r.slope);
r.vel_axis = linspace(-r.v_max,r.v_max,r.Nd);
r.valid_range_mask = true(1,r.Nr);
r.theta_axis = linspace(-r.az_span,r.az_span,32);
r.rx_noise_power_W = repmat(p.noise_power_W,1,2);
noiseCube = (randn(r.Nr,r.Nd,r.n_rx)+1j*randn(r.Nr,r.Nd,r.n_rx))/sqrt(2);
sep = moving_stationary_separator(noiseCube,r);
Pref_sep = sep.raw_rd_power;
Pref_rd = zeros(r.Nr,r.Nd);
for rx = 1:r.n_rx
    [~,~,Prd] = range_doppler_processor(noiseCube(:,:,rx),r,struct('keystone',false,'clutter_method','off'));
Pref_rd = Pref_rd + Prd;
end
ratioDb = 10*log10(median(Pref_sep(Pref_sep>0))/median(Pref_rd(Pref_rd>0)));
report = add(report,'separator and RD front end agree on absolute power', ...
    abs(ratioDb) < 1.0, sprintf('median offset %.3f dB',ratioDb));

% --- 7. keystone leaves a stationary tone untouched ----------------------
Rk = fft(repmat(exp(1j*2*pi*0.13*(0:r.Nr-1).'),1,r.Nd),r.Nr,1);
Rks = keystone_motion_compensation(Rk,r);
err = norm(Rks(:)-Rk(:))/max(norm(Rk(:)),eps);
report = add(report,'keystone is near-identity for zero Doppler', ...
    err < 0.05, sprintf('relative change %.4f',err));

if verbose, print_report(report); end
end

function r = add(r,name,pass,detail)
if nargin < 4, detail = ''; end
r.checks(end+1) = struct('name',name,'pass',logical(pass),'detail',detail);
r.ok = r.ok && logical(pass);
end

function e = rel_err(a,b), e = abs(a-b)/max(abs(b),eps); end

function print_report(r)
fprintf('\n[%s] %s\n',upper(r.name),ternary(r.ok,'PASS','FAIL'));
for i = 1:numel(r.checks)
    c = r.checks(i);
    if isempty(c.detail)
        fprintf('   %-4s %s\n',ternary(c.pass,'ok','FAIL'),c.name);
    else
        fprintf('   %-4s %s  (%s)\n',ternary(c.pass,'ok','FAIL'),c.name,c.detail);
    end
end
end

function y = ternary(c,a,b), if c, y = a; else, y = b; end, end
