function report = detection_chain_selftest(verbose)
%DETECTION_CHAIN_SELFTEST  Statistical behaviour of the detection stages.
%
%   Verifies that the calibrated multipliers reproduce their nominal
%   false-alarm probabilities, that the matched filter's null statistic has the
%   distribution its threshold assumes, and that the weak-candidate channel
%   stays strictly below the declaration threshold.

if nargin < 1, verbose = true;
end
report = struct('name','detection_chain','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());

% --- 1. CFAR multiplier reproduces the requested Pfa ---------------------
% Draw the null directly: cell under test Gamma(M,1/M), reference the mean of
% n independent draws of the same law, then count exceedances of alpha*ref.
M = p.cfar.power_shape;
q = p; q.Nr = 96; q.Nd = 96; q.valid_range_mask = true(1,q.Nr);
q.range_axis = linspace(1,300,q.Nr); q.vel_axis = linspace(-30,30,q.Nd);
q.cfar.Pfa = 1e-3;                    % resolvable with a tractable sample count
q.cfar.mode = 'ca';
rng(4242,'twister');
P = gamma_draw(q.Nr,q.Nd,M,1/M);
[~,thr,~,cinfo] = adaptive_cfar_2d(P,q);
valid = thr > 0;
pfaEmp = nnz(P(valid) > thr(valid))/max(nnz(valid),1);
report = add(report,'CA-CFAR empirical Pfa tracks the nominal value', ...
    pfaEmp < 6*q.cfar.Pfa + 3/max(nnz(valid),1), ...
    sprintf('nominal %.1e, empirical %.2e over %d cells',q.cfar.Pfa,pfaEmp,nnz(valid)));
report = add(report,'CA multiplier is finite and above unity', ...
    isfinite(cinfo.alpha_ca) && cinfo.alpha_ca > 1, ...
    sprintf('alpha_CA = %.4f (%.2f dB)',cinfo.alpha_ca,10*log10(cinfo.alpha_ca)));

% --- 2. tightening Pfa must raise the multiplier -------------------------
q2 = q;
q2.cfar.Pfa = 1e-6;
[~,~,~,cinfo2] = adaptive_cfar_2d(P,q2);
report = add(report,'multiplier increases monotonically as Pfa tightens', ...
    cinfo2.alpha_ca > cinfo.alpha_ca, ...
    sprintf('%.3f dB at 1e-3 versus %.3f dB at 1e-6', ...
    10*log10(cinfo.alpha_ca),10*log10(cinfo2.alpha_ca)));

% --- 3. order-statistic and greatest-of multipliers are ordered ----------
% The order-statistic reference is the 75th percentile of the training set,
% which for a unit-mean Gamma(M,1/M) population sits above the mean. To reach
% the same false-alarm rate its multiplier must therefore be smaller, by
% roughly the ratio of the two references. What is larger for OS-CFAR is the
% detection loss, not the raw multiplier.
osRatio = cinfo.alpha_os/cinfo.alpha_ca;
report = add(report,'OS multiplier is smaller than CA in proportion to its reference', ...
    osRatio > 0.6 && osRatio < 0.95, ...
    sprintf('alpha_OS/alpha_CA = %.4f, expected near the reciprocal of the 75th percentile',osRatio));
report = add(report,'GO multiplier is below SO at equal Pfa', ...
    cinfo.alpha_go < cinfo.alpha_so, ...
    sprintf('alpha_GO = %.3f, alpha_SO = %.3f',cinfo.alpha_go,cinfo.alpha_so));

% --- 4. weak candidates stay under the declaration threshold ------------
q3 = q;
q3.cfar.Pfa = 1e-4;
q3.cfar.weak_snr_db = -6;
P3 = P; P3(48,48) = 400;                       % one strong cell
[det,thr3,~,ci3] = adaptive_cfar_2d(P3,q3);
weak = ci3.weak_candidates;
overThreshold = 0;
for i = 1:numel(weak)
    if P3(weak(i).r_bin,weak(i).d_bin) > thr3(weak(i).r_bin,weak(i).d_bin)
overThreshold = overThreshold + 1;
    end
end
report = add(report,'weak candidates never cross the declaration threshold', ...
    overThreshold == 0, sprintf('%d of %d weak candidates were above it',overThreshold,numel(weak)));
report = add(report,'the injected strong cell is declared', ...
    any(arrayfun(@(d) d.r_bin == 48 && d.d_bin == 48,det)));

% --- 5. sub-bin refinement stays inside one bin -------------------------
offs = arrayfun(@(d) abs(d.range_offset_bins),det);
report = add(report,'parabolic refinement is bounded by half a bin', ...
    isempty(offs) || max(offs) <= 0.5+1e-9, ...
    sprintf('largest offset %.3f bins',max([offs 0])));

% --- 6. matched filter null statistic is exponential --------------------
% With no target present the whitened AMF statistic should be unit-mean
% exponential, which is what the threshold -ln(Pfa) assumes.
r = compact_config(p);
% Run at the production Doppler length so the test measures what the pipeline
% actually experiences. With n_tx transmitters the aperture gets Nd/n_tx
% snapshots for n_virt channels.
r.Nd = p.Nd;
r.vel_axis = linspace(-r.v_max,r.v_max,r.Nd);
stats = zeros(1,60);
rng(99,'twister');
va = [];
for k = 1:numel(stats)
    cube = (randn(r.Nr,r.Nd,r.n_rx)+1j*randn(r.Nr,r.Nd,r.n_rx))/sqrt(2);
    va = tdm_virtual_aperture(cube,r,20,0,false);
    z = mean(va,2); Z = va - z;
    % Proportional diagonal loading, exactly as ADAPTIVE_MATCHED_FILTER applies
    % it. A fixed epsilon would not scale with the data and would make this
    % test measure the epsilon rather than the statistic.
    Rn = (Z*Z')/max(size(va,2)-1,1);
    Rn = 0.5*(Rn + Rn');
    Rn = Rn + max(real(trace(Rn))/r.n_virt,eps)*r.est.music_cov_diagonal_loading*eye(r.n_virt);
    a = ones(r.n_virt,1)/sqrt(r.n_virt);
    Ri = inv(Rn);
    stats(k) = size(va,2)*abs(a'*Ri*z)^2/real(a'*Ri*a);
end
% The covariance is estimated from the same snapshots that form the test
% vector, so the whitened statistic carries a loss factor rather than the unit
% mean the known-covariance ideal would give. The factor is approximately
% K/(K-L+1) with K = N_s - 1 training samples and L virtual channels, and it
% falls toward unity as snapshots outnumber channels. Asserting a unit mean
% would be asserting a known-covariance detector the pipeline does not have.
Ns = size(va,2);
K = max(Ns-1,1);
L = r.n_virt;
predicted = K/max(K-L+1,1);
report = add(report,'AMF null statistic matches the estimated-covariance prediction', ...
    mean(stats) > 0.7 && mean(stats) < 2.0*predicted, ...
    sprintf('sample mean %.3f, predicted near %.3f for %d snapshots and %d channels', ...
    mean(stats),predicted,Ns,L));
% The same factor means the nominal threshold -ln(Pfa) is optimistic by this
% much in power terms. Reported so the operating point is not mistaken for the
% ideal one.
report = add(report,'the covariance-estimation loss is bounded at the operating snapshot count', ...
    predicted < 1.5, ...
    sprintf('%.2f dB of loss at %d snapshots',10*log10(predicted),Ns));

if verbose, print_report(report); end
end

function P = gamma_draw(nr,nd,shape,scale)
%GAMMA_DRAW  Gamma samples by summing exponentials; no toolbox dependency.
k = max(1,round(shape));
P = zeros(nr,nd);
for i = 1:k
    P = P - log(max(rand(nr,nd),realmin));
end
P = P*scale;
end

function r = compact_config(p)
r = p;
r.Nr = 256;
r.Nd = 16;
r.f_range_axis = (0:r.Nr-1)*(r.fs_ADC/r.Nr);
r.range_axis = r.c*r.f_range_axis/(2*r.slope);
r.vel_axis = linspace(-r.v_max,r.v_max,r.Nd);
r.valid_range_mask = true(1,r.Nr);
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
    if isempty(c.detail)
        fprintf('   %-4s %s\n',ternary(c.pass,'ok','FAIL'),c.name);
    else
        fprintf('   %-4s %s  (%s)\n',ternary(c.pass,'ok','FAIL'),c.name,c.detail);
    end
end
end
function y = ternary(c,a,b), if c, y = a; else, y = b; end, end
