function [detections,threshold_map,noise_map,info] = adaptive_cfar_2d(RD_power,p,varargin)
%ADAPTIVE_CFAR_2D  Statistically calibrated two-dimensional CFAR detector.
%
%   [det,thr,noise,info] = ADAPTIVE_CFAR_2D(RD_power,p) thresholds a
%   range-Doppler power map. [det,...] = ADAPTIVE_CFAR_2D(RD_power,p,RefPower)
%   estimates the local background from a second, separately supplied map.
%   Both maps must carry absolute power on the same normalisation, which the
%   shared unitary FFT convention guarantees.
%
%   NULL MODEL
%   Each cell is the non-coherent sum of M = n_rx independent complex Gaussian
%   channel powers, so under H0 the normalised cell is Gamma(M, 1/M) with unit
%   mean. The multiplier alpha is obtained by solving the exact false-alarm
%   integral rather than by assuming a large-sample approximation:
%
%       P_fa(alpha) = Integral_0^inf Q( alpha x ; M, 1/M ) f_ref(x) dx
%
%   with Q the upper incomplete Gamma and f_ref the density of the reference
%   estimator. For a cell-averaging reference over n cells,
%   f_ref = Gamma(Mn, 1/(Mn)); for an order-statistic reference the density of
%   the k-th of n order statistics is used; for greatest-of and smallest-of the
%   density of the max and min of two independent side means is used. Each
%   alpha is solved by bisection and cached.
%
%   REFERENCE SELECTION
%   The window is partitioned into four disjoint training regions: leading and
%   lagging in range, leading and lagging in Doppler. Each region is classified
%   as homogeneous or contaminated by its coefficient of variation, compared
%   against the value the null model predicts, cv_H0 = 1/sqrt(M). This is the
%   variability index of Smith and Varshney extended to two dimensions.
%
%     * all homogeneous and mutually consistent means -> cell averaging over
%       the union, which is the minimum-variance choice;
%     * a clutter edge, detected as a mean ratio outside the configured band ->
%       cell averaging over the quietest homogeneous region, which avoids the
%       threshold being lifted by the bright side;
%     * every region contaminated, the multiple-interferer case -> an order
%       statistic over the full training set, which rejects up to
%       (1 - os_fraction) n interfering cells.
%
%   OUTPUTS
%   Hard detections are local maxima that exceed the calibrated threshold, are
%   grouped in range-Doppler, and are refined to sub-bin accuracy by parabolic
%   interpolation of the log-power. Weak candidates sit deliberately below the
%   declaration threshold and are handed to the track-before-detect branch;
%   they are candidates for temporal integration, never declarations.

if nargin >= 3 && ~isempty(varargin{1})
    RefPower = varargin{1};
else
RefPower = RD_power;
end
validateattributes(RD_power,{'numeric'},{'2d','real','nonnegative','finite'});
validateattributes(RefPower,{'numeric'},{'2d','real','nonnegative','finite','size',size(RD_power)});
[Nr,Nd] = size(RD_power);
modeName = lower(char(get_default_field(p.cfar,'mode','adaptive')));

Tr = round(p.cfar.Tr); Td = round(p.cfar.Td);
Gr = round(p.cfar.Gr); Gd = round(p.cfar.Gd);
H = Tr+Gr;
W = Td+Gd;
if Nr <= 2*H+1 || Nd <= 2*W+1
    error('adaptive_cfar_2d:Window','RD map %dx%d is too small for the configured CFAR window.',Nr,Nd);
end

% Region cardinalities: A/B lead and lag in range, C/D lead and lag in Doppler.
nA = Tr*(2*W+1); nB = nA;
nC = (2*Gr+1)*Td; nD = nC;
nFull = nA+nB+nC+nD;
if nFull < p.cfar.min_reference_cells
    error('adaptive_cfar_2d:ReferenceCells','Only %d CFAR reference cells; at least %d required.', ...
        nFull,p.cfar.min_reference_cells);
end

M    = max(1,round(get_default_field(p.cfar,'power_shape',1)));
Pfa  = p.cfar.Pfa;
kOS  = max(1,min(nFull-1,round(p.cfar.os_fraction*nFull)));
cvH0 = 1/sqrt(M);
viFactor = get_default_field(p.cfar,'heterogeneity_cv',0.8)/max(cvH0,eps);
ratioMax = min(get_default_field(p.cfar,'edge_ratio_high',1.55), ...
    1/max(get_default_field(p.cfar,'edge_ratio_low',0.65),eps));
cleanFloor = min(max(get_default_field(p.cfar,'os_contamination_ratio',0.75),0),1);

% Calibrated multipliers, honouring explicit overrides.
alphaFull = alpha_ca(nFull,Pfa,M,get_default_field(p.cfar,'ca_alpha',NaN));
alphaOS   = alpha_os(nFull,kOS,Pfa,M,get_default_field(p.cfar,'os_alpha',NaN));
alphaGO   = alpha_goso(nA,Pfa,M,true ,get_default_field(p.cfar,'go_alpha',NaN));
alphaSO   = alpha_goso(nA,Pfa,M,false,get_default_field(p.cfar,'so_alpha',NaN));

% ---- integral images give every region mean and variance in O(1) ---------
S1 = integral_image(RefPower);
S2 = integral_image(RefPower.^2);

noise_map     = zeros(Nr,Nd);
threshold_map = zeros(Nr,Nd);
method_map    = zeros(Nr,Nd,'uint8');   % 1 CA, 2 OS, 3 GO, 4 SO, 5 CA-subset
stat_snr      = -Inf(Nr,Nd);

rStart = 1+H;
rStop = Nr-H;
dStart = 1+W;
dStop = Nd-W;
validRows = true(Nr,1);
if isfield(p,'valid_range_mask'), validRows = p.valid_range_mask(:); end

for r = rStart:rStop
    if ~validRows(r), continue; end
    for d = dStart:dStop
        [sA,qA] = region_stats(S1,S2,r-H,r-Gr-1,d-W,d+W);
        [sB,qB] = region_stats(S1,S2,r+Gr+1,r+H,d-W,d+W);
        [sC,qC] = region_stats(S1,S2,r-Gr,r+Gr,d-W,d-Gd-1);
        [sD,qD] = region_stats(S1,S2,r-Gr,r+Gr,d+Gd+1,d+W);

        mu = [sA/nA, sB/nB, sC/max(nC,1), sD/max(nD,1)];
        m2 = [qA/nA, qB/nB, qC/max(nC,1), qD/max(nD,1)];
        cnt = [nA nB nC nD];
        mu = max(mu,realmin);
        cv = sqrt(max(m2./(mu.^2) - 1,0));

        switch modeName
            case 'ca'
                ref = (sA+sB+sC+sD)/nFull; alpha = alphaFull; mth = 1;
            case 'go'
                ref = max(mu(1),mu(2)); alpha = alphaGO; mth = 3;
            case 'so'
                ref = min(mu(1),mu(2)); alpha = alphaSO; mth = 4;
            case 'os'
                ref = order_statistic_reference(RefPower,r,d,H,W,Gr,Gd,kOS);
alpha = alphaOS;
mth = 2;
            case {'adaptive','vi'}
homo = cv <= viFactor*cvH0;
                % The clean part of the window must also be large enough to
                % trust. If contamination has consumed more than the permitted
                % fraction of the reference cells, fall through to the order
                % statistic rather than estimating the background from a
                % small surviving corner.
                cleanFraction = sum(cnt(homo))/nFull;
                if any(homo) && cleanFraction >= cleanFloor
                    hm = mu(homo); hc = cnt(homo);
                    ratio = max(hm)/max(min(hm),realmin);
                    if ratio <= ratioMax
                        % Homogeneous and mutually consistent: pool them.
                        n_used = sum(hc);
                        ref = sum(hm.*hc)/n_used;
mth = 1;
                    else
                        % Clutter edge: threshold on the quietest clean region
                        % so a bright neighbour cannot mask the target.
                        [ref,ix] = min(hm);
                        n_used = hc(ix);
mth = 5;
                    end
                    alpha = alpha_ca(n_used,Pfa,M,NaN);
                else
                    ref = order_statistic_reference(RefPower,r,d,H,W,Gr,Gd,kOS);
alpha = alphaOS;
mth = 2;
                end
            otherwise
                error('adaptive_cfar_2d:Mode','Unsupported CFAR mode: %s. Use ca, os, go, so or adaptive.',modeName);
        end

        ref = max(ref,realmin);
        noise_map(r,d)     = ref;
        threshold_map(r,d) = alpha*ref;
        method_map(r,d)    = mth;
        stat_snr(r,d)      = 10*log10(max(RD_power(r,d),realmin)/ref);
    end
end

% ---- declaration and weak-evidence masks --------------------------------
mask = RD_power > threshold_map & threshold_map > 0 & isfinite(threshold_map);
weakMask = RD_power > noise_map*10^(p.cfar.weak_snr_db/10) & noise_map > 0;
if isfield(p,'valid_range_mask')
    vr = repmat(p.valid_range_mask(:),1,Nd);
mask = mask & vr;
weakMask = weakMask & vr;

% Doppler-edge guard. The periodic spectrum makes the outer bins vulnerable to
% leakage and alias replicas whose velocity cannot be resolved reliably.
dMargin = max(0,round(get_default_field(p.cfar,'valid_doppler_margin_bins',0)));
if dMargin > 0 && Nd > 2*dMargin + 2
    vd = false(1,Nd);
    vd(1+dMargin:Nd-dMargin) = true;
    vdm = repmat(vd,Nr,1);
    mask = mask & vdm;
    weakMask = weakMask & vdm;
end

% A hard declaration is not a weak seed as well.
weakMask = weakMask & ~mask;
end

localMask = local_max_mask(RD_power,mask,p.cfar.local_max_r,p.cfar.local_max_d);
weakR = round(get_default_field(p.tbd,'local_max_range_radius',p.cfar.local_max_r));
weakD = round(get_default_field(p.tbd,'local_max_velocity_radius',p.cfar.local_max_d));
localWeak = local_max_mask(RD_power,weakMask,weakR,weakD);

[r,d] = find(localMask);
score = stat_snr(localMask);
[score,ord] = sort(score,'descend'); r = r(ord); d = d(ord);
if numel(r) > p.cfar.max_detections
    r = r(1:p.cfar.max_detections); d = d(1:p.cfar.max_detections); score = score(1:p.cfar.max_detections);
end

% Point-spread rejection, applied before grouping and on the ranked list, so
% every candidate is tested against the strongest response near it.
%
%   A windowed transform spreads a point target across a mainlobe several cells
%   wide, ringed by sidelobes whose level the window fixes: 31.5 dB for the
%   Hann taper used here. A target well above threshold therefore produces
%   secondary local maxima that clear CFAR on their own merits. They are real
%   energy and they are not separate objects.
%
%   The test is relative, not absolute. A candidate is discarded only when it
%   sits inside the response width of a stronger candidate AND falls below it
%   by more than the window is capable of producing. Two genuine targets of
%   comparable strength never trigger it, and a weak target beside a strong one
%   survives if it is separated by more than the response width.
if logical(get_default_field(p.cfar,'psf_rejection',false)) && numel(r) > 1
    psfR  = max(1,round(get_default_field(p.cfar,'psf_range_bins',6)));
    psfD  = max(1,round(get_default_field(p.cfar,'psf_doppler_bins',4)));
    psfDb = get_default_field(p.cfar,'psf_margin_db',26.0);
    pw = zeros(numel(r),1);
    for i = 1:numel(r), pw(i) = 10*log10(max(RD_power(r(i),d(i)),realmin)); end
    keepPsf = true(numel(r),1);
    for i = 1:numel(r)
        if ~keepPsf(i), continue; end
        near = abs(r-r(i)) <= psfR & abs(d-d(i)) <= psfD;
        near(i) = false;
        % score is sorted descending, so anything later in the list is weaker
        shadow = near & (pw < pw(i) - psfDb);
        keepPsf(shadow) = false;
    end
    nDropped = nnz(~keepPsf);
    r = r(keepPsf); d = d(keepPsf); score = score(keepPsf);
else
    nDropped = 0;
end

% Range-Doppler peak grouping: keep the strongest cell of each cluster.
keep = true(numel(r),1);
for i = 1:numel(r)
    if ~keep(i), continue; end
    near = abs(r-r(i)) <= p.cfar.group_r_bins & abs(d-d(i)) <= p.cfar.group_d_bins;
    near(i) = false;
    keep(near) = false;
end
r = r(keep); d = d(keep); score = score(keep);

subbin = logical(get_default_field(p.cfar,'subbin_interpolation',true));
detections = repmat(empty_detection(),0,1);
for i = 1:numel(r)
    det = empty_detection();
    det.r_bin = r(i); det.d_bin = d(i);
    det.range_bin_center    = p.range_axis(r(i));
    det.velocity_bin_center = p.vel_axis(d(i));
    if subbin
        [det.range,det.range_offset_bins]      = parabolic_axis(RD_power,r(i),d(i),p,'range');
        [det.velocity,det.doppler_offset_bins] = parabolic_axis(RD_power,r(i),d(i),p,'velocity');
    else
det.range = det.range_bin_center;
det.velocity = det.velocity_bin_center;
    end
    det.cfar_snr_db    = score(i);
    det.cfar_threshold = threshold_map(r(i),d(i));
    det.cfar_noise     = noise_map(r(i),d(i));
    det.cfar_mode      = method_name(method_map(r(i),d(i)));
det.subbin_refined = subbin;
    detections(end+1) = det;
end

[wr_,wd_] = find(localWeak);
weakScore = stat_snr(localWeak);
[weakScore,wo] = sort(weakScore,'descend'); wr_ = wr_(wo); wd_ = wd_(wo);
maxWeak = p.tbd.max_candidates_per_frame;
if numel(wr_) > maxWeak
    wr_ = wr_(1:maxWeak); wd_ = wd_(1:maxWeak); weakScore = weakScore(1:maxWeak);
end
weak_candidates = repmat(struct('r_bin',0,'d_bin',0,'range',0,'velocity',0,'snr_db',0),0,1);
for i = 1:numel(wr_)
    weak_candidates(end+1) = struct('r_bin',wr_(i),'d_bin',wd_(i), ...
        'range',p.range_axis(wr_(i)),'velocity',p.vel_axis(wd_(i)),'snr_db',weakScore(i));
end

info = struct();
info.Pfa = Pfa;
info.mode = modeName;
info.power_shape = M;
info.alpha_ca = alphaFull;
info.alpha_os = alphaOS;
info.alpha_go = alphaGO;
info.alpha_so = alphaSO;
info.training_cells = nFull;
info.region_cells   = [nA nB nC nD];
info.os_order = kOS;
info.vi_threshold_cv = viFactor*cvH0;
info.clean_reference_floor = cleanFloor;
info.method_map = method_map;
info.statistical_count = nnz(mask);
info.local_peak_count  = numel(r);
info.psf_rejected_count = nDropped;
info.weak_candidate_count = numel(weak_candidates);
info.threshold_median = median(threshold_map(threshold_map>0),'omitnan');
info.noise_median     = median(noise_map(noise_map>0),'omitnan');
info.reference_power_median = median(RefPower(:),'omitnan');
info.weak_candidates = weak_candidates;
end

% =========================================================================
% Integral-image helpers
% =========================================================================
function S = integral_image(X)
S = zeros(size(X,1)+1,size(X,2)+1);
S(2:end,2:end) = cumsum(cumsum(X,1),2);
end

function [s1,s2] = region_stats(S1,S2,r1,r2,d1,d2)
if r2 < r1 || d2 < d1, s1 = 0;
s2 = 0;
return;
end
s1 = S1(r2+1,d2+1) - S1(r1,d2+1) - S1(r2+1,d1) + S1(r1,d1);
s2 = S2(r2+1,d2+1) - S2(r1,d2+1) - S2(r2+1,d1) + S2(r1,d1);
end

function ref = order_statistic_reference(RefPower,r,d,H,W,Gr,Gd,k)
w = RefPower(r-H:r+H,d-W:d+W);
keep = true(size(w));
keep((H-Gr+1):(H+Gr+1),(W-Gd+1):(W+Gd+1)) = false;
trn = sort(w(keep),'ascend');
ref = max(trn(min(k,numel(trn))),realmin);
end

% =========================================================================
% Exact Pfa calibration
% =========================================================================
function a = alpha_ca(N,Pfa,M,override)
if isfinite(override) && override > 0, a = override; return; end
persistent kN kP kM kA
if isempty(kN), kN = []; kP = []; kM = []; kA = []; end
ix = find(kN == N & kP == Pfa & kM == M,1);
if ~isempty(ix), a = kA(ix); return; end
shapeCut = M; scaleCut = 1/M; shapeRef = M*N; scaleRef = 1/(M*N);
a = bisect_alpha(@(aa) integral(@(x) gamma_survival(aa*x,shapeCut,scaleCut).* ...
    gamma_pdf_local(x,shapeRef,scaleRef),0,Inf,'RelTol',1e-8,'AbsTol',1e-12),Pfa);
kN(end+1) = N; kP(end+1) = Pfa; kM(end+1) = M; kA(end+1) = a;
end

function a = alpha_os(N,k,Pfa,M,override)
if isfinite(override) && override > 0, a = override; return; end
persistent kN kK kP kM kA
if isempty(kN), kN = []; kK = []; kP = []; kM = []; kA = []; end
ix = find(kN == N & kK == k & kP == Pfa & kM == M,1);
if ~isempty(ix), a = kA(ix); return; end
shape = M;
scale = 1/M;
a = bisect_alpha(@(aa) integral(@(x) gamma_survival(aa*x,shape,scale).* ...
    order_pdf(x,N,k,shape,scale),0,Inf,'RelTol',1e-8,'AbsTol',1e-12),Pfa);
kN(end+1) = N; kK(end+1) = k; kP(end+1) = Pfa; kM(end+1) = M; kA(end+1) = a;
end

function a = alpha_goso(n,Pfa,M,goMode,override)
if isfinite(override) && override > 0, a = override; return; end
persistent kN kP kM kGO kSO
if isempty(kN), kN = []; kP = []; kM = []; kGO = []; kSO = []; end
ix = find(kN == n & kP == Pfa & kM == M,1);
if isempty(ix)
    kN(end+1) = n; kP(end+1) = Pfa; kM(end+1) = M; kGO(end+1) = NaN; kSO(end+1) = NaN;
    ix = numel(kN);
    kGO(ix) = goso_alpha(n,Pfa,M,true);
    kSO(ix) = goso_alpha(n,Pfa,M,false);
end
if goMode, a = kGO(ix); else, a = kSO(ix); end
end

function a = goso_alpha(n,Pfa,M,goMode)
% CUT ~ Gamma(M,1/M); each side mean ~ Gamma(Mn,1/(Mn)).
% For the max of two iid side means the density is 2 f(x) F(x); for the min it
% is 2 f(x) (1-F(x)).
shapeCut = max(1,M); scaleCut = 1/max(M,1);
shapeSide = max(1,n*M); scaleSide = 1/max(shapeSide,1);
fun = @(aa) integral(@(x) gamma_survival(aa*x,shapeCut,scaleCut).* ...
    (2*gamma_pdf_local(x,shapeSide,scaleSide).*side_cdf(x,shapeSide,scaleSide,goMode)), ...
    0,Inf,'RelTol',1e-8,'AbsTol',1e-12);
a = bisect_alpha(fun,Pfa);
end

function a = bisect_alpha(fun,Pfa)
lo = 0;
hi = 1;
while safe_eval(fun,hi) > Pfa && hi < 1e6, hi = 2*hi; end
for z = 1:80
    mid = (lo+hi)/2;
    if safe_eval(fun,mid) > Pfa, lo = mid; else, hi = mid; end
end
a = (lo+hi)/2;
end

function v = safe_eval(fun,x)
try
    v = fun(x);
catch
v = 1;
end
if ~isfinite(v), v = 1; end
end

function y = gamma_survival(x,shape,scale)
y = gammainc(max(x,0)/max(scale,eps),shape,'upper');
end

function y = gamma_pdf_local(x,shape,scale)
q = max(x,0); y = zeros(size(q)); pos = q > 0;
y(pos) = exp((shape-1).*log(q(pos)) - q(pos)./scale - gammaln(shape) - shape*log(scale));
end

function y = order_pdf(x,N,k,shape,scale)
F = gammainc(max(x,0)/max(scale,eps),shape,'lower');
f = gamma_pdf_local(x,shape,scale);
logc = gammaln(N+1) - gammaln(k) - gammaln(N-k+1);
logy = logc + (k-1).*log(max(F,realmin)) + (N-k).*log(max(1-F,realmin)) + log(max(f,realmin));
y = exp(logy); y(~isfinite(y)) = 0;
end

function q = side_cdf(x,shape,scale,goMode)
F = gammainc(x/max(scale,eps),shape,'lower');
if goMode, q = F;
else, q = 1-F;
end
end

% =========================================================================
% Peak handling
% =========================================================================
function m = local_max_mask(P,mask,rr,dd)
[Nr,Nd] = size(P); m = false(Nr,Nd);
rr = max(0,round(rr)); dd = max(0,round(dd));
[r,d] = find(mask);
for k = 1:numel(r)
    r1 = max(1,r(k)-rr); r2 = min(Nr,r(k)+rr);
    d1 = max(1,d(k)-dd); d2 = min(Nd,d(k)+dd);
    m(r(k),d(k)) = P(r(k),d(k)) >= max(P(r1:r2,d1:d2),[],'all');
end
end

function [v,delta] = parabolic_axis(P,k,d,p,which)
% Three-point parabolic interpolation of the log-power peak. For a windowed
% FFT main lobe the log magnitude is locally quadratic, so the vertex offset
% estimates the fractional bin position.
delta = 0;
switch which
    case 'range'
        v = p.range_axis(k);
        if k <= 1 || k >= size(P,1), return; end
        y = 10*log10(max(P(k-1:k+1,d),realmin));
step = p.range_resolution_actual;
        base = p.range_axis(k);
    otherwise
        v = p.vel_axis(d);
        if d <= 1 || d >= size(P,2), return; end
        y = 10*log10(max(P(k,d-1:d+1),realmin));
step = p.velocity_resolution_actual;
        base = p.vel_axis(d);
end
y = y(:);
den = y(1) - 2*y(2) + y(3);
if abs(den) > 1e-12
    delta = max(-0.5,min(0.5,0.5*(y(1)-y(3))/den));
end
v = base + delta*step;
end

function s = method_name(k)
labels = {'CA','OS','GO','SO','CA-subset'};
k = max(1,min(numel(labels),double(k)));
s = labels{k};
end

function d = empty_detection()
d = struct('range',0,'velocity',0,'angle_deg',NaN,'r_bin',1,'d_bin',1, ...
    'range_bin_center',0,'velocity_bin_center',0, ...
    'cfar_snr_db',-Inf,'cfar_threshold',0,'cfar_noise',0,'cfar_mode','', ...
    'subbin_refined',false,'amf_stat',0,'amf_db',-Inf,'angle_fft_deg',NaN, ...
    'angle_music_deg',NaN,'angle_deg_std',NaN,'power',0,'snr_db',-Inf, ...
    'range_offset_bins',0,'doppler_offset_bins',0);
end
