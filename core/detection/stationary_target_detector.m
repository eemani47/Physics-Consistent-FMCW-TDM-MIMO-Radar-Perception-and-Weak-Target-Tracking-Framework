function [detections,threshold_map,noise_map,info] = stationary_target_detector(PstatRA,p)
%STATIONARY_TARGET_DETECTOR  Calibrated hit decision on the stationary map.
%
%   Parked vehicles, barriers, poles and gantries are legitimate targets, but
%   they occupy the same Doppler cell as the ground clutter the moving branch
%   deliberately removes. They are therefore detected on their own map: the
%   range-angle power surface produced by beamforming the coherent stationary
%   estimate across the virtual aperture.
%
%   The background statistics of that surface differ from the moving map. Each
%   cell is a single beamformed complex sample, so its power is exponential
%   rather than a sum of channel powers, and the threshold multiplier is
%   calibrated for that null:
%
%       CUT ~ Exp(1),   reference mean ~ Gamma(n, 1/n)
%       P_fa(alpha) = Integral_0^inf exp(-alpha x) f_Gamma(x; n, 1/n) dx
%
%   solved numerically for alpha at the configured stationary P_fa. Setting
%   use_calibrated_cfar to false substitutes the fixed multiplier
%   threshold_scale, which is retained for direct comparison against the
%   literature formulation.
%
%   Detections are two-dimensional local maxima on the range-angle grid, so a
%   single extended reflector produces one hit per resolvable centre rather
%   than a ridge of adjacent cells.

detections = repmat(empty_detection(),0,1);
[Nr,Na] = size(PstatRA);
threshold_map = zeros(Nr,Na);
noise_map     = zeros(Nr,Na);

if ~p.paper.stationary.enabled || isempty(PstatRA)
    info = struct('enabled',false,'candidate_count',0,'local_peak_count',0, ...
        'alpha',NaN,'calibrated',false,'Pfa',get_default_field(p.paper.stationary,'Pfa',NaN), ...
        'threshold_scale',p.paper.stationary.threshold_scale);
return;
end

Rr = round(p.paper.stationary.range_ref_bins);
Ra = round(p.paper.stationary.angle_ref_bins);
Gr = round(p.paper.stationary.range_guard_bins);
Ga = round(p.paper.stationary.angle_guard_bins);
HR = Rr+Gr;
HA = Ra+Ga;
if Nr <= 2*HR+1 || Na <= 2*HA+1
    info = struct('enabled',false,'candidate_count',0,'local_peak_count',0, ...
        'alpha',NaN,'calibrated',false,'Pfa',get_default_field(p.paper.stationary,'Pfa',NaN), ...
        'threshold_scale',p.paper.stationary.threshold_scale);
return;
end

refCount = (2*HR+1)*(2*HA+1) - (2*Gr+1)*(2*Ga+1);
calibrated = logical(get_default_field(p.paper.stationary,'use_calibrated_cfar',true));
Pfa = get_default_field(p.paper.stationary,'Pfa',1e-4);
if calibrated
    alpha = stationary_alpha(refCount,Pfa);
else
alpha = p.paper.stationary.threshold_scale;
end

% Integral images make the reference mean O(1) per cell.
S = zeros(Nr+1,Na+1);
S(2:end,2:end) = cumsum(cumsum(PstatRA,1),2);

validRows = true(Nr,1);
if isfield(p,'valid_range_mask'), validRows = p.valid_range_mask(:); end

for r = 1+HR:Nr-HR
    if ~validRows(r), continue; end
    for a = 1+HA:Na-HA
        outer = box_sum(S,r-HR,r+HR,a-HA,a+HA);
        inner = box_sum(S,r-Gr,r+Gr,a-Ga,a+Ga);
        bg = max((outer-inner)/refCount,realmin);
        noise_map(r,a)     = bg;
        threshold_map(r,a) = bg*alpha;
    end
end

mask = PstatRA > threshold_map & threshold_map > 0;
if isfield(p,'valid_range_mask')
    mask = mask & repmat(p.valid_range_mask(:),1,Na);
end
mask = local_max_mask(PstatRA,mask,p.paper.stationary.local_max_range,p.paper.stationary.local_max_angle);

[r,a] = find(mask);
score = 10*log10(max(PstatRA(mask),realmin)./max(noise_map(mask),realmin));
[score,ord] = sort(score,'descend'); r = r(ord); a = a(ord);
maxD = p.paper.stationary.max_detections;
if numel(r) > maxD, r = r(1:maxD); a = a(1:maxD); score = score(1:maxD); end

zeroBin = nearest_zero_doppler_bin(p.vel_axis);
for k = 1:numel(r)
    d = empty_detection();
    d.r_bin = r(k); d.d_bin = zeroBin;
    d.range = p.range_axis(r(k));
d.range_bin_center = d.range;
d.velocity = 0;
d.velocity_bin_center = 0;
    d.angle_deg = p.theta_axis(a(k));
    d.cfar_snr_db = score(k);
    d.cfar_threshold = threshold_map(r(k),a(k));
    d.cfar_noise = noise_map(r(k),a(k));
    d.cfar_mode = ternary_local(calibrated,'stationary-calibrated','stationary-fixed-scale');
    d.origin = 'stationary';
d.is_hard = true;
    d.quality_score_db = score(k);
    detections(end+1) = d;
end

info = struct('enabled',true,'candidate_count',nnz(mask),'local_peak_count',numel(detections), ...
    'alpha',alpha,'calibrated',calibrated,'Pfa',Pfa,'reference_cells',refCount, ...
    'threshold_scale',p.paper.stationary.threshold_scale, ...
    'noise_median',median(noise_map(noise_map>0),'omitnan'));
end

% =========================================================================
function a = stationary_alpha(N,Pfa)
persistent kN kP kA
if isempty(kN), kN = []; kP = []; kA = []; end
ix = find(kN == N & kP == Pfa,1);
if ~isempty(ix), a = kA(ix); return; end
shape = max(1,N); scale = 1/max(N,1);
lo = 0;
hi = 1;
fun = @(aa) integral(@(x) exp(-aa*x).*gamma_pdf_local(x,shape,scale),0,Inf, ...
    'RelTol',1e-10,'AbsTol',1e-14);
while fun(hi) > Pfa && hi < 1e6, hi = 2*hi; end
for it = 1:80
    mid = (lo+hi)/2;
    if fun(mid) > Pfa, lo = mid; else, hi = mid; end
end
a = (lo+hi)/2;
kN(end+1) = N; kP(end+1) = Pfa; kA(end+1) = a;
end

function y = gamma_pdf_local(x,shape,scale)
q = max(x,0); y = zeros(size(q)); pos = q > 0;
y(pos) = exp((shape-1).*log(q(pos)) - q(pos)./scale - gammaln(shape) - shape*log(scale));
end

function s = box_sum(S,r1,r2,c1,c2)
s = S(r2+1,c2+1) - S(r1,c2+1) - S(r2+1,c1) + S(r1,c1);
end

function tf = local_max_mask(P,mask,rr,aa)
[Nr,Na] = size(P); tf = false(Nr,Na);
rr = max(0,round(rr)); aa = max(0,round(aa));
[ir,ia] = find(mask);
for k = 1:numel(ir)
    r = ir(k); a = ia(k);
    r1 = max(1,r-rr); r2 = min(Nr,r+rr);
    a1 = max(1,a-aa); a2 = min(Na,a+aa);
    tf(r,a) = P(r,a) >= max(P(r1:r2,a1:a2),[],'all');
end
end

function k = nearest_zero_doppler_bin(v), [~,k] = min(abs(v)); end
function y = ternary_local(c,a,b), if c, y = a; else, y = b; end, end

function d = empty_detection()
d = struct('range',0,'velocity',0,'angle_deg',NaN,'r_bin',1,'d_bin',1, ...
    'range_bin_center',0,'velocity_bin_center',0, ...
    'cfar_snr_db',-Inf,'cfar_threshold',0,'cfar_noise',0,'cfar_mode','', ...
    'subbin_refined',false,'amf_stat',0,'amf_db',-Inf,'amf_class','verified', ...
    'angle_fft_deg',NaN,'aoa_fft_deg',NaN,'angle_music_deg',NaN,'music_peak_db',-Inf, ...
    'music_peak_prominence_db',-Inf,'fft_peak_prominence_db',-Inf,'angle_difference_deg',NaN, ...
    'music_theta',zeros(1,0),'music_spectrum',zeros(1,0),'fft_theta',zeros(1,0), ...
    'fft_spectrum',zeros(1,0),'music_info',struct(),'power',0,'snr_db',-Inf, ...
    'range_offset_bins',0,'doppler_offset_bins',0,'origin','stationary','is_hard',true, ...
    'tbd_snr_db',-Inf,'velocity_raw',0,'tdm_alias_offset',0,'tdm_alias_score',NaN, ...
    'tdm_alias_mode','','tdm_alias_angle_preview',NaN,'quality_score_db',-Inf, ...
    'confidence',0,'is_cluster_supported',false,'aoa_quality_db',-Inf);
end
