function [detections,info] = adaptive_matched_filter(detections,rx_cube,p)
%ADAPTIVE_MATCHED_FILTER  Physical template verification of hard CFAR points.
%
%   CFAR answers a purely energetic question: is this cell brighter than its
%   neighbourhood by more than the null distribution allows. It says nothing
%   about whether the energy is spatially consistent with a plane wave arriving
%   from any single direction. Noise spikes, sidelobe leakage and interference
%   residue all pass that first test.
%
%   This stage answers the second question. It forms the virtual aperture at
%   the candidate's range bin and velocity, estimates the interference-plus-
%   noise covariance from the snapshot spread, and evaluates the adaptive
%   matched filter over the steering manifold:
%
%       Lambda(theta) = | a(theta)^H R^-1 z |^2 / ( a(theta)^H R^-1 a(theta) )
%
%       T = N_s * max_theta Lambda(theta)
%
%   The maximiser is the angle estimate; the maximum is the detection
%   statistic. Under H0 the whitened statistic is exponentially distributed
%   with unit mean, so the threshold follows in closed form,
%
%       T > -ln P_fa
%
%   and the operating point is floored by an explicit SNR margin so that the
%   test can be made stricter than its nominal false-alarm rate demands. The
%   effective threshold is the larger of the two and is reported in INFO.
%
%   The search runs coarse-to-fine: a uniform sweep at the coarse step locates
%   the lobe, then a local refinement resolves it to the fine step. The total
%   number of hypotheses is capped so that the cost of one candidate cannot
%   grow without bound when the aperture is wide.
%
%   Sub-CFAR evidence never reaches this function on the normal path. The
%   track-before-detect branch verifies its own trajectories by calling this
%   filter with an explicitly relaxed configuration, which keeps the two
%   evidence channels separate and auditable.

info = struct('candidate_count',numel(detections),'evaluated_count',0,'accepted_count',0, ...
    'rejected_amf_count',0,'gs_rescued_count',0,'noise_variances',[],'angle_search_count',0, ...
    'covariance_loading',[],'threshold_stat',NaN,'threshold_db',NaN, ...
    'failed_candidate_count',0,'dropped_over_budget',0,'whitened',true,'errors',{{}});
if isempty(detections), return; end
validateattributes(rx_cube,{'numeric'},{'3d','nonempty'},mfilename,'rx_cube',2);

% ---- candidate budget ---------------------------------------------------
maxCand = max(1,round(get_default_field(p.detector,'max_candidates',128)));
if numel(detections) > maxCand
    snr = arrayfun(@(d) get_default_field(d,'cfar_snr_db',-Inf),detections);
    [~,ord] = sort(snr,'descend');
    info.dropped_over_budget = numel(detections)-maxCand;
    detections = detections(ord(1:maxCand));
end

% ---- hard-point contract ------------------------------------------------
hardOnly = logical(get_default_field(p.detector,'hard_point_only',true));
if hardOnly
    isHard = arrayfun(@(d) logical(get_default_field(d,'is_hard',true)),detections);
    if ~all(isHard)
        error('adaptive_matched_filter:HardPointOnly', ...
            'Sub-CFAR evidence reached the matched filter while hard_point_only is enabled.');
    end
end

% ---- operating point ----------------------------------------------------
amfPfa = max(min(p.detector.amf_threshold_pfa,0.2),1e-12);
thresholdDb = max(10*log10(-log(amfPfa)),p.detector.min_amf_db);
thresholdStat = 10^(thresholdDb/10);
info.threshold_stat = thresholdStat;
info.threshold_db   = thresholdDb;
whitened = logical(get_default_field(p.detector,'use_whitened_amf',true));
info.whitened = whitened;
minTemplateEnergy = get_default_field(p.detector,'min_template_energy',1e-12);
guardRatio = min(max(get_default_field(p.detector,'noise_estimation_guard_ratio',0.90),0.05),1);

% Robust per-chain noise estimate from the lower guardRatio quantile of the
% dechirped power, which is insensitive to the small fraction of cells that
% actually contain targets or interference.
noise_var = zeros(1,p.n_rx);
for rx = 1:p.n_rx
    z = abs(rx_cube(:,:,rx)).^2;
    q = quantile_local(z(:),guardRatio*0.5);
    % For CN(0,s) power, the q-th quantile is -s ln(1-q).
    noise_var(rx) = max(q/max(-log(1-guardRatio*0.5),eps),realmin);
end
info.noise_variances = noise_var;

gsCfg = get_default_field(p.detector,'gs',struct('enabled',false,'rescue_enabled',false,'rescue_min_db',0));
gsRescueAllowed = logical(get_default_field(gsCfg,'enabled',false)) && ...
                  logical(get_default_field(gsCfg,'rescue_enabled',false));
gsRescueMinDb   = get_default_field(gsCfg,'rescue_min_db',0);

keep = false(1,numel(detections));
for i = 1:numel(detections)
    det = detections(i);
    try
        rb = nearest_index(p.range_axis,det.range);
        va = tdm_virtual_aperture(rx_cube,p,rb,det.velocity,false);
        Ns = size(va,2);
        if Ns < 4
info.failed_candidate_count = info.failed_candidate_count + 1;
continue;
        end
info.evaluated_count = info.evaluated_count + 1;

        z = mean(va,2);
Z = va - z;
        Rn = (Z*Z')/max(Ns-1,1);
        Rn = 0.5*(Rn + Rn');
        loading = max(real(trace(Rn))/max(p.n_virt,1),eps)*p.est.music_cov_diagonal_loading;
        Rn = Rn + loading*eye(p.n_virt);
        info.covariance_loading(end+1) = loading;

        [grid1,grid2] = build_angle_grids(p,det.angle_deg);
        info.angle_search_count = info.angle_search_count + numel(grid1) + numel(grid2);

        [theta0,~] = amf_scan(z,Rn,det.range,grid1,p,whitened,noise_var,minTemplateEnergy);
fine = grid2 + theta0;
        fine = fine(fine >= -p.az_span & fine <= p.az_span);
        if isempty(fine), fine = theta0; end
        [theta,score] = amf_scan(z,Rn,det.range,fine,p,whitened,noise_var,minTemplateEnergy);

        % z averages Ns snapshots, so Cov(z) = R/Ns and the test statistic
        % carries the full coherent snapshot gain.
        stat = Ns*max(real(score),0);
        amf_db = 10*log10(max(stat,realmin));

det.angle_deg      = theta;
det.angle_music_deg= theta;
det.amf_stat       = stat;
det.amf_db         = amf_db;
det.amf_threshold_db = thresholdDb;
        det.power          = real(z'*z);
det.snr_db         = det.cfar_snr_db;

        gsValid  = logical(get_default_field(det,'gs_valid',false));
        gsInterf = logical(get_default_field(det,'gs_interference_detected',false));
        gsDb     = get_default_field(det,'gs_db',-Inf);
det.gs_rescue = gsRescueAllowed && gsValid && gsInterf;

        if isfinite(stat) && stat >= thresholdStat
            keep(i) = true;
            det.amf_class = 'verified';
        elseif det.gs_rescue && gsDb >= gsRescueMinDb
            keep(i) = true;
            det.amf_class = 'gs_rescued';
info.gs_rescued_count = info.gs_rescued_count + 1;
        else
            det.amf_class = 'rejected';
info.rejected_amf_count = info.rejected_amf_count + 1;
        end
        detections(i) = det;
    catch ME
info.failed_candidate_count = info.failed_candidate_count + 1;
        info.errors{end+1} = sprintf('candidate %d: %s',i,ME.message);
    end
end

detections = detections(keep);
info.accepted_count = numel(detections);
end

% =========================================================================
function [coarse,fineOffsets] = build_angle_grids(p,priorAngle)
%BUILD_ANGLE_GRIDS  Coarse sweep plus a local refinement offset grid.
maxHyp = max(8,round(get_default_field(p.detector,'max_angle_hypotheses',361)));
coarse = -p.az_span:p.detector.angle_coarse_step_deg:p.az_span;
if isfinite(priorAngle)
    coarse = unique([coarse priorAngle-4 priorAngle priorAngle+4]);
end
coarse = coarse(coarse >= -p.az_span & coarse <= p.az_span);
half = p.detector.angle_refine_halfspan_deg;
step = p.detector.angle_refine_step_deg;
fineOffsets = -half:step:half;
budget = maxHyp - numel(fineOffsets);
if budget < 8, budget = max(8,round(maxHyp/2)); end
if numel(coarse) > budget
    coarse = coarse(round(linspace(1,numel(coarse),budget)));
end
end

function [thetaBest,scoreBest] = amf_scan(z,Rn,R,thetaGrid,p,whitened,noise_var,minEnergy)
%AMF_SCAN  Maximise the matched-filter statistic over an angular grid.
if whitened
    Ri = pinv_hermitian(Rn,1e-10);
else
    Ri = eye(size(Rn,1))/max(mean(noise_var),realmin);
end
A = steering_bank(p,R,thetaGrid,minEnergy);
num = abs(A'*(Ri*z)).^2;
den = real(sum(conj(A).*(Ri*A),1)).';
scores = num./max(den,realmin);
scores(~isfinite(scores)) = -Inf;
[scoreBest,ix] = max(scores);
if isempty(ix)
thetaBest = 0;
scoreBest = -Inf;
else
    thetaBest = thetaGrid(ix);
end
end

function A = steering_bank(p,R,thetaGrid,minEnergy)
%STEERING_BANK  Exact near-field bistatic steering vectors, unit norm.
R  = max(R,1e-3);
th = thetaGrid(:).';
xt = R*sind(th); yt = R*cosd(th);
A  = complex(zeros(p.n_virt,numel(th)));
vi = 1;
for tx = 1:p.n_tx
    dtx = hypot(xt-p.tx_x(tx),yt);
    for rx = 1:p.n_rx
        drx = hypot(xt-p.rx_x(rx),yt);
        tau = (dtx+drx)/p.c;
        A(vi,:) = exp(1j*2*pi*p.fc*tau)./max(dtx.*drx,eps);
vi = vi + 1;
    end
end
energy = sum(abs(A).^2,1);
if any(energy < minEnergy)
    A(:,energy < minEnergy) = 0;
end
A = A./max(sqrt(energy),eps);
end

function q = quantile_local(x,pr)
x = x(isfinite(x));
if isempty(x), q = 0; return; end
x = sort(x,'ascend');
k = max(1,min(numel(x),ceil(pr*numel(x))));
q = x(k);
end

function i = nearest_index(x,v), [~,i] = min(abs(x-v)); end

function Ri = pinv_hermitian(A,floorFrac)
A = 0.5*(A+A');
[V,D] = eig(A,'vector');
D = real(D);
D = max(D,max(D)*floorFrac);
Ri = V*diag(1./D)*V';
end
