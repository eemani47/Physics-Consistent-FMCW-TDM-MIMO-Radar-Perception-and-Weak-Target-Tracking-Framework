function [detections,info] = generalized_subspace_detector(detections,rx_cube,p)
%GENERALIZED_SUBSPACE_DETECTOR  Spatial detection under MIMO-FMCW interference.
%
%   Implements the low-dimensional spatial model of the generalized-subspace
%   detector for automotive MIMO-FMCW mutual interference. In a TDM schedule a
%   desired echo occupies the transmit-receive product manifold
%
%       s = a_t(theta_t) (x) a_r(theta_t)
%
%   whereas an interferer that enters through the receive array while the
%   victim's own transmitter is firing occupies
%
%       i = a_t(theta_t) (x) a_r(theta_i)
%
%   The two share the transmit factor and differ only in the receive factor,
%   so the interference is confined to a rank-one subspace whose direction is
%   known once theta_i is estimated. Writing the covariance as
%
%       R = sigma^2 ( I + rho i i^H ) ,   rho = eta / sigma^2
%
%   the inverse follows in closed form from the Sherman-Morrison identity,
%
%       R^-1 = ( 1 / sigma^2 ) ( I - rho/(1+rho) i i^H )
%
%   which avoids inverting an estimated full-rank covariance from the small
%   number of snapshots a single range-Doppler cell provides. The detector is
%
%       T = 2 | y^H R^-1 s |^2 / ( sigma^2 s^H R^-1 s )  >  gamma ,
%       P_fa = exp( -gamma / 2 )
%
%   The interference direction is estimated from the receive-only covariance
%   obtained by summing the transmit blocks, subject to a minimum angular
%   separation from the target so that the target is never nulled by its own
%   subspace. The aligned interference power is recovered from the residual
%   covariance after projecting out the desired rank-one component.
%
%   This runs in parallel with the matched filter on hard CFAR points only.
%   Its purpose is to preserve targets that a plain energy test would discard
%   because an interferer inflated their local noise estimate.

info = struct('candidate_count',numel(detections),'evaluated_count',0, ...
    'accepted_count',0,'rescued_count',0,'interference_detected_count',0, ...
    'mean_inr_db',-Inf,'gamma',NaN,'pfa',NaN,'noise_source','','errors',{{}});
if isempty(detections), return; end
validateattributes(rx_cube,{'numeric'},{'3d','nonempty'},mfilename,'rx_cube',2);

cfg = p.detector.gs;
info.pfa   = cfg.pfa;
info.gamma = max(-2*log(cfg.pfa),1);

useMeasured = logical(get_default_field(cfg,'use_measured_noise',true));
if useMeasured
    % Median-based estimate of the per-sample chain noise power. For CN(0,s)
    % power the median is s ln 2, and the median is unaffected by the small
    % fraction of cells carrying targets or interference ridges.
    est = zeros(1,p.n_rx);
    for rx = 1:p.n_rx
        z = abs(rx_cube(:,:,rx)).^2;
        est(rx) = median(z(:),'omitnan')/log(2);
    end
    sigma_chain = max(mean(est),realmin);
    info.noise_source = 'measured median power';
else
    sigma_chain = max(mean(p.rx_noise_power_W),realmin);
    info.noise_source = 'configured kTBF';
end

inrAccum = [];
for i = 1:numel(detections)
    try
        det = detections(i);
        rb = nearest_index(p.range_axis,det.range);
        [va,rb] = tdm_virtual_aperture(rx_cube,p,rb,det.velocity,false);
        Ns = size(va,2);
if Ns < cfg.min_snapshots, continue;
end

        y = mean(va,2);
        C = (va*va')/Ns;  C = 0.5*(C+C');
        sigma2 = sigma_chain/max(Ns,1);      % covariance of the snapshot mean

        thetaT = get_default_field(det,'angle_deg',NaN);
        if ~isfinite(thetaT), thetaT = estimate_spatial_peak(y,p); end
        thetaT = max(-p.az_span,min(p.az_span,thetaT));

        at  = tx_steering(p,thetaT);
        s   = kron(at,rx_steering(p,thetaT)); s = s/max(norm(s),eps);

        % Receive-only covariance: summing the transmit blocks removes the
        % common transmit factor and leaves a pure receive-aperture estimate.
        Crx = zeros(p.n_rx);
        for tx = 1:p.n_tx
            rows = (tx-1)*p.n_rx + (1:p.n_rx);
            V = va(rows,:);
            Crx = Crx + (V*V')/Ns;
        end
        Crx = 0.5*(Crx+Crx');

        thetaGrid = (-p.az_span):cfg.angle_step_deg:p.az_span;
        Ar = rx_steering_bank(p,thetaGrid);
        ps = real(sum(conj(Ar).*(Crx*Ar),1))./max(real(sum(conj(Ar).*Ar,1)),eps);
        [~,ord] = sort(ps,'descend');
thetaI = thetaT;
found = false;
        for q = 1:numel(ord)
            cand = thetaGrid(ord(q));
            if abs(wrap_angle(cand-thetaT)) >= cfg.min_interference_angle_sep_deg
thetaI = cand;
found = true;
break;
            end
        end

        ai = kron(at,rx_steering(p,thetaI)); ai = ai/max(norm(ai),eps);
        Ps = eye(p.n_virt) - (s*s');
        Cres = Ps*C*Ps; Cres = 0.5*(Cres+Cres');
        eta = max(real(ai'*Cres*ai) - sigma_chain,0);
        rho = min(max(eta/max(sigma_chain,eps),0),cfg.max_inr_linear);
        inr_db = 10*log10(max(rho,realmin));

        Rinv = eye(p.n_virt) - (rho/(1+rho))*(ai*ai');
        den  = max(real(s'*Rinv*s),eps);
        stat = 2*abs(y'*Rinv*s)^2/(sigma2*den);
        gs_db = 10*log10(max(real(stat),realmin));

        valid = isfinite(stat) && stat >= info.gamma;
interferenceDetected = found && inr_db >= cfg.interference_inr_threshold_db;

        detections(i).gs_stat  = real(stat);
        detections(i).gs_db    = gs_db;
        detections(i).gs_valid = logical(valid);
        detections(i).gs_interference_angle_deg = thetaI;
        detections(i).gs_target_angle_deg = thetaT;
        detections(i).gs_inr_db = inr_db;
        detections(i).gs_interference_detected = logical(interferenceDetected);
        detections(i).gs_noise_power = sigma2;
        detections(i).gs_r_bin = rb;
        detections(i).gs_enabled = true;

info.evaluated_count = info.evaluated_count + 1;
if valid, info.accepted_count = info.accepted_count + 1;
end
if valid && interferenceDetected, info.rescued_count = info.rescued_count + 1;
end
if interferenceDetected, info.interference_detected_count = info.interference_detected_count + 1;
end
        if isfinite(inr_db), inrAccum(end+1) = inr_db; end
    catch ME
        info.errors{end+1} = sprintf('candidate %d: %s',i,ME.message);
    end
end
if ~isempty(inrAccum), info.mean_inr_db = mean(inrAccum); end
end

% =========================================================================
function theta = estimate_spatial_peak(y,p)
grid = (-p.az_span):1:p.az_span;
sc = zeros(size(grid));
for k = 1:numel(grid)
    a = kron(tx_steering(p,grid(k)),rx_steering(p,grid(k)));
    a = a/max(norm(a),eps);
    sc(k) = abs(a'*y)^2;
end
[~,ix] = max(sc);
theta = grid(ix);
end

function a = tx_steering(p,theta)
a = exp(1j*2*pi*(p.tx_x(:)/p.lambda)*sind(theta));
a = a/max(norm(a),eps);
end

function a = rx_steering(p,theta)
a = exp(1j*2*pi*(p.rx_x(:)/p.lambda)*sind(theta));
a = a/max(norm(a),eps);
end

function A = rx_steering_bank(p,thetaGrid)
A = exp(1j*2*pi*(p.rx_x(:)/p.lambda)*sind(thetaGrid(:).'));
A = A./max(sqrt(sum(abs(A).^2,1)),eps);
end

function i = nearest_index(x,v), [~,i] = min(abs(x-v)); end
function a = wrap_angle(a), a = mod(a+180,360)-180; end
