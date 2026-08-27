function [theta_bf,P_bf,theta_music,P_music,info] = music_aoa_estimator(va_signal,p,varargin)
%MUSIC_AOA_ESTIMATOR  Virtual-aperture angle estimation with three spectra.
%
%   Forms the spatial covariance of the de-rotated virtual-aperture snapshots
%
%       R = (1/N_s) X X^H + delta I ,   delta = mu * trace(R)/L
%
%   optionally applies forward-backward averaging, splits the eigenstructure at
%   an MDL-selected model order and evaluates three spectra on a common angular
%   grid:
%
%     conventional beamformer   P_bf(th)    = | a^H z |^2 / ( a^H a )
%     MUSIC                     P_mu(th)    = 1 / ( a^H U_n U_n^H a )
%     Capon / MVDR              P_cap(th)   = 1 / ( a^H R^-1 a )
%
%   MUSIC supplies resolution, the beamformer supplies an unconditionally
%   stable reference, and Capon supplies an independent cross-check that is
%   sensitive to a different failure mode than MUSIC. Reporting all three lets
%   downstream logic detect the case where a high-resolution peak is an
%   artefact of a poorly conditioned covariance rather than a real source.
%
%   Model order is chosen by the minimum description length criterion
%
%       MDL(k) = -N (L-k) log( g_k / a_k ) + (1/2) k (2L-k) log N
%
%   with g_k and a_k the geometric and arithmetic means of the L-k smallest
%   eigenvalues. Steering vectors use the simulator's exact near-field bistatic
%   convention whenever a reference range is supplied, and fall back to the
%   plane-wave model otherwise.

if isempty(va_signal)
    error('music_aoa_estimator:Empty','Virtual aperture is empty.');
end
if nargin >= 3 && ~isempty(varargin{1}), reference_range = varargin{1}; else, reference_range = Inf; end

X = va_signal;
[L,Ns] = size(X);
if Ns < 2, X = [X X]; Ns = 2; end

% ---- spatial covariance --------------------------------------------------
R = (X*X')/Ns;
R = 0.5*(R + R');
loading = max(real(trace(R))/max(L,1),eps)*get_default_field(p.est,'music_cov_diagonal_loading',1e-3);
R = R + loading*eye(L);
if logical(get_default_field(p.est,'forward_backward',true)) && L > 1
    J = flip(eye(L));
    R = 0.5*(R + J*conj(R)*J);
end

if isfield(p,'virtual_x') && numel(p.virtual_x) == L
    x_virtual = p.virtual_x(:);
else
    x_virtual = (0:L-1).'*p.lambda/2;
end

% ---- angular grids -------------------------------------------------------
maxHyp  = max(64,round(get_default_field(p.detector,'max_angle_hypotheses',361)));
fftPad  = max(1,round(get_default_field(p.est,'fft_zero_pad',8)));
Nbf     = min(max(256,p.n_virt*fftPad),max(maxHyp,256)*8);
Nmu     = min(max(256,round(get_default_field(p.est,'music_grid',2048))),16384);
theta_bf    = linspace(-p.az_span,p.az_span,Nbf);
theta_music = linspace(-p.az_span,p.az_span,Nmu);

zhat = mean(X,2);

% ---- conventional beamformer --------------------------------------------
A_bf = steering_bank(theta_bf,reference_range,p,x_virtual);
P_bf = abs(A_bf'*zhat).^2 ./ max(real(sum(conj(A_bf).*A_bf,1)).',eps);
P_bf = reshape(P_bf,1,[]);
P_bf = 10*log10(max(P_bf/max(max(P_bf),realmin),1e-15));

% ---- eigenstructure and model order --------------------------------------
[V,E] = eig(R,'vector');
E = real(E); [E,ord] = sort(E,'descend'); V = V(:,ord);
modelOrder = lower(char(get_default_field(p.est,'model_order','mdl')));
kmax = max(1,min(get_default_field(p.est,'n_src_max',1),L-1));
if strcmp(modelOrder,'fixed')
n_src = kmax;
else
    n_src = mdl_model_order(E,Ns,L,kmax);
end
minSnapshots = max(2,round(get_default_field(p.est,'min_snapshots',8)));
snapshotSufficient = Ns >= minSnapshots;

Un = V(:,n_src+1:end);
if isempty(Un), Un = zeros(L,1); end
Pn = Un*Un';

% ---- MUSIC and Capon spectra --------------------------------------------
A_mu = steering_bank(theta_music,reference_range,p,x_virtual);
den_music = real(sum(conj(A_mu).*(Pn*A_mu),1));
P_music = 1./max(den_music,1e-15);
P_music = 10*log10(max(P_music/max(P_music),1e-15));

Ri = pinv_hermitian(R,1e-8);
den_capon = real(sum(conj(A_mu).*(Ri*A_mu),1));
P_capon = 1./max(den_capon,1e-15);
P_capon = 10*log10(max(P_capon/max(P_capon),1e-15));

% ---- peak extraction with parabolic refinement ---------------------------
[pk,ix] = max(P_music);
ix = max(1,min(numel(P_music),ix));
dth = theta_music(2)-theta_music(1); off = 0;
if ix > 1 && ix < numel(P_music)
    y1 = P_music(ix-1); y2 = P_music(ix); y3 = P_music(ix+1);
den = y1 - 2*y2 + y3;
    if abs(den) > 1e-12, off = max(-0.5,min(0.5,0.5*(y1-y3)/den)); end
end
theta_peak = theta_music(ix) + off*dth;

[pbf,ibf]  = max(P_bf);      theta_bf_peak = theta_bf(ibf);
[~,icp]    = max(P_capon);   theta_capon   = theta_music(icp);
medM = median(P_music,'omitnan');
medB = median(P_bf,'omitnan');

info = struct();
info.eigenvalues            = E;
info.normalized_eigenvalues = E/max(max(E),eps);
info.n_src                  = n_src;
info.model_order            = modelOrder;
info.condition_number       = max(E)/max(min(E),eps);
info.diagonal_loading       = loading;
info.n_snapshots            = Ns;
info.min_snapshots          = minSnapshots;
info.snapshot_sufficient    = snapshotSufficient;
info.music_peak_db              = pk;
info.music_peak_angle_deg       = theta_peak;
info.music_peak_prominence_db   = pk - medM;
info.beamformer_peak_angle_deg     = theta_bf_peak;
info.beamformer_peak_prominence_db = pbf - medB;
info.capon_peak_angle_deg       = theta_capon;
info.angle_difference_deg       = wrap_angle(theta_peak - theta_bf_peak);
info.music_capon_difference_deg = wrap_angle(theta_peak - theta_capon);
% Legacy aliases retained so existing consumers keep working.
info.fft_peak_angle_deg      = theta_bf_peak;
info.fft_peak_prominence_db  = info.beamformer_peak_prominence_db;

maxPeaks = max(1,round(get_default_field(p.est,'max_aoa_peaks',3)));
info.max_aoa_peaks = maxPeaks;
peakMask = false(size(P_music));
peakMask(2:end-1) = P_music(2:end-1) >= P_music(1:end-2) & P_music(2:end-1) >= P_music(3:end);
peakIdx = find(peakMask);
if ~isempty(peakIdx)
    [~,ordPk] = sort(P_music(peakIdx),'descend');
    peakIdx = peakIdx(ordPk(1:min(maxPeaks,numel(ordPk))));
else
peakIdx = ix;
end
info.music_peak_angles_deg = theta_music(peakIdx);
info.music_peak_values_db  = P_music(peakIdx);
info.capon_spectrum_db     = P_capon;
info.steering_positions    = x_virtual;
info.steering_model = ternary_local(isfinite(reference_range) && reference_range > 0, ...
    'exact near-field bistatic','plane-wave');
end

% =========================================================================
function A = steering_bank(thetaGrid,R,p,x_virtual)
%STEERING_BANK  Unit-norm steering vectors for a whole angular grid.
th = thetaGrid(:).';
if isfinite(R) && R > 0
    R  = max(R,1e-3);
    xt = R*sind(th); yt = R*cosd(th);
    A  = complex(zeros(numel(x_virtual),numel(th)));
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
else
    A = exp(1j*2*pi*(x_virtual/p.lambda)*sind(th));
end
A = A./max(sqrt(sum(abs(A).^2,1)),eps);
end

function k = mdl_model_order(E,N,L,kmax)
if L <= 1 || N <= 1 || isempty(E), k = 1; return; end
kmax = max(1,min([kmax,L-1,N-1]));
vals = max(real(E),eps);
mdl = Inf(1,kmax+1);
for kk = 0:kmax
    tail = vals(kk+1:end); m = numel(tail);
    g = exp(mean(log(tail))); a = mean(tail);
    mdl(kk+1) = -N*m*log(max(g/a,eps)) + 0.5*kk*(2*L-kk)*log(N);
end
[~,ix] = min(mdl);
k = max(1,min(ix-1,kmax));
end

function Ri = pinv_hermitian(A,floorFrac)
[V,D] = eig(0.5*(A+A'),'vector');
D = real(D);
D = max(D,max(D)*floorFrac);
Ri = V*diag(1./D)*V';
end

function a = wrap_angle(a), a = mod(a+180,360)-180; end
function y = ternary_local(c,a,b), if c, y = a; else, y = b; end, end
