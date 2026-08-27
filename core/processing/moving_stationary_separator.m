function out = moving_stationary_separator(rx_cube,p)
%MOVING_STATIONARY_SEPARATOR  Coherent stationary estimation and subtraction.
%
%   Implements the processing structure of Hyun, Jin and Lee (2017), adapted to
%   the physical TDM-MIMO simulator used here:
%
%     range compression -> coherent stationary estimate and subtraction ->
%     Doppler compression for the moving branch, with an independently
%     retained stationary range-angle profile for static reflectors.
%
%   The stationary component is estimated per transmit/receive branch, so the
%   deterministic TDM phase is never averaged across transmitters. For a bin
%   containing only zero-Doppler energy the slow-time mean is a consistent
%   estimate of that energy; a target at non-zero Doppler decorrelates in the
%   same average and survives the subtraction.
%
%       X_s(v) = (1/K) sum_{k in TX slot} R(r, k, rx)
%       X_m    = R - X_s
%
%   Both branches are transformed with the shared unitary, noise-preserving
%   normalisation used by RANGE_DOPPLER_PROCESSOR, so the moving-target power
%   map returned here is on the same absolute scale as the raw reference map
%   built by the range-Doppler front end. That equality is what makes a CFAR
%   threshold estimated on the reference field valid for a cell under test
%   taken from the moving field.
%
%   The stationary range-angle map is formed by digital beamforming across the
%   physical virtual aperture using the exact near-field TX/RX steering
%   convention of the simulator at v = 0.

validateattributes(rx_cube,{'numeric'},{'3d','nonempty'},mfilename,'rx_cube',1);
[Nr,Nd,Nrx] = size(rx_cube);
if Nr ~= p.Nr || Nd ~= p.Nd || Nrx ~= p.n_rx
    error('moving_stationary_separator:Size','RX cube must be Nr-by-Nd-by-n_rx.');
end

winName = get_default_field(p.range_processing,'window','hann');
[wr,wrms] = rd_window(Nr,winName);
[wd,wdms] = rd_window(Nd,winName);
wd = wd.';

% ---- range compression, identical normalisation to the RD front end -------
R = complex(zeros(Nr,Nd,Nrx));
for rx = 1:Nrx
    R(:,:,rx) = fft(rx_cube(:,:,rx).*wr,Nr,1)/(sqrt(Nr)*wrms);
end

useCoherentSubtraction = logical(get_default_field(p.paper,'coherent_subtraction',true));
useCoherentMean        = logical(get_default_field(p.paper,'moving_coherent_mean',true));

% ---- per-TX/RX coherent stationary estimate ------------------------------
Xs_v = complex(zeros(Nr,p.n_virt));
stationary_template = complex(zeros(Nr,Nd,Nrx));
for tx = 1:p.n_tx
chirps = tx:p.n_tx:Nd;
    if isempty(chirps), continue; end
    for rx = 1:Nrx
        vi = (tx-1)*Nrx + rx;
        if useCoherentMean
            xstat = mean(R(:,chirps,rx),2);
        else
            xstat = median(real(R(:,chirps,rx)),2) + 1j*median(imag(R(:,chirps,rx)),2);
        end
        Xs_v(:,vi) = xstat;
        stationary_template(:,chirps,rx) = repmat(xstat,1,numel(chirps));
    end
end

if useCoherentSubtraction
Xm = R - stationary_template;
else
Xm = R;
end

% ---- Doppler compression for both branches -------------------------------
Dmove = complex(zeros(Nr,Nd,Nrx));
Pmove = zeros(Nr,Nd);
Praw  = zeros(Nr,Nd);
for rx = 1:Nrx
    D    = fftshift(fft(Xm(:,:,rx).*wd,Nd,2)/(sqrt(Nd)*wdms),2);
    Draw = fftshift(fft(R (:,:,rx).*wd,Nd,2)/(sqrt(Nd)*wdms),2);
    Dmove(:,:,rx) = D;
    Pmove = Pmove + abs(D).^2;
    Praw  = Praw  + abs(Draw).^2;
end
Pmove(~isfinite(Pmove)) = 0;
Praw(~isfinite(Praw))   = 0;
if isfield(p,'valid_range_mask')
    mask = repmat(p.valid_range_mask(:),1,Nd);
    Pmove(~mask) = 0;
    Praw(~mask)  = 0;
end

% ---- stationary range-angle beamforming ----------------------------------
theta_axis = p.theta_axis;
Na = numel(theta_axis);
PstatRA = zeros(Nr,Na);
validBins = 1:Nr;
if isfield(p,'valid_range_mask'), validBins = find(p.valid_range_mask); end
for ii = 1:numel(validBins)
    r = validBins(ii);
    z = Xs_v(r,:).';
    if norm(z) <= eps, continue; end
    A = stationary_steering_bank(p,p.range_axis(r),theta_axis);   % n_virt x Na
    PstatRA(r,:) = abs(A'*z).^2 .';
end
PstatRA(~isfinite(PstatRA)) = 0;

stationary_range_power = sum(abs(Xs_v).^2,2);
stationary_range_power(~isfinite(stationary_range_power)) = 0;
if isfield(p,'valid_range_mask')
    stationary_range_power(~p.valid_range_mask) = 0;
end

out = struct();
out.range_fft                  = R;
out.stationary_virtual_profile = Xs_v;
out.stationary_template        = stationary_template;
out.moving_range_cube          = Xm;
out.moving_doppler_cube        = Dmove;
out.moving_rd_power            = Pmove;
out.raw_rd_power               = Praw;
out.stationary_range_power     = stationary_range_power;
out.stationary_range_angle_power = PstatRA;
out.theta_axis                 = theta_axis;
out.range_resolution_actual    = p.range_resolution_actual;
out.velocity_resolution_actual = p.velocity_resolution_actual;
out.window_name                = char(winName);
out.power_normalization        = 'unitary_fft_with_window_rms_compensation';
out.stationary_method          = 'per-TX/RX coherent slow-time mean with complex subtraction';
out.moving_method              = 'Doppler FFT after coherent stationary subtraction';
out.stationary_method_dbf      = 'physical TDM-MIMO near-field DBF at v = 0';
end

% =========================================================================
function A = stationary_steering_bank(p,R,thetaGrid)
%STATIONARY_STEERING_BANK  Unit-norm near-field steering vectors, v = 0.
% Uses the exact bistatic delay of the simulator so the estimator and the
% measurement share one convention: a(v) = exp(+j 2 pi f_c tau) / (d_tx d_rx).
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
nrm = sqrt(sum(abs(A).^2,1));
A = A./max(nrm,eps);
end
