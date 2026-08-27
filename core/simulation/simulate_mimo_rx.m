function [rx_cube,truth,meta] = simulate_mimo_rx(p,scene,snr_db)
%SIMULATE_MIMO_RX  TDM-MIMO FMCW equivalent-complex-baseband receiver model.
%
%   Every active TX / target / RX path is formed as a delayed FMCW echo in
%   equivalent complex baseband and explicitly dechirped against the transmitted
%   reference before the ADC model is applied. The model retains propagation
%   delay, carrier phase, bistatic path loss, RCS, physical array geometry,
%   slow-time Doppler evolution and first-order intra-chirp range rate.
%
%   Signal model, for TX m and RX n observing a target at slant ranges
%   d_tx(t), d_rx(t):
%
%       tau(t)   = ( d_tx(t) + d_rx(t) ) / c
%       s_tx(t)  = exp( j pi S t^2 )
%       s_rx(t)  = A(t) exp( -j 2 pi f_c tau(t) ) exp( j pi S (t - tau(t))^2 )
%       s_if(t)  = s_tx(t) conj( s_rx(t) )
%                = A(t) exp( j 2 pi f_c tau(t) ) exp( j pi S ( 2 t tau(t) - tau(t)^2 ) )
%
%   so the beat frequency is S*tau plus the carrier-Doppler contribution
%   f_c d(tau)/dt. Received power follows the bistatic radar equation
%
%       P_r = P_t G^2 lambda^2 sigma / ( (4 pi)^3 d_tx^2 d_rx^2 )
%
%   which collapses to the monostatic R^-4 law for coincident phase centres.
%
%   Receiver noise is a band-limited thermal process: white complex Gaussian
%   samples are shaped to the IF passband so that the injected power equals
%   k T B F exactly, with B the one-sided complex IF bandwidth.

if nargin < 2 || isempty(scene)
targets = p.targets;
elseif isstruct(scene)
    if isfield(scene,'targets'), targets = scene.targets; else, targets = scene; end
else
targets = scene;
end
if nargin < 3 || isempty(snr_db), snr_db = Inf; end
if ~isnumeric(targets) || size(targets,2) ~= 4
    error('simulate_mimo_rx:TargetShape','scene must be an N-by-4 target table or a struct containing .targets.');
end

rng(p.random_seed,'twister');
Nr = p.Nr;
Nd = p.Nd;
Nrx = p.n_rx;
Ntx = p.n_tx;
t_fast = (0:Nr-1).'/p.fs_ADC;
t_slow = (0:Nd-1)*p.Tchirp;
rx_cube = complex(zeros(Nr,Nd,Nrx));
tx_reference = exp(1j*pi*p.slope*(t_fast.^2));

nT = size(targets,1);
truth = repmat(struct('id',0,'range0',0,'velocity',0,'rcs_dbsm',0,'rcs_m2',0, ...
    'angle_deg',0,'x',0,'y',0,'fd',0,'fb0',0,'input_snr_db',Inf, ...
    'power_relative_db',0),1,max(nT,1));
if nT == 0, truth = truth([]); end

% -------------------------------------------------------------------------
% Reference received power per target, averaged over the CPI and over every
% TX/RX pair. This is the anchor for SNR-controlled noise and is deliberately
% target-only: clutter, interference and other targets are excluded.
% -------------------------------------------------------------------------
Pmean = zeros(nT,1);
for ti = 1:nT
    R0 = targets(ti,1); v = targets(ti,2); theta = targets(ti,4); rcs_db = targets(ti,3);
Rm = R0 + v*t_slow;
    if any(Rm <= 0)
        error('simulate_mimo_rx:InvalidTrajectory','Target %d reaches non-positive range within the CPI.',ti);
    end
    sigma = 10^(rcs_db/10);
acc = 0;
cnt = 0;
    for tx = 1:Ntx
chirps = tx:Ntx:Nd;
        if isempty(chirps), continue; end
        Rsel = Rm(chirps); xt = Rsel*sind(theta); yt = Rsel*cosd(theta);
        for rx = 1:Nrx
            dtx = hypot(xt-p.tx_x(tx),yt);
            drx = hypot(xt-p.rx_x(rx),yt);
            pr  = p.P_tx_W*p.G_lin^2*p.lambda^2*sigma ./ ((4*pi)^3 .* max(dtx.^2 .* drx.^2,eps));
            acc = acc + sum(pr); cnt = cnt + numel(pr);
        end
    end
    Pmean(ti) = acc/max(cnt,1);
end
Pweak = max(min(Pmean),realmin);
if nT == 0, Pweak = realmin;
end

% Target fluctuation model. Swerling 0 is deterministic; Swerling 1 draws one
% complex scattering coefficient per coherent processing interval.
scatter = ones(max(nT,1),1);
swModel = char(p.target.swerling_model);
if strcmpi(swModel,'Swerling1')
    scatter = (randn(max(nT,1),1)+1j*randn(max(nT,1),1))/sqrt(2);
elseif ~strcmpi(swModel,'Swerling0')
    error('simulate_mimo_rx:SwerlingModel','Unsupported target fluctuation model: %s',swModel);
end

% -------------------------------------------------------------------------
% TDM-MIMO echo synthesis. One transmitter is active per chirp; the echo is
% formed for that transmitter and every receive chain, then dechirped.
% The inner loop is vectorised over fast time and over the chirps belonging
% to a given transmitter.
% -------------------------------------------------------------------------
for ti = 1:nT
    R0 = targets(ti,1); v = targets(ti,2); rcs_db = targets(ti,3); theta = targets(ti,4);
    sigma = 10^(rcs_db/10);
Rm = R0 + v*t_slow;
    ux = sind(theta);

    truth(ti).id        = ti;
    truth(ti).range0    = R0;
    truth(ti).velocity  = v;
    truth(ti).rcs_dbsm  = rcs_db;
    truth(ti).rcs_m2    = sigma;
    truth(ti).angle_deg = theta;
    truth(ti).x = R0*sind(theta);
    truth(ti).y = R0*cosd(theta);
    truth(ti).fd = 2*v/p.lambda;

tauAcc = 0;
tauDotAcc = 0;
nAcc = 0;
    for tx = 1:Ntx
chirps = tx:Ntx:Nd;
        if isempty(chirps), continue; end
        Rsel = Rm(chirps);
        xt = Rsel*sind(theta); yt = Rsel*cosd(theta);
        for rx = 1:Nrx
            d_tx = hypot(xt-p.tx_x(tx),yt);
            d_rx = hypot(xt-p.rx_x(rx),yt);
            tau0 = (d_tx+d_rx)/p.c;                                   % 1 x K
            dtx_rate = v*(Rsel - p.tx_x(tx)*ux)./max(d_tx,eps);
            drx_rate = v*(Rsel - p.rx_x(rx)*ux)./max(d_rx,eps);
            tau_dot  = (dtx_rate + drx_rate)/p.c;                      % 1 x K

            tauAcc = tauAcc + sum(tau0); tauDotAcc = tauDotAcc + sum(tau_dot);
            nAcc = nAcc + numel(tau0);

            % First-order intra-chirp delay: tau(t) = tau0 + tau_dot t.
            % Accurate to O(v^2) over one chirp and avoids RF-rate sampling.
            TAU = tau0 + t_fast*tau_dot;                               % Nr x K
            DTX = max(d_tx + t_fast*dtx_rate,eps);
            DRX = max(d_rx + t_fast*drx_rate,eps);
            PR  = p.P_tx_W*p.G_lin^2*p.lambda^2*sigma ./ ((4*pi)^3 .* (DTX.^2 .* DRX.^2));
            AMP = sqrt(PR).*scatter(ti);

            IF = AMP .* exp(1j*2*pi*p.fc*TAU) ...
                     .* exp(1j*pi*p.slope*(2*(t_fast.*TAU) - TAU.^2));
            rx_cube(:,chirps,rx) = rx_cube(:,chirps,rx) + IF;
        end
    end
    truth(ti).fb0 = p.slope*(tauAcc/max(nAcc,1)) + p.fc*(tauDotAcc/max(nAcc,1));
    truth(ti).input_snr_db = snr_db;
    truth(ti).power_relative_db = 10*log10(Pmean(ti)/Pweak);
end

% -------------------------------------------------------------------------
% Diffuse stationary clutter. Modelled as a zero-Doppler, range-shaped
% background process with slow-time amplitude modulation, drawn independently
% per receive chain (spatially diffuse rather than a resolvable point target).
% -------------------------------------------------------------------------
if p.clutter.enabled && nT > 0
clutter_power = p.clutter.power_fraction_of_weakest*Pweak;
    cslow = 1 + p.clutter.slow_variation_std*randn(1,Nd);
    range_shape = exp(-((1:Nr).'/Nr)*p.clutter.range_decay_power);
    for rx = 1:Nrx
        c = sqrt(max(clutter_power,eps))*range_shape.*(randn(Nr,1)+1j*randn(Nr,1))/sqrt(2);
        rx_cube(:,:,rx) = rx_cube(:,:,rx) + c*cslow;
    end
end

% -------------------------------------------------------------------------
% Mutual FMCW interference. An uncoordinated emitter sweeping at a different
% slope appears as a chirped ridge at the dechirped IF interface. The emitter
% has a physical direction of arrival, so the receive array observes it with a
% deterministic inter-channel phase progression.
% -------------------------------------------------------------------------
interference_signal = complex(zeros(Nr,Nd));
if p.interference.enabled && nT > 0
k_int   = p.interference.bandwidth_Hz/p.Tchirp;
    phase   = 2*pi*(p.interference.f0_Hz*t_fast + 0.5*k_int*t_fast.^2);
    amp     = p.interference.amplitude_vs_weakest*sqrt(Pweak);
    env     = 0.75 + 0.25*sin(2*pi*t_fast/p.Tchirp).^2;
    int_sig = amp*env.*exp(1j*phase);
    interference_signal = int_sig*ones(1,Nd);
    thetaI  = get_field_local(p.interference,'angle_deg',0);
    for rx = 1:Nrx
        spatial = exp(1j*2*pi*(p.rx_x(rx)/p.lambda)*sind(thetaI));
        rx_cube(:,:,rx) = rx_cube(:,:,rx) + spatial*interference_signal;
    end
end

% -------------------------------------------------------------------------
% Receiver noise. The mode selects how the per-chain noise power is set; the
% shaping is common. Band-limited shaping confines the process to the IF
% passband so that the declared kTBF power and the sampled process agree.
% -------------------------------------------------------------------------
noiseMode = get_field_local(p.noise,'mode','physical_rx_chain');
noise_power_used = reshape(p.rx_noise_power_W,1,1,Nrx);
snr_used = Inf;
switch noiseMode
    case 'none'
        noise_power_used = zeros(1,1,Nrx);
    case 'fixed_awgn'
        noise_power_used = repmat(max(p.noise.fixed_power_W,realmin),1,1,Nrx);
    case 'snr_awgn'
snr_used = p.noise.snr_db;
        if isfinite(snr_db), snr_used = snr_db; end
        noise_power_used = repmat(Pweak ./ 10.^(snr_used/10),1,1,Nrx);
    case 'physical_rx_chain'
        if isfinite(snr_db)
            noise_power_used = repmat(Pweak ./ 10.^(snr_db/10),1,1,Nrx);
snr_used = snr_db;
        end
    otherwise
        error('simulate_mimo_rx:NoiseModel','Unsupported noise mode: %s',noiseMode);
end

bandLimited = logical(get_field_local(p.noise,'band_limited',true));
passMask = if_passband_mask(p);
independentChains = logical(get_field_local(p.noise,'independent_rx_chains',true));
if p.noise.enabled && ~strcmp(noiseMode,'none')
    if independentChains
        % Physical case: each receive chain has its own amplifier and mixer,
        % so the thermal processes are mutually independent and the array
        % sees spatially white noise.
        for rx = 1:Nrx
            sigma2 = max(noise_power_used(1,1,rx),realmin);
            rx_cube(:,:,rx) = rx_cube(:,:,rx) + shaped_noise(Nr,Nd,sigma2,passMask,bandLimited);
        end
    else
        % Fully correlated case: one realisation shared by every chain, which
        % makes the noise look like a broadside source. Useful for probing how
        % much the spatial detectors rely on noise whiteness.
        common = shaped_noise(Nr,Nd,1,passMask,bandLimited);
        for rx = 1:Nrx
            sigma2 = max(noise_power_used(1,1,rx),realmin);
            rx_cube(:,:,rx) = rx_cube(:,:,rx) + sqrt(sigma2)*common;
        end
    end
noiseInjected = true;
else
    noise_power_used = zeros(1,1,Nrx);
noiseInjected = false;
end

% -------------------------------------------------------------------------
% Metadata.
% -------------------------------------------------------------------------
noise_power_scalar = mean(noise_power_used(:));
meta = struct();
meta.signal_chain   = 'equivalent_complex_baseband_TX_chirp -> propagation_delay -> RX_echo -> dechirp/mixer -> complex_ADC';
meta.waveform_model = p.waveform.model;
meta.dechirp_enabled= p.waveform.dechirp_enabled;
meta.dechirp_sign   = p.waveform.dechirp_sign;
meta.beat_frequency_sign = p.waveform.beat_frequency_sign;
meta.carrier_sampled_directly = p.waveform.carrier_sampled_directly;
meta.dechirp_model  = 'analytic equivalent complex baseband; the RF carrier is never sampled';
meta.tx_reference_chirp = tx_reference;
meta.t_fast = t_fast;
meta.t_slow = t_slow;
meta.noise_power_W = noise_power_scalar;
meta.noise_power_per_rx_W = reshape(noise_power_used,1,[]);
meta.noise_enabled = noiseInjected;
meta.noise_band_limited = bandLimited;
meta.noise_independent_rx_chains = independentChains;
meta.noise_passband_bins = nnz(passMask);
meta.noise_model = sprintf('%s | independent complex circular AWGN per RX chain',noiseMode);
meta.noise_injection_point = p.noise.injection_point;
meta.noise_figure_per_rx_dB = p.rx_noise_figure_dB;
meta.noise_temperature_per_rx_K = p.rx_noise_temperature_K;
meta.snr_db_request = snr_used;
meta.snr_definition = 'weakest-target target-only mean received power / injected AWGN power';
meta.target_powers_W = Pmean;
meta.reference_target_power_W = Pweak;
meta.target_snr_db = 10*log10(Pmean./max(noise_power_scalar,realmin));
meta.motion_model = 'constant radial velocity with first-order intra-chirp bistatic delay rate';
meta.interference_injected = any(abs(interference_signal(:)) > 0);
meta.interference_signal = interference_signal;
meta.interference_angle_deg = get_field_local(p.interference,'angle_deg',0);
meta.array_geometry = p.array_geometry;
meta.swerling_model = swModel;
meta.rcs_units = get_field_local(p.target,'rcs_units','m^2');
end

% =========================================================================
function mask = if_passband_mask(p)
%IF_PASSBAND_MASK  Logical mask of FFT bins inside the physical IF passband.
% The dechirped support spans [-f_doppler_max, +f_beat_max]; an anti-alias
% filter ahead of the ADC passes exactly that band.
f = (0:p.Nr-1)*(p.fs_ADC/p.Nr);
f(f > p.fs_ADC/2) = f(f > p.fs_ADC/2) - p.fs_ADC;   % signed bin frequencies
mask = (f >= -p.f_doppler_max) & (f <= p.f_beat_max);
mask = reshape(mask,[],1);
if ~any(mask), mask(1) = true; end
end

function n = shaped_noise(Nr,Nd,sigma2,passMask,bandLimited)
%SHAPED_NOISE  Complex circular Gaussian noise of total power sigma2 per sample.
if ~bandLimited
    n = sqrt(sigma2/2).*(randn(Nr,Nd)+1j*randn(Nr,Nd));
return;
end
K = nnz(passMask);
% X[k] iid CN(0,s) on K passband bins gives E|ifft(X)|^2 = K s / Nr^2,
% so s = sigma2 Nr^2 / K delivers exactly sigma2 per time sample.
s = sigma2*Nr^2/K;
X = zeros(Nr,Nd);
X(passMask,:) = sqrt(s/2).*(randn(K,Nd)+1j*randn(K,Nd));
n = ifft(X,Nr,1);
end

function v = get_field_local(s,f,d)
if isstruct(s) && isscalar(s) && isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
