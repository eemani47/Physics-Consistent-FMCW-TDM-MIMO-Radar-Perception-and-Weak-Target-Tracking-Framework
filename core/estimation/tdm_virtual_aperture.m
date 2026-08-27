function [va_signal,r_bin,info] = tdm_virtual_aperture(rx_cube,p,r_bin,velocity_mps,normalize)
%TDM_VIRTUAL_APERTURE  Physical TDM-MIMO virtual aperture with Doppler de-rotation.
%
%   In a time-division MIMO schedule the transmitters fire in turn, so the
%   virtual element formed by transmitter m and receiver n is sampled at a
%   different instant from the element formed by transmitter m' and the same
%   receiver. A target with radial velocity v therefore imprints a
%   transmitter-dependent phase
%
%       phi_m = 2 pi f_d t_m ,      f_d = 2 v / lambda
%
%   on top of the spatial phase the aperture is meant to measure. Left
%   uncorrected, that term rotates the array manifold and biases every angle
%   estimate. This function de-rotates each chirp by its own slow-time instant
%   for the supplied velocity hypothesis, then stacks the transmit blocks into
%   the physical virtual aperture.
%
%   The range transform uses the shared unitary, noise-preserving
%   normalisation, so the returned snapshots are on the same absolute power
%   scale as the receiver noise model. Detectors that compare a statistic
%   against k T B F therefore remain calibrated.
%
%   Outputs
%     va_signal  n_virt-by-ceil(Nd/n_tx) complex snapshot matrix
%     r_bin      clamped range bin actually used
%     info       snapshot bookkeeping and the applied Doppler correction

if nargin < 5 || isempty(normalize), normalize = false; end
[Nr,Nd,Nrx] = size(rx_cube);
if Nr ~= p.Nr || Nd ~= p.Nd || Nrx ~= p.n_rx
    error('tdm_virtual_aperture:CubeSize','rx_cube must be Nr-by-Nd-by-n_rx.');
end
validateattributes(r_bin,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(velocity_mps,{'numeric'},{'scalar','real','finite'});

r_bin = max(1,min(Nr,round(r_bin)));
[wr,wrms] = rd_window(Nr,get_default_field(p.range_processing,'window','hann'));
R = fft(rx_cube.*wr,p.Nr,1)/(sqrt(p.Nr)*wrms);

nslow = ceil(Nd/p.n_tx);
va_signal = complex(zeros(p.n_virt,nslow));
fd = 2*velocity_mps/p.lambda;
for tx = 1:p.n_tx
chirps = tx:p.n_tx:Nd;
    nCh = numel(chirps);
if nCh == 0, continue;
end
    phase_comp = exp(-1j*2*pi*fd*((chirps-1)*p.Tchirp));
    for rx = 1:p.n_rx
        vi = (tx-1)*p.n_rx + rx;
        s = reshape(R(r_bin,chirps,rx),1,[]);
        va_signal(vi,1:nCh) = s.*phase_comp;
    end
end

scaleApplied = 1;
if normalize
    scaleApplied = sqrt(max(mean(abs(va_signal(:)).^2),eps));
va_signal = va_signal/scaleApplied;
end

if nargout > 2
    info = struct('r_bin',r_bin,'range_m',p.range_axis(r_bin), ...
        'velocity_hypothesis_mps',velocity_mps,'doppler_hz',fd, ...
        'snapshots',nslow,'normalized',normalize,'normalization_scale',scaleApplied, ...
        'power_normalization','unitary_fft_with_window_rms_compensation');
end
end
