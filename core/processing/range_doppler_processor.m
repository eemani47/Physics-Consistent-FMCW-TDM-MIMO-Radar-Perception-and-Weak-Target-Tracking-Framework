function [RD_map,range_profile,RD_power,range_fft,aux] = range_doppler_processor(Mix2D,p,opts)
%RANGE_DOPPLER_PROCESSOR  Calibrated complex-IQ range/Doppler transformation.
%
%   Operates on one receive chain of dechirped fast-time / slow-time samples
%   and returns range-Doppler power in physical units. Both transforms use the
%   shared unitary, noise-preserving normalisation
%
%       X(f) = FFT{ x[n] w[n] } / ( sqrt(N) * g_rms )
%
%   so that white input of variance sigma^2 yields bins of variance sigma^2.
%   RD_power therefore carries absolute power and is the quantity CFAR sees.
%   RD_map is a display-normalised copy in dB; nothing downstream thresholds it.
%
%   Outputs
%     RD_map        display-normalised range-Doppler map, dB relative to peak
%     range_profile slow-time-averaged range profile, dB relative to peak
%     RD_power      absolute range-Doppler power after the configured clutter filter
%     range_fft     complex range spectrum before Doppler processing
%     aux           intermediate products, window gains and power bookkeeping

if nargin < 3 || isempty(opts), opts = struct(); end
if ndims(Mix2D) ~= 2 || size(Mix2D,1) ~= p.Nr || size(Mix2D,2) ~= p.Nd
    error('range_doppler_processor:InputSize','Mix2D must be Nr-by-Nd complex samples.');
end
opts = with_default(opts,'keystone',p.range_processing.keystone);
opts = with_default(opts,'clutter_method',p.range_processing.clutter_method);
opts = with_default(opts,'window',get_default_field(p.range_processing,'window','hann'));

alpha_hp = get_default_field(p.range_processing,'highpass_alpha',0.92);

X_raw = Mix2D;
X     = clutter_filter(X_raw,opts.clutter_method,alpha_hp);

[wr,wrms] = rd_window(p.Nr,opts.window);
[wd,wdms] = rd_window(p.Nd,opts.window);
wd = wd.';

% Range compression.
R_raw = fft(X_raw.*(wr*wd),p.Nr,1)/(sqrt(p.Nr)*wrms);
R     = fft(X    .*(wr*wd),p.Nr,1)/(sqrt(p.Nr)*wrms);
range_fft = R;

% Range-cell migration compensation across the CPI.
if opts.keystone
    Rks_raw = keystone_motion_compensation(R_raw,p);
    Rks     = keystone_motion_compensation(R,p);
else
Rks_raw = R_raw;
Rks     = R;
end

% Doppler compression.
D_raw = fftshift(fft(Rks_raw,p.Nd,2)/(sqrt(p.Nd)*wdms),2);
D     = fftshift(fft(Rks    ,p.Nd,2)/(sqrt(p.Nd)*wdms),2);

RD_power_pre = abs(D_raw).^2;
RD_power     = abs(D).^2;
RD_power_pre(~isfinite(RD_power_pre)) = 0;
RD_power(~isfinite(RD_power))         = 0;

if isfield(p,'valid_range_mask')
    RD_power_pre(~p.valid_range_mask,:) = 0;
    RD_power(~p.valid_range_mask,:)     = 0;
end

scale = max(RD_power(:));
if isempty(scale) || ~isfinite(scale) || scale <= 0, scale = 1; end
RD_map = 10*log10(max(RD_power/scale,1e-15));

rp = mean(abs(Rks),2);
if isfield(p,'valid_range_mask'), rp(~p.valid_range_mask) = 0; end
rpm = max(rp);
if isempty(rpm) || ~isfinite(rpm) || rpm <= 0, rpm = 1; end
range_profile = 20*log10(max(rp/rpm,1e-15));

aux = struct();
aux.range_fft_keystone      = Rks;
aux.range_fft_keystone_raw  = Rks_raw;
aux.doppler_fft             = D;
aux.doppler_fft_raw         = D_raw;
aux.raw_range_fft           = R_raw;
aux.window_name             = char(opts.window);
aux.window_range            = wr;
aux.window_doppler          = wd;
aux.window_rms_gain         = wrms;
aux.window_doppler_rms_gain = wdms;
aux.window_coherent_gain_range   = mean(wr);
aux.window_coherent_gain_doppler = mean(wd);
aux.range_bin_spacing       = p.range_resolution_actual;
aux.velocity_bin_spacing    = p.velocity_resolution_actual;
aux.RD_power_pre_clutter    = RD_power_pre;
aux.RD_power_post_clutter   = RD_power;
aux.input_power             = mean(abs(X_raw(:)).^2);
aux.post_clutter_power      = mean(abs(X(:)).^2);
aux.power_normalization     = 'unitary_fft_with_window_rms_compensation';
end

function s = with_default(s,f,v)
if ~isfield(s,f) || isempty(s.(f)), s.(f) = v; end
end
