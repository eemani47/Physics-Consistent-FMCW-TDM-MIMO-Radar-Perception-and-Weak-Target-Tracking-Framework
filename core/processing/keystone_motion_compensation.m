function Xks = keystone_motion_compensation(X,p)
%KEYSTONE_MOTION_COMPENSATION  Keystone resampling for range-cell migration.
%
%   A target closing at radial velocity v moves v*T_cpi metres during the
%   coherent processing interval. Once that displacement becomes comparable to
%   a range cell the echo smears across range bins and the Doppler transform
%   loses coherent gain. In the range-frequency / slow-time domain the coupled
%   phase of a moving point scatterer is
%
%       phi(f_b,t) = 2 pi ( f_c + f_b ) ( 2 v / c ) t
%
%   The keystone transform removes the coupling without knowing v by rescaling
%   slow time independently in every range-frequency bin,
%
%       t' = t * f_c / ( f_c + f_b )
%
%   after which the residual phase depends on t' alone and migration becomes
%   velocity independent. The correction grows linearly across the beat band
%   and is largest at maximum range.
%
%   X is the Nr-by-Nd range spectrum produced by the range FFT. Resampling uses
%   a Lanczos-windowed sinc kernel evaluated on the circular slow-time grid,
%   which preserves the Doppler spectrum where linear interpolation of the real
%   and imaginary parts would broaden it.

[Nr,Nd] = size(X);
if Nd < 4
Xks = X;
return;
end

fb = zeros(Nr,1);
n  = min(Nr,numel(p.f_range_axis));
fb(1:n) = p.f_range_axis(1:n);
scale = p.fc ./ max(p.fc + fb,eps);         % Nr x 1, marginally below unity

idx = 0:Nd-1;
TQ   = scale*idx;                            % Nr x Nd resampled positions
base = floor(TQ);
frac = TQ - base;

A = 4;                                       % Lanczos half-width in samples
Xks = complex(zeros(Nr,Nd));
for j = -A+1:A
    d = frac - j;                            % distance from tap to sample
    w = lanczos_kernel(d,A);
    src = mod(base + j,Nd) + 1;              % circular slow-time support
    lin = (src-1)*Nr + repmat((1:Nr).',1,Nd);
    Xks = Xks + w.*X(lin);
end

% Renormalise so a constant slow-time sequence is reproduced exactly; this
% removes the small kernel truncation bias.
wsum = zeros(Nr,Nd);
for j = -A+1:A
    wsum = wsum + lanczos_kernel(frac - j,A);
end
Xks = Xks ./ max(abs(wsum),eps) .* sign_safe(wsum);
Xks(~isfinite(Xks)) = 0;
end

function w = lanczos_kernel(x,a)
w = sinc_local(x).*sinc_local(x/a);
w(abs(x) >= a) = 0;
end

function y = sinc_local(x)
y = ones(size(x));
nz = abs(x) > 1e-12;
y(nz) = sin(pi*x(nz))./(pi*x(nz));
end

function s = sign_safe(w)
s = ones(size(w));
s(w < 0) = -1;
end
