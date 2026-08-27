function [w,rms_gain,coherent_gain] = rd_window(N,name)
%RD_WINDOW  Single source of truth for analysis windows and their gains.
%
%   [w,rms_gain,coherent_gain] = RD_WINDOW(N,name) returns an N-point analysis
%   window normalised to unit mean, together with its RMS gain
%
%       g_rms = sqrt( mean( |w|^2 ) )
%
%   and its coherent gain mean(w) (unity by construction).
%
%   Every stage that forms a range or Doppler FFT obtains its window here and
%   divides the transform by sqrt(N)*g_rms. That combination is unitary in the
%   noise sense: a white input of variance sigma^2 produces bins of variance
%   sigma^2 regardless of N or of the window choice. Because all stages share
%   this convention, power maps produced by different modules are directly
%   comparable, which is what allows a CFAR reference estimated on one field to
%   threshold a cell under test taken from another.

if nargin < 2 || isempty(name), name = 'hann'; end
N = max(1,round(N));
if N <= 1
    w = ones(N,1); rms_gain = 1; coherent_gain = 1; return;
end
n = (0:N-1).';
switch lower(char(name))
    case {'hann','hanning'}
        w = 0.5 - 0.5*cos(2*pi*n/(N-1));
    case 'hamming'
        w = 0.54 - 0.46*cos(2*pi*n/(N-1));
    case {'blackman','blackmanharris'}
        w = 0.42 - 0.5*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1));
    case {'rect','rectangular','none','boxcar'}
        w = ones(N,1);
    case 'taylor'
        % Two-term Taylor-like taper with a low close-in sidelobe floor.
        w = 1 - 0.6*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1));
    otherwise
        error('rd_window:Unknown','Unsupported analysis window: %s',char(name));
end
m = mean(w);
if ~isfinite(m) || m <= 0, m = 1; end
w = w/m;
coherent_gain = mean(w);
rms_gain = sqrt(mean(abs(w).^2));
if ~isfinite(rms_gain) || rms_gain <= 0, rms_gain = 1; end
end
