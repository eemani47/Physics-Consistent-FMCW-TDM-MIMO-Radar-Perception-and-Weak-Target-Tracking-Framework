function [dechirped,tx_ref,rx_echo] = fmcw_complex_baseband_dechirp(t_fast,slope,fc,tau,amplitude)
%FMCW_COMPLEX_BASEBAND_DECHIRP Generate an FMCW echo and dechirp it.
% The RF carrier is represented analytically in equivalent complex baseband;
% this avoids impossible direct sampling of a 77 GHz carrier at ADC rates.
%
% TX reference:
%   s_tx(t) = exp(j*pi*S*t^2)
%
% Delayed received analytic signal:
%   s_rx(t) = A*exp(-j*2*pi*fc*tau)*exp(j*pi*S*(t-tau)^2)
%
% Dechirp/mixer:
%   s_if(t) = s_tx(t) * conj(s_rx(t)) / A
%            = exp(j*2*pi*fc*tau)
%              * exp(j*pi*S*(2*t*tau - tau^2))
%
% The resulting beat frequency is +S*tau.  For a moving target, tau is
% evaluated at each chirp's slow-time sample, so the carrier phase evolution
% across chirps naturally carries the Doppler information.

validateattributes(t_fast,{'numeric'},{'column','real','finite'},mfilename,'t_fast',1);
validateattributes(slope,{'numeric'},{'scalar','real','finite','positive'},mfilename,'slope',2);
validateattributes(fc,{'numeric'},{'scalar','real','finite','positive'},mfilename,'fc',3);
validateattributes(tau,{'numeric'},{'vector','real','finite','nonnegative'},mfilename,'tau',4);
validateattributes(amplitude,{'numeric'},{'scalar','real','finite','nonnegative'},mfilename,'amplitude',5);

% Equivalent complex-baseband chirps. No RF carrier is numerically sampled.
tx_ref = exp(1j*pi*slope*(t_fast.^2));
rx_echo = amplitude .* exp(-1j*2*pi*fc*tau) .* exp(1j*pi*slope*((t_fast-tau).^2));
dechirped = tx_ref .* conj(rx_echo);

dechirped = reshape(dechirped,[],1);
tx_ref = reshape(tx_ref,[],1);
rx_echo = reshape(rx_echo,[],1);

end
