function varargout = tdm_mimo_processing(mode,varargin)
%TDM_MIMO_PROCESSING  Dispatcher for the TDM-MIMO subsystem.
%
%   Modes
%     'resolve'   -> TDM_VELOCITY_RESOLVER  (Doppler-ambiguity resolution)
%     'aperture'  -> TDM_VIRTUAL_APERTURE   (virtual-aperture formation)
%
%   This function holds no algorithm of its own. It exists so that callers can
%   address the subsystem through one stable entry point while the canonical
%   implementations live in exactly one file each.

if nargin < 1, error('tdm_mimo_processing:Mode','A mode string is required.'); end
switch lower(char(mode))
    case 'resolve'
        [varargout{1:max(nargout,1)}] = tdm_velocity_resolver(varargin{:});
    case 'aperture'
        [varargout{1:max(nargout,1)}] = tdm_virtual_aperture(varargin{:});
    otherwise
        error('tdm_mimo_processing:Mode','Unknown mode "%s". Use resolve or aperture.',char(mode));
end
end
