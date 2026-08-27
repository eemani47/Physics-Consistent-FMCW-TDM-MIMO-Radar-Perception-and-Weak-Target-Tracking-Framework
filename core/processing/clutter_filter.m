function Mix2D_clean = clutter_filter(Mix2D, method, varargin)
%CLUTTER_REMOVAL  Stationary-clutter suppression with size-safe methods.
%   'off'       no suppression
%   'dc_cancel' slow-time mean removal
%   'mti_2tap'  first difference with edge replication
%   'mti_3tap'  second difference with edge replication
%   'highpass'  mean + first-order IIR high-pass in slow time

if nargin < 2 || isempty(method), method='off'; end
if nargin >= 3 && ~isempty(varargin{1}), alpha=double(varargin{1}); else, alpha=0.92; end
alpha=min(max(alpha,0),0.9999);
method=lower(string(method));

switch method
    case "off"
Mix2D_clean = Mix2D;
    case "dc_cancel"
        Mix2D_clean = Mix2D - mean(Mix2D,2);
    case "mti_2tap"
        y = diff(Mix2D,1,2);
        Mix2D_clean = [y(:,1),y];
    case "mti_3tap"
        y = diff(Mix2D,2,2);
        Mix2D_clean = [y(:,1),y(:,1),y];
    case "highpass"
        Mix2D_clean = zeros(size(Mix2D),'like',Mix2D);
        Mix2D_clean(:,1)=Mix2D(:,1);
        for k=2:size(Mix2D,2)
            Mix2D_clean(:,k)=alpha*(Mix2D_clean(:,k-1)+Mix2D(:,k)-Mix2D(:,k-1));
        end
    otherwise
        error('clutter_filter:unknownMethod','Unknown clutter method: %s',method);
end
end
