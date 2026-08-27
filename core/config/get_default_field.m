function v = get_default_field(s, f, d)
%GETFIELD_DEFAULT Safely return struct field or default value.
% This helper is intentionally a standalone function so it is available to
% all pipeline modules, not just to a caller's local-function scope.

if nargin < 3
    error('get_default_field:NotEnoughInputs', ...
        'Usage: get_default_field(struct, field, defaultValue).');
end

if isstruct(s) && isscalar(s) && isfield(s, f)
    val = s.(f);
    if ~isempty(val)
v = val;
return;
    end
end

v = d;
end
