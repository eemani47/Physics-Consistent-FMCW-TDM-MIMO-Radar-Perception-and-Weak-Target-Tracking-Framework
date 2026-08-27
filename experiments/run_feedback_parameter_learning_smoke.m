function L = run_feedback_parameter_learning_smoke(varargin)
%RUN_FEEDBACK_PARAMETER_LEARNING_SMOKE  Fast end-to-end check of the learner.
%
%   One scene, one SNR point, two iterations, no validation campaign. This
%   exercises the cache, the controllers, the lexicographic acceptance test
%   and the persistence path in a few minutes, so a failure in any of them
%   surfaces before a long run is started.
%
%   Example
%     L = run_feedback_parameter_learning_smoke('frames',6);

opts = struct('frames',6,'verbose',true,'persist',true);
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(opts,key)
        error('run_feedback_parameter_learning_smoke:Args','Unknown option "%s".',key);
    end
    opts.(key) = varargin{k+1};
end

if opts.verbose
    fprintf('\n[SMOKE] 1 scene at 0 dB | 2 iterations | validation off\n');
end

L = run_feedback_parameter_learning('trials',1,'frames',opts.frames, ...
    'snr_values',0,'iterations',2,'validation',false, ...
    'persist',opts.persist,'verbose',opts.verbose);

if opts.persist
    if ~exist(L.learned_default_path,'file')
        error('run_feedback_parameter_learning_smoke:Persist', ...
            'Learned-default file was not written: %s',L.learned_default_path);
    end
    S = load(L.learned_default_path,'tuned');
    if ~isfield(S,'tuned') || ~isstruct(S.tuned)
        error('run_feedback_parameter_learning_smoke:Persist', ...
            'Persistent file is missing the "tuned" variable: %s',L.learned_default_path);
    end
    if opts.verbose
        fprintf('\n[SMOKE] PASS. Learned defaults at %s\n',L.learned_default_path);
        fprintf('[SMOKE] Restart MATLAB and launch the interface; the tuning tabs will show these values.\n');
    end
end
end
