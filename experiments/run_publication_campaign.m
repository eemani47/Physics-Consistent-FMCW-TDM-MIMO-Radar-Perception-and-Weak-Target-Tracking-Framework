function S = run_publication_campaign(varargin)
%RUN_PUBLICATION_CAMPAIGN  One-command held-out Monte Carlo campaign.
%
%   Runs the test split against the frozen baseline parameters, or against the
%   learned parameters if feedback learning has been run and persisted.
%
%   Profiles
%     'quick'        2 trials x 4 frames over 5 SNR points   ~40 frames
%     'standard'    50 trials x 12 frames over 16 SNR points ~9,600 frames
%     'publication' 500 trials x 16 frames over 16 SNR points ~128,000 frames
%
%   Time the quick profile first and extrapolate. Every trial re-runs the
%   complete physical simulator and the entire downstream chain, so cost
%   scales linearly with total frames and a publication run is a multi-day
%   commitment. There is no checkpointing; split a long campaign across SNR
%   points into separate output directories if the machine may be interrupted.
%
%   Example
%     S = run_publication_campaign('profile','standard');

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('profile','standard','output_dir',fullfile(pwd,'docs','results'), ...
    'use_learned',true,'verbose',true,'config',struct());
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(opts,key)
        error('run_publication_campaign:Args','Unknown option "%s".',key);
    end
    opts.(key) = varargin{k+1};
end

switch lower(char(opts.profile))
    case 'quick',       trials = 2;   frames = 4;  snr = -6:6:18;
    case 'standard',    trials = 50;  frames = 12; snr = -10:2:20;
    case 'publication', trials = 500; frames = 16; snr = -10:2:20;
    otherwise
        error('run_publication_campaign:Profile', ...
            'profile must be quick, standard or publication.');
end

cfg = opts.config;
learnedNote = 'frozen baseline parameters';
if opts.use_learned
    lp = fullfile(root,'core','config','learned_defaults.mat');
    if exist(lp,'file')
        L = load(lp,'tuned');
        if isfield(L,'tuned') && isstruct(L.tuned)
            cfg = radar_experiment_common('merge',L.tuned,cfg);
            learnedNote = 'learned parameters from core/config/learned_defaults.mat';
        end
    end
end

totalFrames = trials*frames*numel(snr);
if opts.verbose
    fprintf('\n============================================================\n');
    fprintf(' PUBLICATION CAMPAIGN\n');
    fprintf(' profile=%s | %d trials x %d frames x %d SNR points = %d frames\n', ...
        opts.profile,trials,frames,numel(snr),totalFrames);
    fprintf(' parameters: %s\n',learnedNote);
    fprintf('============================================================\n');
end

S = run_radar_publication_monte_carlo('split','test','snr_values',snr, ...
    'trials',trials,'frames',frames,'config',cfg, ...
    'output_dir',opts.output_dir,'verbose',opts.verbose);
S.profile = opts.profile;
S.parameter_source = learnedNote;
save(fullfile(S.output_dir,'publication_campaign.mat'),'S','-v7.3');
if opts.verbose
    fprintf('\nCampaign complete. Results: %s\n',S.output_dir);
end
end
