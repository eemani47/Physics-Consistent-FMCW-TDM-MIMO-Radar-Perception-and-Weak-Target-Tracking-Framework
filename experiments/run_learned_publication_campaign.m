function C = run_learned_publication_campaign(varargin)
%RUN_LEARNED_PUBLICATION_CAMPAIGN  Train, validate, then test on held-out scenes.
%
%   Runs the complete lifecycle in one command:
%
%     1. feedback learning on the train split
%     2. independent validation on the validation split, inside the learner
%     3. a held-out test campaign on the test split with the learned parameters
%
%   The three splits occupy disjoint seed spaces, so the reported test result
%   is measured on scenes the tuning process has never seen. Reporting a
%   tuned parameter set on the scenes it was tuned against measures the
%   fitting procedure, not the radar.
%
%   Profiles
%     'quick'        1 iteration set, small campaign, for verifying the flow
%     'standard'     a usable tuning run with a moderate test campaign
%     'publication'  a full tuning run and a large held-out campaign
%
%   Example
%     C = run_learned_publication_campaign('profile','standard');

root = radar_experiment_common('root');
addpath(genpath(root));

opts = struct('profile','standard','mode','zero_false', ...
    'output_dir',fullfile(pwd,'docs','results'),'verbose',true);
for k = 1:2:numel(varargin)
    key = lower(char(varargin{k}));
    if ~isfield(opts,key)
        error('run_learned_publication_campaign:Args','Unknown option "%s".',key);
    end
    opts.(key) = varargin{k+1};
end

switch lower(char(opts.profile))
    case 'quick'
L_trials = 1;
L_frames = 6;
L_iter = 3;
L_snr = 6;
T_trials = 2;
T_frames = 6;
T_snr = -6:6:18;
    case 'standard'
        L_trials = 3; L_frames = 10; L_iter = 8;  L_snr = [0 6 12 18];
T_trials = 20;
T_frames = 12;
T_snr = -10:4:20;
    case 'publication'
        L_trials = 6; L_frames = 12; L_iter = 14; L_snr = [-4 0 6 12 18];
T_trials = 200;
T_frames = 16;
T_snr = -10:2:20;
    otherwise
        error('run_learned_publication_campaign:Profile', ...
            'profile must be quick, standard or publication.');
end

if opts.verbose
    fprintf('\n############################################################\n');
    fprintf(' LEARNED CAMPAIGN | profile=%s | mode=%s\n',opts.profile,opts.mode);
    fprintf(' 1. learn on TRAIN   2. validate on VALIDATION   3. report on TEST\n');
    fprintf('############################################################\n');
end

L = run_feedback_parameter_learning('mode',opts.mode,'trials',L_trials, ...
    'frames',L_frames,'iterations',L_iter,'snr_values',L_snr, ...
    'validation',true,'validation_trials',max(2,L_trials), ...
    'persist',true,'verbose',opts.verbose, ...
    'output_dir',fullfile(opts.output_dir,'feedback_learning'));

if opts.verbose
    fprintf('\n[CAMPAIGN] Held-out test campaign with the learned parameters.\n');
end
T = run_radar_publication_monte_carlo('split','test','snr_values',T_snr, ...
    'trials',T_trials,'frames',T_frames,'config',L.best_tuned, ...
    'output_dir',opts.output_dir,'verbose',opts.verbose);

C = struct('learning',L,'test',T,'options',opts, ...
    'learned_default_path',L.learned_default_path);
save(fullfile(opts.output_dir,'learned_campaign.mat'),'C','-v7.3');
plot_convergence(L,opts.output_dir);

if opts.verbose
    fprintf('\n############################################################\n');
    fprintf(' TRAIN  Pd=%.4f  false/frame=%.4f\n', ...
        L.best_metrics.pd,L.best_metrics.false_per_frame);
    fprintf(' TEST   Pd=%.4f  false/frame=%.4f   (pooled over %d conditions)\n', ...
        mean([T.summary.pd],'omitnan'),mean([T.summary.false_per_frame],'omitnan'), ...
        numel(T.summary));
    fprintf(' Learned defaults persisted to %s\n',L.learned_default_path);
    fprintf('############################################################\n');
end
end

function plot_convergence(L,outdir)
%PLOT_CONVERGENCE  Detection probability and false objects across iterations.
try
    it = [L.trace.iteration];
    pd = arrayfun(@(t) t.metrics.pd,L.trace);
    fp = arrayfun(@(t) t.metrics.false_per_frame,L.trace);
    f = figure('Visible','off','Color','w','Position',[100 100 820 420]);
    yyaxis left;  plot(it,pd,'-o','LineWidth',1.8); ylabel('detection probability');
    yyaxis right; plot(it,fp,'-s','LineWidth',1.8); ylabel('false objects / frame');
    xlabel('learning iteration'); grid on;
    title('Closed-loop convergence');
    legend({'P_d','false objects / frame'},'Location','best');
    if ~exist(outdir,'dir'), mkdir(outdir); end
    saveas(f,fullfile(outdir,'learning_convergence.png'));
    close(f);
catch
end
end
