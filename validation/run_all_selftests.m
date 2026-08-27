function [ok,results] = run_all_selftests(varargin)
%RUN_ALL_SELFTESTS  Execute the full validation suite.
%
%   [ok,results] = RUN_ALL_SELFTESTS() runs every self-test and returns a
%   single pass flag together with the individual reports.
%
%   Name-value options
%     'verbose'   print each report as it completes (default true)
%     'include'   cell array of suite names to run
%     'exclude'   cell array of suite names to skip
%     'cfar_pfa'  also run the long false-alarm study (default false; it is
%                 excluded by default because a meaningful estimate of a
%                 1e-5 false-alarm rate needs far more samples than a
%                 development run should spend)
%
%   Every suite here is behavioural or structural. None of them asserts on the
%   presence of a string in a source file, because such a check passes while
%   the code beneath it is wrong, and fails when the code is merely reworded.

opt = struct('verbose',true,'include',{{}},'exclude',{{}},'cfar_pfa',false);
for i = 1:2:numel(varargin)
    key = lower(char(varargin{i}));
    if isfield(opt,key), opt.(key) = varargin{i+1}; end
end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));

suites = { ...
    'radar_integrity',   @() radar_integrity_selftest(false); ...
    'detection_chain',   @() detection_chain_selftest(false); ...
    'estimation',        @() estimation_selftest(false); ...
    'track_before_detect',@() tbd_selftest(false); ...
    'evaluation',        @() evaluation_selftest(false); ...
    'pipeline',          @() pipeline_selftest(false); ...
    'contracts',         @() contract_selftest(false); ...
    'experiments',       @() experiments_selftest(false); ...
    'integrity',         @() integrity_selftest(false)};

if opt.cfar_pfa
    suites(end+1,:) = {'cfar_pfa',@() cfar_pfa_report()};
end
if ~isempty(opt.include)
    suites = suites(ismember(suites(:,1),opt.include),:);
end
if ~isempty(opt.exclude)
    suites = suites(~ismember(suites(:,1),opt.exclude),:);
end

results = repmat(struct('name','','ok',false,'checks',repmat(struct('name','','pass',false,'detail',''),0,1),'error','','seconds',0),0,1);
if opt.verbose
    fprintf('\n================ VALIDATION SUITE ================\n');
end
for s = 1:size(suites,1)
    name = suites{s,1};
    rec = struct('name',name,'ok',false,'checks',repmat(struct('name','','pass',false,'detail',''),0,1),'error','','seconds',0);
t0 = tic;
    try
        r = suites{s,2}();
rec.ok = r.ok;
rec.checks = r.checks;
    catch ME
rec.ok = false;
        rec.error = sprintf('%s (%s)',ME.message,ME.identifier);
    end
    rec.seconds = toc(t0);
    results(end+1) = rec;
    if opt.verbose, print_suite(rec); end
end

ok = all([results.ok]);
if opt.verbose
nPass = 0;
nTotal = 0;
    for i = 1:numel(results)
        c = results(i).checks;
        if isempty(c) || ~isfield(c,'pass'), continue; end
        nPass = nPass + nnz([c.pass]);
        nTotal = nTotal + numel(c);
    end
    fprintf('\n-------------------------------------------------\n');
    fprintf('SUITES %d/%d passed   CHECKS %d/%d passed   %.1f s\n', ...
        nnz([results.ok]),numel(results),nPass,nTotal,sum([results.seconds]));
    fprintf('OVERALL: %s\n',ternary(ok,'PASS','FAIL'));
    fprintf('=================================================\n\n');
end
end

function r = cfar_pfa_report()
out = validate_cfar_pfa();
r = struct('name','cfar_pfa','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
for i = 1:numel(out.modes)
    m = out.modes(i);
pass = m.pfa_ci_low <= m.pfa_target && m.pfa_target <= m.pfa_ci_high;
    r.checks(end+1) = struct('name',sprintf('%s empirical Pfa brackets the target',m.mode), ...
        'pass',pass,'detail',sprintf('%.3e in [%.3e, %.3e]',m.pfa_empirical,m.pfa_ci_low,m.pfa_ci_high));
r.ok = r.ok && pass;
end
end

function print_suite(rec)
fprintf('\n[%s] %s  (%.2f s)\n',upper(rec.name),ternary(rec.ok,'PASS','FAIL'),rec.seconds);
if ~isempty(rec.error)
    fprintf('   ERROR %s\n',rec.error);
return;
end
for i = 1:numel(rec.checks)
    c = rec.checks(i);
    if isempty(c.detail)
        fprintf('   %-4s %s\n',ternary(c.pass,'ok','FAIL'),c.name);
    else
        fprintf('   %-4s %s  (%s)\n',ternary(c.pass,'ok','FAIL'),c.name,c.detail);
    end
end
end

function y = ternary(c,a,b), if c, y = a; else, y = b; end, end
