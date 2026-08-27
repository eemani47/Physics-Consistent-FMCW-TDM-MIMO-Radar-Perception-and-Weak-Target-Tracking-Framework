function report = integrity_selftest(verbose)
%INTEGRITY_SELFTEST  The radar is not allowed to cheat.
%
%   A simulation can produce excellent numbers dishonestly: by letting truth
%   reach the detector, by carrying a previous frame's answer forward as this
%   frame's, or by capping counts so the metric cannot express a failure.
%   These tests assert the absence of each, structurally and behaviourally,
%   so a later change cannot reintroduce one quietly.

if nargin < 1, verbose = true; end
report = struct('name','integrity','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
root = fileparts(fileparts(mfilename('fullpath')));

% --- 1. truth never reaches a formation stage ---------------------------
dirs = {'core/detection','core/estimation','core/tbd','core/tracking', ...
        'core/processing','core/interference'};
offenders = {};
for d = 1:numel(dirs)
    files = dir(fullfile(root,dirs{d},'*.m'));
    for k = 1:numel(files)
        txt = strip_comments(fileread(fullfile(files(k).folder,files(k).name)));
        if ~isempty(regexp(txt,'\<truth\w*\>','once'))
            offenders{end+1} = files(k).name;
        end
    end
end
report = add(report,'no detection, estimation, weak-target or tracking file references truth', ...
    isempty(offenders),describe(offenders));

trk = fileread(fullfile(root,'core','tracking','radar_object_tracker.m'));
report = add(report,'the tracker accepts no truth argument', ...
    contains(trk,'radar_object_tracker(frame_data,~,p,opts)'), ...
    'the second positional argument is explicitly discarded');

rt = strip_comments(fileread(fullfile(root,'run_radar_realtime.m')));
fd = regexp(rt,'frameData\s*=\s*struct\([^;]*\)','match');
report = add(report,'the live frame handed to the tracker carries measurements only', ...
    ~any(cellfun(@(s) contains(s,'truth'),fd)));

% --- 2. no frame borrows a previous frame's answer ----------------------
report = add(report,'the point-cloud display never falls back to an earlier frame', ...
    ~contains(trk,'last_nonempty'),'the final frame is reported as-is, empty if empty');

p = radar_configuration(struct());
% The Doppler length must exceed the CFAR window span or the detector rejects
% the map before the behaviour under test is reached.
q = p;
q.Nr = 512;
q.Nd = max(64,2^nextpow2(2*(q.cfar.Td+q.cfar.Gd)+8));
q.f_range_axis = (0:q.Nr-1)*(q.fs_ADC/q.Nr);
q.range_axis = q.c*q.f_range_axis/(2*q.slope);
q.PRF = 1/q.Tchirp;
q.fd_axis = (-floor(q.Nd/2):ceil(q.Nd/2)-1)*(q.PRF/q.Nd);
q.vel_axis = q.lambda*q.fd_axis/2;
q.track.dt = q.Nd*q.Tchirp;
q.valid_range_mask = q.range_axis <= q.R_max;
q.theta_axis = linspace(-q.az_span,q.az_span,128);
q.interference.enabled = false; q.clutter.enabled = false;
blank = 1e-9*(randn(q.Nr,q.Nd,q.n_rx)+1j*randn(q.Nr,q.Nd,q.n_rx));
quiet = struct('clean',blank,'raw',blank);
[objQ,~,infoQ] = radar_object_tracker({quiet},{},q,struct('finalize',true));
report = add(report,'a signal-free frame produces no objects', ...
    isempty(objQ),sprintf('%d objects reported',numel(objQ)));
% A constant false-alarm-rate detector applied to noise produces false alarms
% at the configured rate. Demanding zero points from a noise frame would be
% demanding that the detector stop being a CFAR. What must hold is that the
% count stays consistent with the rate the operating point specifies, and that
% none of it survives to become an object - which the check above asserts.
nCells = nnz(q.valid_range_mask)*q.Nd;
expected = q.cfar.Pfa*nCells;
budget = max(5,10*expected);
nPts = numel(infoQ.live_hard_points);
report = add(report,'noise-only detections stay within the configured false-alarm rate', ...
    nPts <= budget, ...
    sprintf('%d points over %d cells; %.2f expected at Pfa %.0e, budget %d', ...
    nPts,nCells,expected,q.cfar.Pfa,round(budget)));

report = add(report,'objects record whether they were measured this frame', ...
    contains(trk,'o.coasted = t.missed > 0;'), ...
    'a report at a predicted state is never presented as a fresh detection');
report = add(report,'the coasted count is published with the objects', ...
    contains(trk,'coasted_object_count'));

% --- 3. no cap silently decides the outcome -----------------------------
report = add(report,'the verification budget exceeds the detection budget', ...
    p.detector.max_candidates >= p.cfar.max_detections, ...
    sprintf('%d candidates against %d detections', ...
    p.detector.max_candidates,p.cfar.max_detections));
report = add(report,'a bound verification budget is reported, not absorbed', ...
    contains(trk,'VerificationBudget'));
report = add(report,'nothing obliges the radar to report a minimum object count', ...
    isempty(regexp(trk,'min_objects|force_object|guarantee_object','once')));

% --- 4. duplicate gates match the resolution the array has --------------
report = add(report,'the angular duplicate gate lies inside one beamwidth', ...
    p.track.duplicate_angle_deg <= p.beamwidth_deg, ...
    sprintf('%.2f deg gate, %.2f deg beamwidth', ...
    p.track.duplicate_angle_deg,p.beamwidth_deg));
report = add(report,'the range duplicate gate spans at least two range cells', ...
    p.track.duplicate_range_m >= 2*p.range_resolution_actual, ...
    sprintf('%.2f m gate, %.2f m resolution', ...
    p.track.duplicate_range_m,p.range_resolution_actual));

% --- 5. the evaluator cannot be satisfied twice by one target ----------
truth = [100 5 10 0];
one = struct('range',100.5,'velocity',5,'angle_deg',0);
E1 = radar_object_evaluation(one,truth,{},p);
two = struct('range',{100.5,101.0},'velocity',{5,5},'angle_deg',{0,0});
E2 = radar_object_evaluation(two,truth,{},p);
report = add(report,'one truth object can be matched at most once', ...
    E1.matched == 1 && E2.matched == 1 && E2.false_objects == 1, ...
    sprintf('two reports on one target give %d matched and %d false', ...
    E2.matched,E2.false_objects));
report = add(report,'detection probability cannot exceed one', ...
    E2.pd <= 1+1e-12,sprintf('Pd = %.4f',E2.pd));

% --- 6. a duplicated report is diagnosed as a split, not an invention ---
stage = {struct('rd_power_moving',[],'rd_power_reference',[], ...
    'cfar_moving',one,'amf_moving',one,'stationary_hits',repmat(one,0,1), ...
    'amf_stationary',repmat(one,0,1),'groups',one)};
obs = truth_observability(truth,stage,{},two,p);
report = add(report,'a second report on a matched target is classed as splitting', ...
    get_default_field(obs.mechanism_counts,'split',0) >= 1, ...
    sprintf('split=%d spurious=%d', ...
    get_default_field(obs.mechanism_counts,'split',0), ...
    get_default_field(obs.mechanism_counts,'spurious',0)));

if verbose, print_report(report); end
end

% =========================================================================
function txt = strip_comments(txt)
lines = strsplit(txt,newline);
for i = 1:numel(lines)
    c = strfind(lines{i},'%');
    if ~isempty(c), lines{i} = lines{i}(1:c(1)-1); end
end
txt = strjoin(lines,newline);
end

function s = describe(c)
if isempty(c), s = ''; return; end
s = sprintf('%d file(s): %s',numel(c),strjoin(unique(c),', '));
end

function r = add(r,name,pass,detail)
if nargin < 4, detail = ''; end
r.checks(end+1) = struct('name',name,'pass',logical(pass),'detail',detail);
r.ok = r.ok && logical(pass);
end

function print_report(r)
fprintf('\n[%s] %s\n',upper(r.name),tern(r.ok,'PASS','FAIL'));
for i = 1:numel(r.checks)
    c = r.checks(i);
    if isempty(c.detail)
        fprintf('   %-4s %s\n',tern(c.pass,'ok','FAIL'),c.name);
    else
        fprintf('   %-4s %s  (%s)\n',tern(c.pass,'ok','FAIL'),c.name,c.detail);
    end
end
end
function y = tern(c,a,b), if c, y = a; else, y = b; end, end
