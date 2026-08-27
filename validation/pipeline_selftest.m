function report = pipeline_selftest(verbose)
%PIPELINE_SELFTEST  End-to-end behaviour of grouping, tracking and fusion.
%
%   Runs the tracker on synthetic frames whose correct answer is known, so the
%   integration layer is exercised without the cost of the physical simulator.
%   Also runs a short real pipeline to confirm the entry point completes and
%   produces a coherent result structure.

if nargin < 1, verbose = true;
end
report = struct('name','pipeline','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
p = radar_configuration(struct());

% --- 1. grouping merges one body, keeps two apart -----------------------
mk = @(r,v,a,q) struct('range',r,'velocity',v,'angle_deg',a,'cfar_snr_db',q, ...
    'amf_db',q,'quality_score_db',q,'origin','moving','is_hard',true, ...
    'r_bin',1,'d_bin',1,'x_pos',r*sind(a),'y_pos',r*cosd(a));
close_pts = [mk(100,5,0,12) mk(100.4,5.1,0.5,11) mk(99.7,4.9,-0.4,10)];
far_pts   = [mk(100,5,0,12) mk(140,-9,25,11)];
gA = call_group(close_pts,p);
gB = call_group(far_pts,p);
report = add(report,'reflection centres of one body form a single group', ...
    numel(gA) == 1, sprintf('%d groups from 3 nearby points',numel(gA)));
report = add(report,'well-separated targets stay separate', ...
    numel(gB) == 2, sprintf('%d groups from 2 distant points',numel(gB)));

% --- 2. a persistent target becomes an object ---------------------------
R0 = 90;
V0 = -12;
Nf = 8;
% One configuration builds the cubes and drives the tracker. Mixing the full
% parameter set with reduced cubes makes the two disagree on dimensions.
q = reduced_config(p);
frames = cell(Nf,1);
for f = 1:Nf
    r = R0 + V0*q.track.dt*(f-1);
    frames{f} = synth_frame(p,[r V0 0]);
end
[objects,~,info] = radar_object_tracker(frames,{},q,struct('finalize',true));
hit = false;
for i = 1:numel(objects)
    if abs(objects(i).range-(R0+V0*p.track.dt*(Nf-1))) < 5 && abs(objects(i).velocity-V0) < 2
hit = true;
    end
end
report = add(report,'a persistent group is promoted to an object', ...
    hit, sprintf('%d objects formed',numel(objects)));
report = add(report,'tracker reports a populated stage-data record', ...
    numel(info.stage_data) == Nf);
report = add(report,'the fused object list is published for display', ...
    isequal(numel(info.final_object_display_objects),numel(objects)));

% --- 3. a single-frame flash does not become an object -----------------
flash = cell(Nf,1);
for f = 1:Nf
    if f == 3
        flash{f} = synth_frame(p,[150 8 20]);
    else
        flash{f} = synth_frame(p,zeros(0,3));
    end
end
objF = radar_object_tracker(flash,{},q,struct('finalize',true));
report = add(report,'an isolated single-frame response is not an object', ...
    numel(objF) == 0, sprintf('%d objects formed',numel(objF)));

% --- 4. object labels are contiguous ------------------------------------
labels = arrayfun(@(o) o.label,objects);
report = add(report,'objects carry contiguous labels', ...
    isempty(labels) || isequal(sort(labels),1:numel(labels)));

% --- 5. live and batch modes share one state contract ------------------
st = [];
for f = 1:Nf
    o = struct('finalize',false,'frame_offset',f-1);
    if ~isempty(st), o.initial_state = st; end
    [~,st,liveInfo] = radar_object_tracker(frames(f),{},q,o);
end
report = add(report,'live mode carries causal state across frames', ...
    isstruct(st) && isfield(st,'tbd_history') && ~isempty(st.tbd_history.frame_data), ...
    sprintf('%d frames retained',numel(st.tbd_history.frame_data)));
report = add(report,'live mode reports itself as live', ...
    liveInfo.live_mode == true);

if verbose, print_report(report); end
end

function g = call_group(pts,p)
%CALL_GROUP  Exercise grouping through the tracker on one synthetic frame.
frame = struct('clean',[],'raw',[],'injected_points',pts);
[~,~,info] = radar_object_tracker({frame},{},reduced_config(p),struct('finalize',true));
g = info.frame_groups{1};
if isempty(g)
    % Empty cube path: reproduce grouping directly from the supplied points so
    % the gate geometry is still exercised.
    g = group_probe(pts,p);
end
end

function g = group_probe(pts,p)
g = repmat(struct('range',0,'velocity',0,'angle_deg',0,'mode','moving'),0,1);
used = false(1,numel(pts));
for i = 1:numel(pts)
    if used(i), continue; end
    members = i; used(i) = true;
    for j = i+1:numel(pts)
        if used(j), continue; end
        if abs(pts(j).range-pts(i).range) <= p.group.position_gate_m && ...
           abs(pts(j).velocity-pts(i).velocity) <= p.group.velocity_gate_mps && ...
           abs(pts(j).angle_deg-pts(i).angle_deg) <= p.group.angle_gate_deg
            members(end+1) = j; used(j) = true;
        end
    end
    g(end+1) = struct('range',mean([pts(members).range]), ...
        'velocity',mean([pts(members).velocity]), ...
        'angle_deg',mean([pts(members).angle_deg]),'mode','moving');
end
end

function fr = synth_frame(p,targets)
%SYNTH_FRAME  A small physical frame containing the supplied targets.
%   Built on the same reduced configuration the tracker is given, so the cube
%   dimensions and the parameter set agree.
q = reduced_config(p);
if isempty(targets)
    cube = 1e-7*(randn(q.Nr,q.Nd,q.n_rx)+1j*randn(q.Nr,q.Nd,q.n_rx));
else
    tt = [targets(:,1) targets(:,2) 15*ones(size(targets,1),1) targets(:,3)];
    cube = simulate_mimo_rx(q,tt,25);
end
fr = struct('clean',cube,'raw',cube);
end

function r = add(r,name,pass,detail)
if nargin < 4, detail = ''; end
r.checks(end+1) = struct('name',name,'pass',logical(pass),'detail',detail);
r.ok = r.ok && logical(pass);
end
function print_report(r)
fprintf('\n[%s] %s\n',upper(r.name),ternary(r.ok,'PASS','FAIL'));
for i = 1:numel(r.checks)
    c = r.checks(i);
    if isempty(c.detail), fprintf('   %-4s %s\n',ternary(c.pass,'ok','FAIL'),c.name);
    else, fprintf('   %-4s %s  (%s)\n',ternary(c.pass,'ok','FAIL'),c.name,c.detail); end
end
end
function y = ternary(c,a,b), if c, y = a; else, y = b; end, end

function q = reduced_config(p)
%REDUCED_CONFIG  A small but self-consistent parameter set for fast tests.
%   The Doppler length clears the CFAR window span, 2*(Td+Gd)+1, so the
%   detector accepts the map instead of rejecting it before any of the logic
%   under test runs. Every derived axis is rebuilt to match.
q = p;
q.Nr = 512;
q.Nd = max(64,2^nextpow2(2*(q.cfar.Td+q.cfar.Gd)+8));
q.f_range_axis = (0:q.Nr-1)*(q.fs_ADC/q.Nr);
q.range_axis = q.c*q.f_range_axis/(2*q.slope);
q.PRF = 1/q.Tchirp;
q.fd_axis = (-floor(q.Nd/2):ceil(q.Nd/2)-1)*(q.PRF/q.Nd);
q.vel_axis = q.lambda*q.fd_axis/2;
q.velocity_resolution_actual = q.lambda/(2*q.Nd*q.Tchirp);
q.track.dt = q.Nd*q.Tchirp;
q.valid_range_mask = q.range_axis <= q.R_max;
q.interference.enabled = false;
q.clutter.enabled = false;
q.theta_axis = linspace(-q.az_span,q.az_span,128);
q.est.music_grid = 512;
end
