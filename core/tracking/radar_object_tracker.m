function [objects,state,info] = radar_object_tracker(frame_data,~,p,opts)
%RADAR_OBJECT_TRACKER  Point cloud, grouping, tracking and object formation.
%
%   This is the integration layer of the perception stack. It turns per-frame
%   radar measurements into persistent objects through an explicit hierarchy of
%   evidence, in which every stage answers a question the previous stage could
%   not.
%
%     1. Calibrated CFAR asks whether a cell is brighter than its local
%        background allows. It produces hard candidates only.
%     2. The adaptive matched filter and the generalized subspace detector ask
%        whether that energy is spatially consistent with a wavefront from a
%        single direction. Sub-CFAR evidence never enters this stage on the
%        normal path.
%     3. TDM Doppler-ambiguity resolution and high-resolution angle estimation
%        turn verified detections into a measured point cloud.
%     4. Grouping asks which points belong to the same physical body, because
%        one vehicle produces several reflection centres.
%     5. The group tracker asks whether a group persists along a kinematically
%        admissible trajectory, using a Kalman filter with normalised
%        innovation squared gating.
%     6. Object formation asks whether accumulated evidence justifies declaring
%        an object, applying stricter gates than track confirmation.
%     7. Track-before-detect runs alongside as an independent weak-target
%        channel. Its trajectories never enter the point tracker; they are
%        fused only at object level, and only where no confirmed object
%        already owns the same physical response.
%
%   Truth is never supplied to any formation stage. It is used exclusively by
%   the offline evaluator, after this function has returned.
%
%   [objects,state,info] = RADAR_OBJECT_TRACKER(frame_data,~,p,opts)
%
%   opts.finalize       true for batch mode, false for causal frame-by-frame
%   opts.frame_offset   global frame index of the first supplied frame
%   opts.initial_state  carried tracker state for live operation

if nargin < 4 || isempty(opts), opts = struct(); end
Nf = numel(frame_data);
finalizeRun = get_default_field(opts,'finalize',true);
frameOffset = get_default_field(opts,'frame_offset',0);

if isfield(opts,'initial_state') && ~isempty(opts.initial_state)
state = opts.initial_state;
else
    state = empty_state();
end
if ~isfield(state,'tbd_history') || ~isstruct(state.tbd_history)
    state.tbd_history = struct('frame_data',{{}},'evidence',{{}},'frame_info',{{}}, ...
        'global_frames',zeros(1,0));
end

allFramePoints = cell(Nf,1);
allFrameGroups = cell(Nf,1);
frameInfo      = cell(Nf,1);
trackDiag      = cell(Nf,1);
tbdEvidence    = cell(Nf,1);
stageData      = cell(Nf,1);
gsInterferenceFrames = 0;
gsRescuedTotal = 0;

lastPaper = struct(); lastRdClean = []; lastRdAux = struct();

for frame = 1:Nf
    if live_stop_requested(), break; end
    [cleanCube,rawCube] = unpack_frame(frame_data{frame});
    if isempty(cleanCube)
        allFramePoints{frame} = detection_array_template();
        allFrameGroups{frame} = group_array_template();
        frameInfo{frame} = empty_frame_info();
        trackDiag{frame} = empty_track_diag();
        tbdEvidence{frame} = struct([]);
        stageData{frame} = empty_stage_data();
continue;
    end

    [Pmove,Pref,rdAux,paperProc,ifInfo] = build_rd_sum(cleanCube,rawCube,p);
lastPaper = paperProc;
lastRdClean = Pmove;
lastRdAux = rdAux;
    if ~finalizeRun
        drawnow limitrate; if live_stop_requested(), break; end
    end

    fi = empty_frame_info();
fi.interference_info = ifInfo;

    % ---- moving branch: hard candidates ---------------------------------
    % With moving_use_cfar disabled the moving branch is suppressed entirely
    % and only the stationary decomposition contributes, which isolates the
    % contribution of each branch when studying where objects come from.
    if logical(get_default_field(p.paper,'moving_use_cfar',true))
        [cfarDet,thrMap,noiseMap,cinfo] = adaptive_cfar_2d(Pmove,p,Pref);
    else
        cfarDet = []; thrMap = zeros(size(Pmove)); noiseMap = zeros(size(Pmove));
        cinfo = struct('weak_candidates',[],'statistical_count',0,'mode','disabled');
    end
fi.cfar_info = cinfo;
    fi.hard_cfar_count = numel(cfarDet);
    hardMoving = standardize_detections(cfarDet,p,'moving');

    % ---- spatial verification -------------------------------------------
    gsInfoMoving = struct('evaluated_count',0,'accepted_count',0,'rescued_count',0, ...
        'interference_detected_count',0,'mean_inr_db',-Inf);
    if p.detector.gs.enabled && ~isempty(hardMoving)
        [hardMoving,gsInfoMoving] = generalized_subspace_detector(hardMoving,cleanCube,p);
        if gsInfoMoving.interference_detected_count > 0
gsInterferenceFrames = gsInterferenceFrames + 1;
        end
    end
    [verifiedMoving,ainfo] = adaptive_matched_filter(hardMoving,cleanCube,p);
    if get_default_field(ainfo,'dropped_over_budget',0) > 0
        % The budget bounds a pathological frame. If it binds in normal
        % operation it is silently deciding which candidates get tested, so it
        % announces itself rather than absorbing the truncation.
        warning('radar_object_tracker:VerificationBudget', ...
            ['Frame %d: %d CFAR candidates dropped before verification ' ...
             '(detector.max_candidates = %d). Raise the budget.'], ...
            frame,ainfo.dropped_over_budget,p.detector.max_candidates);
    end
    gsRescuedTotal = gsRescuedTotal + get_default_field(ainfo,'gs_rescued_count',0);
fi.amf_info = ainfo;
    fi.amf_verified_count = numel(verifiedMoving);

    % ---- TDM ambiguity, angle, quality ----------------------------------
    if ~isempty(verifiedMoving)
        [verifiedMoving,tdmInfo] = tdm_mimo_processing('resolve',verifiedMoving,cleanCube,p);
fi.tdm_info = tdmInfo;
        verifiedMoving = angle_refinement(verifiedMoving,cleanCube,p);
        verifiedMoving = detection_quality_filter(verifiedMoving,p);
    end
    verifiedMoving = canonicalize_detections(verifiedMoving);
    for i = 1:numel(verifiedMoving), verifiedMoving(i).origin = 'moving'; end

    % ---- stationary branch ----------------------------------------------
    stationaryRaw = detection_array_template();
    verifiedStationary = detection_array_template();
gsInfoStationary = gsInfoMoving;
gsInfoStationary.evaluated_count = 0;
    if p.paper.stationary.enabled
        sdet = stationary_target_detector(paperProc.stationary_range_angle_power,p);
        stationaryRaw = standardize_detections(sdet,p,'stationary');
        if ~isempty(stationaryRaw)
            if p.detector.gs.enabled
                [stationaryRaw,gsInfoStationary] = generalized_subspace_detector(stationaryRaw,cleanCube,p);
            end
            verifiedStationary = adaptive_matched_filter(stationaryRaw,cleanCube,p);
            verifiedStationary = angle_refinement(verifiedStationary,cleanCube,p);
            verifiedStationary = detection_quality_filter(verifiedStationary,p);
        end
    end
    verifiedStationary = canonicalize_detections(verifiedStationary);
    for i = 1:numel(verifiedStationary)
        verifiedStationary(i).origin = 'stationary';
        verifiedStationary(i).velocity = 0;
    end

    verified = [verifiedMoving(:); verifiedStationary(:)];

    stageData{frame} = struct( ...
        'cfar_moving',hardMoving,'amf_moving',verifiedMoving,'rd_power_moving',Pmove, ...
        'rd_power_reference',Pref,'noise_map',noiseMap,'stationary_hits',stationaryRaw, ...
        'amf_stationary',verifiedStationary,'groups',group_array_template(), ...
        'gs_moving',gsInfoMoving,'gs_stationary',gsInfoStationary);

    % ---- grouping and tracking -------------------------------------------
    if logical(get_default_field(p.paper,'coarse_grouping_before_tracking',true))
        groups = group_point_cloud(verified,p);
    else
        groups = points_as_groups(verified,p);
    end
    stageData{frame}.groups = groups;
    fi.group_count = numel(groups);
    fi.accepted_measurement_count = numel(verified);

globalFrame = frameOffset + frame;
    if logical(get_default_field(p.paper,'separate_track_processes',true))
        movingGroups     = groups(strcmp({groups.mode},'moving'));
        stationaryGroups = groups(strcmp({groups.mode},'stationary'));
        [state.moving,tdM]     = update_group_tracker(state.moving,movingGroups,p,globalFrame,'moving');
        [state.stationary,tdS] = update_group_tracker(state.stationary,stationaryGroups,p,globalFrame,'stationary');
    else
        [state.moving,tdM] = update_group_tracker(state.moving,groups,p,globalFrame,'moving');
        tdS = empty_track_diag();
        state.stationary = empty_track_state();
    end
state.frame = globalFrame;
    state.tracks = [state.moving.tracks(:).' state.stationary.tracks(:).'];

    allFramePoints{frame} = verified;
    allFrameGroups{frame} = groups;
    frameInfo{frame} = fi;
    trackDiag{frame} = merge_track_diag(tdM,tdS);
    tbdEvidence{frame} = build_tbd_evidence(Pmove,Pref,noiseMap,thrMap,cinfo,verified,p);

    state = push_tbd_history(state,frame_data{frame},tbdEvidence{frame}, ...
        struct('rd_power_moving',Pmove,'rd_power_reference',Pref,'noise_map',noiseMap), ...
        globalFrame,p);
end

% =========================================================================
% Object formation
% =========================================================================
info = empty_info();
info.frame_count = Nf;
info.frame_detections = allFramePoints;
info.frame_groups = allFrameGroups;
info.frame_info = frameInfo;
info.track_diagnostics = trackDiag;
info.stage_data = stageData;
info.global_frame = state.frame;
info.live_mode = ~finalizeRun;
info.detector = 'VI-CFAR -> AMF/GS -> TDM/MUSIC -> grouping -> NIS-gated tracker -> TBD fusion';
info.gs_interference_frames = gsInterferenceFrames;
info.gs_rescued_count = gsRescuedTotal;
info.final_paper_processing = lastPaper;
info.final_rd_power_clean = lastRdClean;
info.final_rd_aux = lastRdAux;
if isstruct(lastPaper) && isfield(lastPaper,'stationary_range_angle_power')
info.final_stationary_range_angle_power = lastPaper.stationary_range_angle_power;
info.final_stationary_range_power = lastPaper.stationary_range_power;
end

movingObjects = extract_group_objects(state.moving,p,state.frame,'moving');
stationaryObjects = extract_group_objects(state.stationary,p,state.frame,'stationary');
% Keep object aggregation shape-invariant.  Individual extraction routes can
% legitimately return either an empty 0x1 struct array or a non-empty row
% array after dynamic promotion.  Canonicalise both to column vectors before
% vertical concatenation so the object schema and ordering are unchanged.
groupObjects = [movingObjects(:); stationaryObjects(:)];
info.moving_track_count     = numel(state.moving.tracks);
info.stationary_track_count = numel(state.stationary.tracks);
info.group_track_count      = info.moving_track_count + info.stationary_track_count;
info.moving_point_count     = count_origin(allFramePoints,'moving');
info.stationary_point_count = count_origin(allFramePoints,'stationary');
info.moving_group_count     = count_group_mode(allFrameGroups,'moving');
info.stationary_group_count = count_group_mode(allFrameGroups,'stationary');
info.group_measurement_count= info.moving_group_count + info.stationary_group_count;
info.measurement_count      = info.group_measurement_count;
info.confirmed_count        = nnz([state.tracks.confirmed]);
info.gnn_confirmed_count    = info.confirmed_count;
info.gnn_confirmed_track_count = info.confirmed_count;
info.track_count            = numel(state.tracks);

if finalizeRun
    [dpPaths,dpInfo]   = dynamic_programming_tbd(tbdEvidence,p);
    [cohPaths,cohInfo] = coherent_tbd_detector(frame_data,p,collect_frame_maps(stageData,tbdEvidence));
    tbdPaths = merge_tbd_paths(dpPaths,cohPaths,p);
info.tbd = dpInfo;
info.live_coherent_tbd_info = cohInfo;
else
    [tbdPaths,liveInfo,cohInfo] = live_tbd_pass(state,p);
info.tbd = liveInfo;
info.live_tbd_info = liveInfo;
info.live_coherent_tbd_info = cohInfo;
info.live_tbd_paths = tbdPaths;
end

tbdPaths = validate_tbd_path_angles(tbdPaths,p);
tbdObjects = paths_to_objects(tbdPaths,p,groupObjects,frame_data);
info.candidate_trajectory_count = numel(tbdPaths);
info.tbd_confirmed_count = numel(tbdObjects);
info.tbd_state_cells = numel(state.tbd_history.frame_data);

objects = fuse_objects(groupObjects,tbdObjects,p);
objects = relabel_objects(objects);
info.final_object_count = numel(objects);
% How many reported objects are coasting on a prediction this frame. A run in
% which this is persistently large is reporting inference, not measurement.
if isempty(objects)
    info.coasted_object_count = 0;
else
    info.coasted_object_count = nnz(arrayfun(@(o) logical(get_default_field(o,'coasted',false)),objects));
end
info.live_hard_objects = groupObjects;
info.live_tbd_objects = tbdObjects;
% Strictly the final frame's point cloud. Falling back to an earlier frame
% would display stale detections as though they belonged to this one.
if isempty(allFramePoints)
    info.live_hard_points = detection_array_template();
else
    info.live_hard_points = allFramePoints{end};
end
info.live_display_objects = objects;
info.final_object_display_objects = objects;
end

% =========================================================================
% Range-Doppler front end
% =========================================================================
function [Pmove,Pref,rdAux,paperProc,ifInfo] = build_rd_sum(cleanCube,rawCube,p)
%BUILD_RD_SUM  Interference suppression, then the two coherent power maps.
%
%   The moving-target map and the CFAR reference map are produced by one call
%   to the coherent separator, so they cannot drift apart in normalisation.
%   The reference is the unsubtracted field: it carries the same noise
%   statistics as the cell under test but is not depleted by stationary
%   subtraction, which makes it the correct estimate of the local background.

ifInfo = struct('applied',false,'passes',0,'mask_fraction',0,'reduction_db',0);
cube = cleanCube;

if p.interference.hough_tf_enabled
passes = 1;
    if logical(get_default_field(p.interference,'iterative_enabled',false))
        passes = max(1,round(get_default_field(p.interference,'iterative_passes',1)));
    end
    minRed = get_default_field(p.interference,'iterative_min_power_reduction_db',1.0);
    for pass = 1:passes
        before = mean(abs(cube(:)).^2) + eps;
maskFrac = 0;
        for rx = 1:size(cube,3)
            [cube(:,:,rx),hi] = hough_interference_mitigator(cube(:,:,rx),p);
            maskFrac = maskFrac + hi.mask_fraction/size(cube,3);
        end
        after = mean(abs(cube(:)).^2) + eps;
        red = 10*log10(before/after);
ifInfo.applied = true;
ifInfo.passes = pass;
ifInfo.mask_fraction = maskFrac;
ifInfo.reduction_db = ifInfo.reduction_db + red;
if red < minRed, break;
end
    end
end

paperProc = moving_stationary_separator(cube,p);
Pmove = paperProc.moving_rd_power;
Pref  = paperProc.raw_rd_power;

% One display-side transform on the first chain, for RX/ADC diagnostics.
if isempty(rawCube), rawCube = cleanCube; end
[~,~,~,~,rdAux] = range_doppler_processor(rawCube(:,:,1),p);
end

% =========================================================================
% Detection normalisation
% =========================================================================
function det = standardize_detections(in,p,origin)
%STANDARDIZE_DETECTIONS  Coerce detector output to the tracker's contract.
%   Sub-bin refined range and velocity are preserved. The bin indices travel
%   alongside as integers for anything that needs to index a map.
det = detection_array_template();
for i = 1:numel(in)
    d = detection_template();
    src = in(i);
    f = fieldnames(src);
    for k = 1:numel(f)
        if isfield(d,f{k}), d.(f{k}) = src.(f{k}); end
    end
    d.r_bin = clamp_index(get_default_field(src,'r_bin',1),numel(p.range_axis));
    d.d_bin = clamp_index(get_default_field(src,'d_bin',1),numel(p.vel_axis));
    if ~isfinite(d.range) || d.range <= 0
        d.range = p.range_axis(d.r_bin);
    end
    if ~isfinite(d.velocity)
        d.velocity = p.vel_axis(d.d_bin);
    end
    if strcmp(origin,'stationary'), d.velocity = 0; end
d.origin = origin;
d.is_hard = true;
    det(end+1) = d;
end
end

function det = canonicalize_detections(in)
det = detection_array_template();
for i = 1:numel(in)
    d = detection_template();
    f = fieldnames(in(i));
    for k = 1:numel(f)
        if isfield(d,f{k}), d.(f{k}) = in(i).(f{k}); end
    end
    % A missing bearing is carried as NaN, not replaced by boresight. The
    % quality filter rejects it; substituting zero here would put an
    % unresolved detection at zero degrees and call it a measurement.
    d.x_pos = d.range*sind(d.angle_deg);
    d.y_pos = d.range*cosd(d.angle_deg);
    det(end+1) = d;
end
end

% =========================================================================
% Grouping
% =========================================================================
function groups = group_point_cloud(det,p)
%GROUP_POINT_CLOUD  Merge reflection centres belonging to one physical body.
%
%   A vehicle spans several range and angle cells, so a single object produces
%   a cluster of points. Grouping them before tracking prevents one car from
%   becoming several objects. Compatibility is tested in range, radial
%   velocity and bearing, optionally with a Cartesian cross-range term so that
%   two points at similar range but opposite bearings are never merged.
%
%   Gates widen with range when adaptive_position_gain is non-zero, because a
%   fixed angular error subtends a larger cross-range distance far away.

groups = group_array_template();
if isempty(det), return; end

minPoints  = max(1,round(get_default_field(p.group,'min_points',1)));
maxPoints  = max(1,round(get_default_field(p.group,'max_points_per_group',32)));
useCart    = logical(get_default_field(p.group,'use_cartesian',true));
commonV    = logical(get_default_field(p.group,'enforce_common_radial_velocity',true));
growth     = logical(get_default_field(p.group,'enable_centroid_growth',true));
posGain    = get_default_field(p.group,'adaptive_position_gain',0);
weightFloor= get_default_field(p.group,'weight_floor_db',2.0);
maxCost    = get_default_field(p.group,'max_cluster_cost',1.0);

for modeCell = {'moving','stationary'}
    modeName = modeCell{1};
    sel = find(strcmp({det.origin},modeName));
    if isempty(sel), continue; end
    sub = det(sel);
    quality = arrayfun(@(d) max(get_default_field(d,'quality_score_db',d.cfar_snr_db),-Inf),sub);
    [~,ord] = sort(quality,'descend');
    sub = sub(ord); sel = sel(ord);

    if strcmp(modeName,'stationary')
        vGate = get_default_field(p.group,'stationary_velocity_gate_mps',p.group.velocity_gate_mps);
    else
        vGate = get_default_field(p.group,'moving_velocity_gate_mps',p.group.velocity_gate_mps);
    end

    used = false(1,numel(sub));
    for i = 1:numel(sub)
        if used(i), continue; end
        members = i; used(i) = true;
        cR = sub(i).range; cV = sub(i).velocity; cA = sub(i).angle_deg;
        for j = i+1:numel(sub)
            if used(j) || numel(members) >= maxPoints, continue; end
            rGate = p.group.position_gate_m*(1 + posGain*cR/max(p.R_max,eps));
            cost = group_cost(sub(j),cR,cV,cA,rGate,vGate,p.group.angle_gate_deg,useCart);
            if ~isfinite(cost) || cost > maxCost, continue; end
            if commonV && abs(sub(j).velocity - cV) > vGate, continue; end
            members(end+1) = j; used(j) = true;
            if growth
                w = group_weights(sub(members),weightFloor);
                cR = sum(w.*[sub(members).range]);
                cV = sum(w.*[sub(members).velocity]);
                cA = circular_mean([sub(members).angle_deg],w);
            end
        end
        if numel(members) < minPoints, continue; end
        groups(end+1) = build_group(sub(members),sel(members),modeName,weightFloor);
    end
end
end

function groups = points_as_groups(det,p)
groups = group_array_template();
for i = 1:numel(det)
    groups(end+1) = build_group(det(i),i,det(i).origin,0);
end
end

function c = group_cost(d,cR,cV,cA,rGate,vGate,aGate,useCart)
dr = abs(d.range - cR)/max(rGate,eps);
dv = abs(d.velocity - cV)/max(vGate,eps);
da = 0;
if isfinite(d.angle_deg) && isfinite(cA)
    da = abs(wrap_angle(d.angle_deg - cA))/max(aGate,eps);
end
c = max([dr dv da]);
if useCart && isfinite(d.angle_deg) && isfinite(cA)
    dx = d.range*sind(d.angle_deg) - cR*sind(cA);
    dy = d.range*cosd(d.angle_deg) - cR*cosd(cA);
    c = max(c,hypot(dx,dy)/max(rGate,eps));
end
end

function w = group_weights(members,floorDb)
%GROUP_WEIGHTS Robust quality weighting for a non-empty detector member set.
% A group can legitimately arrive with no finite quality score (for example,
% after a downstream diagnostic field is unavailable). Such a group must still
% receive a well-defined uniform weighting rather than triggering an indexed
% assignment with an empty right-hand side.
q = arrayfun(@(d) max(get_default_field(d,'quality_score_db',d.cfar_snr_db),-Inf),members);
q = q(:).';
if isempty(q)
    w = zeros(1,0);
return;
end
finite_q = isfinite(q);
if ~any(finite_q)
    w = ones(1,numel(q))/numel(q);
return;
end
q(~finite_q) = min(q(finite_q),[],'omitnan');
q_floor = min(q,[],'omitnan');
w = max(q - q_floor + max(floorDb,0.1),0.1);
s = sum(w);
if ~isfinite(s) || s <= 0
    w = ones(1,numel(q))/numel(q);
else
w = w/s;
end
end

function g = build_group(members,idx,modeName,weightFloor)
w = group_weights(members,weightFloor);
g = group_template();
g.range    = sum(w.*[members.range]);
g.velocity = sum(w.*[members.velocity]);
g.angle_deg= circular_mean([members.angle_deg],w);
g.x_pos = g.range*sind(g.angle_deg);
g.y_pos = g.range*cosd(g.angle_deg);
g.range_spread_m      = spread([members.range]);
g.velocity_spread_mps = spread([members.velocity]);
g.angle_spread_deg    = angle_spread([members.angle_deg]);
g.reflection_count    = numel(members);
g.member_indices      = idx(:).';
g.cfar_snr_db = max([members.cfar_snr_db]);
g.amf_db      = max([members.amf_db]);
g.quality_db  = max(arrayfun(@(d) get_default_field(d,'quality_score_db',-Inf),members));
g.is_hard = true;
g.mode = modeName;
end

% =========================================================================
% Group tracker
% =========================================================================
function [ts,trackDiag] = update_group_tracker(ts,groups,p,globalFrame,modeName)
%UPDATE_GROUP_TRACKER  Kalman prediction, NIS-gated association and lifecycle.
%
%   State is the radial pair x = [r; v] under a constant-velocity model
%
%       F = [1 dt; 0 1] ,   Q = q G G^T ,   G = [dt^2/2; dt]
%
%   with q the process acceleration variance. Both state components are
%   measured directly, so H = I and the innovation covariance is
%
%       S = P^- + R ,   R = diag(sigma_r^2, sigma_v^2)
%
%   Association uses the normalised innovation squared
%
%       epsilon = nu^T S^-1 nu
%
%   which is chi-square distributed with two degrees of freedom under a correct
%   association. Gating on epsilon rather than on fixed windows makes the gate
%   adapt automatically: a freshly born track with large covariance accepts a
%   wider innovation than a well-established one. A cheap per-axis box gate
%   runs first to avoid evaluating the quadratic form for obviously distant
%   pairs.
%
%   Bearing is not part of the linear state, because the measurement is
%   nonlinear in it and the radial pair is what the constant-velocity model
%   actually describes. It is filtered separately by a scalar Kalman recursion
%   on the unit circle, which averages correctly across the wrap.

if isempty(ts) || ~isstruct(ts), ts = empty_track_state(); end
trackDiag = empty_track_diag();
trackDiag.measurement_count = numel(groups);

dt = p.track.dt;
F  = [1 dt; 0 1];
G  = [0.5*dt^2; dt];
Q  = p.track.q*(G*G');
R  = diag([p.track.measurement_sigma_range^2, p.track.measurement_sigma_velocity^2]);
Ra = p.track.measurement_sigma_angle^2;
Qa = get_default_field(p.track,'process_sigma_angle_deg',1.5)^2;

% ---- predict -------------------------------------------------------------
for k = 1:numel(ts.tracks)
    ts.tracks(k).x = F*ts.tracks(k).x;
    ts.tracks(k).P = F*ts.tracks(k).P*F' + Q;
    ts.tracks(k).range    = ts.tracks(k).x(1);
    ts.tracks(k).velocity = ts.tracks(k).x(2);
    ts.tracks(k).Pa = ts.tracks(k).Pa + Qa;
end

% ---- gated global nearest neighbour --------------------------------------
nT = numel(ts.tracks); nM = numel(groups);
cost = inf(nT,nM);
for k = 1:nT
    S = ts.tracks(k).P + R;
    Sinv = inv_2x2(S);
    for m = 1:nM
        nu = [groups(m).range - ts.tracks(k).x(1); groups(m).velocity - ts.tracks(k).x(2)];
        if abs(nu(1)) > p.track.gate_range_sigma*sqrt(S(1,1)) + p.group.position_gate_m, continue; end
        if abs(nu(2)) > p.track.gate_velocity_sigma*sqrt(S(2,2)) + p.group.velocity_gate_mps, continue; end
        nis = nu'*Sinv*nu;
if nis > p.track.gate_nis, continue;
end
        if isfinite(ts.tracks(k).angle_deg) && isfinite(groups(m).angle_deg)
            da = abs(wrap_angle(groups(m).angle_deg - ts.tracks(k).angle_deg));
            if da > p.track.gate_angle_sigma*sqrt(ts.tracks(k).Pa + Ra) + p.group.angle_gate_deg, continue; end
            nis = nis + (da^2)/max(ts.tracks(k).Pa + Ra,eps);
        end
        cost(k,m) = nis;
    end
end
assign = greedy_assign(cost);

% ---- update --------------------------------------------------------------
matchedMeas = false(1,nM);
for k = 1:nT
    m = assign(k);
    if m == 0
        ts.tracks(k).missed = ts.tracks(k).missed + 1;
        ts.tracks(k) = push_history(ts.tracks(k),false,-Inf,-Inf,NaN,0,globalFrame,group_template(),p);
continue;
    end
    matchedMeas(m) = true;
trackDiag.matched_count = trackDiag.matched_count + 1;
    g = groups(m);

    S = ts.tracks(k).P + R;
    K = ts.tracks(k).P/S;
    nu = [g.range - ts.tracks(k).x(1); g.velocity - ts.tracks(k).x(2)];
    ts.tracks(k).x = ts.tracks(k).x + K*nu;
    ts.tracks(k).P = (eye(2) - K)*ts.tracks(k).P*(eye(2)-K)' + K*R*K';
    ts.tracks(k).range = ts.tracks(k).x(1);
    ts.tracks(k).velocity = ts.tracks(k).x(2);

    if isfinite(g.angle_deg)
        if ~isfinite(ts.tracks(k).angle_deg)
            ts.tracks(k).angle_deg = g.angle_deg; ts.tracks(k).Pa = Ra;
        else
            Ka = ts.tracks(k).Pa/(ts.tracks(k).Pa + Ra);
            ts.tracks(k).angle_deg = wrap_angle(ts.tracks(k).angle_deg + ...
                Ka*wrap_angle(g.angle_deg - ts.tracks(k).angle_deg));
            ts.tracks(k).Pa = (1-Ka)*ts.tracks(k).Pa;
        end
    end

strongHit = g.amf_db >= p.track.min_strong_amf_db && g.cfar_snr_db >= p.track.min_strong_cfar_db;
    ts.tracks(k).missed = 0;
    ts.tracks(k).hits = ts.tracks(k).hits + 1;
    ts.tracks(k).hard_hits = ts.tracks(k).hard_hits + double(g.is_hard);
    ts.tracks(k).strong_hits = ts.tracks(k).strong_hits + double(strongHit);
    ts.tracks(k) = push_history(ts.tracks(k),true,g.amf_db,g.cfar_snr_db,g.angle_deg, ...
        g.reflection_count,globalFrame,g,p);
    ts.tracks(k).last_frame = globalFrame;
    ts.tracks(k).last_group = g;
end

% ---- birth ---------------------------------------------------------------
for m = 1:nM
    if matchedMeas(m), continue; end
    if groups(m).amf_db < p.track.birth_min_amf_db, continue; end
    t = empty_track();
t.id = ts.next_id;
ts.next_id = ts.next_id + 1;
    t.x = [groups(m).range; groups(m).velocity];
    t.P = diag([max(4*p.track.measurement_sigma_range^2,1), ...
                max(4*p.track.measurement_sigma_velocity^2,1)]);
    t.range = groups(m).range; t.velocity = groups(m).velocity;
    t.angle_deg = groups(m).angle_deg; t.Pa = Ra;
t.hits = 1;
    t.hard_hits = double(groups(m).is_hard);
    t.strong_hits = double(groups(m).amf_db >= p.track.min_strong_amf_db && ...
                           groups(m).cfar_snr_db >= p.track.min_strong_cfar_db);
t.mode = modeName;
    t = push_history(t,true,groups(m).amf_db,groups(m).cfar_snr_db,groups(m).angle_deg, ...
        groups(m).reflection_count,globalFrame,groups(m),p);
    t.last_frame = globalFrame; t.last_group = groups(m);
    ts.tracks(end+1) = t;
trackDiag.birth_count = trackDiag.birth_count + 1;
end

% ---- confirmation --------------------------------------------------------
for k = 1:numel(ts.tracks)
    if ts.tracks(k).confirmed, continue; end
    win = min(numel(ts.tracks(k).hit_history),p.track.group_confirm_window);
if win == 0, continue;
end
    recent = ts.tracks(k).hit_history(end-win+1:end);
    if nnz(recent) < p.track.group_confirm_hits, continue; end
    st = track_statistics(ts.tracks(k));
relax = 0;
    if ts.tracks(k).strong_hits >= 1
        relax = get_default_field(p.track,'strong_evidence_bonus',0);
    end
if st.mean_amf   < p.track.min_confirmation_mean_amf_db - relax, continue;
end
if st.mean_cfar  < p.track.min_confirmation_mean_cfar_db, continue;
end
if st.angle_support < p.track.min_confirmation_angle_support, continue;
end
if st.angle_std     > p.track.max_confirmation_angle_std_deg, continue;
end
    ts.tracks(k).confirmed = true;
    ts.tracks(k).status = 'confirmed';
trackDiag.confirmed_transitions = trackDiag.confirmed_transitions + 1;
end

% ---- pruning, capacity and duplicate merging -----------------------------
keep = true(1,numel(ts.tracks));
for k = 1:numel(ts.tracks)
limit = p.track.group_max_missed;
    if ~ts.tracks(k).confirmed
        limit = min(limit,get_default_field(p.track,'provisional_max_missed',limit));
    end
    if ts.tracks(k).missed > limit, keep(k) = false; end
end
trackDiag.pruned_count = nnz(~keep);
ts.tracks = ts.tracks(keep);
ts.tracks = merge_duplicate_tracks(ts.tracks,p);
trackDiag.merged_duplicate_count = numel(keep) - trackDiag.pruned_count - numel(ts.tracks);

ts.frame = globalFrame;
trackDiag.post_track_count = numel(ts.tracks);
trackDiag.post_confirmed_count = nnz([ts.tracks.confirmed]);
end

function t = push_history(t,hit,amf,cfar,ang,groupSize,frame,g,p)
t.hit_history(end+1)   = hit;
t.score_history(end+1) = amf;
t.cfar_history(end+1)  = cfar;
t.angle_history(end+1) = ang;
t.group_size_history(end+1) = groupSize;
t.history_frame(end+1) = frame;
t.history_groups(end+1) = g;
maxH = max(2,round(get_default_field(p.track,'max_history_frames',16)));
if numel(t.hit_history) > maxH
    cut = numel(t.hit_history) - maxH;
    t.hit_history(1:cut) = []; t.score_history(1:cut) = [];
    t.cfar_history(1:cut) = []; t.angle_history(1:cut) = [];
    t.group_size_history(1:cut) = []; t.history_frame(1:cut) = [];
    t.history_groups(1:cut) = [];
end
end

function st = track_statistics(t)
hit = logical(t.hit_history);
st = struct('support_fraction',0,'mean_amf',-Inf,'mean_cfar',-Inf, ...
    'angle_support',0,'angle_std',Inf,'hits',t.hits,'hard_hits',t.hard_hits, ...
    'strong_hits',t.strong_hits,'reflection_count',1,'unique_frames',0);
if isempty(hit), return; end
st.support_fraction = mean(hit);
s = t.score_history(hit); s = s(isfinite(s));
c = t.cfar_history(hit);  c = c(isfinite(c));
a = t.angle_history(hit); a = a(isfinite(a));
if ~isempty(s), st.mean_amf = mean(s); end
if ~isempty(c), st.mean_cfar = mean(c); end
st.angle_support = numel(a)/max(nnz(hit),1);
st.angle_std = angle_spread(a);
gs = t.group_size_history(hit);
if ~isempty(gs), st.reflection_count = max(1,round(mean(gs))); end
st.unique_frames = numel(unique(t.history_frame(hit)));
end

function tracks = merge_duplicate_tracks(tracks,p)
if numel(tracks) < 2, return; end
keep = true(1,numel(tracks));
for i = 1:numel(tracks)
    if ~keep(i), continue; end
    for j = i+1:numel(tracks)
        if ~keep(j), continue; end
        if abs(tracks(i).range - tracks(j).range) <= p.track.duplicate_range_m && ...
           abs(tracks(i).velocity - tracks(j).velocity) <= p.track.duplicate_velocity_mps && ...
           angle_close(tracks(i).angle_deg,tracks(j).angle_deg,p.track.duplicate_angle_deg)
            if track_rank(tracks(j)) > track_rank(tracks(i))
                tracks(i) = absorb_track(tracks(j),tracks(i)); keep(j) = false;
            else
                tracks(i) = absorb_track(tracks(i),tracks(j)); keep(j) = false;
            end
        end
    end
end
tracks = tracks(keep);
end

function r = track_rank(t)
r = double(t.confirmed)*1e6 + t.hits*100 + t.strong_hits*10 + t.hard_hits;
end

function a = absorb_track(a,b)
a.hits = max(a.hits,b.hits);
a.hard_hits = max(a.hard_hits,b.hard_hits);
a.strong_hits = max(a.strong_hits,b.strong_hits);
a.confirmed = a.confirmed || b.confirmed;
if a.confirmed, a.status = 'confirmed'; end
a.missed = min(a.missed,b.missed);
end

% =========================================================================
% Object formation from tracks
% =========================================================================
function objects = extract_group_objects(ts,p,globalFrame,modeName)
%EXTRACT_GROUP_OBJECTS  Promote confirmed tracks that carry enough evidence.
%
%   Track confirmation is a statement about persistence. Object existence is a
%   stricter statement that also requires accumulated detection quality, so a
%   track that survives on marginal hits is not automatically an object. Three
%   promotion routes exist:
%
%     * the primary gate, on hits, support fraction, mean matched-filter and
%       CFAR evidence, and angular support and spread;
%     * a recovery gate for tracks whose evidence per frame is lower but whose
%       persistence is much higher, which is the signature of a real but weak
%       target rather than a fluctuating false alarm;
%     * a persistent-strong gate for well-established tracks passing through a
%       temporary miss.
%
%   Near the end of the processed range the gates tighten and the recovery
%   routes are withdrawn, because the range walls are where sidelobe and
%   wrap-around artefacts concentrate.

objects = empty_object_array();
if isempty(ts) || ~isfield(ts,'tracks'), return; end
isStationary = strcmp(modeName,'stationary');
edgeRange = p.track.range_edge_fraction*p.R_max;

for k = 1:numel(ts.tracks)
    t = ts.tracks(k);
if ~t.confirmed, continue;
end
    st = track_statistics(t);
isEdge = t.range >= edgeRange;

    if isStationary
        gate = struct('hits',p.track.stationary_group_final_min_hits, ...
            'support',p.track.stationary_group_final_min_support, ...
            'amf',p.track.stationary_group_final_mean_amf_db, ...
            'cfar',p.track.stationary_group_final_mean_cfar_db, ...
            'angsup',p.track.stationary_group_final_angle_support, ...
            'angstd',p.track.stationary_group_final_angle_std_deg);
    else
        gate = struct('hits',p.track.group_final_min_hits, ...
            'support',p.track.group_final_min_support, ...
            'amf',p.track.group_final_mean_amf_db, ...
            'cfar',p.track.group_final_mean_cfar_db, ...
            'angsup',p.track.group_final_angle_support, ...
            'angstd',p.track.group_final_angle_std_deg);
    end

    accept = st.hits >= gate.hits && st.support_fraction >= gate.support && ...
             st.mean_amf >= gate.amf && st.mean_cfar >= gate.cfar && ...
st.angle_support >= gate.angsup && st.angle_std <= gate.angstd;
    route = 'primary';

    if ~accept && ~isEdge
        if st.hits >= p.track.group_recovery_min_hits && ...
           st.support_fraction >= p.track.group_recovery_min_support && ...
           st.mean_amf >= p.track.group_recovery_mean_amf_db && ...
           st.mean_cfar >= p.track.group_recovery_mean_cfar_db && ...
           st.angle_support >= p.track.group_recovery_angle_support && ...
           st.angle_std <= p.track.group_recovery_angle_std_deg
            accept = true; route = 'recovery';
        elseif st.hits >= p.track.persistent_recovery_min_hits && ...
               st.support_fraction >= p.track.persistent_recovery_min_support && ...
               st.hard_hits >= p.track.persistent_recovery_min_hard_hits && ...
               st.mean_amf >= p.track.persistent_recovery_mean_amf_db && ...
               st.angle_support >= p.track.persistent_recovery_angle_support && ...
               st.angle_std <= p.track.persistent_recovery_angle_std_deg && ...
               t.missed <= p.track.persistent_recovery_max_missed
            accept = true; route = 'persistent';
        end
    end

    if isEdge
        accept = st.hits >= p.track.edge_min_hits && ...
                 st.strong_hits >= p.track.edge_min_strong_hits && ...
                 st.mean_amf >= p.track.edge_min_mean_amf_db && ...
st.mean_cfar >= p.track.edge_min_mean_cfar_db;
        route = 'edge';
    end
if ~accept, continue;
end

    o = empty_object();
o.id = t.id;
o.range = t.range;
o.velocity = t.velocity;
o.angle_deg = t.angle_deg;
if isStationary, o.velocity = 0;
end
    o.x_pos = o.range*sind(o.angle_deg);
    o.y_pos = o.range*cosd(o.angle_deg);
o.score_db = st.mean_amf;
o.amf_db = st.mean_amf;
o.cfar_snr_db = st.mean_cfar;
o.hits = st.hits;
o.hard_hits = st.hard_hits;
o.confirmed = true;
o.missed = t.missed;
    o.source = ['group_tracker:' modeName ':' route];
    o.track_status = t.status;
    % A track that did not associate this frame is reported at its Kalman
    % prediction rather than at a measurement. That is what a tracker is for,
    % but it is flagged so it can never be mistaken for a fresh detection.
    o.coasted = t.missed > 0;
    o.frames_since_measurement = t.missed;
o.is_edge_range = isEdge;
o.reflection_count = st.reflection_count;
o.measurement_count = st.hits;
    o.unique_frame_count = max(st.unique_frames,1);
    o.extent_range_m = get_default_field(t.last_group,'range_spread_m',0);
    o.extent_cross_range_m = o.range*abs(sind(get_default_field(t.last_group,'angle_spread_deg',0)));
    objects(end+1) = o;
end
end

% =========================================================================
% Track-before-detect integration
% =========================================================================
function ev = build_tbd_evidence(Pmove,Pref,noiseMap,thrMap,cinfo,verified,p)
%BUILD_TBD_EVIDENCE  Weak-cell evidence with its own local reference window.
%
%   The dynamic-programming branch is given a normalised power for every weak
%   candidate. That normalisation uses a reference window sized independently
%   of the CFAR window, because the two stages answer different questions: the
%   CFAR window is tuned to protect a single-frame false-alarm rate, while the
%   TBD reference only needs a stable local mean over which to integrate.

ev = struct([]);
if isempty(cinfo) || ~isfield(cinfo,'weak_candidates') || isempty(cinfo.weak_candidates)
return;
end
wc = cinfo.weak_candidates;
S = zeros(size(Pref,1)+1,size(Pref,2)+1);
S(2:end,2:end) = cumsum(cumsum(Pref,1),2);
[Nr,Nd] = size(Pref);
RR = round(get_default_field(p.tbd,'reference_range_radius',8));
RV = round(get_default_field(p.tbd,'reference_velocity_radius',8));
GR = round(get_default_field(p.tbd,'reference_guard_range',2));
GV = round(get_default_field(p.tbd,'reference_guard_velocity',2));

out = repmat(tbd_evidence_template(),0,1);
for i = 1:numel(wc)
    r = wc(i).r_bin; d = wc(i).d_bin;
    r1 = max(1,r-RR); r2 = min(Nr,r+RR);
    d1 = max(1,d-RV); d2 = min(Nd,d+RV);
    g1 = max(1,r-GR); g2 = min(Nr,r+GR);
    h1 = max(1,d-GV); h2 = min(Nd,d+GV);
    outer = box_sum(S,r1,r2,d1,d2); nOuter = (r2-r1+1)*(d2-d1+1);
    inner = box_sum(S,g1,g2,h1,h2); nInner = (g2-g1+1)*(h2-h1+1);
    n = max(nOuter-nInner,1);
    ref = max((outer-inner)/n,realmin);
    if ref <= 0, ref = max(noiseMap(r,d),realmin); end

    q = tbd_evidence_template();
    q.range = wc(i).range; q.velocity = wc(i).velocity;
q.r_bin = r;
q.d_bin = d;
    q.power = Pmove(r,d); q.noise_power = ref;
    q.evidence_db = 10*log10(max(q.power,realmin)/ref);
    if logical(get_default_field(p.tbd,'range_compensation_enabled',false))
        q.range_prior_db = range_prior(q.range,p);
q.evidence_db = q.evidence_db + q.range_prior_db;
    end
    [own,ownAmf] = hard_point_owns(q.range,q.velocity,verified,p);
if own, continue;
end
q.amf_db = ownAmf;
q.is_hard = false;
    out(end+1) = q;
end
if ~isempty(out), ev = out; end
end

function [tf,amfDb] = hard_point_owns(r,v,verified,p)
%HARD_POINT_OWNS  Cost-based ownership test against verified hard points.
%   A verified detection already accounts for the energy at its location. Weak
%   evidence within a normalised distance of it is the same physical response,
%   not a second object, so it is withheld from the weak-target channel.
tf = false;
amfDb = NaN;
if isempty(verified) || ~logical(get_default_field(p.tbd,'suppress_near_hard',true)), return; end
sr = max(get_default_field(p.tbd,'match_range_sigma_m',1),eps);
sv = max(get_default_field(p.tbd,'match_velocity_sigma_mps',1),eps);
maxCost = get_default_field(p.tbd,'max_verified_match_cost',9.0);
minAmf  = get_default_field(p.tbd,'near_hard_suppress_amf_db',-Inf);
best = Inf;
for i = 1:numel(verified)
    c = ((r - verified(i).range)/sr)^2 + ((v - verified(i).velocity)/sv)^2;
    if c < best, best = c; amfDb = verified(i).amf_db; end
    if c <= maxCost && verified(i).amf_db >= minAmf, tf = true; return; end
end
end

function db = range_prior(r,p)
start = get_default_field(p.tbd,'range_compensation_start_fraction',0.7)*p.R_max;
maxDb = get_default_field(p.tbd,'range_compensation_max_db',6);
if r <= start, db = 0;
return;
end
db = maxDb*min((r-start)/max(p.R_max-start,eps),1);
end

function paths = merge_tbd_paths(dpPaths,cohPaths,p)
%MERGE_TBD_PATHS  Union of the two weak-target channels.
%   Each branch contributes independently. A branch returning nothing must
%   never remove the other branch's trajectories.
paths = repmat(empty_tbd_path(),0,1);
for i = 1:numel(dpPaths)
    paths(end+1) = normalize_tbd_path(dpPaths(i),'dp');
end
for i = 1:numel(cohPaths)
    paths(end+1) = normalize_tbd_path(cohPaths(i),'coherent');
end
% Suppress duplicates between the two branches at the object-fusion gate.
keep = true(1,numel(paths));
for i = 1:numel(paths)
    if ~keep(i), continue; end
    for j = i+1:numel(paths)
        if ~keep(j), continue; end
        if fusion_cost(paths(i).final_range,paths(i).final_velocity, ...
                       paths(j).final_range,paths(j).final_velocity,p) <= ...
           get_default_field(p.tbd,'object_fusion_gate',2.5)
            if paths(j).score > paths(i).score, paths(i) = paths(j); end
            keep(j) = false;
        end
    end
end
paths = paths(keep);
end

function q = normalize_tbd_path(src,branch)
q = empty_tbd_path();
f = fieldnames(src);
for k = 1:numel(f)
    if isfield(q,f{k}), q.(f{k}) = src.(f{k}); end
end
q.branch = branch;
if ~isfinite(q.final_range) && ~isempty(q.ranges), q.final_range = q.ranges(end); end
if ~isfinite(q.final_velocity) && ~isempty(q.velocities), q.final_velocity = q.velocities(end); end
if ~isfinite(q.final_angle), q.final_angle = get_default_field(src,'angle_deg',NaN); end
end

function paths = validate_tbd_path_angles(paths,p)
%VALIDATE_TBD_PATH_ANGLES  Reject trajectories that fan out in bearing.
%   A path can be perfectly consistent in range and Doppler and still be an
%   artefact. Requiring the bearing to hold together across the path removes
%   sidelobe and multipath replicas that a purely kinematic test accepts.
keep = true(1,numel(paths));
maxStd = get_default_field(p.tbd,'path_angle_max_std_deg',8.0);
for i = 1:numel(paths)
    a = paths(i).angles; a = a(isfinite(a));
    if numel(a) >= 2
        s = angle_spread(a);
        paths(i).angle_std = s;
        if s > maxStd, keep(i) = false; end
    end
end
paths = paths(keep);
end

function objects = paths_to_objects(paths,p,existing,frame_data)
%PATHS_TO_OBJECTS  Promote weak-target trajectories that clear their gates.
%
%   A trajectory carries range and Doppler evidence. It does not, by itself,
%   carry a bearing: the dynamic-programming branch integrates power in the
%   range-Doppler plane and never forms a spatial estimate. An object report
%   without a bearing is not a located object, so rather than substituting
%   boresight, the bearing is measured here from the aperture at the path's own
%   final state. A path whose bearing does not resolve is not promoted.
objects = empty_object_array();
minAmf = get_default_field(p.tbd,'path_promotion_min_amf_db',2.5);
farStart = get_default_field(p.tbd,'far_range_recovery_start_m',0.75*p.R_max);
farAmf   = get_default_field(p.tbd,'far_range_recovery_min_integrated_amf_db',5.0);
trajOn   = logical(get_default_field(p.tbd,'trajectory_recovery_enabled',true));
trajAmf  = get_default_field(p.tbd,'trajectory_recovery_min_integrated_amf_db',5.0);
trajSup  = get_default_field(p.tbd,'trajectory_recovery_min_support_fraction',0.60);

for i = 1:numel(paths)
    q = paths(i);
if q.length < p.tbd.min_path_frames, continue;
end
if q.support_fraction < p.tbd.min_path_support_fraction, continue;
end

score = q.score;
    if strcmp(q.branch,'coherent'), score = max(score,q.coherent_score_db); end
promote = score >= p.tbd.path_promotion_score;
    if strcmp(q.branch,'coherent')
promote = q.coherent_score_db >= p.tbd.coherent.path_promotion_score_db;
    end
    if ~promote && trajOn && isfinite(q.mean_amf) && ...
       q.mean_amf >= trajAmf && q.support_fraction >= trajSup
promote = true;
    end
if ~promote, continue;
end

    % Trajectory-level template verification. The per-frame matched-filter
    % threshold is relaxed to the configured trajectory false-alarm rate,
    % because the path already supplies temporal evidence the single-frame
    % test does not have; it is not removed.
    pathAmfFloor = max(10*log10(-log(max(min(get_default_field(p.tbd,'path_amf_pfa',0.1),0.5),1e-12))), ...
                       get_default_field(p.tbd,'path_min_amf_db',-Inf));
    if isfinite(q.mean_amf)
        if q.mean_amf < max(minAmf,pathAmfFloor), continue; end
    elseif ~strcmp(q.branch,'coherent')
        continue;
    end
    % Bearing. The coherent branch already votes on angle along the path; the
    % dynamic-programming branch does not, so it is measured once here at the
    % path's final state. Range and Doppler consistency alone cannot separate a
    % real trajectory from a sidelobe or multipath replica.
    theta = q.final_angle;
    if isfinite(theta) && q.angle_support > 0
        % This path voted on bearing across its frames; require the votes to
        % have covered enough of them to mean anything.
        if q.angle_support < get_default_field(p.tbd,'min_promotion_angle_support',0.50)
            continue;
        end
    else
        theta = measure_path_bearing(q,frame_data,p);
    end
    if ~isfinite(theta), continue; end
    q.final_angle = theta;
    if q.final_range >= farStart && (~isfinite(q.mean_amf) || q.mean_amf < farAmf), continue; end
    if object_already_owned(q,existing,p), continue; end

    o = empty_object();
o.id = 0;
o.range = q.final_range;
o.velocity = q.final_velocity;
o.angle_deg = q.final_angle;
    % A trajectory with no angular evidence is not a located object. Filling
    % the bearing in with boresight turns an unresolved path into a confident
    % report pointing straight ahead, which is how a weak-target branch
    % manufactures a false object at zero degrees.
    if ~isfinite(o.angle_deg), continue; end
    o.x_pos = o.range*sind(o.angle_deg);
    o.y_pos = o.range*cosd(o.angle_deg);
o.score_db = score;
o.amf_db = q.mean_amf;
    % Keep invalid numerical explosions out of reported diagnostics.
o.cfar_snr_db = max(min(10*log10(max(q.mean_z,realmin)),120),-60);
o.hits = q.length;
o.tbd_hits = q.length;
o.confirmed = true;
    o.source = ['tbd:' q.branch];
    o.track_status = 'tbd';
o.measurement_count = q.length;
o.unique_frame_count = q.length;
o.is_edge_range = o.range >= p.track.range_edge_fraction*p.R_max;
    objects(end+1) = o;
end
end

function theta = measure_path_bearing(q,frame_data,p)
%MEASURE_PATH_BEARING  Spatial estimate at a weak-target path's final state.
%   Forms the virtual aperture at the path's own range bin and velocity, so the
%   TDM Doppler phase is removed for the hypothesis the path actually asserts,
%   and accepts the high-resolution peak only if it is prominent enough to be a
%   source rather than a ripple in a flat spectrum. Runs once per candidate
%   path, not per cell, so the cost is a handful of apertures per frame.
theta = NaN;
if isempty(frame_data), return; end
fi = numel(frame_data);
if ~isempty(q.frame_indices)
    fi = min(max(q.frame_indices(end),1),numel(frame_data));
end
[cube,~] = unpack_frame(frame_data{fi});
if isempty(cube), return; end
try
    [~,rb] = min(abs(p.range_axis - q.final_range));
    va = tdm_virtual_aperture(cube,p,rb,q.final_velocity,false);
    [~,~,~,~,ai] = music_aoa_estimator(va,p,q.final_range);
    promFloor = get_default_field(p.tbd,'path_angle_min_prominence_db',0.5);
    if ai.music_peak_prominence_db >= promFloor && ai.snapshot_sufficient
        theta = ai.music_peak_angle_deg;
    elseif ai.beamformer_peak_prominence_db >= promFloor
        theta = ai.beamformer_peak_angle_deg;
    end
catch
    theta = NaN;
end
if isfinite(theta) && abs(theta) > p.az_span, theta = NaN; end
end

function tf = object_already_owned(q,existing,p)
tf = false;
minHits = get_default_field(p.tbd,'exclusion_track_min_hits',2);
gate = get_default_field(p.tbd,'object_fusion_gate',2.5);
rangeGate = get_default_field(p.tbd,'object_fusion_range_m',2.0);
for i = 1:numel(existing)
    if existing(i).hits < minHits, continue; end
    if fusion_cost(existing(i).range,existing(i).velocity,q.final_range,q.final_velocity,p) <= gate
        tf = true;
        return;
    end
    if abs(existing(i).range - q.final_range) <= rangeGate
        tf = true;
        return;
    end
end
end

function objects = fuse_objects(groupObjects,tbdObjects,p)
objects = groupObjects(:);
for i = 1:numel(tbdObjects)
dup = false;
    for j = 1:numel(objects)
        if abs(objects(j).range - tbdObjects(i).range) <= p.tbd.object_fusion_range_m && ...
           abs(objects(j).velocity - tbdObjects(i).velocity) <= p.tbd.object_fusion_velocity_mps && ...
           angle_close(objects(j).angle_deg,tbdObjects(i).angle_deg,p.tbd.object_fusion_angle_deg)
dup = true;
break;
        end
    end
    if ~dup, objects(end+1) = tbdObjects(i); end
end
% Final duplicate pass across the union.
keep = true(1,numel(objects));
for i = 1:numel(objects)
    if ~keep(i), continue; end
    for j = i+1:numel(objects)
        if ~keep(j), continue; end
        if abs(objects(i).range - objects(j).range) <= p.track.duplicate_range_m && ...
           abs(objects(i).velocity - objects(j).velocity) <= p.track.duplicate_velocity_mps && ...
           angle_close(objects(i).angle_deg,objects(j).angle_deg,p.track.duplicate_angle_deg)
            if objects(j).score_db > objects(i).score_db, objects(i) = objects(j); end
            keep(j) = false;
        end
    end
end
objects = objects(keep);
end

function objects = relabel_objects(objects)
for i = 1:numel(objects), objects(i).label = i; end
end

% =========================================================================
% Live-mode support
% =========================================================================
function state = push_tbd_history(state,frameData,evidence,frameMaps,globalFrame,p)
h = state.tbd_history;
h.frame_data{end+1} = frameData;
h.evidence{end+1} = evidence;
h.frame_info{end+1} = frameMaps;
h.global_frames(end+1) = globalFrame;
maxH = max(2,round(get_default_field(p.track,'max_history_frames',16)));
if numel(h.frame_data) > maxH
    cut = numel(h.frame_data) - maxH;
    h.frame_data(1:cut) = []; h.evidence(1:cut) = [];
    h.frame_info(1:cut) = []; h.global_frames(1:cut) = [];
end
state.tbd_history = h;
end

function [paths,dpInfo,cohInfo] = live_tbd_pass(state,p)
%LIVE_TBD_PASS  Causal weak-target search over the retained history.
%   Live mode runs the same two algorithms as batch mode over a bounded window
%   of past frames. No future frame is ever consulted, and no reduced search
%   budget is substituted.
h = state.tbd_history;
paths = repmat(empty_tbd_path(),0,1);
dpInfo = empty_live_tbd_info();
cohInfo = struct('enabled',false,'accepted_path_count',0);
if numel(h.evidence) < p.tbd.min_path_frames, return; end
% Search a bounded sliding window rather than the whole retained buffer, so
% per-frame cost reaches a ceiling instead of climbing with frame number.
w = max(p.tbd.min_path_frames,round(get_default_field(p.tbd,'live_window_frames',8)));
n = numel(h.evidence);
sel = max(1,n-w+1):n;
[dpPaths,dpInfo] = dynamic_programming_tbd(h.evidence(sel),p);
[cohPaths,cohInfo] = coherent_tbd_detector(h.frame_data(sel),p,h.frame_info(sel));
paths = merge_tbd_paths(dpPaths,cohPaths,p);
end

function maps = collect_frame_maps(stageData,tbdEvidence)
maps = cell(numel(stageData),1);
for f = 1:numel(stageData)
    s = stageData{f};
    if isstruct(s) && isfield(s,'rd_power_moving')
        maps{f} = struct('rd_power_moving',s.rd_power_moving, ...
            'rd_power_reference',get_default_field(s,'rd_power_reference',[]), ...
            'noise_map',get_default_field(s,'noise_map',[]));
    else
        maps{f} = struct('rd_power_moving',[],'rd_power_reference',[],'noise_map',get_default_field(s,'noise_map',[]));
    end
end
end

% =========================================================================
% Small helpers
% =========================================================================
function [cleanCube,rawCube] = unpack_frame(f)
cleanCube = []; rawCube = [];
if isstruct(f)
    if isfield(f,'clean'), cleanCube = f.clean; end
    if isfield(f,'raw'), rawCube = f.raw; end
    if isempty(cleanCube), cleanCube = rawCube; end
    if isempty(rawCube), rawCube = cleanCube; end
else
cleanCube = f;
rawCube = f;
end
end

function tf = live_stop_requested()
tf = false;
try
    tf = isappdata(0,'FMCW_GUI_STOP') && getappdata(0,'FMCW_GUI_STOP');
catch
tf = false;
end
end

function s = box_sum(S,r1,r2,c1,c2)
s = S(r2+1,c2+1) - S(r1,c2+1) - S(r2+1,c1) + S(r1,c1);
end

function Sinv = inv_2x2(S)
d = S(1,1)*S(2,2) - S(1,2)*S(2,1);
if abs(d) < eps, Sinv = pinv(S); return; end
Sinv = [S(2,2) -S(1,2); -S(2,1) S(1,1)]/d;
end

function assign = greedy_assign(cost)
[nT,nM] = size(cost);
assign = zeros(1,nT);
if nT == 0 || nM == 0, return;
end
usedM = false(1,nM);
[vals,idx] = sort(cost(:),'ascend');
for q = 1:numel(vals)
    if ~isfinite(vals(q)), break; end
    [k,m] = ind2sub([nT nM],idx(q));
    if assign(k) ~= 0 || usedM(m), continue; end
    assign(k) = m; usedM(m) = true;
end
end

function n = count_origin(framePoints,name)
n = 0;
for f = 1:numel(framePoints)
    d = framePoints{f};
    if isempty(d), continue; end
    n = n + nnz(strcmp({d.origin},name));
end
end

function n = count_group_mode(frameGroups,name)
n = 0;
for f = 1:numel(frameGroups)
    g = frameGroups{f};
    if isempty(g), continue; end
    n = n + nnz(strcmp({g.mode},name));
end
end

function td = merge_track_diag(a,b)
td = empty_track_diag();
f = fieldnames(td);
for k = 1:numel(f)
    td.(f{k}) = get_default_field(a,f{k},0) + get_default_field(b,f{k},0);
end
end

function k = clamp_index(v,n)
if ~isfinite(v), v = 1; end
k = max(1,min(n,round(v)));
end

function s = spread(v)
v = v(isfinite(v));
if numel(v) < 2, s = 0; else, s = max(v)-min(v); end
end

function s = angle_spread(a)
a = a(isfinite(a));
if numel(a) < 2, s = 0; return; end
mu = atan2d(mean(sind(a)),mean(cosd(a)));
s = sqrt(mean(wrap_angle(a-mu).^2));
end

function m = circular_mean(a,w)
ok = isfinite(a);
if ~any(ok), m = NaN; return; end
a = a(ok);
if nargin < 2 || isempty(w), w = ones(size(a)); else, w = w(ok); end
if sum(w) <= 0, w = ones(size(a)); end
m = atan2d(sum(w.*sind(a)),sum(w.*cosd(a)));
end

function tf = angle_close(a,b,gate)
if ~isfinite(a) || ~isfinite(b), tf = true; return; end
tf = abs(wrap_angle(a-b)) <= gate;
end

function a = wrap_angle(a), a = mod(a+180,360)-180; end

function c = fusion_cost(r1,v1,r2,v2,p)
%FUSION_COST  Normalised range/velocity distance used for duplicate tests.
%   Expressed in gate units so one scalar threshold governs fusion regardless
%   of the physical resolution of the current radar design.
sr = max(get_default_field(p.tbd,'object_fusion_range_m',1),eps);
sv = max(get_default_field(p.tbd,'object_fusion_velocity_mps',1),eps);
c = sqrt(((r1-r2)/sr)^2 + ((v1-v2)/sv)^2);
end

% =========================================================================
% Templates
% =========================================================================
function s = empty_state()
s = struct('moving',empty_track_state(),'stationary',empty_track_state(), ...
    'tracks',empty_track_array(),'frame',0, ...
    'tbd_history',struct('frame_data',{{}},'evidence',{{}},'frame_info',{{}},'global_frames',zeros(1,0)));
end

function s = empty_track_state()
s = struct('tracks',empty_track_array(),'frame',0,'next_id',1);
end

function t = empty_track()
t = struct('id',0,'x',zeros(2,1),'P',eye(2),'Pa',9,'range',0,'velocity',0,'angle_deg',NaN, ...
    'missed',0,'hits',0,'hard_hits',0,'strong_hits',0,'hit_history',false(1,0), ...
    'score_history',zeros(1,0),'cfar_history',zeros(1,0),'angle_history',nan(1,0), ...
    'group_size_history',zeros(1,0),'confirmed',false,'status','tentative','mode','moving', ...
    'last_frame',0,'last_group',group_template(),'history_frame',zeros(1,0), ...
    'history_groups',group_array_template());
end
function a = empty_track_array(), a = repmat(empty_track(),0,1); end

function d = detection_template()
d = struct('range',0,'velocity',0,'angle_deg',NaN,'r_bin',1,'d_bin',1, ...
    'range_bin_center',0,'velocity_bin_center',0,'subbin_refined',false, ...
    'cfar_snr_db',-Inf,'cfar_threshold',0,'cfar_noise',0,'cfar_mode','', ...
    'amf_stat',0,'amf_db',-Inf,'amf_class','rejected','amf_threshold_db',NaN, ...
    'angle_fft_deg',NaN,'aoa_fft_deg',NaN,'angle_music_deg',NaN,'music_peak_db',-Inf, ...
    'music_peak_prominence_db',-Inf,'fft_peak_prominence_db',-Inf,'angle_difference_deg',NaN, ...
    'music_theta',zeros(1,0),'music_spectrum',zeros(1,0),'fft_theta',zeros(1,0), ...
    'fft_spectrum',zeros(1,0),'music_info',struct(),'power',0,'snr_db',-Inf, ...
    'range_offset_bins',0,'doppler_offset_bins',0,'origin','cfar','is_hard',true, ...
    'tbd_snr_db',-Inf,'velocity_raw',NaN,'tdm_alias_offset',0,'tdm_alias_score',NaN, ...
    'tdm_alias_mode','','tdm_alias_angle_preview',NaN,'tdm_alias_coherence',NaN, ...
    'quality_score_db',-Inf,'confidence',0,'is_cluster_supported',false,'aoa_quality_db',-Inf, ...
    'x_pos',NaN,'y_pos',NaN, ...
    'gs_stat',0,'gs_db',-Inf,'gs_valid',false,'gs_interference_angle_deg',NaN, ...
    'gs_target_angle_deg',NaN,'gs_inr_db',-Inf,'gs_interference_detected',false, ...
    'gs_noise_power',0,'gs_r_bin',1,'gs_enabled',false,'gs_rescue',false,'gs_stage',false);
end
function a = detection_array_template(), a = repmat(detection_template(),0,1); end

function g = group_template()
g = struct('range',0,'velocity',0,'angle_deg',NaN,'x_pos',NaN,'y_pos',NaN, ...
    'range_spread_m',0,'velocity_spread_mps',0,'angle_spread_deg',0, ...
    'reflection_count',1,'member_indices',1,'cfar_snr_db',-Inf,'amf_db',-Inf, ...
    'quality_db',-Inf,'is_hard',true,'mode','moving');
end
function a = group_array_template(), a = repmat(group_template(),0,1); end

function o = empty_object()
o = struct('id',0,'label',0,'range',0,'velocity',0,'angle_deg',NaN,'score_db',-Inf, ...
    'hits',0,'confirmed',false,'missed',0,'x_pos',NaN,'y_pos',NaN,'cfar_snr_db',-Inf, ...
    'amf_db',-Inf,'hard_hits',0,'tbd_hits',0,'source','group_tracker', ...
    'source_is_placeholder',false,'track_status','inactive','is_edge_range',false, ...
    'extent_range_m',0,'extent_cross_range_m',0,'reflection_count',1, ...
    'coasted',false,'frames_since_measurement',0, ...
    'measurement_count',1,'unique_frame_count',1);
end
function a = empty_object_array(), a = repmat(empty_object(),0,1); end

function q = tbd_evidence_template()
q = struct('range',0,'velocity',0,'r_bin',1,'d_bin',1,'power',0,'noise_power',1, ...
    'evidence_db',-Inf,'range_prior_db',0,'angle_deg',NaN,'amf_db',NaN, ...
    'is_hard',false,'origin','tbd');
end

function q = empty_tbd_path()
q = struct('frame_indices',zeros(1,0),'candidate_indices',zeros(1,0),'length',0, ...
    'score',-Inf,'support_fraction',0,'ranges',zeros(1,0),'velocities',zeros(1,0), ...
    'angles',zeros(1,0),'z',zeros(1,0),'evidence',zeros(1,0),'amf_db',zeros(1,0), ...
    'hard_mask',false(1,0),'mean_llr',0,'sum_llr',0,'mean_z',0,'angle_support',0, ...
    'angle_std',NaN,'hard_support',0,'mean_amf',NaN,'final_range',NaN, ...
    'final_velocity',NaN,'final_angle',NaN,'confirmation_score',-Inf, ...
    'coherent_score_db',-Inf,'angle_std_deg',NaN,'angle_deg',NaN,'branch','dp');
end

function f = empty_frame_info()
f = struct('cfar_info',struct(),'amf_info',struct(),'tdm_info',struct(), ...
    'interference_info',struct(),'hard_cfar_count',0,'amf_verified_count',0, ...
    'group_count',0,'accepted_measurement_count',0);
end

function d = empty_track_diag()
d = struct('measurement_count',0,'matched_count',0,'birth_count',0, ...
    'confirmed_transitions',0,'pruned_count',0,'post_track_count',0, ...
    'post_confirmed_count',0,'merged_duplicate_count',0);
end

function s = empty_stage_data()
s = struct('cfar_moving',detection_array_template(),'amf_moving',detection_array_template(), ...
    'rd_power_moving',[],'rd_power_reference',[],'noise_map',[],'stationary_hits',detection_array_template(), ...
    'amf_stationary',detection_array_template(),'groups',group_array_template(), ...
    'gs_moving',struct(),'gs_stationary',struct());
end

function info = empty_live_tbd_info()
info = struct('enabled',false,'frame_count',0,'candidate_counts',zeros(1,0), ...
    'total_candidates',0,'path_count',0,'accepted_path_count',0, ...
    'suppressed_candidate_count',0,'max_terminal_score',-Inf,'path_scores',zeros(1,0), ...
    'path_lengths',zeros(1,0),'path_support',zeros(1,0), ...
    'method','Gamma GLR dynamic-programming TBD');
end

function info = empty_info()
info = struct('frame_count',0,'frame_detections',{{}},'frame_groups',{{}},'frame_info',{{}}, ...
    'track_diagnostics',{{}},'stage_data',{{}},'global_frame',0,'live_mode',false, ...
    'gs_interference_frames',0,'gs_rescued_count',0,'final_paper_processing',struct(), ...
    'final_rd_power_clean',[],'final_rd_aux',struct(), ...
    'final_stationary_range_angle_power',[],'final_stationary_range_power',[], ...
    'moving_track_count',0,'stationary_track_count',0,'group_track_count',0, ...
    'moving_point_count',0,'stationary_point_count',0,'moving_group_count',0, ...
    'stationary_group_count',0,'group_measurement_count',0,'measurement_count',0, ...
    'confirmed_count',0,'gnn_confirmed_count',0,'gnn_confirmed_track_count',0, ...
    'track_count',0,'tbd',struct(),'live_coherent_tbd_info',struct(), ...
    'live_tbd_paths',[],'candidate_trajectory_count',0,'tbd_confirmed_count',0, ...
    'tbd_state_cells',0,'final_object_count',0,'coasted_object_count',0,'live_hard_objects',[], ...
    'live_tbd_objects',[],'live_hard_points',[],'live_display_objects',[], ...
    'final_object_display_objects',[],'live_tbd_object_error','', ...
    'live_tbd_info',struct(),'hypotheses',[],'input_count',0,'output_count',0, ...
    'removed',0,'rejected_low_snr',0,'rejected_invalid_angle',0, ...
    'estimated_cardinality',0,'selection_mode','','detector','', ...
    'weak_candidates',[],'track_lifecycle',struct());
end
