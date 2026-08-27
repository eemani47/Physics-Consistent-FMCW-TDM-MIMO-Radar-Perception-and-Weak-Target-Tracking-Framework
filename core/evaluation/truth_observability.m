function obs = truth_observability(truth,stageData,frameGroups,objects,p)
%TRUTH_OBSERVABILITY  Stage attribution for missed truth and false objects.
%
%   Counting a miss tells you that something failed. It does not tell you what.
%   This module follows every true target through the pipeline and records the
%   deepest stage at which measurable evidence for it still existed, so a miss
%   can be attributed rather than merely tallied:
%
%     PRE_CFAR   energy is present in the range-Doppler field, but no
%                declaration was made - a threshold or normalisation issue
%     CFAR       a hard candidate existed but did not survive verification -
%                a spatial-consistency or covariance issue
%     AMF        a verified point existed but did not join a group - a
%                grouping-gate issue
%     GROUP      a group existed but never became a persistent object - a
%                tracking or object-existence issue
%     OBJECT     the target was reported
%
%   The same machinery runs in reverse for false objects, recording the
%   earliest stage at which each spurious report can first be seen. Together
%   these answer the two questions that matter when tuning a radar: where did
%   the target go, and where did the ghost come from.
%
%   This function is diagnostic only. It runs after object formation and its
%   output never re-enters the pipeline.

obs = struct('per_target',struct([]),'stage_counts',struct('pre_cfar',0,'cfar',0, ...
    'amf',0,'group',0,'object',0,'absent',0),'false_objects',struct([]), ...
    'false_stage_counts',struct('moving_cfar',0,'stationary_cfar',0,'moving_amf',0, ...
    'group',0,'track',0,'unattributed',0), ...
    'mechanism_counts',struct('split',0,'spurious',0));

truth = normalize_truth(truth);
nT = numel(truth);
Nf = numel(stageData);
if nT == 0, return;
end

gates = get_default_field(p,'obs',struct());
gR = get_default_field(gates,'range_gate_m',1.5);
gV = get_default_field(gates,'velocity_gate_mps',1.25);
gA = get_default_field(gates,'angle_gate_deg',5.0);
ggR = get_default_field(gates,'group_range_gate_m',2.0);
ggV = get_default_field(gates,'group_velocity_gate_mps',1.75);
ggA = get_default_field(gates,'group_angle_gate_deg',5.0);
preDb = get_default_field(gates,'pre_cfar_min_db',3.0);

per = repmat(target_record(),1,nT);
for ti = 1:nT
    rec = target_record();
rec.id = ti;
    rec.range = truth(ti).range;
    rec.velocity = truth(ti).velocity;
    rec.angle_deg = truth(ti).angle_deg;
rec.max_pre_cfar_db = -Inf;

    for f = 1:Nf
        s = stageData{f};
        if ~isstruct(s), continue; end
        % Pre-CFAR: local peak-to-background excess at the truth cell.
        Pm = get_default_field(s,'rd_power_moving',[]);
        Pr = get_default_field(s,'rd_power_reference',[]);
        if ~isempty(Pm)
            db = local_excess_db(Pm,Pr,rec.range,rec.velocity,p);
            rec.max_pre_cfar_db = max(rec.max_pre_cfar_db,db);
if db >= preDb, rec.pre_cfar_frames = rec.pre_cfar_frames + 1;
end
        end
        % Keep moving/stationary diagnostic arrays independent. Their runtime
        % structures may carry branch-specific fields, so concatenating them as
        % MATLAB struct arrays is not a valid operation. The attribution logic
        % only needs the per-branch match result and maximum evidence.
        cfMoving = as_array(get_default_field(s,'cfar_moving',[]));
        cfStationary = as_array(get_default_field(s,'stationary_hits',[]));
        if match_any(cfMoving,rec,gR,gV,gA,false) || ...
                match_any(cfStationary,rec,gR,gV,gA,false)
rec.cfar_frames = rec.cfar_frames + 1;
            rec.max_cfar_db = max(rec.max_cfar_db, ...
                max(max_matching_field(cfMoving,rec,gR,gV,gA,false,'cfar_snr_db'), ...
                    max_matching_field(cfStationary,rec,gR,gV,gA,false,'cfar_snr_db')));
        end
        amMoving = as_array(get_default_field(s,'amf_moving',[]));
        amStationary = as_array(get_default_field(s,'amf_stationary',[]));
        if match_any(amMoving,rec,gR,gV,gA,true) || ...
                match_any(amStationary,rec,gR,gV,gA,true)
rec.amf_frames = rec.amf_frames + 1;
            rec.max_amf_db = max(rec.max_amf_db, ...
                max(max_matching_field(amMoving,rec,gR,gV,gA,true,'amf_db'), ...
                    max_matching_field(amStationary,rec,gR,gV,gA,true,'amf_db')));
        end
        gr = as_array(get_default_field(s,'groups',[]));
        if isempty(gr) && numel(frameGroups) >= f, gr = as_array(frameGroups{f}); end
        if match_any(gr,rec,ggR,ggV,ggA,true)
rec.group_frames = rec.group_frames + 1;
            rec.max_group_quality_db = max(rec.max_group_quality_db, max_matching_field(gr,rec,ggR,ggV,ggA,true,'quality_db'));
        end
    end

    rec.object_matched = match_any(as_array(objects),rec,ggR,ggV,ggA,true);
    rec.deepest_stage = deepest(rec);
    obs.stage_counts.(lower(rec.deepest_stage)) = obs.stage_counts.(lower(rec.deepest_stage)) + 1;
    per(ti) = rec;
end
obs.per_target = per;

% ---- false-object attribution -------------------------------------------
matchedObj = false(1,numel(objects));
for ti = 1:nT
    for oi = 1:numel(objects)
        if matchedObj(oi), continue; end
        if within(objects(oi),per(ti),ggR,ggV,ggA,true)
            matchedObj(oi) = true; break;
        end
    end
end

fo = repmat(false_record(),0,1);
for oi = 1:numel(objects)
    if matchedObj(oi), continue; end
    r = false_record();
r.object_index = oi;
    r.range = objects(oi).range;
    r.velocity = objects(oi).velocity;
    r.angle_deg = objects(oi).angle_deg;
    r.source = get_default_field(objects(oi),'source','');
    probe = struct('range',r.range,'velocity',r.velocity,'angle_deg',r.angle_deg);
    for f = 1:Nf
        s = stageData{f};
        if ~isstruct(s), continue; end
        if match_any(as_array(get_default_field(s,'cfar_moving',[])),probe,gR,gV,gA,false)
r.seen_moving_cfar = true;
end
        if match_any(as_array(get_default_field(s,'stationary_hits',[])),probe,gR,gV,gA,false)
r.seen_stationary_cfar = true;
end
        if match_any(as_array(get_default_field(s,'amf_moving',[])),probe,gR,gV,gA,true)
r.seen_moving_amf = true;
end
        if match_any(as_array(get_default_field(s,'groups',[])),probe,ggR,ggV,ggA,true)
r.seen_group = true;
end
    end
    r.origin_stage = earliest(r);
    % Mechanism, which is a different question from stage and needs the
    % opposite correction. A report sitting inside the match gate of a truth
    % object that another report already claimed is the same physical response
    % counted twice: a resolution or fusion failure. A report with no truth
    % nearby is an invention. Tightening a detection threshold fixes the
    % second and does nothing for the first.
    r.mechanism = 'spurious';
    for ti = 1:nT
        dR = abs(per(ti).range - r.range);
        dV = abs(per(ti).velocity - r.velocity);
        dA = abs(mod(per(ti).angle_deg - r.angle_deg + 180,360) - 180);
        if dR <= ggR && dV <= ggV && dA <= ggA
            r.mechanism = 'split';
            r.nearest_truth_index = ti;
            r.nearest_truth_distance_m = dR;
            break;
        end
    end
    obs.mechanism_counts.(r.mechanism) = obs.mechanism_counts.(r.mechanism) + 1;
    obs.false_stage_counts.(r.origin_stage) = obs.false_stage_counts.(r.origin_stage) + 1;
    fo(end+1) = r;
end
obs.false_objects = fo;
end

% =========================================================================
function s = deepest(rec)
if rec.object_matched,       s = 'OBJECT';   return; end
if rec.group_frames > 0,     s = 'GROUP';    return; end
if rec.amf_frames > 0,       s = 'AMF';      return; end
if rec.cfar_frames > 0,      s = 'CFAR';     return; end
if rec.pre_cfar_frames > 0,  s = 'PRE_CFAR'; return; end
s = 'ABSENT';
end

function s = earliest(r)
if r.seen_moving_cfar,      s = 'moving_cfar';     return; end
if r.seen_stationary_cfar,  s = 'stationary_cfar'; return; end
if r.seen_moving_amf,       s = 'moving_amf';      return; end
if r.seen_group,            s = 'group';           return; end
if ~isempty(r.source),      s = 'track';           return; end
s = 'unattributed';
end

function db = local_excess_db(Pm,Pr,range,velocity,p)
%LOCAL_EXCESS_DB  Peak-to-background excess in a small window at the truth cell.
db = -Inf;
[~,rb] = min(abs(p.range_axis - range));
[~,db_] = min(abs(p.vel_axis - velocity));
[Nr,Nd] = size(Pm);
r1 = max(1,rb-1); r2 = min(Nr,rb+1);
d1 = max(1,db_-1); d2 = min(Nd,db_+1);
pk = max(Pm(r1:r2,d1:d2),[],'all');
if isempty(Pr) || ~isequal(size(Pr),size(Pm))
    bg = median(Pm(Pm>0),'omitnan');
else
    R1 = max(1,rb-12); R2 = min(Nr,rb+12);
    D1 = max(1,db_-12); D2 = min(Nd,db_+12);
    win = Pr(R1:R2,D1:D2);
    % Only cells that carry data may set the background. The map is held at
    % zero outside the processed band, and a window straddling that edge would
    % otherwise yield a zero background and an infinite reported excess.
    wv = win(isfinite(win) & win > 0);
    if isempty(wv)
        bg = NaN;
    else
        bg = median(wv);
    end
end
if isfinite(pk) && isfinite(bg) && bg > 0
    db = 10*log10(max(pk,realmin)/bg);
end
end

function tf = match_any(arr,rec,gR,gV,gA,useAngle)
tf = false;
for i = 1:numel(arr)
    if within(arr(i),rec,gR,gV,gA,useAngle), tf = true; return; end
end
end

function v = max_matching_field(arr,rec,gR,gV,gA,useAngle,field)
v = -Inf;
for i = 1:numel(arr)
    if within(arr(i),rec,gR,gV,gA,useAngle)
        v = max(v,get_default_field(arr(i),field,-Inf));
    end
end
end

function tf = within(a,rec,gR,gV,gA,useAngle)
tf = false;
if abs(get_default_field(a,'range',NaN) - rec.range) > gR, return; end
if abs(get_default_field(a,'velocity',NaN) - rec.velocity) > gV, return; end
if useAngle
    th = get_default_field(a,'angle_deg',NaN);
    if isfinite(th) && isfinite(rec.angle_deg)
        if abs(mod(th-rec.angle_deg+180,360)-180) > gA, return; end
    end
end
tf = true;
end

function a = as_array(x)
if isempty(x), a = repmat(struct('range',NaN,'velocity',NaN,'angle_deg',NaN),0,1); return; end
a = x(:);
end

function t = normalize_truth(truth)
t = repmat(struct('range',0,'velocity',0,'angle_deg',0),0,1);
if isempty(truth), return; end
if isnumeric(truth)
    for i = 1:size(truth,1)
        t(end+1) = struct('range',truth(i,1),'velocity',truth(i,2), ...
            'angle_deg',truth(i,min(4,size(truth,2))));
    end
return;
end
for i = 1:numel(truth)
    t(end+1) = struct('range',get_default_field(truth(i),'range', ...
        get_default_field(truth(i),'range0',NaN)), ...
        'velocity',get_default_field(truth(i),'velocity',NaN), ...
        'angle_deg',get_default_field(truth(i),'angle_deg',NaN));
end
end

function r = target_record()
r = struct('id',0,'range',NaN,'velocity',NaN,'angle_deg',NaN, ...
    'pre_cfar_frames',0,'cfar_frames',0,'amf_frames',0,'group_frames',0, ...
    'object_matched',false,'max_pre_cfar_db',-Inf,'max_cfar_db',-Inf, ...
    'max_amf_db',-Inf,'max_group_quality_db',-Inf,'deepest_stage','ABSENT');
end

function r = false_record()
r = struct('object_index',0,'range',NaN,'velocity',NaN,'angle_deg',NaN,'source','', ...
    'mechanism','spurious','nearest_truth_index',0,'nearest_truth_distance_m',Inf, ...
    'seen_moving_cfar',false,'seen_stationary_cfar',false,'seen_moving_amf',false, ...
    'seen_group',false,'origin_stage','unattributed');
end
