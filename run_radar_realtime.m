function run_radar_realtime()
%RUN_FMCW_REALTIME Causal frame-by-frame FMCW radar simulation.
% Each frame is generated, processed, tracked, and displayed before the next
% frame is acquired. The loop runs until the GUI requests STOP.
root=fileparts(mfilename('fullpath'));
project_root=root;
addpath(genpath(project_root)); if ~isempty(root), addpath(root,'-begin'); end
% START fast path: the GUI passes the already-solved runtime configuration in
% memory. Direct command-line invocation safely falls back to gui_config.mat.
p=[]; try, p=getappdata(0,'FMCW_GUI_RUNTIME_P'); catch, end
if isempty(p) || ~isstruct(p)
    G=load(fullfile(project_root,'gui','gui_config.mat'),'config'); cfg=G.config;
    p=radar_configuration(cfg);
end
% Full-fidelity live mode. No silent reduction of angular/MUSIC/GS search
% resolution is applied; live and offline processing use the same configured
% estimator budgets.
p.theta_axis=linspace(-p.az_span,p.az_span,p.N_angle);

state=[]; frame=0; framePeriod=double(p.Nd)*double(p.Tchirp);
clean_cube=[]; % Reused across frames to reduce heap allocation and memory traffic.

setappdata(0,'FMCW_GUI_RT_ACTIVE',true); setappdata(0,'FMCW_GUI_CLOSE_REQUEST',false); setappdata(0,'FMCW_GUI_PAUSED',false); setappdata(0,'FMCW_GUI_STEP_REQUEST',false);
cleanup=onCleanup(@() safeClearRt());
fprintf('\n---------------- REAL-TIME CAUSAL RADAR ----------------\n');
fprintf('Each frame is acquired -> detected -> tracked -> displayed. Press STOP RADAR to end.\n');
while true
    if getStop(), break; end
    [frame,shouldRun]=waitForLiveControl(frame);
if ~shouldRun, break;
end
frame=frame+1;
tFrame=tic;
tRx=tic;
pf=p;
pf.random_seed=p.random_seed+frame;
    elapsed=double(frame-1)*double(p.Nd)*double(p.Tchirp);
    target_state=double(p.targets); target_state(:,1)=target_state(:,1)+target_state(:,2)*elapsed; pf.targets=target_state;
snr_override=Inf;
    if strcmp(p.noise.mode,'snr_awgn'), snr_override=p.noise.snr_db; end
    [rx_cube,truth,meta]=simulate_mimo_rx(pf,pf.targets,snr_override);
    rxTime=toc(tRx);
drawnow limitrate;
    if getStop(), break; end;
drawnow limitrate;
    if getStop(), break; end
    truthGui=truth_to_matrix(truth,pf.targets);
    notify_gui_progress(struct('phase','simulation','frame',frame,'Nframes',0,'live',true,'pf',pf,'truth',truthGui,'rx_cube',rx_cube,'meta',meta,'sim_time_s',elapsed,'frame_period_s',framePeriod));
tPre=tic;
    if isempty(clean_cube) || ~isequal(size(clean_cube),size(rx_cube)) || ~strcmp(class(clean_cube),class(rx_cube))
        clean_cube=complex(zeros(size(rx_cube),'like',rx_cube));
    else
        clean_cube(:)=0;
    end
    for rx=1:p.n_rx
        if getStop(), break; end
        [clean_cube(:,:,rx),~]=interference_mitigator(rx_cube(:,:,rx),pf);
drawnow limitrate;
        if getStop(), break; end
    end
    preTime=toc(tPre);
drawnow limitrate;
    if getStop(), break; end
    % The frame handed to the tracker carries measurements only. Truth is
    % deliberately absent: a value the detection chain can reach is a value a
    % later change can accidentally use.
    frameData=struct('clean',clean_cube,'raw',rx_cube);
    notify_gui_progress(struct('phase','detection','frame',frame,'Nframes',0,'live',true,'pf',pf,'Pmove',[],'truth',truthGui,'sim_time_s',elapsed,'frame_period_s',framePeriod));
    opts=struct('finalize',false,'initial_state',state,'frame_offset',frame-1,'total_frames',0,'notify_detection',false);
    try
        tDet=tic; [objects,state,info]=radar_object_tracker({frameData},[],pf,opts);
        detTime=toc(tDet);
drawnow limitrate;
        if getStop(), break; end
    catch ME
        fprintf(2,'[REALTIME] Frame %d detection error: %s\n',frame,ME.message);
        for kk=1:min(numel(ME.stack),8), fprintf(2,'  at %s:%d\n',ME.stack(kk).name,ME.stack(kk).line); end
        % Preserve a usable live display for this frame rather than terminating the radar loop.
        objects=empty_live_object_array(); state=state; detTime=toc(tDet);
        info=struct('frame_info',{{struct('rd_power_clean',[],'paper_processing',struct(),'hard_cfar_count',0,'amf_verified_count',0,'group_count',0,'accepted_measurement_count',0)}}, ...
            'frame_detections',{{struct([])}});
    end
    % Build payload field-by-field. This avoids MATLAB struct-array expansion and
    % the cryptic "Field names must be non-empty..." error when a nested result
    % happens to be a non-scalar struct/empty value.
    payload=struct();
    payload.phase='live_result';
payload.live=true;
payload.frame=frame;
payload.Nframes=0;
payload.pf=pf;
payload.truth=truthGui;
payload.meta=meta;
payload.sim_time_s=elapsed;
payload.frame_period_s=framePeriod;
    payload.wall_processing_s=toc(tFrame);
    payload.processing_fps=1/max(payload.wall_processing_s,eps);
    payload.timing=struct('rx_generation_s',rxTime,'preprocess_s',preTime,'detection_s',detTime,'tracking_s',detTime,'gui_s',NaN);
payload.objects=objects;
payload.info=info;
    payload.hard_objects=getfield_safe_local(info,'live_hard_objects',struct([]));
    payload.hard_measurement_points=getfield_safe_local(info,'live_hard_points',struct([]));
    payload.tbd_objects=getfield_safe_local(info,'live_tbd_objects',struct([]));
    % Plot/mark the actual final radar-object union, not every verified CFAR/AMF point.
    % Raw measurements remain separately available as hard_measurement_points.
    % The tracker publishes the fused final-object list under this name; the
    % fallback keeps older cached state loadable.
    payload.display_objects=getfield_safe_local(info,'final_object_display_objects',payload.objects);
    payload.tbd_info=getfield_safe_local(info,'live_tbd_info',struct());
    payload.coherent_tbd_info=getfield_safe_local(info,'live_coherent_tbd_info',struct());
    payload.tbd_object_error=getfield_safe_local(info,'live_tbd_object_error','');
    payload.cfar_points=struct([]); payload.amf_points=struct([]); payload.groups=struct([]); payload.stage_data=struct([]);
    try
        if isfield(info,'stage_data') && ~isempty(info.stage_data)
            st=info.stage_data{1};
            if isstruct(st), payload.stage_data=st; if isfield(st,'cfar_moving'), payload.cfar_points=st.cfar_moving; end; if isfield(st,'amf_moving'), payload.amf_points=st.amf_moving; end; if isfield(st,'groups'), payload.groups=st.groups; end; end
        end
    catch
    end
    fi=struct('frame_index',frame,'rd_power_clean',[],'paper_processing',struct(), ...
        'hard_cfar_count',0,'amf_verified_count',0,'group_count',0, ...
        'accepted_measurement_count',0,'live_tbd_object_count',numel(getfield_safe_local(info,'live_tbd_objects',struct([]))));
    if isstruct(info) && isfield(info,'frame_info') && ~isempty(info.frame_info)
        try
            tmpFi=info.frame_info{1};
            if isstruct(tmpFi), tmpFi.frame_index=frame; end
            if isstruct(tmpFi) && isscalar(tmpFi)
fi=tmpFi;
            end
        catch
        end
    end
    fi.live_tbd_objects=getfield_safe_local(info,'live_tbd_objects',struct([]));
    tbdInfoFi=getfield_safe_local(info,'live_tbd_info',struct());
    fi.live_tbd_candidate_count=getfield_safe_local(tbdInfoFi,'total_candidates',0);
    fi.live_tbd_path_count=numel(getfield_safe_local(info,'live_tbd_paths',struct([])));
    fi.live_tbd_object_error=getfield_safe_local(info,'live_tbd_object_error','');
    fi.live_hard_point_count=numel(getfield_safe_local(info,'live_hard_points',struct([])));
payload.frame_info=fi;
    payload.Pmove=getfield_safe_local(fi,'rd_power_clean',[]);
    payload.paperProc=getfield_safe_local(fi,'paper_processing',struct());
    % Guaranteed display payload: use the completed frame's actual clean RX
    % data as a fallback for GUI maps when a detector stage returns incomplete
    % diagnostics. This does not replace detection; it only keeps the live
    % measurement maps visible for the completed frame.
    if isempty(payload.Pmove) && ~isempty(clean_cube)
        try
            [payload.Pmove,~,~,payload.paperProc]=localBuildDisplayMaps(clean_cube,pf);
        catch ME
            fprintf(2,'[REALTIME] Frame %d display-map fallback warning: %s\n',frame,ME.message);
        end
    end
    if ~isstruct(payload.paperProc)
        payload.paperProc=struct();
    end
    if isstruct(info) && isfield(info,'frame_detections') && ~isempty(info.frame_detections)
        try, payload.verified=info.frame_detections{1}; catch, payload.verified=struct([]); end
    else
        payload.verified=struct([]);
    end
payload.rx_cube=rx_cube;
payload.clean_cube=clean_cube;
    hardN=numel(getfield_safe_local(info,'live_hard_objects',struct([])));
    tbdN=numel(getfield_safe_local(info,'live_tbd_objects',struct([])));
    tbdInfoLocal=getfield_safe_local(info,'live_tbd_info',struct());
tbdCand=0; tbdPathsN=numel(getfield_safe_local(info,'live_tbd_paths',struct([])));
if isstruct(tbdInfoLocal) && isfield(tbdInfoLocal,'total_candidates'), tbdCand=tbdInfoLocal.total_candidates; end
if isstruct(tbdInfoLocal) && isfield(tbdInfoLocal,'path_count'), tbdPathsN=tbdInfoLocal.path_count; end
fprintf('LIVE F%02d | sim %.3f ms | RX %.2f ms | prep %.2f ms | detect/track %.2f ms | total %.2f ms | hard %d | TBDcand %d | TBDpaths %d | TBDobj %d | objects %d\n',frame,elapsed*1e3,rxTime*1e3,preTime*1e3,detTime*1e3,payload.wall_processing_s*1e3,hardN,tbdCand,tbdPathsN,tbdN,numel(objects));
if ~isempty(payload.tbd_object_error), fprintf(2,'[REALTIME] Frame %d TBD object formation error: %s\n',frame,payload.tbd_object_error); end
    notify_gui_progress(payload);
drawnow limitrate;
    if getStop(), break; end
    % Simulate the acquisition cadence: process one frame, then wait only if
    % processing finished before the next physical frame boundary.
    remt=framePeriod-toc(tFrame);
    while remt>0
drawnow;
        if getStop(), break; end
        dt=min(remt,0.02); pause(dt);
        remt=framePeriod-toc(tFrame);
    end
end
fprintf('REAL-TIME RADAR STOPPED at frame %d.\n',frame);
end
function tf=getStop()
tf=false; try, tf=logical(getappdata(0,'FMCW_GUI_STOP_REQUEST')); catch, end
try, tf=tf || logical(getappdata(0,'FMCW_GUI_CLOSE_REQUEST')); catch, end
end
function notify_gui_progress(payload)
try, cb=getappdata(0,'FMCW_GUI_PROGRESS_CALLBACK'); if ~isempty(cb) && isa(cb,'function_handle'), cb(payload); end
catch ME, fprintf(2,'GUI progress callback warning: %s\n',ME.message); end
end
function a=empty_live_object_array(), a=struct('id',{},'range',{},'velocity',{},'angle_deg',{},'x_pos',{},'y_pos',{}); end

function v=getfield_safe_local(s,f,d)
v=d;
try
    if isstruct(s) && isfield(s,f)
        v=s.(f);
    end
catch
end
end

function [Pmove,Pref,auxSum,paperProc]=localBuildDisplayMaps(cleanCube,p)
%LOCALBUILDDISPLAYMAPS Build the same live measurement maps from the completed
% clean RX cube. This is display-only fallback; detector decisions remain in
% radar_object_tracker.
Pmove=zeros(p.Nr,p.Nd); Pref=zeros(p.Nr,p.Nd); auxSum=cell(1,p.n_rx);
for rx=1:p.n_rx
    [~,~,Prd,~,auxR]=range_doppler_processor(cleanCube(:,:,rx),p,struct('keystone',p.range_processing.keystone,'clutter_method','off'));
    Pmove=Pmove+max(real(Prd),0); Pref=Pref+max(real(Prd),0); auxSum{rx}=auxR;
end
if isfield(p,'paper') && p.paper.enabled
    paperProc=moving_stationary_separator(cleanCube,p);
Pmove=paperProc.moving_rd_power;
else
    paperProc=struct('moving_rd_power',Pmove,'stationary_range_power',zeros(p.Nr,1),...
        'stationary_range_angle_power',zeros(p.Nr,numel(p.theta_axis)));
end
Pmove(~isfinite(Pmove))=0; Pref(~isfinite(Pref))=0;
end

function safeClearRt()
try, rmappdata(0,'FMCW_GUI_RUNTIME_P'); catch, end
try, rmappdata(0,'FMCW_GUI_RT_ACTIVE'); end
try, rmappdata(0,'FMCW_GUI_STOP_REQUEST'); end
try, rmappdata(0,'FMCW_GUI_PAUSED'); end
try, rmappdata(0,'FMCW_GUI_STEP_REQUEST'); end
try, rmappdata(0,'FMCW_GUI_CLOSE_REQUEST'); end
end

function truthGui=truth_to_matrix(truth,fallback)
truthGui=[];
try
    if isnumeric(truth) && size(truth,2)==4
        truthGui=double(truth);
    elseif isstruct(truth)
        truthGui=zeros(numel(truth),4);
        for k=1:numel(truth)
            r=getfield_safe(truth(k),'range0',NaN); v=getfield_safe(truth(k),'velocity',NaN); rcs=getfield_safe(truth(k),'rcs_dbsm',NaN); a=getfield_safe(truth(k),'angle_deg',NaN);
            truthGui(k,:)=[r v rcs a];
        end
    end
catch
    truthGui=[];
end
if isempty(truthGui) && isnumeric(fallback) && size(fallback,2)==4, truthGui=double(fallback); end
end
function v=getfield_safe(s,f,d)
v=d; try, if isfield(s,f) && ~isempty(s.(f)), v=s.(f); end, catch, end
end

function [frame,shouldRun]=waitForLiveControl(frame)
shouldRun=true;
while logical(getApp('FMCW_GUI_PAUSED',false)) && ~getStop()
    if logical(getApp('FMCW_GUI_STEP_REQUEST',false))
        setappdata(0,'FMCW_GUI_STEP_REQUEST',false);
        notify_gui_progress(struct('phase','paused_step','frame',frame,'Nframes',0,'live',true));
return;
    end
    notify_gui_progress(struct('phase','paused','frame',frame,'Nframes',0,'live',true));
    drawnow; pause(0.05);
end
if getStop(), shouldRun=false; end
end
function v=getApp(name,default)
v=default; try, if isappdata(0,name), v=getappdata(0,name); end, catch, end
end
