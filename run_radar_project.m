function result = run_radar_project(config_override, run_options)
%RUN_RADAR_PROJECT Execute the full FMCW/TDM-MIMO signal-processing pipeline.
%
% Backward compatible:
%   result = run_radar_project
%
% Benchmark / experiment mode:
%   result = run_radar_project(config_override, run_options)
%
% config_override is merged on top of gui_config.mat when supplied.
% run_options fields:
%   Nframes            - number of frames (default: GUI/default)
%   show_figures       - logical (default: GUI/default)
%   snr_override       - finite dB value or Inf (default: derived from config)
%   save_diagnostics   - logical (default: true)
%   save_results       - logical (default: true)
%   notify_gui         - logical (default: true)
%
% Truth is used only after object formation for offline evaluation.
root=fileparts(mfilename('fullpath'));
addpath(genpath(root));
if ~isempty(root), addpath(root,'-begin'); end
clearvars -except ans root config_override run_options;

if nargin < 1 || isempty(config_override), config_override=struct(); end
if nargin < 2 || isempty(run_options), run_options=struct(); end

localFns={'radar_configuration','validate_radar_config','get_default_field','simulate_mimo_rx', ...
    'fmcw_complex_baseband_dechirp','rd_window','range_doppler_processor','clutter_filter', ...
    'keystone_motion_compensation','moving_stationary_separator','adaptive_cfar_2d', ...
    'adaptive_matched_filter','generalized_subspace_detector','stationary_target_detector', ...
    'detection_quality_filter','music_aoa_estimator','angle_refinement','tdm_virtual_aperture', ...
    'tdm_velocity_resolver','tdm_mimo_processing','interference_mitigator', ...
    'hough_interference_mitigator','dynamic_programming_tbd','coherent_tbd_detector', ...
    'radar_object_tracker','radar_object_evaluation','truth_observability'};
for ii=1:numel(localFns)
    pp=which(localFns{ii});
    if isempty(pp) || ~startsWith(strrep(pp,'\','/'),strrep(root,'\','/'))
        error('run_radar_project:PathCollision',...
            'MATLAB is resolving %s outside the project directory: %s',localFns{ii},pp);
    end
end

showFigures=true; Nframes=16; snr_override=Inf; cfg=struct();
if exist(fullfile(root,'gui','gui_config.mat'),'file')
    G=load(fullfile(root,'gui','gui_config.mat'),'config');
    if isfield(G,'config') && isstruct(G.config), cfg=G.config; end
    if isfield(cfg,'show_figures'), showFigures=logical(cfg.show_figures); end
    if isfield(cfg,'Nframes'), Nframes=round(cfg.Nframes); end
end
% Apply persistent learned parameters after the GUI baseline but before
% explicit per-run overrides. This makes the learned values true defaults
% for every subsequent run without preventing intentional experiments from
% supplying their own detector/tracker parameters.
% radar_configuration adopts the learned algorithm sections itself, so no
% second merge is needed here. Doing it twice risked promoting the training
% scene and array geometry recorded alongside the tuned parameters.
cfg=merge_struct_local(cfg,config_override);
if isfield(run_options,'Nframes') && ~isempty(run_options.Nframes), Nframes=round(run_options.Nframes); end
if isfield(run_options,'show_figures') && ~isempty(run_options.show_figures), showFigures=logical(run_options.show_figures); end
if isfield(run_options,'snr_override') && ~isempty(run_options.snr_override)
    snr_override=double(run_options.snr_override);
elseif strcmp(radar_noise_mode(cfg),'snr_awgn')
snr_override=cfg.noise_level;
elseif isfield(cfg,'snr_override_enabled') && logical(cfg.snr_override_enabled) && isfield(cfg,'snr_override_db') && ~isempty(cfg.snr_override_db)
snr_override=cfg.snr_override_db;
else
snr_override=Inf;
end
saveDiagnostics=true;
saveResults=true;
notifyGUI=true;
if isfield(run_options,'save_diagnostics'), saveDiagnostics=logical(run_options.save_diagnostics); end
if isfield(run_options,'save_results'), saveResults=logical(run_options.save_results); end
if isfield(run_options,'notify_gui'), notifyGUI=logical(run_options.notify_gui); end

% Offline/benchmark runs must not inherit stale GUI stop/close flags left in
% MATLAB application data by a previously closed or cancelled GUI run.
% GUI-driven runs retain the normal stop/close signaling behavior.
restoreGuiFlags=struct('has_stop',false,'stop',false,'has_close',false,'close',false);
if ~notifyGUI
    restoreGuiFlags=local_suppress_gui_stop_flags();
    cleanupGuiFlags=onCleanup(@()local_restore_gui_stop_flags(restoreGuiFlags));
end

p=radar_configuration(cfg);
set(groot,'defaultFigureVisible',ternary(showFigures,'on','off'));
fprintf('\n============================================================\n');
fprintf('  FMCW AUTOMOTIVE RADAR — PHYSICAL SIGNAL-PROCESSING PIPELINE\n');
fprintf('  Physical TX/RX propagation + wideband FMCW + paper MTI + spatial GS interference detector + coherent weak-target integration + CFAR point cloud + group tracking + TBD\n');
fprintf('============================================================\n');
fprintf('fc=%.1f GHz | B=%.1f MHz | Tchirp=%.3f us\n',p.fc/1e9,p.B/1e6,p.Tchirp*1e6);
fprintf('Fs=%.3f MHz | Nr=%d | Nd=%d | virtual aperture=%d\n',p.fs_ADC/1e6,p.Nr,p.Nd,p.n_virt);
fprintf('dR nominal=%.2f m | actual=%.3f m | dV=%.3f m/s | same-TX TDM v_amb=+/-%.2f m/s\n',p.range_resolution_nominal,p.range_resolution_actual,p.velocity_resolution_actual,p.tdm_unambiguous_velocity);
switch char(p.noise.mode)
    case 'fixed_awgn', noiseDesc=sprintf('fixed=%.2f dBm/RX',p.noise.fixed_power_dBm);
    case 'snr_awgn', noiseDesc=sprintf('SNR=%.2f dB',p.noise.snr_db);
    case 'physical_rx_chain', noiseDesc=sprintf('thermal=%.2f dBm/RX',10*log10(max(p.noise_power_W,realmin)/1e-3));
    otherwise, noiseDesc='disabled';
end
fprintf('NOISE: model=%s | enabled=%d | %s | NF=%.1f dB | T=%.0f K | BW=%.2f MHz | RX=%d | Pfa=%.2e | GS=%d | Hough-TF=%d | Paper-MTI=%d | Stationary=%d\n',char(p.noise.mode),p.noise.enabled,noiseDesc,p.NF_dB,p.temp,p.noise_bandwidth/1e6,p.n_rx,p.cfar.Pfa,p.detector.gs.enabled,p.interference.hough_tf_enabled,p.paper.enabled,p.paper.stationary.enabled);

clean_frames=cell(Nframes,1); truth_last=[]; pf_last=[]; sim_meta=cell(Nframes,1); cancelled=false;
for frame=1:Nframes
    if local_gui_stop_requested()
        cancelled=true;
        fprintf('Offline evaluation cancelled by GUI request.\n');
        break
    end
pf=p;
pf.random_seed=p.random_seed+frame;
    elapsed=double(frame-1)*double(p.Nd)*double(p.Tchirp);
    target_state=double(p.targets);
    target_state(:,1)=target_state(:,1)+target_state(:,2)*double(elapsed);
pf.targets=target_state;
    [rx_cube,truth,meta]=simulate_mimo_rx(pf,pf.targets,snr_override);
    clean_cube=complex(zeros(size(rx_cube)));
    for rx=1:p.n_rx
        [clean_cube(:,:,rx),~]=interference_mitigator(rx_cube(:,:,rx),pf);
    end
    clean_frames{frame}=struct('clean',clean_cube,'raw',rx_cube,'truth',pf.targets);
    truth_last=truth; pf_last=pf; sim_meta{frame}=meta;
    fprintf('SIM %02d/%02d | RX-noise mean %.3e W | RX-noise range [%+.3e,%+.3e] W | target SNR range [%+.1f,%+.1f] dB | interference=%d\n', ...
        frame,Nframes,mean(meta.noise_power_per_rx_W),min(meta.noise_power_per_rx_W),max(meta.noise_power_per_rx_W),min(meta.target_snr_db),max(meta.target_snr_db),meta.interference_injected);
    if notifyGUI, notify_gui_progress(struct('phase','simulation','frame',frame,'Nframes',Nframes,'pf',pf,'truth',truth,'meta',meta)); end
    if local_gui_stop_requested()
        cancelled=true;
        fprintf('Offline evaluation cancelled by GUI request.\n');
        break
    end
end

if cancelled || isempty(pf_last)
    result=struct('objects',[],'state',[],'info',struct('cancelled',true),'truth',truth_last,'params',pf_last,'object_eval',struct(),'clean_frames',{clean_frames(1:max(0,find(~cellfun(@isempty,clean_frames),1,'last')))});
    return
end

fprintf('\n---------------- DETECTION / ESTIMATION / TRACKING ----------------\n');
if local_gui_stop_requested()
    result=struct('objects',[],'state',[],'info',struct('cancelled',true),'truth',truth_last,'params',pf_last,'object_eval',struct(),'clean_frames',{clean_frames});
    return
end
[objects,state,info]=radar_object_tracker(clean_frames,{},pf_last,struct());
[object_eval,obs]=radar_object_evaluation(objects,pf_last.targets,info.stage_data,pf_last,info.frame_groups);
falseObs=obs.false_objects;
info.false_object_observability=falseObs;
info.truth_observability=obs;
% Use the exact moving-target RD field consumed by the detector.
% radar_object_tracker caches this field in info.final_rd_power_clean.
if isfield(info,'final_rd_power_clean') && ~isempty(info.final_rd_power_clean)
finalPclean=info.final_rd_power_clean;
else
    finalPclean=[];
end
fprintf('\n---------------- FINAL OFFLINE EVALUATION ----------------\n');
for t=1:object_eval.truth_count
    d=object_eval.assignment(t);
    if d>0
        z=objects(d);
        fprintf('T%d -> R%d | R %.2f -> %.2f (%+.2f) | V %+.2f -> %+.2f (%+.2f) | Az %+.2f -> %+.2f (%+.2f)\n',...
            t,d,pf_last.targets(t,1),z.range,z.range-pf_last.targets(t,1),...
            pf_last.targets(t,2),z.velocity,z.velocity-pf_last.targets(t,2),...
            pf_last.targets(t,4),z.angle_deg,wrap_angle(z.angle_deg-pf_last.targets(t,4)));
    else
        fprintf('T%d | MISSED\n',t);
    end
end
fprintf('Truth=%d | Radar objects=%d | Matched=%d | False=%d | Missed=%d | Pd=%.1f%%\n',...
    object_eval.truth_count,object_eval.radar_object_count,object_eval.matched_count,object_eval.false_object_count,object_eval.missed_count,object_eval.object_pd);
fprintf('RMSE | Range=%.4f m | Velocity=%.4f m/s | Angle=%.4f deg\n',object_eval.range_rmse,object_eval.velocity_rmse,object_eval.angle_rmse);
fprintf('STAGE ATTRIBUTION (offline diagnostic only)\n');
pt=obs.per_target;
for oo=1:numel(pt)
    fprintf('  T%d | R=%6.2f m | preCFAR=%2d | CFAR=%2d | AMF=%2d | groups=%2d | maxPre=%+6.2f dB | deepest=%s\n', ...
        pt(oo).id,pt(oo).range,pt(oo).pre_cfar_frames,pt(oo).cfar_frames,pt(oo).amf_frames, ...
        pt(oo).group_frames,pt(oo).max_pre_cfar_db,pt(oo).deepest_stage);
end
sc=obs.stage_counts;
fprintf('  Deepest stage reached | OBJECT=%d | GROUP=%d | AMF=%d | CFAR=%d | PRE_CFAR=%d | ABSENT=%d\n', ...
    sc.object,sc.group,sc.amf,sc.cfar,sc.pre_cfar,sc.absent);
movingPts=info.moving_point_count;
stationaryPts=info.stationary_point_count;
movGroups=info.moving_group_count;
statGroups=info.stationary_group_count;
fprintf('Points=%d | Moving points=%d | Stationary points=%d | Groups/frame total=%d | Moving groups=%d | Stationary groups=%d | Group tracks=%d | GNN objects=%d | TBD paths=%d | TBD objects=%d | Final objects=%d | Detector=%s\n',info.measurement_count,movingPts,stationaryPts,info.group_measurement_count,movGroups,statGroups,info.group_track_count,info.gnn_confirmed_count,info.candidate_trajectory_count,info.tbd_confirmed_count,numel(objects),info.detector);
if isfield(info,'tbd')
    fprintf('TBD | candidates=%d | accepted paths=%d | max score=%.3f | mean path support=%.3f\n',info.tbd.total_candidates,info.tbd.accepted_path_count,info.tbd.max_terminal_score,mean_or_zero(info.tbd.path_support));
ci=info.live_coherent_tbd_info;
    if isstruct(ci) && isfield(ci,'total_candidates')
        fprintf('COH-TBD | seeds=%d | accepted paths=%d | best score=%.2f dB | threshold=%.2f dB (%s)\n', ...
            ci.total_candidates,ci.accepted_path_count,ci.best_score_db, ...
            getfield_default_local(ci,'threshold_db',NaN),getfield_default_local(ci,'threshold_source',''));
    end
end
if isfield(info,'gs_rescued_count')
    fprintf('GS-SPATIAL | interference-affected frames=%d | GS-rescued hard candidates=%d\n',info.gs_interference_frames,info.gs_rescued_count);
end
if object_eval.false_object_count>0
    fprintf('FALSE OBJECTS (offline diagnostic only): %d\n',object_eval.false_object_count);
    used=false(1,numel(objects));
    for tt=1:object_eval.truth_count, dd=object_eval.assignment(tt); if dd>0, used(dd)=true; end, end
    for ff=find(~used)
        stage='unattributed';
        if ~isempty(falseObs)
            ix=find([falseObs.object_index]==ff,1);
            if ~isempty(ix), stage=falseObs(ix).origin_stage; end
        end
        fprintf('  F%d | R=%.2f m | V=%+.2f m/s | Az=%+.2f deg | hits=%d | hard=%d | TBD=%d | AMF=%.2f dB | CFAR=%.2f dB | source=%s | origin=%s\n',ff,objects(ff).range,objects(ff).velocity,objects(ff).angle_deg,objects(ff).hits,objects(ff).hard_hits,objects(ff).tbd_hits,objects(ff).amf_db,objects(ff).cfar_snr_db,objects(ff).source,stage);
    end
end

if saveDiagnostics
    try
        outdir=fullfile(pwd,'results'); if ~exist(outdir,'dir'),mkdir(outdir);end
        diagnostics=struct('info',info,'truth_last',truth_last,'pf',pf_last,'sim_meta',{sim_meta},'object_eval',object_eval);
        save(fullfile(outdir,'radar_diagnostics.mat'),'diagnostics','-v7.3');
    catch ME
        fprintf('Diagnostics save warning: %s\n',ME.message);
    end
end

if showFigures
    P=zeros(p.Nr,p.Nd);
    for rx=1:p.n_rx
        [cleanLast,~]=unpack_frame_for_plot(clean_frames{end}); [~,~,Prd]=range_doppler_processor(cleanLast(:,:,rx),p,struct('keystone',true,'clutter_method','dc_cancel'));
P=P+Prd;
    end
    figure('Name','Physics Radar','Color','w','Position',[60 60 1500 900]);
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    nexttile; imagesc(p.vel_axis,p.range_axis,10*log10(max(P/max(P(:)+eps),1e-15))); axis xy; colorbar; xlabel('Velocity (m/s)'); ylabel('Range (m)'); title('Calibrated RD evidence');
nexttile;
hold on;
grid on;
    if ~isempty(objects), plot([objects.velocity],[objects.range],'ro','LineWidth',1.5); end
    plot(pf_last.targets(:,2),pf_last.targets(:,1),'k+','LineWidth',1.5); xlabel('Velocity (m/s)'); ylabel('Range (m)'); title('Final objects / truth'); legend('Radar','Truth');
    nexttile; plot_object_metric(objects,pf_last.targets,'angle');
    nexttile; axis off; text(0,1,sprintf('Detector: %s\nPfa=%.2e\nThermal noise=%.3e W\nMeasurements=%d\nConfirmed=%d\nTruth used only offline',info.detector,p.cfar.Pfa,p.noise_power_W,info.measurement_count,info.confirmed_count),'VerticalAlignment','top','FontSize',11);
end

result=struct('objects',objects,'state',state,'info',info,'truth',truth_last,'params',pf_last,'object_eval',object_eval,'clean_frames',{clean_frames});
finalPclean=info.final_rd_power_clean;
finalCleanCube=[];
if ~isempty(clean_frames) && ~isempty(clean_frames{end})
    [finalCleanCube,~]=unpack_frame_for_plot(clean_frames{end});
end
last=struct('objects',objects,'object_info',info,'truth',truth_last,'pf',pf_last,'state',state,'object_eval',object_eval,...
    'measurement_point_count',info.measurement_count,'formation_point_count',numel(objects),'cfar_info',struct('bypassed',false),...
    'tbd_info',info,'truth_observability',obs,'detection_heatmap',struct('Pclean',finalPclean,'frame_index',Nframes),...
    'paper_processing',info.final_paper_processing,...
    'final_clean_cube',finalCleanCube,'waveform',struct('B',pf_last.B,'Tchirp',pf_last.Tchirp,'Nd',pf_last.Nd,'Nr',pf_last.Nr,'range_resolution',pf_last.range_resolution_actual,'velocity_resolution',pf_last.velocity_resolution_actual,'tdm_unambiguous_velocity',pf_last.tdm_unambiguous_velocity));
result.last=last;
if saveResults
    try
        save(fullfile(pwd,'fmcw_demo_results_v17.mat'),'last','-v7.3');
    catch ME
        fprintf('GUI compatibility save warning: %s\n',ME.message);
    end
end
end

function tf=local_gui_stop_requested()
tf=false;
try
    if isappdata(0,'FMCW_GUI_STOP_REQUEST')
        tf=logical(getappdata(0,'FMCW_GUI_STOP_REQUEST'));
    end
catch
end
end

function saved=local_suppress_gui_stop_flags()
saved=struct('has_stop',false,'stop',false,'has_close',false,'close',false);
try
    saved.has_stop=isappdata(0,'FMCW_GUI_STOP_REQUEST');
    if saved.has_stop, saved.stop=logical(getappdata(0,'FMCW_GUI_STOP_REQUEST')); end
    saved.has_close=isappdata(0,'FMCW_GUI_CLOSE_REQUEST');
    if saved.has_close, saved.close=logical(getappdata(0,'FMCW_GUI_CLOSE_REQUEST')); end
    setappdata(0,'FMCW_GUI_STOP_REQUEST',false);
    setappdata(0,'FMCW_GUI_CLOSE_REQUEST',false);
catch
end
end

function local_restore_gui_stop_flags(saved)
try
    if saved.has_stop
        setappdata(0,'FMCW_GUI_STOP_REQUEST',saved.stop);
    elseif isappdata(0,'FMCW_GUI_STOP_REQUEST')
        rmappdata(0,'FMCW_GUI_STOP_REQUEST');
    end
    if saved.has_close
        setappdata(0,'FMCW_GUI_CLOSE_REQUEST',saved.close);
    elseif isappdata(0,'FMCW_GUI_CLOSE_REQUEST')
        rmappdata(0,'FMCW_GUI_CLOSE_REQUEST');
    end
catch
end
end

function notify_gui_progress(payload)
try
    cb=getappdata(0,'FMCW_GUI_PROGRESS_CALLBACK');
    if ~isempty(cb) && isa(cb,'function_handle'), cb(payload); end
catch ME
    fprintf(2,'GUI progress callback warning: %s\n',ME.message);
end
end

function plot_object_metric(objects,truth,mode)
E=radar_object_evaluation(objects,truth); a=E.assignment; idx=1:size(truth,1); est=nan(size(idx));
for t=1:numel(idx)
    d=a(t); if d>0
        if strcmp(mode,'range'),est(t)=objects(d).range; elseif strcmp(mode,'velocity'),est(t)=objects(d).velocity; else,est(t)=objects(d).angle_deg; end
    end
end
if strcmp(mode,'range'),tv=truth(:,1); ylabel('Range (m)'); ttl='Range — Truth vs Radar'; elseif strcmp(mode,'velocity'),tv=truth(:,2); ylabel('Velocity (m/s)'); ttl='Velocity — Truth vs Radar'; else, tv=truth(:,4); ylabel('Azimuth (deg)'); ttl='Azimuth — Truth vs Radar'; end
plot(idx,tv,'k*-','DisplayName','Truth'); hold on; plot(idx,est,'ro-','DisplayName','Radar'); grid on; legend('Location','best'); xlabel('Object'); title(ttl);
end
function [clean,raw]=unpack_frame_for_plot(f)
if isstruct(f), clean=f.clean; raw=f.raw; else, clean=f; raw=f; end
end
function v=getfield_default_local(s,f,d), if ~isstruct(s) || ~isscalar(s) || ~isfield(s,f),v=d;return;end; val=s.(f); if isempty(val),v=d;else,v=val;end,end
function x=mean_or_zero(v), if isempty(v),x=0; else,x=mean(v,'omitnan'); end,end
function out=merge_struct_local(base,ov)
out=base;
if isempty(ov) || ~isstruct(ov), return; end
f=fieldnames(ov);
for k=1:numel(f)
    key=f{k}; val=ov.(key);
    if isstruct(val) && isfield(out,key) && isstruct(out.(key))
        out.(key)=merge_struct_local(out.(key),val);
    else
        out.(key)=val;
    end
end
end
function m=radar_noise_mode(cfg)
m='';
if ~isstruct(cfg) || ~isfield(cfg,'noise_model') || isempty(cfg.noise_model), return; end
m=lower(strrep(char(cfg.noise_model),' ','_'));
if any(strcmp(m,{'snr-controlled_awgn','snr_awgn','snr'})), m='snr_awgn'; end
end
function a=wrap_angle(a), a=mod(a+180,360)-180; end
function x=ternary(c,a,b), if c,x=a;else,x=b;end,end
