function fig = fmcw_radar_gui_impl
% FMCW_GUI - Dynamic FMCW automotive-radar design GUI.
% GUI is a design/evaluation layer over the canonical radar algorithms.

root = fileparts(mfilename('fullpath'));
fig = uifigure('Name','FMCW Automotive Radar — Signal Processing Studio', ...
    'Position',[40 40 1450 900], 'Color',[0.96 0.96 0.96], 'Visible','on');
fig.CloseRequestFcn=@closeGui;
setappdata(0,'FMCW_GUI_FIGURE',fig);
drawnow;

C.bg = [0.96 0.96 0.96]; C.panel=[1 1 1]; C.accent=[0.10 0.43 0.72];
C.good=[0.84 0.95 0.86]; C.warn=[1.00 0.93 0.72]; C.bad=[1.00 0.84 0.84];
% UI label factory: avoid nested-function name collisions in MATLAB static workspaces.
makeRadarLabel = @(parent,varargin) feval('uilabel',parent,varargin{:});

outer = uigridlayout(fig,[1 2]); outer.Padding=[8 8 8 8]; outer.ColumnWidth={760,'1x'}; outer.ColumnSpacing=10;
leftPanel=uipanel(outer,'Title','Radar Design','BackgroundColor',C.panel); leftPanel.Layout.Column=1;
rightPanel=uipanel(outer,'Title','Final Radar Output','BackgroundColor',C.panel); rightPanel.Layout.Column=2;

leftOuter=uigridlayout(leftPanel,[3 1]); leftOuter.Padding=[8 8 8 8]; leftOuter.RowHeight={'1x',108,52}; leftOuter.RowSpacing=8;
viewport=uipanel(leftOuter,'BorderType','line','Scrollable','on','BackgroundColor',C.panel); viewport.Layout.Row=1;
canvas=uipanel(viewport,'BorderType','none','BackgroundColor',C.panel); canvas.Position=[0 0 735 2165];
content=uigridlayout(canvas,[6 1]); content.RowHeight={275,300,230,230,300,760}; content.ColumnWidth={'1x'}; content.Padding=[5 5 5 8]; content.RowSpacing=10;

% ---------- Status details (full-length, non-clipped) ----------
statusPanel=uipanel(leftOuter,'Title','Status Details','BackgroundColor',[0.97 0.98 0.99]); statusPanel.Layout.Row=2;
statusGrid=uigridlayout(statusPanel,[2 1]); statusGrid.RowHeight={66,18}; statusGrid.Padding=[6 4 6 4]; statusGrid.RowSpacing=2;
statusDetails=uitextarea(statusGrid,'Editable','off','Value',{'READY','Configure parameters, then start LIVE.','Errors and warnings appear here.'}); statusDetails.Layout.Row=1; statusDetails.FontName='Monospaced'; statusDetails.FontSize=10; statusDetails.WordWrap='on';
pipelineLabel=makeRadarLabel(statusGrid,'Text','Pipeline: idle','FontWeight','bold'); pipelineLabel.Layout.Row=2;
% ---------- Footer: fixed run / status ----------
footer=uipanel(leftOuter,'BackgroundColor',[0.94 0.96 0.98]); footer.Layout.Row=3;
fg=uigridlayout(footer,[1 8]); fg.ColumnWidth={118,70,58,58,74,'1x',72,72}; fg.Padding=[6 5 6 5]; fg.ColumnSpacing=4;
runBtn=uibutton(fg,'Text','START LIVE','FontWeight','bold','ButtonPushedFcn',@runClicked); runBtn.Layout.Column=1; runBtn.BackgroundColor=C.accent; runBtn.FontColor='w';
pauseBtn=uibutton(fg,'Text','PAUSE','FontWeight','bold','ButtonPushedFcn',@pauseClicked); pauseBtn.Layout.Column=2; pauseBtn.Enable='off';
stepBtn=uibutton(fg,'Text','STEP','FontWeight','bold','ButtonPushedFcn',@stepClicked); stepBtn.Layout.Column=3; stepBtn.Enable='off';
stopBtn=uibutton(fg,'Text','STOP','FontWeight','bold','ButtonPushedFcn',@stopClicked); stopBtn.Layout.Column=4; stopBtn.Enable='off';
evalBtn=uibutton(fg,'Text','EVAL 16','FontWeight','bold','ButtonPushedFcn',@evalClicked); evalBtn.Layout.Column=5;
statusLabel=makeRadarLabel(fg,'Text','Design OK','FontWeight','bold'); statusLabel.Layout.Column=6; statusLabel.WordWrap='off';
readyLabel=makeRadarLabel(fg,'Text','READY','HorizontalAlignment','center','FontWeight','bold'); readyLabel.Layout.Column=7; readyLabel.BackgroundColor=C.good;

% ---------- Section 1 ----------
scenePanel=uipanel(content,'Title','1 — Scene / Objects','BackgroundColor',C.panel); scenePanel.Layout.Row=1;
s1outer=uigridlayout(scenePanel,[1 1]); s1outer.Padding=[2 2 2 2];
s1viewport=uipanel(s1outer,'BorderType','none','Scrollable','off','BackgroundColor',C.panel); s1viewport.Layout.Row=1;
s1canvas=uipanel(s1viewport,'BorderType','none','BackgroundColor',C.panel); s1canvas.Position=[0 0 720 265];
sg=uigridlayout(s1canvas,[3 1]); sg.Padding=[6 6 6 6]; sg.RowHeight={218,34,20};
sceneTable=uitable(sg,'ColumnName',{'Object','Range (m)','Velocity (m/s)','RCS (%)','Azimuth (deg)'}, ...
    'ColumnEditable',[false true true true true],'RowName',[],'ColumnWidth',{120,145,160,110,155},'Data',randomScene()); sceneTable.Layout.Row=1;
sceneTable.CellSelectionCallback=@(~,e) setappdata(sceneTable,'SelectedRows',e.Indices);
btns=uigridlayout(sg,[1 5]); btns.Layout.Row=2; btns.ColumnWidth={110,125,120,130,'1x'}; btns.Padding=[0 0 0 0];
addBtn=uibutton(btns,'Text','Add object','ButtonPushedFcn',@addObject); addBtn.Layout.Column=1;
remBtn=uibutton(btns,'Text','Remove selected','ButtonPushedFcn',@removeObject); remBtn.Layout.Column=2;
randBtn=uibutton(btns,'Text','New scene','ButtonPushedFcn',@newRandomScene); randBtn.Layout.Column=3;
resetBtn=uibutton(btns,'Text','Reference scene','ButtonPushedFcn',@resetScene); resetBtn.Layout.Column=4;
sceneCount=makeRadarLabel(btns,'Text',sprintf('%d objects',size(sceneTable.Data,1)),'HorizontalAlignment','right'); sceneCount.Layout.Column=5;
hint=makeRadarLabel(sg,'Text','RCS: 0–100% maps to 0–20 dBsm.'); hint.Layout.Row=3; hint.FontColor=[0.25 0.25 0.25];

% ---------- Section 2 ----------
paramPanel=uipanel(content,'Title','2 — Radar Parameters (section scroll)','BackgroundColor',C.panel); paramPanel.Layout.Row=2;
sec2=uigridlayout(paramPanel,[1 1]); sec2.Padding=[2 2 2 2];
paramViewport=uipanel(sec2,'BorderType','none','Scrollable','off','BackgroundColor',C.panel); paramViewport.Layout.Row=1;
paramCanvas=uipanel(paramViewport,'BorderType','none','BackgroundColor',C.panel); paramCanvas.Position=[0 0 720 285];
pg=uigridlayout(paramCanvas,[12 4]); pg.RowHeight=repmat({21},1,12); pg.ColumnWidth={210,145,95,80}; pg.Padding=[8 8 8 8]; pg.RowSpacing=2; pg.ColumnSpacing=5;
labels={'Carrier frequency','Maximum range','Range resolution','Maximum velocity','Velocity resolution','Bandwidth','Chirp slope','Chirp duration','ADC sample rate','ADC samples / chirp','Frame length','Azimuth span'};
keys={'fc','R_max','R_res','v_max','v_res','B','slope','Tchirp','Fs','Nr','Nd','az_span'};
unitItems={{'GHz','MHz'},{'m','km'},{'m','cm'},{'m/s','km/h'},{'m/s','km/h'},{'MHz','GHz'},{'MHz/us','GHz/s'},{'us','ms'},{'MHz','GHz'},{'samples','kSamples'},{'chirps','kChirps'},{'deg'}};
defs=[77,300,0.50,60,0.9375,300,18.4928,16.2225,126.35,2048,128,70];
lims=[[24 90];[10 2000];[0.05 20];[1 150];[0.05 10];[1 2000];[0.01 100];[1 500];[0.5 500];[256 65536];[32 8192];[10 89]];
S.params=struct(); S.busy=false; S.controls={}; S.designOK=false; S.uiState='INITIALIZING'; S.lastError=''; S.stopRequested=false;
for i=1:numel(keys)
    lab=makeRadarLabel(pg,'Text',labels{i}); lab.Layout.Row=i; lab.Layout.Column=1;
    sp=uispinner(pg); sp.Value=defs(i); sp.Limits=lims(i,:); sp.Step=stepFor(i); sp.ValueDisplayFormat='%.6g'; sp.Layout.Row=i; sp.Layout.Column=2;
    dd=uidropdown(pg,'Items',unitItems{i},'Value',unitItems{i}{1}); dd.Layout.Row=i; dd.Layout.Column=3;
    cb=uicheckbox(pg,'Text','Lock','Value',false); cb.Layout.Row=i; cb.Layout.Column=4;
    ps=struct('label',lab,'spinner',sp,'unit',dd,'lock',cb,'lastUnit',unitItems{i}{1}); S.params.(keys{i})=ps;
    sp.ValueChangedFcn=@(~,~) parameterChanged(keys{i}); dd.ValueChangedFcn=@(~,~) unitChanged(keys{i}); cb.ValueChangedFcn=@(~,~) lockChanged(keys{i});
    S.controls{end+1}=sp; S.controls{end+1}=dd; S.controls{end+1}=cb;
end

% ---------- Section 3 ----------
derPanel=uipanel(content,'Title','3 — Derived values / physical checks (read-only)','BackgroundColor',C.panel); derPanel.Layout.Row=3;
s3outer=uigridlayout(derPanel,[1 1]); s3outer.Padding=[2 2 2 2];
s3viewport=uipanel(s3outer,'BorderType','none','Scrollable','off','BackgroundColor',C.panel); s3viewport.Layout.Row=1;
s3canvas=uipanel(s3viewport,'BorderType','none','BackgroundColor',C.panel); s3canvas.Position=[0 0 720 220];
dg=uigridlayout(s3canvas,[6 3]); dg.ColumnWidth={250,180,'1x'}; dg.RowHeight=[repmat({30},1,5),40]; dg.Padding=[8 8 8 8]; dg.RowSpacing=3;
derivedNames={'Maximum beat frequency','ADC Nyquist requirement (2× beat)','ADC-limited range capability','Doppler unambiguous velocity (±)','Design status'};
derivedUnits={'MHz','MHz','m','m/s',''};
D=struct();
for i=1:5
    a=makeRadarLabel(dg,'Text',derivedNames{i}); a.Layout.Row=i; a.Layout.Column=1;
    e=uieditfield(dg,'text','Editable','off'); e.Layout.Row=i; e.Layout.Column=2; e.BackgroundColor=[0.95 0.95 0.95]; e.Value='—';
    u=makeRadarLabel(dg,'Text',derivedUnits{i}); u.Layout.Row=i; u.Layout.Column=3;
    D.(derivedKey(i))=e;
end
note=makeRadarLabel(dg,'Text','Derived checks only. 2048 fast-time samples/chirp baseline; bandwidth sets nominal range resolution.'); note.Layout.Row=6; note.Layout.Column=[1 3]; note.FontColor=[0.25 0.25 0.25];

% ---------- Section 4 ----------
hwPanel=uipanel(content,'Title','4 — Hardware / Simulation (section scroll)','BackgroundColor',C.panel); hwPanel.Layout.Row=4;
sec4=uigridlayout(hwPanel,[1 1]); sec4.Padding=[2 2 2 2];
hwViewport=uipanel(sec4,'BorderType','none','Scrollable','off','BackgroundColor',C.panel); hwViewport.Layout.Row=1;
hwCanvas=uipanel(hwViewport,'BorderType','none','BackgroundColor',C.panel); hwCanvas.Position=[0 0 720 220];
hg=uigridlayout(hwCanvas,[6 4]); hg.RowHeight=repmat({30},1,6); hg.ColumnWidth={190,150,100,90}; hg.Padding=[8 10 8 10]; hg.RowSpacing=3;
S.hw=struct(); addHw('TX count','n_tx',2,1,'count',[1 8]); addHw('RX count','n_rx',4,2,'count',[1 16]); addHw('Random seed','seed',47,3,'',[0 999999]); addHw('Angle grid','N_angle',1024,4,'samples',[128 8192]); addHw('Evaluation frames','Nframes',16,5,'frames',[1 32]); addHw('Show intermediate figures','show_fig',0,6,'0/1',[0 1]);

% ---------- Section 5 ----------
noisePanel=uipanel(content,'Title','5 — Receiver Noise / SNR (physical model)','BackgroundColor',C.panel); noisePanel.Layout.Row=5;
sec5=uigridlayout(noisePanel,[1 1]); sec5.Padding=[2 2 2 2];
ng=uigridlayout(sec5,[8 4]); ng.Layout.Row=1; ng.Layout.Column=1; ng.RowHeight=repmat({28},1,8); ng.ColumnWidth={210,150,105,'1x'}; ng.Padding=[8 10 8 10]; ng.RowSpacing=4; ng.ColumnSpacing=6;
S.noise=struct();
S.noise.enabled=uicheckbox(ng,'Text','Enable noise','Value',true); S.noise.enabled.Layout.Row=1; S.noise.enabled.Layout.Column=1;
S.noise.model=uidropdown(ng,'Items',{'Physical thermal AWGN','SNR-controlled AWGN','Fixed AWGN power','No noise'},'Value','Physical thermal AWGN','Tooltip','Choose the receiver-noise model.'); S.noise.model.Layout.Row=1; S.noise.model.Layout.Column=[2 4];
S.noise.NF_dB=addNoise('Noise figure',10,2,'dB',[0 30]);
S.noise.temp_K=addNoise('Receiver temperature',290,3,'K',[50 1000]);
S.noise.level=addNoise('Noise level',20,4,'dB',[0 80]);
noiseUnit=makeRadarLabel(ng,'Text','SNR dB','HorizontalAlignment','left'); noiseUnit.Layout.Row=4; noiseUnit.Layout.Column=3;
fixedLabel=makeRadarLabel(ng,'Text','Fixed AWGN power','HorizontalAlignment','left'); fixedLabel.Layout.Row=5; fixedLabel.Layout.Column=1;
S.noise.fixed=uispinner(ng); S.noise.fixed.Value=-92; S.noise.fixed.Limits=[-140 -20]; S.noise.fixed.Step=1; S.noise.fixed.ValueDisplayFormat='%.6g'; S.noise.fixed.Layout.Row=5; S.noise.fixed.Layout.Column=2;
fixedUnit=makeRadarLabel(ng,'Text','dBm','HorizontalAlignment','left'); fixedUnit.Layout.Row=5; fixedUnit.Layout.Column=3;
noiseBandwidthLabel=makeRadarLabel(ng,'Text','BW: Fs/2 | post-dechirp','FontColor',[0.25 0.25 0.25]); noiseBandwidthLabel.Layout.Row=6; noiseBandwidthLabel.Layout.Column=[1 4];
noiseInfo=makeRadarLabel(ng,'Text','Mode: thermal = kTB×NF','FontColor',[0.25 0.25 0.25]); noiseInfo.Layout.Row=7; noiseInfo.Layout.Column=[1 4];
noiseHint=makeRadarLabel(ng,'Text','SNR/fixed modes set AWGN level directly.','FontColor',[0.25 0.25 0.25]); noiseHint.Layout.Row=8; noiseHint.Layout.Column=[1 4];
S.controls{end+1}=S.noise.enabled; S.controls{end+1}=S.noise.model; S.controls{end+1}=S.noise.fixed;
S.noise.enabled.ValueChangedFcn=@(~,~) noiseChanged(); S.noise.model.ValueChangedFcn=@(~,~) noiseChanged();

% ---------- Section 6 ----------
tunePanel=uipanel(content,'Title',['6 — Detection / Estimation / Tracking / Filtering Controls    [' learnedBannerText() ']'],'BackgroundColor',C.panel); tunePanel.Layout.Row=6;
tuneOuter=uigridlayout(tunePanel,[2 1]); tuneOuter.Padding=[4 4 4 4]; tuneOuter.RowHeight={34,'1x'}; tuneOuter.RowSpacing=4;
presetBar=uigridlayout(tuneOuter,[1 4]); presetBar.Layout.Row=1; presetBar.ColumnWidth={110,180,'1x',180}; presetBar.Padding=[4 2 4 2];
presetLabel=makeRadarLabel(presetBar,'Text','Tuning preset','FontWeight','bold'); presetLabel.Layout.Column=1;
S.det=struct();
S.det.preset=uidropdown(presetBar,'Items',{'Validated baseline','Balanced recall','Strict precision','Weak-target TBD','Custom'},'Value','Validated baseline','ValueChangedFcn',@applyDetectionPreset); S.det.preset.Layout.Column=2;
presetNote=makeRadarLabel(presetBar,'Text','All controls below are saved into gui_config.mat','FontColor',[0.25 0.25 0.25]); presetNote.Layout.Column=3;
S.det.status=makeRadarLabel(presetBar,'Text','Live + offline','HorizontalAlignment','right','FontWeight','bold'); S.det.status.Layout.Column=4;

tuneTabs=uitabgroup(tuneOuter); tuneTabs.Layout.Row=2;
% CFAR / weak candidate controls
t=uitab(tuneTabs,'Title','CFAR'); g=uigridlayout(t,[9 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({31},1,9); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetDropdown(g,1,1,2,{'CFAR mode','adaptive|ca|os|go|so'},'cfar','mode','adaptive');
addDetSpinner(g,1,4,5,{'Pfa',1e-5,[1e-8 1e-1],1e-6,'%.2e',''},'cfar','Pfa');
addDetSpinner(g,2,1,2,{'Range training cells',12,[1 64],1,'%.0f','cells'},'cfar','Tr');
addDetSpinner(g,2,4,5,{'Doppler training cells',16,[1 64],1,'%.0f','cells'},'cfar','Td');
addDetSpinner(g,3,1,2,{'Range guard cells',2,[0 16],1,'%.0f','cells'},'cfar','Gr');
addDetSpinner(g,3,4,5,{'Doppler guard cells',2,[0 16],1,'%.0f','cells'},'cfar','Gd');
addDetSpinner(g,4,1,2,{'OS fraction',0.75,[0.5 0.99],0.01,'%.2f','fraction'},'cfar','os_fraction');
addDetSpinner(g,4,4,5,{'Weak SNR threshold',-6,[-30 10],0.5,'%.1f','dB'},'cfar','weak_snr_db');
addDetSpinner(g,5,1,2,{'Minimum CFAR quality SNR',0,[-20 30],0.5,'%.1f','dB'},'cfar','min_snr_db');
addDetSpinner(g,5,4,5,{'Max CFAR detections',256,[16 2048],16,'%.0f','points'},'cfar','max_detections');
addDetSpinner(g,6,1,2,{'Min reference cells',32,[8 512],1,'%.0f','cells'},'cfar','min_reference_cells');
addDetSpinner(g,6,4,5,{'Local max range radius',1,[0 8],1,'%.0f','bins'},'cfar','local_max_r');
addDetSpinner(g,7,1,2,{'Local max Doppler radius',1,[0 8],1,'%.0f','bins'},'cfar','local_max_d');
addDetSpinner(g,7,4,5,{'Heterogeneity CV',0.8,[0.1 5],0.05,'%.2f',''},'cfar','heterogeneity_cv');
addDetSpinner(g,8,1,2,{'OS contamination ratio',0.75,[0.1 1],0.01,'%.2f',''},'cfar','os_contamination_ratio');
addDetSpinner(g,8,4,5,{'RD grouping range',2,[0 8],1,'%.0f','bins'},'cfar','group_r_bins');
addDetSpinner(g,9,1,2,{'RD grouping Doppler',2,[0 8],1,'%.0f','bins'},'cfar','group_d_bins');
% Physical verification / GS / AMF
t=uitab(tuneTabs,'Title','Verification'); g=uigridlayout(t,[8 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({31},1,8); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetSpinner(g,1,1,2,{'AMF Pfa',1e-3,[1e-6 0.2],1e-4,'%.2e',''},'detector','amf_threshold_pfa');
addDetSpinner(g,1,4,5,{'Minimum AMF',7,[0 25],0.5,'%.1f','dB'},'detector','min_amf_db');
addDetSpinner(g,2,1,2,{'Angle coarse step',2,[0.25 10],0.25,'%.2f','deg'},'detector','angle_coarse_step_deg');
addDetSpinner(g,2,4,5,{'Angle refine halfspan',3,[0.5 15],0.25,'%.2f','deg'},'detector','angle_refine_halfspan_deg');
addDetSpinner(g,3,1,2,{'Angle refine step',0.25,[0.05 2],0.05,'%.2f','deg'},'detector','angle_refine_step_deg');
addDetSpinner(g,3,4,5,{'Max verified candidates',128,[16 1024],16,'%.0f','points'},'detector','max_candidates');
addDetCheckbox(g,4,1,2,'GS spatial detector','detector_gs','enabled',true);
addDetSpinner(g,4,4,5,{'GS Pfa',1e-3,[1e-6 0.2],1e-4,'%.2e',''},'detector_gs','pfa');
addDetSpinner(g,5,1,2,{'GS angle step',1,[0.25 5],0.25,'%.2f','deg'},'detector_gs','angle_step_deg');
addDetSpinner(g,5,4,5,{'Min interference separation',8,[2 25],1,'%.1f','deg'},'detector_gs','min_interference_angle_sep_deg');
addDetSpinner(g,6,1,2,{'Interference INR threshold',3,[-10 30],0.5,'%.1f','dB'},'detector_gs','interference_inr_threshold_db');
addDetSpinner(g,6,4,5,{'GS rescue minimum',0,[-10 20],0.5,'%.1f','dB'},'detector_gs','rescue_min_db');
addDetCheckbox(g,7,1,2,'GS rescue','detector_gs','rescue_enabled',true);
addDetSpinner(g,7,4,5,{'Max INR',1e4,[10 1e8],10,'%.0f','linear'},'detector_gs','max_inr_linear');

% AoA / TDM estimation
t=uitab(tuneTabs,'Title','AoA / TDM'); g=uigridlayout(t,[9 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({31},1,9); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetSpinner(g,1,1,2,{'MUSIC grid',2048,[256 8192],256,'%.0f','samples'},'est','music_grid');
addDetSpinner(g,1,4,5,{'MUSIC prominence',3,[0 20],0.5,'%.1f','dB'},'est','music_min_prom_db');
addDetSpinner(g,2,1,2,{'MUSIC diagonal loading',1e-3,[1e-6 0.1],1e-4,'%.2e',''},'est','music_cov_diagonal_loading');
addDetSpinner(g,2,4,5,{'Maximum sources',3,[1 7],1,'%.0f','sources'},'est','n_src_max');
addDetDropdown(g,3,1,2,{'Model order','mdl|fixed'},'est','model_order','mdl');
addDetSpinner(g,3,4,5,{'Maximum AoA peaks',3,[1 8],1,'%.0f','peaks'},'est','max_aoa_peaks');
addDetSpinner(g,4,1,2,{'FFT zero pad factor',8,[1 64],1,'%.0f','x'},'est','fft_zero_pad');
addDetSpinner(g,4,4,5,{'Minimum snapshots',8,[4 128],1,'%.0f','snapshots'},'est','min_snapshots');
addDetSpinner(g,5,1,2,{'TDM alias span',2,[0 5],1,'%.0f','orders'},'detector','tdm_alias_span');
addDetSpinner(g,5,4,5,{'Alias score margin',1,[0 10],0.25,'%.2f','dB'},'detector','tdm_alias_score_margin_db');
addDetSpinner(g,6,1,2,{'TDM coherence weight',4,[0 20],0.5,'%.1f','weight'},'detector','tdm_coherence_weight');
addDetCheckbox(g,6,4,5,'Use whitened AMF','detector','use_whitened_amf',true);

% TBD controls
t=uitab(tuneTabs,'Title','TBD'); g=uigridlayout(t,[11 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({30},1,11); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetCheckbox(g,1,1,2,'Enable DP TBD','tbd','enabled',true);
addDetSpinner(g,1,4,5,{'Minimum path frames',6,[3 16],1,'%.0f','frames'},'tbd','min_path_frames');
addDetSpinner(g,2,1,2,{'Minimum path support',0.65,[0.4 1],0.05,'%.2f','fraction'},'tbd','min_path_support_fraction');
addDetSpinner(g,2,4,5,{'Path minimum score',7,[0 50],0.5,'%.1f','score'},'tbd','path_min_score');
addDetSpinner(g,3,1,2,{'Path promotion score',12,[0 80],0.5,'%.1f','score'},'tbd','path_promotion_score');
addDetSpinner(g,3,4,5,{'Maximum gap frames',2,[0 5],1,'%.0f','frames'},'tbd','max_gap_frames');
addDetSpinner(g,4,1,2,{'Path angle std',8,[0 20],0.5,'%.1f','deg'},'tbd','path_angle_max_std_deg');
addDetSpinner(g,4,4,5,{'Minimum angle prominence',0.5,[0 10],0.25,'%.2f','dB'},'tbd','path_angle_min_prominence_db');
addDetCheckbox(g,5,1,2,'Trajectory AMF recovery','tbd','trajectory_recovery_enabled',true);
addDetSpinner(g,5,4,5,{'Trajectory AMF minimum',5,[0 20],0.5,'%.1f','dB'},'tbd','trajectory_recovery_min_integrated_amf_db');
addDetCheckbox(g,6,1,2,'Suppress near verified hard points','tbd','suppress_near_hard',true);
addDetSpinner(g,6,4,5,{'Verified hard AMF floor',7,[0 25],0.5,'%.1f','dB'},'tbd','near_hard_suppress_amf_db');
addDetSpinner(g,7,1,2,{'TBD max candidates/frame',256,[32 2048],32,'%.0f','candidates'},'tbd','max_candidates_per_frame');
addDetSpinner(g,7,4,5,{'TBD max confirmed paths',16,[1 64],1,'%.0f','paths'},'tbd','max_confirmed_paths');
addDetCheckbox(g,8,1,2,'Enable coherent TBD','tbd_coherent','enabled',true);
addDetSpinner(g,8,4,5,{'Coherent min frames',5,[3 16],1,'%.0f','frames'},'tbd_coherent','min_path_frames');
addDetSpinner(g,9,1,2,{'Coherent support',0.70,[0.4 1],0.05,'%.2f','fraction'},'tbd_coherent','min_support_fraction');
addDetSpinner(g,9,4,5,{'Coherent seed threshold',-12,[-25 5],0.5,'%.1f','dB'},'tbd_coherent','seed_threshold_db');
addDetSpinner(g,10,1,2,{'Coherent path score',6,[0 30],0.5,'%.1f','score'},'tbd_coherent','path_score_threshold');
addDetSpinner(g,10,4,5,{'Coherent score threshold',5.5,[0 20],0.5,'%.1f','dB'},'tbd_coherent','coherent_score_threshold_db');
addDetSpinner(g,11,1,2,{'Coherent promotion score',8,[0 30],0.5,'%.1f','dB'},'tbd_coherent','path_promotion_score_db');
addDetSpinner(g,11,4,5,{'Coherent max seeds/frame',96,[16 512],16,'%.0f','seeds'},'tbd_coherent','max_seeds_per_frame');

% Tracker / object formation
t=uitab(tuneTabs,'Title','Track / Object'); g=uigridlayout(t,[8 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({31},1,8); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetSpinner(g,1,1,2,{'Track confirm hits',2,[1 6],1,'%.0f','hits'},'track','group_confirm_hits');
addDetSpinner(g,1,4,5,{'Track confirmation window',4,[2 10],1,'%.0f','frames'},'track','group_confirm_window');
addDetSpinner(g,2,1,2,{'Maximum missed frames',2,[0 6],1,'%.0f','frames'},'track','group_max_missed');
addDetSpinner(g,2,4,5,{'Tentative max missed',1,[0 5],1,'%.0f','frames'},'track','provisional_max_missed');
addDetSpinner(g,3,1,2,{'Final min hits',2,[1 10],1,'%.0f','hits'},'track','group_final_min_hits');
addDetSpinner(g,3,4,5,{'Final support',0.50,[0 1],0.05,'%.2f','fraction'},'track','group_final_min_support');
addDetSpinner(g,4,1,2,{'Final mean AMF',7,[0 25],0.5,'%.1f','dB'},'track','group_final_mean_amf_db');
addDetSpinner(g,4,4,5,{'Final mean CFAR',4,[0 20],0.5,'%.1f','dB'},'track','group_final_mean_cfar_db');
addDetSpinner(g,5,1,2,{'Final angle support',0.50,[0 1],0.05,'%.2f','fraction'},'track','group_final_angle_support');
addDetSpinner(g,5,4,5,{'Final angle std',8,[0 20],0.5,'%.1f','deg'},'track','group_final_angle_std_deg');
addDetSpinner(g,6,1,2,{'Recovery min hits',5,[2 12],1,'%.0f','hits'},'track','group_recovery_min_hits');
addDetSpinner(g,6,4,5,{'Recovery support',0.80,[0 1],0.05,'%.2f','fraction'},'track','group_recovery_min_support');
addDetSpinner(g,7,1,2,{'Recovery mean AMF',7,[0 25],0.5,'%.1f','dB'},'track','group_recovery_mean_amf_db');
addDetSpinner(g,7,4,5,{'Recovery mean CFAR',4,[0 20],0.5,'%.1f','dB'},'track','group_recovery_mean_cfar_db');
addDetSpinner(g,8,1,2,{'Duplicate angle gate',2.5,[0.5 10],0.25,'%.2f','deg'},'track','duplicate_angle_deg');

% Filtering / interference / stationary controls
t=uitab(tuneTabs,'Title','Filtering'); g=uigridlayout(t,[9 6]); g.Padding=[8 8 8 8]; g.RowHeight=repmat({30},1,9); g.ColumnWidth={190,105,45,190,105,'1x'};
addDetDropdown(g,1,1,2,{'Clutter filter','off|dc_cancel|mti_2tap|mti_3tap|highpass'},'range_processing','clutter_method','dc_cancel');
addDetSpinner(g,1,4,5,{'High-pass alpha',0.92,[0 0.999],0.01,'%.3f',''},'range_processing','highpass_alpha');
addDetCheckbox(g,2,1,2,'Paper coherent subtraction','paper','coherent_subtraction',true);
addDetCheckbox(g,2,4,5,'Stationary detector','paper_stationary','enabled',true);
addDetSpinner(g,3,1,2,{'Stationary threshold scale',10,[1 30],0.5,'%.1f','× background'},'paper_stationary','threshold_scale');
addDetSpinner(g,3,4,5,{'Stationary max detections',64,[8 512],8,'%.0f','points'},'paper_stationary','max_detections');
addDetCheckbox(g,4,1,2,'Hough TF interference mitigation','interference','hough_tf_enabled',true);
addDetSpinner(g,4,4,5,{'Hough candidate ratio',6,[2 20],0.5,'%.1f','×'},'interference','hough_candidate_ratio');
addDetSpinner(g,5,1,2,{'Hough minimum points',8,[2 50],1,'%.0f','points'},'interference','hough_min_points');
addDetSpinner(g,5,4,5,{'Hough minimum score',35,[5 200],5,'%.0f','score'},'interference','hough_min_score');
addDetSpinner(g,6,1,2,{'Hough mask width',0.035,[0.005 0.2],0.005,'%.3f','normalized'},'interference','hough_mask_width');
addDetSpinner(g,6,4,5,{'Hough max mask fraction',0.12,[0.01 0.5],0.01,'%.2f','fraction'},'interference','hough_max_mask_fraction');
addDetSpinner(g,7,1,2,{'Hough safety blend',0.35,[0 1],0.05,'%.2f','fraction'},'interference','hough_safety_blend');
addDetSpinner(g,7,4,5,{'Hough min output fraction',0.45,[0.05 1],0.05,'%.2f','fraction'},'interference','hough_min_output_fraction');
addDetCheckbox(g,8,1,2,'Iterative interference passes','interference','iterative_enabled',true);
addDetSpinner(g,8,4,5,{'Iterative passes',2,[1 5],1,'%.0f','passes'},'interference','iterative_passes');
addDetSpinner(g,9,1,2,{'Minimum power reduction',1,[0 10],0.5,'%.1f','dB'},'interference','iterative_min_power_reduction_db');

% ---------- Right output ----------
rg=uigridlayout(rightPanel,[4 1]); rg.RowHeight={78,48,'1x',42}; rg.Padding=[10 10 10 10]; rg.RowSpacing=8;
banner=uipanel(rg,'Title','Run status','BackgroundColor',[0.96 0.97 0.99]); banner.Layout.Row=1;
bg=uigridlayout(banner,[2 4]); bg.RowHeight={28,'1x'}; bg.ColumnWidth={150,170,125,'1x'}; bg.Padding=[8 5 8 5]; bg.RowSpacing=2;
summary1=makeRadarLabel(bg,'Text','Truth: —','FontWeight','bold'); summary1.Layout.Row=1; summary1.Layout.Column=1;
summary2=makeRadarLabel(bg,'Text','Radar: —','FontWeight','bold'); summary2.Layout.Row=1; summary2.Layout.Column=2;
summary3=makeRadarLabel(bg,'Text','Matched: —','FontWeight','bold'); summary3.Layout.Row=1; summary3.Layout.Column=3;
summary4=makeRadarLabel(bg,'Text','Status: READY','FontWeight','bold'); summary4.Layout.Row=1; summary4.Layout.Column=4;
simTimeLabel=makeRadarLabel(bg,'Text','Sim: 0.000 ms','FontColor',[0.2 0.2 0.2]); simTimeLabel.Layout.Row=2; simTimeLabel.Layout.Column=1;
rateLabel=makeRadarLabel(bg,'Text','Frame: — | FPS: —','FontColor',[0.2 0.2 0.2]); rateLabel.Layout.Row=2; rateLabel.Layout.Column=2;
stageTimeLabel=makeRadarLabel(bg,'Text','Process: —','FontColor',[0.2 0.2 0.2]); stageTimeLabel.Layout.Row=2; stageTimeLabel.Layout.Column=[3 4];

% ---------- Display controls ----------
viewPanel=uipanel(rg,'Title','Display controls','BackgroundColor',C.panel); viewPanel.Layout.Row=2;
vg=uigridlayout(viewPanel,[2 4]); vg.RowHeight={20,20}; vg.ColumnWidth={'1x','1x','1x','1x'}; vg.Padding=[4 2 4 2]; vg.RowSpacing=1; vg.ColumnSpacing=6;
showTruthCB=uicheckbox(vg,'Text','Truth overlay','Value',true); showTruthCB.Layout.Row=1; showTruthCB.Layout.Column=1;
showRadarCB=uicheckbox(vg,'Text','Radar detections','Value',true); showRadarCB.Layout.Row=1; showRadarCB.Layout.Column=2;
showTracksCB=uicheckbox(vg,'Text','Tracks','Value',false); showTracksCB.Layout.Row=1; showTracksCB.Layout.Column=3;
showCFARCB=uicheckbox(vg,'Text','CFAR','Value',false); showCFARCB.Layout.Row=1; showCFARCB.Layout.Column=4;
showAMFCB=uicheckbox(vg,'Text','AMF','Value',false); showAMFCB.Layout.Row=2; showAMFCB.Layout.Column=1;
showGroupsCB=uicheckbox(vg,'Text','Groups','Value',false); showGroupsCB.Layout.Row=2; showGroupsCB.Layout.Column=2;
showTrailsCB=uicheckbox(vg,'Text','Radar history','Value',false); showTrailsCB.Layout.Row=2; showTrailsCB.Layout.Column=3;
showTruthCB.ValueChangedFcn=@displayChanged;
showRadarCB.ValueChangedFcn=@displayChanged;
showTracksCB.ValueChangedFcn=@displayChanged;
showCFARCB.ValueChangedFcn=@displayChanged;
showAMFCB.ValueChangedFcn=@displayChanged;
showGroupsCB.ValueChangedFcn=@displayChanged;
showTrailsCB.ValueChangedFcn=@displayChanged;

tabs=uitabgroup(rg); tabs.Layout.Row=3;
tab=uitab(tabs,'Title','Truth vs Radar'); tg=uigridlayout(tab,[3 1]); tg.RowHeight={215,'1x',22}; tg.Padding=[6 6 6 6]; tg.RowSpacing=4;
resultTable=uitable(tg,'ColumnName',{'Truth','Radar object','Range truth','Range radar','ΔR','Velocity truth','Velocity radar','ΔV','Angle truth','Angle radar','ΔAz','Status'},'RowName',[]); resultTable.Layout.Row=1; resultTable.FontSize=9;
chart=uigridlayout(tg,[1 2]); chart.Layout.Row=2; chart.ColumnWidth={'1.25x','1x'}; chart.Padding=[0 0 0 0]; chart.ColumnSpacing=10; axRA=uiaxes(chart); axV=uiaxes(chart); axRA.Layout.Column=1; axV.Layout.Column=2;
for ax=[axRA axV], grid(ax,'on'); box(ax,'on'); end
xlabel(axRA,'Range (m)'); ylabel(axRA,'Angle (deg)'); title(axRA,'Range-Angle'); xlabel(axV,'Range (m)'); ylabel(axV,'Velocity (m/s)'); title(axV,'Range-Velocity');
plotNote=makeRadarLabel(tg,'Text','o = real / truth    + = radar detection','HorizontalAlignment','center','FontSize',9,'FontColor',[0.25 0.25 0.25]); plotNote.Layout.Row=3;

tabM=uitab(tabs,'Title','Metrics'); mg=uigridlayout(tabM,[9 2]); mg.ColumnWidth={250,'1x'}; mg.RowHeight=repmat({42},1,9); mg.Padding=[15 15 15 15];
metricLabels={'Truth objects','Radar objects','Matched objects','False outputs','Missed objects','Object Pd','Range RMSE','Velocity RMSE','Angle RMSE'}; metricFields=cell(1,9);
for i=1:9, l=makeRadarLabel(mg,'Text',metricLabels{i},'FontWeight','bold'); l.Layout.Row=i; l.Layout.Column=1; e=makeRadarLabel(mg,'Text','—','HorizontalAlignment','right'); e.Layout.Row=i; e.Layout.Column=2; metricFields{i}=e; end

% ---------- Detection heat map ----------
tabHM=uitab(tabs,'Title','Detection Heat Map'); hmGrid=uigridlayout(tabHM,[3 1]); hmGrid.RowHeight={44,'1x',34}; hmGrid.Padding=[8 8 8 8];
hmInfo=makeRadarLabel(hmGrid,'Text','Moving RD | CFAR map | white = verified','FontWeight','bold'); hmInfo.Layout.Row=1;
hmAx=uiaxes(hmGrid); hmAx.Layout.Row=2; grid(hmAx,'on'); xlabel(hmAx,'Radial velocity (m/s)'); ylabel(hmAx,'Range (m)'); title(hmAx,'Moving-Target Detection Heat Map — Final Frame'); colorbar(hmAx);
hmFooter=makeRadarLabel(hmGrid,'Text','No heat map available yet.','HorizontalAlignment','left'); hmFooter.Layout.Row=3;

% ---------- Paper detection view ----------
tabPaper=uitab(tabs,'Title','Paper Detection'); pg=uigridlayout(tabPaper,[2 2]); pg.RowHeight={'1x',34}; pg.ColumnWidth={'1x','1x'}; pg.Padding=[8 8 8 8];
paperMoveAx=uiaxes(pg); paperMoveAx.Layout.Row=1; paperMoveAx.Layout.Column=1; grid(paperMoveAx,'on'); xlabel(paperMoveAx,'Radial velocity (m/s)'); ylabel(paperMoveAx,'Range (m)'); title(paperMoveAx,'Moving path: coherent subtraction → Doppler'); colorbar(paperMoveAx);
paperStatAx=uiaxes(pg); paperStatAx.Layout.Row=1; paperStatAx.Layout.Column=2; grid(paperStatAx,'on'); xlabel(paperStatAx,'Azimuth (deg)'); ylabel(paperStatAx,'Range (m)'); title(paperStatAx,'Stationary path: coherent profile → DBF'); colorbar(paperStatAx);
paperFooter=makeRadarLabel(pg,'Text','Paper maps: moving + stationary','HorizontalAlignment','left'); paperFooter.Layout.Row=2; paperFooter.Layout.Column=[1 2];

% ---------- BEV map ----------
tabMap=uitab(tabs,'Title','BEV Map'); mapGrid=uigridlayout(tabMap,[2 1]); mapGrid.RowHeight={40,'1x'}; mapGrid.Padding=[8 8 8 8];
mapInfo=makeRadarLabel(mapGrid,'Text','F# | red + truth | black o radar','FontWeight','bold'); mapInfo.Layout.Row=1;
bevAx=uiaxes(mapGrid); bevAx.Layout.Row=2; grid(bevAx,'on'); axis(bevAx,'equal'); xlabel(bevAx,'Cross-range x (m)'); ylabel(bevAx,'Forward range y (m)'); title(bevAx,'Bird''s-Eye Radar Object Map');

% ---------- RX / ADC diagnostic ----------
tabRX=uitab(tabs,'Title','RX / ADC'); rxGrid=uigridlayout(tabRX,[3 2]); rxGrid.RowHeight={42,'1x',34}; rxGrid.ColumnWidth={'1x','1x'}; rxGrid.Padding=[8 8 8 8];
rxInfo=makeRadarLabel(rxGrid,'Text','Current frame | dechirped complex RX/ADC data | actual simulated samples','FontWeight','bold'); rxInfo.Layout.Row=1; rxInfo.Layout.Column=[1 2];
rxWaveAx=uiaxes(rxGrid); rxWaveAx.Layout.Row=2; rxWaveAx.Layout.Column=1; grid(rxWaveAx,'on'); xlabel(rxWaveAx,'ADC sample'); ylabel(rxWaveAx,'Amplitude'); title(rxWaveAx,'RX1 | Current Chirp');
rxSpecAx=uiaxes(rxGrid); rxSpecAx.Layout.Row=2; rxSpecAx.Layout.Column=2; grid(rxSpecAx,'on'); xlabel(rxSpecAx,'Beat frequency (MHz)'); ylabel(rxSpecAx,'Power (dB)'); title(rxSpecAx,'RX1 | Current Chirp Spectrum');
rxFooter=makeRadarLabel(rxGrid,'Text','No RX frame yet.','HorizontalAlignment','left'); rxFooter.Layout.Row=3; rxFooter.Layout.Column=[1 2];

% ---------- Diagnostics / performance ----------
tabDiag=uitab(tabs,'Title','Diagnostics'); dgx=uigridlayout(tabDiag,[4 1]); dgx.RowHeight={34,180,'1x',42}; dgx.Padding=[8 8 8 8]; dgx.RowSpacing=6;
dgHeader=makeRadarLabel(dgx,'Text','Live pipeline timing, detector counts, and frame history','FontWeight','bold'); dgHeader.Layout.Row=1;
pipelineTable=uitable(dgx,'ColumnName',{'Stage','State','Time (ms)'},'RowName',[], 'Data',cell(0,3)); pipelineTable.Layout.Row=2; pipelineTable.ColumnWidth={220,120,110};
diagText=uitextarea(dgx,'Editable','off','Value',{'No completed frame yet.','Run LIVE to populate diagnostics.'}); diagText.Layout.Row=3; diagText.FontName='Monospaced'; diagText.FontSize=11;
histGrid=uigridlayout(dgx,[1 6]); histGrid.Layout.Row=4; histGrid.ColumnWidth={95,95,90,90,'1x',120}; histGrid.Padding=[0 0 0 0];
histPrev=uibutton(histGrid,'Text','STEP BACK','ButtonPushedFcn',@historyBack); histPrev.Layout.Column=1; histPrev.Enable='off';
histNext=uibutton(histGrid,'Text','STEP FWD','ButtonPushedFcn',@historyForward); histNext.Layout.Column=2; histNext.Enable='off';
histLive=uibutton(histGrid,'Text','LATEST','ButtonPushedFcn',@historyLive); histLive.Layout.Column=3; histLive.Enable='off';
histFrame=uispinner(histGrid,'Value',1,'Limits',[1 1],'Step',1,'ValueChangedFcn',@historySelect); histFrame.Layout.Column=4; histFrame.Enable='off';
histInfo=makeRadarLabel(histGrid,'Text','History: —','HorizontalAlignment','left'); histInfo.Layout.Column=5;
clearHistBtn=uibutton(histGrid,'Text','CLEAR HISTORY','ButtonPushedFcn',@clearHistory); clearHistBtn.Layout.Column=6; clearHistBtn.Enable='off';

% ---------- Detection diagnostics ----------
tabDetDiag=uitab(tabs,'Title','Detection Diagnostics'); ddg=uigridlayout(tabDetDiag,[3 1]); ddg.RowHeight={260,'1x',42}; ddg.Padding=[8 8 8 8]; ddg.RowSpacing=6;
detDiagTable=uitable(ddg,'ColumnName',{'Target','Range','CFAR','AMF','Groups','Max CFAR','Max AMF','Deepest stage'},'RowName',[],'Data',cell(0,8)); detDiagTable.Layout.Row=1; detDiagTable.ColumnWidth={70,90,60,60,70,90,90,150}; detDiagTable.CellSelectionCallback=@detDiagSelected;
detDiagDetail=uitextarea(ddg,'Editable','off','Value',{'Select a truth target after a completed frame.','The diagnostic is post-detection only and never feeds the radar pipeline.'}); detDiagDetail.Layout.Row=2; detDiagDetail.FontName='Monospaced'; detDiagDetail.FontSize=11;
detDiagFooter=makeRadarLabel(ddg,'Text','No completed-frame diagnostics yet.','HorizontalAlignment','left'); detDiagFooter.Layout.Row=3;

% ---------- Performance ----------
tabPerf=uitab(tabs,'Title','Performance'); pgf=uigridlayout(tabPerf,[3 1]); pgf.RowHeight={180,180,'1x'}; pgf.Padding=[8 8 8 8]; pgf.RowSpacing=6;
perfTable=uitable(pgf,'ColumnName',{'Stage','Time (ms)','Share','Status'},'RowName',[],'Data',cell(0,4)); perfTable.Layout.Row=1; perfTable.ColumnWidth={260,110,110,130};
perfSummary=uitextarea(pgf,'Editable','off','Value',{'No completed frame yet.','Performance compares simulated frame period against end-to-end processing time.'}); perfSummary.Layout.Row=2; perfSummary.FontName='Monospaced'; perfSummary.FontSize=11;
perfFooter=makeRadarLabel(pgf,'Text','Real-time factor: —','HorizontalAlignment','left'); perfFooter.Layout.Row=3;

outFooter=makeRadarLabel(rg,'Text','No run yet. Live mode updates every frame.','HorizontalAlignment','left'); outFooter.Layout.Row=4;

S.controls=[S.controls, {sceneTable,addBtn,remBtn,resetBtn}];
% Live-display state: keep a fixed reference scale and previous frame so frame-to-frame changes are visible.
S.live=struct('refPeak',[],'prevP',[],'frame',0,'displayedFrame',0,'nframes',0,'mode','live','history',{{}},'historyMax',10,'historyIndex',0,'processingActive',false,'paused',false,'lastTiming',struct(),'lastObs',struct([]));
noiseChanged();
    refreshDesign(false);

    function runClicked(~,~)
        if ~guiAlive() || S.busy, return; end
        mutex=getRunMutex();
        try
            if mutex.isLocked()
                setUiState('ERROR','A radar run is already active.');
return;
            end
            mutex.lock();
        catch ME
            setUiState('ERROR',['Radar run mutex unavailable: ' ME.message]);
return;
        end
        runMutexCleanup=onCleanup(@() safeUnlockMutex(mutex));
S.busy=true;
S.stopRequested=false;
runCompleted=false;
runStopped=false;
        try
            % The scene shown in Section 1 is the scene that runs. A new random
            % scene is produced at launch and on demand from the New scene
            % button, never silently at START, so an edited table is honoured.
            c=prepareAndSaveConfig();
            if ~c.ok
S.busy=false;
                setUiState('BLOCKED',c.reason);
return;
            end
            % Pass the exact solved GUI configuration to the causal live loop.
            % This keeps the runtime truth and the Section 1 scene identical.
            setappdata(0,'FMCW_GUI_RUNTIME_P',radar_configuration(c.config));
            setappdata(0,'FMCW_GUI_STOP_REQUEST',false); setappdata(0,'FMCW_GUI_CLOSE_REQUEST',false); setappdata(0,'FMCW_GUI_PAUSED',false); setappdata(0,'FMCW_GUI_STEP_REQUEST',false);
            clearOutputViews(); S.live.refPeak=[]; S.live.prevP=[]; S.live.frame=0; S.lastError='';
            setappdata(0,'FMCW_GUI_PROGRESS_CALLBACK',@updateLiveProgress);
            cleanupProgress=onCleanup(@() safeRmAppdata(0,'FMCW_GUI_PROGRESS_CALLBACK'));
S.live.paused=false;
S.live.processingActive=true;
            setUiState('RUNNING','Live radar is running. STOP safely exits at the next processing checkpoint.');
            run_radar_realtime();
            runStopped=logical(S.stopRequested);
runCompleted=~runStopped;
            safeRmAppdata(0,'FMCW_GUI_RUNTIME_P');
        catch ME
S.lastError=ME.message;
            fprintf(2,'[FMCW GUI] LIVE ERROR: %s\n',ME.message);
            for kk=1:min(numel(ME.stack),8), fprintf(2,'  at %s:%d\n',ME.stack(kk).name,ME.stack(kk).line); end
            if guiAlive()
S.busy=false;
S.live.processingActive=false;
                setUiState('ERROR',sprintf('LIVE ERROR: %s\nThe GUI is ready to recover; last good frame remains available.\nSee Command Window for full stack.',ME.message));
            end
        end
        if ~guiAlive(), return; end
S.busy=false;
S.live.processingActive=false;
        if runCompleted
            setUiState('READY',sprintf('LIVE stopped normally after frame %d.',S.live.frame));
            setStatusDetails('STOPPED',sprintf('Last displayed frame: F%d. History retained: %d frame(s).',S.live.displayedFrame,numel(S.live.history)));
        elseif runStopped
            setUiState('STOPPED',sprintf('STOP completed at frame %d.',S.live.frame));
            setStatusDetails('STOPPED',sprintf('Last displayed frame: F%d. History retained: %d frame(s).',S.live.displayedFrame,numel(S.live.history)));
        end
        refreshHistoryControls();
drawnow;
    end

    function stopClicked(~,~)
        if ~guiAlive() || ~S.live.processingActive, return; end
        try, setappdata(0,'FMCW_GUI_STOP_REQUEST',true); catch, end
S.stopRequested=true;
        setappdata(0,'FMCW_GUI_PAUSED',false); setappdata(0,'FMCW_GUI_STEP_REQUEST',false); S.live.paused=false;
        if strcmpi(S.uiState,'EVAL')
            setUiState('STOPPING','CANCEL EVAL requested. The batch will exit at the next frame-safe checkpoint.');
        else
            setUiState('STOPPING','STOP requested. The current safe checkpoint will exit the live loop; no new frame will start.');
        end
try, drawnow limitrate;
catch, end
    end

    function closeGui(~,~)
        % Immediate GUI shutdown. Detach callbacks first; the running engine
        % observes FMCW_GUI_STOP_REQUEST at frame/stage checkpoints and exits
        % without touching deleted handles.
        try, setappdata(0,'FMCW_GUI_STOP_REQUEST',true); catch, end
        try, setappdata(0,'FMCW_GUI_PAUSED',false); setappdata(0,'FMCW_GUI_STEP_REQUEST',false); catch, end
        try, rmappdata(0,'FMCW_GUI_PROGRESS_CALLBACK'); catch, end
        try, rmappdata(0,'FMCW_GUI_FIGURE'); catch, end
        try, fig.CloseRequestFcn=[]; delete(fig); catch, end
    end

    function pauseClicked(~,~)
        if ~S.live.processingActive || ~strcmpi(S.uiState,'RUNNING') && ~strcmpi(S.uiState,'PAUSED'), return; end
        cur=false; try, cur=logical(getappdata(0,'FMCW_GUI_PAUSED')); catch, end; cur=~cur;
        setappdata(0,'FMCW_GUI_PAUSED',cur); S.live.paused=cur;
        if cur
            setUiState('PAUSED','LIVE paused at a frame boundary. RESUME continues; STEP processes exactly one next frame.');
            if isvalid(pauseBtn), pauseBtn.Text='RESUME'; end
        else
            setUiState('RUNNING','LIVE resumed. Processing continues at the next frame boundary.');
            if isvalid(pauseBtn), pauseBtn.Text='PAUSE'; end
        end
drawnow;
    end
    function stepClicked(~,~)
        if ~S.live.processingActive || ~strcmpi(S.uiState,'PAUSED'), return; end
        setappdata(0,'FMCW_GUI_PAUSED',true); setappdata(0,'FMCW_GUI_STEP_REQUEST',true); S.live.paused=true;
        setUiState('PAUSED','STEP queued. One complete frame will be acquired, processed, tracked, and displayed.');
        if isvalid(pauseBtn), pauseBtn.Text='RESUME'; end
drawnow;
    end
    function displayChanged(~,~)
        if ~isempty(S.live.history)
            showHistorySnapshot(S.live.historyIndex);
        end
    end
    function detDiagSelected(~,e)
        try
            if isempty(e.Indices) || isempty(S.live.lastObs), return; end
            row=e.Indices(1); if row<1 || row>numel(S.live.lastObs), return; end
            d=S.live.lastObs(row);
            cfarHits=getObsField(d,'cfar_frames',getObsField(d,'cfar_hits',0));
            amfHits=getObsField(d,'amf_frames',getObsField(d,'amf_hits',0));
            groupHits=getObsField(d,'group_frames',getObsField(d,'group_hits',0));
            deepest=getObsField(d,'deepest_stage',getObsField(d,'status','ABSENT'));
            if groupHits>0
                reason='Reached grouped point cloud; no final object in this frame.';
            elseif amfHits>0
                reason='AMF evidence present, but verification/grouping did not form an object.';
            elseif cfarHits>0
                reason='CFAR hit, but later verification rejected the candidate.';
            else
                reason='No target-consistent CFAR hit inside the observability gates.';
            end
            detDiagDetail.Value={ ...
                sprintf('Target T%d',getObsField(d,'id',getObsField(d,'target_index',row))), ...
                sprintf('Range %.3f m | Velocity %.3f m/s | Angle %.2f deg', ...
                    getObsField(d,'range',getObsField(d,'truth_range',NaN)), ...
                    getObsField(d,'velocity',getObsField(d,'truth_velocity',NaN)), ...
                    getObsField(d,'angle_deg',getObsField(d,'truth_angle',NaN))), ...
                sprintf('CFAR hits %d | max %.2f dB',cfarHits,getObsField(d,'max_cfar_db',getObsField(d,'max_pre_cfar_db',-Inf))), ...
                sprintf('AMF hits %d',amfHits), ...
                sprintf('Groups %d | max %.2f dB',groupHits,getObsField(d,'max_group_quality_db',-Inf)), ...
                ['Deepest stage: ' char(deepest)], ...
                ['Why: ' reason]};
        catch
        end
    end

    function evalClicked(~,~)
        if ~guiAlive() || S.busy, return; end
        S.busy=true; S.live.processingActive=true; S.lastError='';
        try
            c=prepareAndSaveConfig();
            if ~c.ok
S.busy=false;
S.live.processingActive=false;
                setUiState('BLOCKED',c.reason);
return;
            end
            setappdata(0,'FMCW_GUI_STOP_REQUEST',false);
            setappdata(0,'FMCW_GUI_CLOSE_REQUEST',false);
            setappdata(0,'FMCW_GUI_PROGRESS_CALLBACK',@updateLiveProgress); cleanupProgress=onCleanup(@() safeRmAppdata(0,'FMCW_GUI_PROGRESS_CALLBACK'));
            clearOutputViews(); S.live.refPeak=[]; S.live.prevP=[]; S.live.frame=0;
            setUiState('EVAL',sprintf('Evaluation: %d-frame batch. RUN controls are locked until completion.',round(S.hw.Nframes.spinner.Value)));
            result=run_radar_project();
            if ~isstruct(result) || ~isfield(result,'last'), error('GUI:InvalidResult','Evaluation returned no final result.'); end
            renderFinal(struct('last',result.last));
S.busy=false;
S.live.processingActive=false;
            setUiState('COMPLETE','Evaluation complete. Final results are shown.');
        catch ME
S.lastError=ME.message;
            fprintf(2,'[FMCW GUI] Evaluation message: %s\n',ME.message); for kk=1:min(numel(ME.stack),5), fprintf(2,'  at %s:%d\n',ME.stack(kk).name,ME.stack(kk).line); end
S.busy=false;
S.live.processingActive=false;
            if strcmp(ME.identifier,'run_radar_project:UserStopped')
                setUiState('CANCELLED',sprintf('Evaluation cancelled at frame %d. Any completed diagnostics remain on disk; the GUI is ready to start again.',S.live.frame));
            else
                setUiState('ERROR',sprintf('EVAL ERROR: %s\nThe GUI is unlocked so you can correct the configuration or return to LIVE.\nSee Command Window for full stack.',ME.message));
            end
        end
        if guiAlive(), refreshHistoryControls(); drawnow; end
    end

    function c=prepareAndSaveConfig()
        c=solveConfig(readGUI());
if ~c.ok, return;
end
        scene=readScene();
        if isempty(scene)||any(~isfinite(scene(:))), c.ok=false; c.reason='Scene contains invalid values.'; return; end
        if any(scene(:,1)>c.config.R_max), c.ok=false; c.reason='A scene object exceeds maximum range.'; return; end
        if any(abs(scene(:,2))>c.config.v_max), c.ok=false; c.reason='A scene object exceeds maximum radial velocity.'; return; end
        if any(abs(scene(:,4))>c.config.az_span), c.ok=false; c.reason='A scene object exceeds azimuth span.'; return; end
        c.config.targets=scene; c.config.n_tx=round(S.hw.n_tx.spinner.Value); c.config.n_rx=round(S.hw.n_rx.spinner.Value); c.config.random_seed=round(S.hw.seed.spinner.Value); c.config.N_angle=round(S.hw.N_angle.spinner.Value); c.config.Nframes=round(S.hw.Nframes.spinner.Value); c.config.noise_enabled=logical(S.noise.enabled.Value); c.config.noise_model=char(S.noise.model.Value); c.config.noise_level=S.noise.level.Value; c.config.noise_fixed_power_dBm=S.noise.fixed.Value; c.config.NF_dB=S.noise.NF_dB.Value; c.config.temp_K=S.noise.temp_K.Value; c.config.snr_override_db=S.noise.level.Value; c.config.snr_override_enabled=false; c.config.show_figures=logical(S.hw.show_fig.spinner.Value);
        c.det=readDetectionGUI();
        [detOK,detReason]=validateDetectionGUI(c.det);
if ~detOK, c.ok=false;
c.reason=detReason;
return;
end
        detNames=fieldnames(c.det);
        for di=1:numel(detNames)
            dn=detNames{di};
            if any(strcmp(dn,{'preset','status'})), continue; end
            c.config.(dn)=c.det.(dn);
        end
        config=c.config;
        % Single canonical location. run_radar_project, run_radar_realtime and
        % the experiment layer all read gui/gui_config.mat; writing anywhere
        % else silently detaches the interface from the pipeline.
        cfgDir=fullfile(root,'gui');
        if ~exist(cfgDir,'dir'), mkdir(cfgDir); end
        save(fullfile(cfgDir,'gui_config.mat'),'config','-mat');
        stale=fullfile(root,'gui_config.mat');
        if exist(stale,'file'), delete(stale); end
        applySolved(c,true);
    end

    function lockUI(tf)
        if ~guiAlive(), return; end
        % Global run-state lock: while a run is active, every control is disabled.
        % When the run ends, Section 2 locks and Section 4 hardware locks are
        % respected instead of blindly re-enabling every widget.
        for k=1:numel(S.controls)
            try
                h=S.controls{k};
                if ~isvalid(h), continue; end
                if tf
                    h.Enable='off';
                else
keepOff=false;
                    if isgraphics(h,'matlab.ui.control.Spinner')
                        f=fieldnames(S.params);
                        for ii=1:numel(f)
                            key=f{ii};
                            if isfield(S.params,key) && isequal(S.params.(key).spinner,h)
                                keepOff=logical(S.params.(key).lock.Value);
break;
                            end
                        end
                        if ~keepOff
                            f=fieldnames(S.hw);
                            for ii=1:numel(f)
                                key=f{ii};
                                if isfield(S.hw,key) && isequal(S.hw.(key).spinner,h)
                                    keepOff=logical(S.hw.(key).lock.Value);
break;
                                end
                            end
                        end
                    end
                    if keepOff
                        h.Enable='off';
                    else
                        h.Enable='on';
                    end
                end
            catch
            end
        end
        % Lock checkboxes themselves must remain usable after a run so users
        % can unlock a parameter. They are only globally disabled during a run.
        if ~tf
            try
                f=fieldnames(S.params);
                for ii=1:numel(f), if isvalid(S.params.(f{ii}).lock), S.params.(f{ii}).lock.Enable='on'; end, end
            catch, end
            try
                f=fieldnames(S.hw);
                for ii=1:numel(f), if isvalid(S.hw.(f{ii}).lock), S.hw.(f{ii}).lock.Enable='on'; end, end
            catch, end
        end
        try, if isvalid(runBtn), runBtn.Enable=ternary(tf,'off','on'); end; catch, end
        try, if isvalid(evalBtn), evalBtn.Enable=ternary(tf,'off','on'); end; catch, end
    end
    function parameterChanged(~)
if S.busy, return;
end
        refreshDesign(false);
    end
    function unitChanged(key)
if S.busy, return;
end
        ps=S.params.(key); oldSI=ps.spinner.Value*scale(ps.lastUnit); newUnit=ps.unit.Value; newVal=oldSI/scale(newUnit); lim=unitLimitsSI(key)/scale(newUnit); ps.spinner.Limits=lim; ps.spinner.Value=clamp(newVal,lim(1),lim(2)); ps.lastUnit=newUnit; S.params.(key)=ps; refreshDesign(false);
    end
    function lockChanged(key)
        cb=S.params.(key).lock; if cb.Value, cb.Text='Locked ✓'; cb.FontWeight='bold'; else, cb.Text='Lock'; cb.FontWeight='normal'; end; refreshDesign(false);
    end
    function noiseChanged()
if S.busy, return;
end
        mode=char(S.noise.model.Value);
        isNone=strcmp(mode,'No noise'); isSNR=strcmp(mode,'SNR-controlled AWGN'); isFixed=strcmp(mode,'Fixed AWGN power');
        S.noise.NF_dB.Enable=ternary(isNone,'off','on'); S.noise.temp_K.Enable=ternary(isNone,'off','on');
        S.noise.level.Enable=ternary(isSNR,'on','off'); S.noise.fixed.Enable=ternary(isFixed,'on','off');
        if isSNR, noiseLabel.Text='Noise / SNR'; elseif isFixed, noiseLabel.Text='Noise power'; else, noiseLabel.Text='Noise / SNR'; end
        if isSNR
            noiseUnit.Text='SNR dB'; noiseInfo.Text=sprintf('SNR AWGN | %.1f dB',S.noise.level.Value);
        elseif isFixed
            noiseUnit.Text='dBm'; noiseInfo.Text=sprintf('Fixed AWGN | %.1f dBm/RX',S.noise.fixed.Value);
        elseif isNone
            noiseUnit.Text='—'; noiseInfo.Text='Noise disabled';
        else
            noiseUnit.Text='—'; noiseInfo.Text=sprintf('Thermal AWGN | NF %.1f dB | T %.0f K',S.noise.NF_dB.Value,S.noise.temp_K.Value);
        end
        bw=max(S.params.Fs.spinner.Value*scale(S.params.Fs.unit.Value)/2,eps); thermalW=1.380649e-23*S.noise.temp_K.Value*bw*10^(S.noise.NF_dB.Value/10); noiseBandwidthLabel.Text=sprintf('BW %.2f MHz | thermal %.2f dBm/RX',bw/1e6,10*log10(max(thermalW,realmin)/1e-3));
    end
    function refreshDesign(updateDerived)
        c=solveConfig(readGUI());
        if c.ok
S.designOK=true;
            applySolved(c,updateDerived);
            if ~S.busy || strcmpi(S.uiState,'BLOCKED') || strcmpi(S.uiState,'ERROR')
                setUiState('READY','Configuration is physically consistent and ready to run.');
            end
        else
S.designOK=false;
            if updateDerived || ~S.busy
                D.DesignStatus.Value='NOT POSSIBLE'; D.DesignStatus.BackgroundColor=C.bad;
            end
            if ~S.busy
                setUiState('BLOCKED',c.reason);
            end
        end
    end
    function applySolved(c,showDerived)
        keys2={'fc','R_max','R_res','v_max','v_res','B','slope','Tchirp','Fs','Nr','Nd'};
        for ii=1:numel(keys2)
            k=keys2{ii}; if ~S.params.(k).lock.Value, setSI(k,c.config.(k)); end
        end
        if showDerived
            D.MaximumBeatFrequency.Value=sprintf('%.3f',c.fbeat/1e6);
            D.ComplexIQBeatLimit.Value=sprintf('%.3f',2*c.fbeat/1e6);
            D.ADCLimitedRangeCapability.Value=sprintf('%.2f',c.range_limit);
            D.DopplerUnambiguousVelocity.Value=sprintf('±%.2f',c.v_amb_tdm);
            D.DesignStatus.Value='PHYSICALLY CONSISTENT'; D.DesignStatus.BackgroundColor=C.good;
        end
    end
    function c=solveConfig(x)
        c=struct('ok',false,'reason',''); c.config=x; c.B=[]; c.T=[]; c.slope=[]; c.Fs=[]; c.Nr=[]; c.Nd=[]; c.fbeat=[]; c.range_res_actual=[]; c.vel_res_actual=[]; c.range_limit=[]; c.v_amb=[]; c.v_amb_tdm=[];
fc=x.fc;
lambda=299792458/fc;
tol=2e-4;
        L=@(k) S.params.(k).lock.Value;
        % Range pair: B paired with R_res
        if L('B') && L('R_res')
            if abs(x.B-299792458/(2*x.R_res))>max(tol*x.B,10), c.reason='Locked bandwidth and range resolution are inconsistent.'; return; end
        elseif L('B')
            x.R_res=299792458/(2*x.B);
        else
            x.B=299792458/(2*x.R_res);
        end
        % Velocity pair: Tchirp paired with Vmax via v_amb
        if L('Tchirp') && L('v_max')
            tNeeded=lambda/(4*x.v_max); if abs(x.Tchirp-tNeeded)>max(tol*x.Tchirp,1e-9), c.reason='Locked chirp duration and maximum velocity are inconsistent.'; return; end
        elseif L('Tchirp')
            x.v_max=lambda/(4*x.Tchirp);
        else
            x.Tchirp=lambda/(4*x.v_max);
        end
        % Slope relation B = S*T
        if L('slope')
            if L('B') && L('Tchirp')
                if abs(x.B/x.Tchirp-x.slope)>max(tol*x.slope,1e6), c.reason='Locked slope conflicts with locked bandwidth and chirp duration.'; return; end
            elseif L('B')
                x.Tchirp=x.B/x.slope; if ~L('v_max'), x.v_max=lambda/(4*x.Tchirp); end
            elseif L('Tchirp')
                x.B=x.slope*x.Tchirp; if ~L('R_res'), x.R_res=299792458/(2*x.B); end
            else
                x.B=x.slope*x.Tchirp; x.R_res=299792458/(2*x.B);
            end
        else
x.slope=x.B/x.Tchirp;
        end
        % ADC range constraint: fbeat = S*2R/c + fD.
fD=2*x.v_max/lambda;
fbeat=2*x.slope*x.R_max/299792458+fD;
        if L('Fs')
            if x.Fs <= 2.05*fbeat, c.reason=sprintf('Locked ADC sample rate is too low: %.3f MHz; require > %.3f MHz (2× max beat) for conservative Nyquist margin.',x.Fs/1e6,2*fbeat/1e6); return; end
        else
x.Fs=2.50*fbeat;
        end
        % ADC sample count relation Nr = power-of-two around Fs*T.
minNr=2048;
minFsForNr=minNr/x.Tchirp;
        if ~L('Fs'), x.Fs=max(2.50*fbeat,minFsForNr); end
        requiredNr=2^nextpow2(max(minNr,ceil(x.Fs*x.Tchirp)));
        if L('Nr')
            nr=round(x.Nr); if nr<2048 || bitand(nr,nr-1)~=0, c.reason='Locked ADC samples/chirp must be a power of two >= 2048.'; return; end
            if abs(nr-x.Fs*x.Tchirp)>0.5 && L('Fs') && L('Tchirp')
                c.reason='Locked ADC samples/chirp, sample rate and chirp duration are inconsistent.'; return;
            end
            if ~L('Fs'), x.Fs=nr/x.Tchirp; fbeat=2*x.slope*x.R_max/299792458+fD; if x.Fs<=2.05*fbeat, c.reason=sprintf('Locked ADC sample count with current chirp duration gives %.3f MHz, below the conservative Nyquist requirement %.3f MHz.',x.Fs/1e6,2*fbeat/1e6); return; end, end
        else
x.Nr=requiredNr;
        end
        % Doppler resolution relation v_res = lambda/(2*Nd*T)
        if L('Nd')
            nd=round(x.Nd); if nd<32 || bitand(nd,nd-1)~=0, c.reason='Locked frame length must be a power of two >= 32.'; return; end
            vcalc=lambda/(2*nd*x.Tchirp);
            if L('v_res') && abs(vcalc-x.v_res)>max(tol*x.v_res,1e-6), c.reason='Locked velocity resolution conflicts with locked frame length and chirp duration.'; return; end
            if ~L('v_res'), x.v_res=vcalc; end
        else
            if L('v_res'), x.Nd=2^nextpow2(max(32,ceil(lambda/(2*x.v_res*x.Tchirp))));
            else, x.Nd=128; x.v_res=lambda/(2*x.Nd*x.Tchirp); end
        end
        % Core bounds
        if x.R_max<=x.R_res || x.B<=0 || x.Tchirp<=0 || x.slope<=0 || x.Fs<=0, c.reason='Range, bandwidth and chirp parameters are inconsistent.'; return; end
        if x.v_max>lambda/(4*x.Tchirp)*(1+tol), c.reason='Maximum velocity exceeds the Doppler-unambiguous velocity for the chosen chirp duration.'; return; end
        rangeLimit=299792458*(x.Fs-abs(fD))/(2*x.slope);
        nTxGui=max(1,round(S.hw.n_tx.spinner.Value));
        vAmbTdm=lambda/(4*nTxGui*x.Tchirp);
        if x.R_max>0.98*rangeLimit, c.reason=sprintf('Requested maximum range %.2f m exceeds the current ADC-limited capability %.2f m.',x.R_max,rangeLimit); return; end
        x.Nr=round(x.Nr); x.Nd=round(x.Nd);
        c.config=x; c.B=x.B; c.T=x.Tchirp; c.slope=x.slope; c.Fs=x.Fs; c.Nr=x.Nr; c.Nd=x.Nd; c.fbeat=2*x.slope*x.R_max/299792458+fD; c.range_res_actual=299792458*(x.Fs/x.Nr)/(2*x.slope); c.vel_res_actual=lambda/(2*x.Nd*x.Tchirp); c.range_limit=rangeLimit; c.v_amb=lambda/(4*x.Tchirp); c.v_amb_tdm=vAmbTdm; c.ok=true;
    end
    function x=readGUI()
        x=struct(); f={'fc','R_max','R_res','v_max','v_res','B','slope','Tchirp','Fs','Nr','Nd','az_span'};
        for i=1:numel(f), x.(f{i})=S.params.(f{i}).spinner.Value*scale(S.params.(f{i}).unit.Value); end
    end
    function scene=readScene()
        d=sceneTable.Data; n=size(d,1); scene=zeros(n,4);
        for i=1:n
            scene(i,1)=toNum(d{i,2});
            scene(i,2)=toNum(d{i,3});
            scene(i,3)=pctToRcsDbsm(toNum(d{i,4}));
            scene(i,4)=toNum(d{i,5});
        end
    end
    function setSI(k,v)
        ps=S.params.(k); ps.spinner.Value=clamp(v/scale(ps.unit.Value),ps.spinner.Limits(1),ps.spinner.Limits(2)); S.params.(k)=ps;
    end
    function z=scale(u)
        switch char(u), case 'GHz',z=1e9; case 'MHz',z=1e6; case 'm',z=1; case 'km',z=1e3; case 'cm',z=1e-2; case 'm/s',z=1; case 'km/h',z=1000/3600; case 'MHz/us',z=1e12; case 'GHz/s',z=1e9; case 'us',z=1e-6; case 'ms',z=1e-3; case 'samples',z=1; case 'kSamples',z=1e3; case 'chirps',z=1; case 'kChirps',z=1e3; otherwise,z=1; end
    end
    function lim=unitLimitsSI(k)
        switch k
            case 'fc', lim=[24e9 90e9]; case 'R_max',lim=[10 2000]; case 'R_res',lim=[0.05 20]; case 'v_max',lim=[1 150]; case 'v_res',lim=[0.05 10]; case 'B',lim=[1e6 2e9]; case 'slope',lim=[1e10 1e14]; case 'Tchirp',lim=[1e-6 5e-4]; case 'Fs',lim=[0.5e6 500e6]; case 'Nr',lim=[256 65536]; case 'Nd',lim=[32 8192]; case 'az_span',lim=[10 89]; otherwise,lim=[-inf inf]; end
    end
    function detectionChanged(~,~)
if S.busy, return;
end
        S.det.preset.Value='Custom';
        S.det.status.Text='Custom tuning';
    end
    function applyDetectionPreset(~,~)
        if ~isfield(S,'det') || ~isfield(S.det,'preset'), return; end
        name=char(S.det.preset.Value);
        switch name
            case 'Validated baseline'
                setDet('cfar','Pfa',1e-5); setDet('cfar','weak_snr_db',-6); setDet('cfar','min_snr_db',-20); setDet('detector','min_amf_db',7);
                setDet('tbd','min_path_frames',6); setDet('tbd','min_path_support_fraction',0.65); setDet('tbd','path_min_score',7); setDet('tbd','path_promotion_score',12); setDet('tbd','suppress_near_hard',true); setDet('tbd','near_hard_suppress_amf_db',0);
                setDet('tbd_coherent','min_path_frames',5); setDet('tbd_coherent','min_support_fraction',0.70); setDet('tbd_coherent','path_score_threshold',6); setDet('tbd_coherent','coherent_score_threshold_db',5.5); setDet('tbd_coherent','path_promotion_score_db',8);
                setDet('track','group_recovery_min_hits',5); setDet('track','group_recovery_min_support',0.80); setDet('track','group_recovery_mean_amf_db',7); setDet('track','group_recovery_mean_cfar_db',4); setDet('track','provisional_max_missed',1);
                S.det.status.Text='Validated baseline';
            case 'Balanced recall'
                setDet('cfar','Pfa',1e-5); setDet('cfar','weak_snr_db',-6); setDet('detector','min_amf_db',7);
                setDet('tbd','min_path_frames',4); setDet('tbd','min_path_support_fraction',0.60); setDet('tbd','path_promotion_score',9.5);
                setDet('tbd_coherent','min_path_frames',4); setDet('tbd_coherent','min_support_fraction',0.60); setDet('tbd_coherent','path_promotion_score_db',6.5);
                setDet('track','group_final_mean_amf_db',7); setDet('track','group_final_mean_cfar_db',4); setDet('track','provisional_max_missed',2);
                S.det.status.Text='Balanced recall';
            case 'Strict precision'
                setDet('cfar','Pfa',1e-6); setDet('cfar','weak_snr_db',-4); setDet('detector','min_amf_db',9);
                setDet('tbd','min_path_frames',6); setDet('tbd','min_path_support_fraction',0.70); setDet('tbd','path_promotion_score',12);
                setDet('tbd_coherent','min_path_frames',5); setDet('tbd_coherent','min_support_fraction',0.75); setDet('tbd_coherent','path_promotion_score_db',8);
                setDet('track','group_final_mean_amf_db',8); setDet('track','group_final_mean_cfar_db',5); setDet('track','provisional_max_missed',1);
                S.det.status.Text='Strict precision';
            case 'Weak-target TBD'
                setDet('cfar','Pfa',2e-5); setDet('cfar','weak_snr_db',-9); setDet('detector','min_amf_db',6);
                setDet('tbd','min_path_frames',4); setDet('tbd','min_path_support_fraction',0.55); setDet('tbd','path_min_score',5); setDet('tbd','path_promotion_score',8);
                setDet('tbd_coherent','min_path_frames',4); setDet('tbd_coherent','min_support_fraction',0.55); setDet('tbd_coherent','seed_threshold_db',-14); setDet('tbd_coherent','path_promotion_score_db',5.5);
                setDet('track','group_final_mean_amf_db',6); setDet('track','group_final_mean_cfar_db',3); setDet('track','provisional_max_missed',2);
                S.det.status.Text='Weak-target TBD';
        end
drawnow;
    end
    function setDet(group,key,val)
        if ~isfield(S.det,group), return; end
        h=S.det.(group);
        if isstruct(h) && isfield(h,key)
            if isgraphics(h.(key)), h.(key).Value=val; end
        end
    end
    function out=readDetectionGUI()
        out=struct();
        groups=fieldnames(S.det);
        for gi=1:numel(groups)
            gname=groups{gi}; if any(strcmp(gname,{'preset','status'})), continue; end
            src=S.det.(gname); vals=struct(); if ~isstruct(src), continue; end
            fs=fieldnames(src);
            for fi=1:numel(fs)
                fn=fs{fi}; v=src.(fn);
                if isgraphics(v) && isprop(v,'Value'), vals.(fn)=v.Value; end
            end
            switch gname
                case 'detector_gs'
                    if ~isfield(out,'detector'), out.detector=struct(); end
out.detector.gs=vals;
                case 'tbd_coherent'
                    if ~isfield(out,'tbd'), out.tbd=struct(); end
out.tbd.coherent=vals;
                case 'paper_stationary'
                    if ~isfield(out,'paper'), out.paper=struct(); end
out.paper.stationary=vals;
                otherwise
                    out.(gname)=vals;
            end
        end
    end
    function [ok,reason]=validateDetectionGUI(d)
        ok=true; reason='';
        try
            if d.cfar.Pfa<=0 || d.cfar.Pfa>=1, ok=false; reason='CFAR Pfa must be between 0 and 1.'; return; end
            if d.detector.amf_threshold_pfa<=0 || d.detector.amf_threshold_pfa>=1, ok=false; reason='AMF Pfa must be between 0 and 1.'; return; end
            if d.tbd.min_path_frames<3, ok=false; reason='TBD minimum path length must be at least 3 frames.'; return; end
            if d.tbd.coherent.min_path_frames<3, ok=false; reason='Coherent TBD minimum path length must be at least 3 frames.'; return; end
            if d.tbd.min_path_support_fraction<=0 || d.tbd.min_path_support_fraction>1, ok=false; reason='TBD support fraction is invalid.'; return; end
        catch ME
            ok=false; reason=['Detection controls invalid: ' ME.message];
        end
    end
    function addDetSpinner(g,row,c1,c2,spec,group,key)
        lab=makeRadarLabel(g,'Text',spec{1}); lab.Layout.Row=row; lab.Layout.Column=c1;
        sp=uispinner(g); sp.Value=tunedDefault(group,key,spec{2},spec{3});
        sp.Limits=spec{3}; sp.Step=spec{4}; sp.ValueDisplayFormat=spec{5}; sp.Layout.Row=row; sp.Layout.Column=c1+1;
        u=makeRadarLabel(g,'Text',spec{6}); u.Layout.Row=row; u.Layout.Column=c1+2; u.HorizontalAlignment='left';
sp.ValueChangedFcn=@detectionChanged;
        if ~isfield(S.det,group), S.det.(group)=struct(); end
        S.det.(group).(key)=sp; S.controls{end+1}=sp;
    end
    function addDetDropdown(g,row,c1,c2,spec,group,key,defaultValue)
        lab=makeRadarLabel(g,'Text',spec{1}); lab.Layout.Row=row; lab.Layout.Column=c1;
        items=strsplit(spec{2},'|');
        dv=tunedDefault(group,key,defaultValue,[]);
        if ~any(strcmp(items,char(string(dv)))), dv=defaultValue; end
        dd=uidropdown(g,'Items',items,'Value',char(string(dv))); dd.Layout.Row=row; dd.Layout.Column=c1+1;
        u=makeRadarLabel(g,'Text',''); u.Layout.Row=row; u.Layout.Column=c1+2;
dd.ValueChangedFcn=@detectionChanged;
        if ~isfield(S.det,group), S.det.(group)=struct(); end
        S.det.(group).(key)=dd; S.controls{end+1}=dd;
    end
    function addDetCheckbox(g,row,c1,c2,label,group,key,defaultValue)
        cb=uicheckbox(g,'Text',label,'Value',logical(tunedDefault(group,key,defaultValue,[]))); cb.Layout.Row=row; cb.Layout.Column=[c1 c1+2];
cb.ValueChangedFcn=@detectionChanged;
        if ~isfield(S.det,group), S.det.(group)=struct(); end
        S.det.(group).(key)=cb; S.controls{end+1}=cb;
    end
    function v=tunedDefault(group,key,fallback,limits)
        % Resolve a control's initial value from the live radar model, which
        % already carries any parameters persisted by feedback learning. The
        % literal in the control specification is only a fallback for fields
        % the model does not publish, so the interface always opens showing
        % the parameters a run will actually use.
v=fallback;
        try
            P=modelDefaults();
            switch group
                case 'detector_gs', sec='detector'; sub='gs';
                case 'tbd_coherent', sec='tbd'; sub='coherent';
                case 'paper_stationary', sec='paper'; sub='stationary';
                otherwise, sec=group; sub='';
            end
            if ~isfield(P,sec), return; end
            node=P.(sec);
            if ~isempty(sub)
                if ~isfield(node,sub), return; end
                node=node.(sub);
            end
            if ~isfield(node,key), return; end
            cand=node.(key);
            if isempty(cand), return; end
            if isnumeric(cand) && isscalar(cand) && isfinite(cand)
                v=double(cand);
                if ~isempty(limits) && numel(limits)==2
                    v=min(max(v,limits(1)),limits(2));
                end
            elseif islogical(cand) && isscalar(cand)
v=cand;
            elseif ischar(cand) || isstring(cand)
                v=char(cand);
            end
        catch
        end
    end
    function P=modelDefaults()
        % Built once per session. radar_configuration applies persisted
        % learned parameters as a default layer, so this is the single place
        % the interface has to look.
        persistent cached
        if isempty(cached)
            try, cached=radar_configuration(struct()); catch, cached=struct(); end
        end
P=cached;
    end
    function s=learnedBannerText()
        s='Baseline parameters';
        try
            root=fileparts(fileparts(mfilename('fullpath')));
            lp=fullfile(root,'core','config','learned_defaults.mat');
            if exist(lp,'file')
                d=dir(lp);
                s=sprintf('Learned parameters loaded (%s)',datestr(d.datenum,'yyyy-mm-dd HH:MM'));
            end
        catch
        end
    end
    function addHw(name,key,val,row,unitTxt,limits)
        l=makeRadarLabel(hg,'Text',name); l.Layout.Row=row; l.Layout.Column=1;
        sp=uispinner(hg); sp.Value=val; sp.Limits=limits; sp.Layout.Row=row; sp.Layout.Column=2;
        u=makeRadarLabel(hg,'Text',unitTxt); u.Layout.Row=row; u.Layout.Column=3;
        cb=uicheckbox(hg,'Text','Lock','Value',false); cb.Layout.Row=row; cb.Layout.Column=4;
        S.hw.(key)=struct('spinner',sp,'unit',u,'lock',cb);
        S.controls{end+1}=sp; S.controls{end+1}=cb;
        sp.ValueChangedFcn=@(~,~) hwChanged(key);
        cb.ValueChangedFcn=@(~,~) hwLockChanged(key);
    end
    function hwChanged(key)
        if S.busy || ~isfield(S.hw,key), return; end
        % Hardware values are direct runtime inputs. Refresh the physical
        % design checks so dependent Section 3 diagnostics (notably TDM
        % unambiguous velocity) update immediately.
        refreshDesign(false);
    end
    function hwLockChanged(key)
        if ~isfield(S.hw,key), return; end
        h=S.hw.(key);
        if h.lock.Value
            h.lock.Text='Locked ✓'; h.lock.FontWeight='bold'; h.spinner.Enable='off';
        else
            h.lock.Text='Lock'; h.lock.FontWeight='normal'; h.spinner.Enable='on';
        end
        S.hw.(key)=h;
        refreshDesign(false);
    end
    function sp=addNoise(name,val,row,unitTxt,limits)
        l=makeRadarLabel(ng,'Text',name); l.Layout.Row=row; l.Layout.Column=1; sp=uispinner(ng); sp.Value=val; sp.Limits=limits; sp.Layout.Row=row; sp.Layout.Column=2; u=makeRadarLabel(ng,'Text',unitTxt); u.Layout.Row=row; u.Layout.Column=3; S.controls{end+1}=sp; sp.ValueChangedFcn=@(~,~) noiseChanged();
    end
    function addObject(~,~)
        % Add a fully specified target rather than a placeholder. The row is
        % drawn from the same admissible bounds a random scene uses and is
        % placed clear of the targets already present, so it is immediately
        % runnable without further editing.
        d=sceneTable.Data; n=size(d,1); idx=n+1;
        lim=sceneLimits();
        existingR=zeros(n,1); existingA=zeros(n,1);
        for i=1:n, existingR(i)=toNum(d{i,2}); existingA(i)=toNum(d{i,5}); end
R=lim.Rmin;
A=0;
        for guard=1:200
            R=lim.Rmin+(lim.Rmax-lim.Rmin)*rand;
A=-0.88*lim.azSpan+1.76*lim.azSpan*rand;
if n==0, break;
end
            clash=abs(R-existingR)<2.5 & abs(A-existingA)<4;
            if ~any(clash), break; end
        end
        V=(2*rand-1)*lim.vLimit;
if rand<0.25, V=0;
end
        d(idx,:)={sprintf('Object %d',idx),round(R,2),round(V,2),dbsmToPct(6+12*rand),round(A,2)};
sceneTable.Data=d;
        sceneCount.Text=sprintf('%d objects',idx);
        setappdata(sceneTable,'SelectedRows',idx);
        setStatus(sprintf('Added Object %d at %.1f m, %.1f m/s, %.1f deg.',idx,R,V,A));
    end
    function newRandomScene(~,~)
        sceneTable.Data=randomScene();
        sceneCount.Text=sprintf('%d objects',size(sceneTable.Data,1));
        setStatus('New random scene generated.');
    end
    function removeObject(~,~)
        sel=getappdata(sceneTable,'SelectedRows');
d=sceneTable.Data;
        if isempty(sel), setStatus('Select a scene row first.'); return; end
        if size(d,1)<=1, setStatus('Keep at least one object.'); return; end
        d(sel(1),:)=[];
        for i=1:size(d,1), d{i,1}=sprintf('Object %d',i); end
sceneTable.Data=d;
        sceneCount.Text=sprintf('%d objects',size(d,1));
    end
    function resetScene(~,~)
        sceneTable.Data=defaultScene();
        sceneCount.Text='8 objects';
        setStatus('Fixed reference scene restored.');
    end

    function setUiState(state,msg)
        if ~guiAlive(), return; end
        S.uiState=upper(char(state));
        if nargin<2, msg=''; end
        msg=char(string(msg));
        % Centralized state machine: every pathway updates the same controls.
        % This prevents contradictory states such as BLOCKED while LIVE is processing.
        try
            lockControls=any(strcmp(S.uiState,{'RUNNING','PAUSED','STOPPING','EVAL','STARTING'}));
            lockUI(lockControls);
        catch
        end
        try
            runBtn.Enable='on'; runBtn.Text='START LIVE'; runBtn.BackgroundColor=C.accent;
            pauseBtn.Enable='off'; pauseBtn.Text='PAUSE';
            stepBtn.Enable='off'; stopBtn.Enable='off'; stopBtn.Text='STOP'; evalBtn.Enable='on'; evalBtn.Text=sprintf('EVAL %d',round(S.hw.Nframes.spinner.Value));
            summary4.BackgroundColor=[1 1 1];
        catch
        end
        switch S.uiState
            case 'INITIALIZING'
                setStatus('Initializing GUI'); setStatusDetails('INITIALIZING',msg); pipelineLabel.Text='Pipeline: initializing';
                readyLabel.Text='INIT'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off';
            case 'READY'
                S.designOK=true; setStatus('Design OK'); setStatusDetails('READY',msg); pipelineLabel.Text='Pipeline: idle — ready to start';
                readyLabel.Text='READY'; readyLabel.BackgroundColor=C.good; summary4.Text='Status: READY';
            case 'BLOCKED'
                S.designOK=false; setStatus(['NOT POSSIBLE: ' msg]); setStatusDetails('BLOCKED',msg); pipelineLabel.Text='Pipeline: blocked — correct configuration to continue';
                readyLabel.Text='BLOCKED'; readyLabel.BackgroundColor=C.bad; summary4.Text='Status: BLOCKED'; summary4.BackgroundColor=C.bad; runBtn.Enable='off'; evalBtn.Enable='off';
            case 'STARTING'
                setStatus('Starting live radar'); setStatusDetails('STARTING',msg); pipelineLabel.Text='Pipeline: starting';
                readyLabel.Text='STARTING'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off'; stopBtn.Enable='on';
            case 'RUNNING'
                setStatus('LIVE radar running'); setStatusDetails('LIVE',msg); pipelineLabel.Text='Pipeline: acquiring / processing';
                readyLabel.Text='LIVE'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off'; pauseBtn.Enable='on'; stopBtn.Enable='on'; summary4.Text='Status: LIVE';
            case 'PAUSED'
                setStatus('LIVE paused'); setStatusDetails('PAUSED',msg); pipelineLabel.Text='Pipeline: paused at frame boundary';
                readyLabel.Text='PAUSED'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off'; pauseBtn.Enable='on'; stepBtn.Enable='on'; stopBtn.Enable='on'; summary4.Text='Status: PAUSED';
            case 'STOPPING'
                setStatus('STOP requested'); setStatusDetails('STOPPING',msg); pipelineLabel.Text='Pipeline: stopping safely';
                readyLabel.Text='STOPPING'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off'; stopBtn.Enable='off'; summary4.Text='Status: STOPPING';
            case 'STOPPED'
                setStatus('Live radar stopped'); setStatusDetails('STOPPED',msg); pipelineLabel.Text='Pipeline: stopped — ready to start again';
                readyLabel.Text='STOPPED'; readyLabel.BackgroundColor=C.good; summary4.Text='Status: STOPPED';
            case 'EVAL'
                setStatus('Evaluation running'); setStatusDetails('EVALUATION',msg); pipelineLabel.Text='Pipeline: offline evaluation';
                readyLabel.Text='EVAL'; readyLabel.BackgroundColor=C.warn; runBtn.Enable='off'; evalBtn.Enable='off'; stopBtn.Enable='on'; stopBtn.Text='CANCEL EVAL'; summary4.Text='Status: EVAL';
            case 'COMPLETE'
                setStatus('Evaluation complete'); setStatusDetails('COMPLETE',msg); pipelineLabel.Text='Pipeline: evaluation complete';
                readyLabel.Text='COMPLETE'; readyLabel.BackgroundColor=C.good; summary4.Text='Status: COMPLETE';
            case 'ERROR'
                setStatus('Runtime error'); setStatusDetails('ERROR',msg); pipelineLabel.Text='Pipeline: error — last valid output retained';
                readyLabel.Text='ERROR'; readyLabel.BackgroundColor=C.bad; summary4.Text='Status: ERROR'; summary4.BackgroundColor=C.bad;
            case 'CANCELLED'
                setStatus('Evaluation cancelled'); setStatusDetails('CANCELLED',msg); pipelineLabel.Text='Pipeline: evaluation cancelled — ready to start again';
                readyLabel.Text='CANCELLED'; readyLabel.BackgroundColor=C.warn; summary4.Text='Status: CANCELLED';
            case 'HISTORY'
                setStatus('History review'); setStatusDetails('HISTORY',msg); pipelineLabel.Text='Pipeline: history review';
                readyLabel.Text='HISTORY'; readyLabel.BackgroundColor=C.warn; summary4.Text='Status: HISTORY';
            otherwise
                setStatus('Unknown UI state'); setStatusDetails('ERROR','Unknown UI state. GUI controls reset to a recoverable state.');
                readyLabel.Text='ERROR'; readyLabel.BackgroundColor=C.bad; runBtn.Enable='on'; evalBtn.Enable='on';
        end
        try
            refreshHistoryControls();
        catch
        end
drawnow limitrate;
    end

    function refreshHistoryControls()
        if ~guiAlive(), return; end
        active=S.live.processingActive; hasHist=~isempty(S.live.history);
        en=(~active && hasHist && ~strcmpi(S.uiState,'EVAL'));
        histPrev.Enable=ternary(en,'on','off'); histNext.Enable=ternary(en,'on','off'); histLive.Enable=ternary(en,'on','off'); histFrame.Enable=ternary(en,'on','off'); clearHistBtn.Enable=ternary(en,'on','off');
        if hasHist
            histFrame.Limits=[1 numel(S.live.history)]; histFrame.Value=max(1,min(numel(S.live.history),max(1,S.live.historyIndex))); histInfo.Text=sprintf('History: %d frame(s)',numel(S.live.history));
        else
            histInfo.Text='History: —'; histFrame.Limits=[1 1]; histFrame.Value=1;
        end
    end

    function setStatus(msg)
        if ~guiAlive(), return; end
        msg=char(string(msg));
        try, if isvalid(statusLabel), statusLabel.Text=msg; end; catch, end
    end
    function setStatusDetails(kind,msg)
        if ~guiAlive(), return; end
        try
            k=char(string(kind));
            raw=cellstr(splitlines(string(msg)));
            raw=raw(~cellfun(@(x) isempty(strtrim(x)),raw));
            if isempty(raw), raw={''}; end
            % UI status is intentionally limited to exactly three short lines.
            if numel(raw)>=2
                line2=strtrim(raw{1});
                line3=strtrim(strjoin(raw(2:end),' | '));
            else
                line2=strtrim(raw{1}); line3='';
            end
            % Keep the right-hand status readable; full details remain in Command Window/log.
            line2=compactStatusLine(line2,92);
            line3=compactStatusLine(line3,92);
            statusDetails.Value={k,line2,line3};
        catch
        end
    end
    function s=compactStatusLine(s,n)
        s=char(string(s));
        s=strrep(s,sprintf('\n'),' | ');
        s=strrep(s,char(13),' ');
        s=strtrim(s);
if nargin<2, n=92;
end
        if numel(s)>n, s=[s(1:max(1,n-1)) '…']; end
    end
    function tf=guiAlive()
tf=false;
        try, tf=isvalid(fig) && isvalid(statusLabel); catch, tf=false; end
    end
    function m=getRunMutex()
        if isappdata(0,'FMCW_GUI_RUN_MUTEX')
            m=getappdata(0,'FMCW_GUI_RUN_MUTEX');
            if ~isa(m,'java.util.concurrent.locks.ReentrantLock')
                m=java.util.concurrent.locks.ReentrantLock();
                setappdata(0,'FMCW_GUI_RUN_MUTEX',m);
            end
        else
            m=java.util.concurrent.locks.ReentrantLock();
            setappdata(0,'FMCW_GUI_RUN_MUTEX',m);
        end
    end
    function safeUnlockMutex(m)
        try
            if ~isempty(m) && m.isHeldByCurrentThread(), m.unlock(); end
        catch
        end
    end
    function k=derivedKey(i), kNames={'MaximumBeatFrequency','ComplexIQBeatLimit','ADCLimitedRangeCapability','DopplerUnambiguousVelocity','DesignStatus'}; k=kNames{i}; end
    function v=stepFor(i), v=[0.1 1 0.01 1 0.01 0.1 0.01 0.1 0.1 64 32 1 1]; v=v(i); end
    function x=clamp(x,a,b),x=min(max(x,a),b);end
    function x=toNum(v),if isnumeric(v),x=v;else,x=str2double(string(v));end,end
    function d=defaultScene()
        % Fixed reference scene. Useful when a run has to be reproduced
        % exactly, or when comparing two parameter sets on identical geometry.
        d={'Object 1',30,5,dbsmToPct(10),-50;
           'Object 2',45,-12,dbsmToPct(12),-18;
           'Object 3',65,-8,dbsmToPct(11),35;
           'Object 4',82,7,dbsmToPct(8),10;
           'Object 5',105,14,dbsmToPct(12),-5;
           'Object 6',125,20,dbsmToPct(16),28;
           'Object 7',155,-2,dbsmToPct(13),-36;
           'Object 8',175,4,dbsmToPct(11),50};
    end

    function d=randomScene(n)
        % A fresh, physically admissible scene. Drawn from the clock, so every
        % launch presents a different geometry and a run is never an accidental
        % repeat of the tuning scene. Targets are separated by more than one
        % resolution cell in range or angle, otherwise the truth table itself
        % would be ambiguous and any score computed against it meaningless.
        if nargin<1 || isempty(n), n=randi([5 9]); end
        rng('shuffle');
        lim=sceneLimits();
        R=zeros(n,1); A=zeros(n,1);
        for i=1:n
            for guard=1:200
                R(i)=lim.Rmin+(lim.Rmax-lim.Rmin)*rand;
                A(i)=-0.88*lim.azSpan+1.76*lim.azSpan*rand;
if i==1, break;
end
                clash=abs(R(i)-R(1:i-1))<2.5 & abs(A(i)-A(1:i-1))<4;
                if ~any(clash), break; end
            end
        end
        V=(2*rand(n,1)-1)*lim.vLimit;
        % A quarter of the scene is static: parked cars and roadside structure
        % are the case the stationary branch exists to handle.
        nStat=max(1,round(0.25*n));
        V(randperm(n,nStat))=0;
        RCS=6+12*rand(n,1);
        d=cell(n,5);
        for i=1:n
            d(i,:)={sprintf('Object %d',i),round(R(i),2),round(V(i),2), ...
                    dbsmToPct(RCS(i)),round(A(i),2)};
        end
    end

    function lim=sceneLimits()
        % Physical bounds the scene must respect, read from the current design
        % rather than hard-coded, so editing the waveform moves the limits too.
        lim=struct('Rmin',15,'Rmax',250,'vLimit',20,'azSpan',60);
        try
            cfg=readGUI();
            Rmax=toNum(cfg.R_max); if isfinite(Rmax)&&Rmax>20, lim.Rmax=0.85*Rmax; end
            lim.Rmin=max(10,0.05*lim.Rmax);
            c0=299792458; fcv=toNum(cfg.fc); vmax=toNum(cfg.v_max); nTx=max(1,round(toNum(cfg.n_tx)));
            if isfinite(fcv)&&isfinite(vmax)&&vmax>0
                lam=c0/fcv; Tc=lam/(4*vmax); vAmb=lam/(4*nTx*Tc);
                lim.vLimit=0.85*min(vAmb,vmax);
            end
            az=toNum(cfg.az_span); if isfinite(az)&&az>5, lim.azSpan=min(az,85); end
        catch
        end
    end
    function p=dbsmToPct(dbsm), p=100*min(max(dbsm,0),20)/20; end
    function dbsm=pctToRcsDbsm(p), p=min(max(p,0),100); dbsm=20*p/100; end
    function u=ternary(c,a,b),if c,u=a;else,u=b;end,end

    function clearOutputViews()
        % Hard reset every output view at the START of every run so a second run
        % can never display stale results from the previous run.
        resultTable.Data=cell(0,12);
        summary1.Text='Truth: —'; summary2.Text='Radar objects: —'; summary3.Text='Matched: —'; summary4.Text='Status: RUNNING...';
summary4.BackgroundColor=C.warn;
        try
            cbs=findall(fig,'Type','ColorBar');
            if ~isempty(cbs), delete(cbs(isvalid(cbs))); end
        catch
        end
        for ax=[axRA axV]
            cla(ax,'reset'); ax.Visible='on'; grid(ax,'on');
        end
        xlabel(axRA,'Cross-range x (m)'); ylabel(axRA,'Forward range y (m)'); title(axRA,'Range + Angle — waiting for fresh run');
        xlabel(axV,'Range (m)'); ylabel(axV,'Velocity (m/s)'); title(axV,'Range-Velocity — waiting for fresh run'); plotNote.Text='o = real / truth    + = radar detection';
        cla(hmAx,'reset'); hmAx.Visible='on'; grid(hmAx,'on');
        title(hmAx,'Waiting | Final RD'); xlabel(hmAx,'Radial velocity (m/s)'); ylabel(hmAx,'Range (m)');
        hmFooter.Text='Fresh run | old RD cleared';
        cla(paperMoveAx,'reset'); paperMoveAx.Visible='on'; grid(paperMoveAx,'on');
        cla(paperStatAx,'reset'); paperStatAx.Visible='on'; grid(paperStatAx,'on');
        title(paperMoveAx,'Waiting | Paper Moving'); xlabel(paperMoveAx,'Radial velocity (m/s)'); ylabel(paperMoveAx,'Range (m)');
        title(paperStatAx,'Waiting | Paper Stationary'); xlabel(paperStatAx,'Azimuth (deg)'); ylabel(paperStatAx,'Range (m)');
        paperFooter.Text='Fresh run | old paper maps cleared';
        cla(bevAx,'reset'); bevAx.Visible='on'; grid(bevAx,'on'); axis(bevAx,'equal');
        title(bevAx,'Waiting | BEV'); xlabel(bevAx,'Cross-range x (m)'); ylabel(bevAx,'Forward range y (m)');
        cla(rxWaveAx,'reset'); cla(rxSpecAx,'reset');
        title(rxWaveAx,'Waiting | RX1 Current Chirp'); xlabel(rxWaveAx,'ADC sample'); ylabel(rxWaveAx,'Amplitude');
        title(rxSpecAx,'Waiting | RX1 Spectrum'); xlabel(rxSpecAx,'Beat frequency (MHz)'); ylabel(rxSpecAx,'Power (dB)');
        rxFooter.Text='Fresh run | RX/ADC view cleared';
        S.live.refPeak=[]; S.live.prevP=[]; S.live.frame=0; S.live.displayedFrame=0; S.live.nframes=0; S.live.mode='live'; S.live.history={}; S.live.historyIndex=0; S.live.processingActive=false; S.live.paused=false; S.live.lastTiming=struct(); S.live.lastObs=struct([]); setappdata(0,'FMCW_GUI_PAUSED',false); setappdata(0,'FMCW_GUI_STEP_REQUEST',false); pauseBtn.Enable='off'; pauseBtn.Text='PAUSE'; stepBtn.Enable='off'; readyLabel.Text='LIVE'; readyLabel.BackgroundColor=C.warn;
        statusDetails.Value={'LIVE','Outputs cleared.','Waiting for Frame 1.'}; pipelineLabel.Text='Pipeline: idle'; simTimeLabel.Text='Sim: 0.000 ms'; rateLabel.Text='Frame: — | FPS: —'; stageTimeLabel.Text='Process: —';
        pipelineTable.Data=cell(0,3); diagText.Value={'No completed frame yet.','Run LIVE to populate diagnostics.'}; detDiagTable.Data=cell(0,8); detDiagDetail.Value={'Select a truth target after a completed frame.','The diagnostic is post-detection only and never feeds the radar pipeline.'}; perfTable.Data=cell(0,4); perfSummary.Value={'No completed frame yet.','Performance compares simulated frame period against end-to-end processing time.'}; perfFooter.Text='Real-time factor: —'; S.live.lastObs=struct([]); histFrame.Limits=[1 1]; histFrame.Value=1; histInfo.Text='History: —'; histPrev.Enable='off'; histNext.Enable='off'; histLive.Enable='off'; histFrame.Enable='off'; clearHistBtn.Enable='off';
        outFooter.Text='Fresh run | outputs cleared'; drawnow;
    end

    function updateLiveProgress(payload)
        % Live rendering is atomic per completed frame:
        if ~guiAlive(), return; end
        % keep the last completed frame visible while the next frame is being processed.
        if ~isstruct(payload), return; end
        try
            phase=get_default_field(payload,'phase','processing');
            frame=get_default_field(payload,'frame',0);
            if strcmpi(phase,'paused') || strcmpi(phase,'paused_step')
                if S.live.displayedFrame>0, summary4.Text=sprintf('F%d displayed | PAUSED',S.live.displayedFrame); end
                pipelineLabel.Text='Pipeline: paused at frame boundary'; readyLabel.Text='PAUSED'; readyLabel.BackgroundColor=C.warn; pauseBtn.Text='RESUME'; stepBtn.Enable='on';
                setStatusDetails('PAUSED',sprintf('Frame boundary pause\nLast displayed: F%d\nSTEP processes exactly one next frame.',S.live.displayedFrame)); drawnow limitrate; return;
            end
            if strcmpi(phase,'simulation')
                setStatus(sprintf('LIVE F%d | RX acquisition',frame));
                pipelineLabel.Text=sprintf('Pipeline: F%d | RX acquisition',frame);
                if S.live.displayedFrame>0, summary4.Text=sprintf('F%d displayed | F%d acquiring',S.live.displayedFrame,frame); else, summary4.Text=sprintf('F%d acquiring | no completed frame',frame); end;
                setStatusDetails('LIVE',sprintf('Frame %d\nSim time: %.3f ms\nRX acquisition in progress...',frame,1000*get_default_field(payload,'sim_time_s',(frame-1)*get_default_field(payload,'frame_period_s',0))));
drawnow limitrate;
return;
            end
            if strcmpi(phase,'detection')
                setStatus(sprintf('LIVE F%d | CFAR > GS > AMF > AoA > Track',frame));
                pipelineLabel.Text=sprintf('Pipeline: F%d | CFAR → GS → AMF → AoA → Track',frame);
                if S.live.displayedFrame>0
                    summary4.Text=sprintf('F%d displayed | F%d processing',S.live.displayedFrame,frame);
                    outFooter.Text=sprintf('F%d displayed | processing F%d',S.live.displayedFrame,frame);
                else
                    summary4.Text=sprintf('F%d processing | no completed frame',frame);
                    outFooter.Text=sprintf('Processing F%d | no completed frame',frame);
                end
                setStatusDetails('LIVE',sprintf('Frame %d\nSim time: %.3f ms\nDetection/tracking pipeline in progress...',frame,1000*get_default_field(payload,'sim_time_s',(frame-1)*get_default_field(payload,'frame_period_s',0))));
drawnow limitrate;
return;
            end
            if ~strcmpi(phase,'live_result'), return; end

            % Build one immutable completed-frame snapshot for ALL live views.
            truth=get_default_field(payload,'truth',[]);
            objects=get_default_field(payload,'objects',[]);
            displayObjects=get_default_field(payload,'display_objects',objects);
            hardObjects=get_default_field(payload,'hard_objects',get_default_field(payload,'verified',struct([])));
            tbdObjects=get_default_field(payload,'tbd_objects',struct([]));
            fi=get_default_field(payload,'frame_info',struct());
            p=lastSafeParams(payload);
            P=get_default_field(payload,'Pmove',[]);
            PP=get_default_field(payload,'paperProc',struct());
            timing=get_default_field(payload,'timing',struct());
            stageDataCurrent=get_default_field(payload,'stage_data',struct()); obs=struct([]);
            if ~isempty(truth) && size(truth,2)==4 && isstruct(p) && ~isempty(stageDataCurrent)
                try, [~,obs]=radar_object_evaluation([],truth,{stageDataCurrent},p,[]); catch, obs=struct([]); end
            end
            simTime=get_default_field(payload,'sim_time_s',(frame-1)*get_default_field(payload,'frame_period_s',0));
            framePeriod=get_default_field(payload,'frame_period_s',0);
            wallProc=get_default_field(payload,'wall_processing_s',NaN);
            fps=get_default_field(payload,'processing_fps',NaN);
            if ~isstruct(PP), PP=struct(); end

            if ~isempty(truth)
                truth=double(truth);
                if size(truth,2)~=4 && size(truth,1)==4, truth=truth.'; end
            end
            if isempty(truth) && isstruct(p) && isfield(p,'targets'), truth=double(p.targets); end

guiTic=tic;
            % Render every view from the SAME completed frame. Nothing is cleared
            % while the next frame is being acquired or processed.
            if ~isempty(truth) && size(truth,2)==4
                summary1.Text=sprintf('Truth: %d',size(truth,1));
            else
                summary1.Text='Truth: —';
            end
            summary2.Text=sprintf('Radar objects: %d (Hard %d + TBD %d)',numel(objects),numel(hardObjects),numel(tbdObjects));
            nMatch=countLiveMatches(truth,objects);
            summary3.Text=sprintf('Matched objects: %d | Measurements: %d',nMatch,numel(get_default_field(payload,'hard_measurement_points',hardObjects)));
S.live.nframes=frame;
            simTimeLabel.Text=sprintf('Sim: %.3f ms',1000*simTime);
            rateLabel.Text=sprintf('Frame: F%d | FPS: %s',frame,rmseValue(fps));
            stageTimeLabel.Text=sprintf('Process: %s ms',rmseValue(1000*wallProc));
            setStatusDetails('FRAME COMPLETE',sprintf('Frame %d\nSim time: %.3f ms\nHard objects: %d | TBD objects: %d | Radar objects: %d\nHard measurements: %d | Matched: %d\nProcessing: %s ms\nFrame period: %.3f ms',frame,1000*simTime,numel(hardObjects),numel(tbdObjects),numel(objects),numel(get_default_field(payload,'hard_measurement_points',hardObjects)),nMatch,rmseValue(1000*wallProc),1000*framePeriod));
            updatePipelineDiagnostics(fi,timing,wallProc,numel(hardObjects),numel(tbdObjects),numel(objects));
            updateDetectionDiagnostics(obs,frame);
            updatePerformanceDiagnostics(timing,wallProc,framePeriod,frame);

            renderStageSafely('Live RD',@() renderLiveHeatMap(P,p,fi,frame,displayObjects,payload));
            renderStageSafely('Live Paper',@() renderLivePaper(PP,p,frame,payload));
            if ~isempty(truth) && size(truth,2)==4
                renderStageSafely('Live Range-Angle',@() plotRangeAngleLive(axRA,truth,displayObjects,frame,get_default_field(payload,'cfar_points',struct([])),get_default_field(payload,'amf_points',struct([])),get_default_field(payload,'groups',struct([]))));
                renderStageSafely('Live Velocity',@() plotVelocityLive(axV,truth,displayObjects,frame));
                renderStageSafely('Live BEV',@() renderTruthBEV(truth,displayObjects,p,frame,get_default_field(payload,'cfar_points',struct([])),get_default_field(payload,'amf_points',struct([])),get_default_field(payload,'groups',struct([]))));
                renderStageSafely('Truth vs Radar table',@() updateLiveTruthRadarTable(truth,objects,frame));
                renderStageSafely('Metrics',@() updateMetricsPanel(truth,objects,frame));
            else
                renderStageSafely('Live BEV',@() renderTruthBEV([],objects,p,frame));
            end
            renderStageSafely('Live RX',@() renderLiveRX(payload,p,frame));
            timing.gui_s=toc(guiTic); updatePipelineDiagnostics(fi,timing,wallProc,numel(hardObjects),numel(tbdObjects),numel(objects)); updatePerformanceDiagnostics(timing,wallProc,framePeriod,frame);
            pushHistorySnapshot(struct('frame',frame,'simTime',simTime,'framePeriod',framePeriod,'truth',truth,'objects',objects,'display_objects',displayObjects,'tbd_objects',tbdObjects,'hard_objects',hardObjects,'obs',obs,'Pmove',P,'paperProc',PP,'p',p,'frame_info',fi,'timing',timing,'wallProc',wallProc,'fps',fps,'match',nMatch,'cfar_points',get_default_field(payload,'cfar_points',struct([])),'amf_points',get_default_field(payload,'amf_points',struct([])),'groups',get_default_field(payload,'groups',struct([]))));

            % Commit the displayed-frame pointer only after all views consumed the snapshot.
S.live.frame=frame;
S.live.displayedFrame=frame;
            setStatus(sprintf('LIVE F%d | displayed',frame));
            pipelineLabel.Text=sprintf('Pipeline: F%d | displayed',frame);
            summary4.Text=sprintf('LIVE F%d | displayed',frame);
drawnow limitrate;
        catch ME
            fprintf(2,'[FMCW GUI] Live update warning: %s\n',ME.message);
            for kk=1:min(numel(ME.stack),5), fprintf(2,'  at %s:%d\n',ME.stack(kk).name,ME.stack(kk).line); end
            setStatusDetails('LIVE UPDATE WARNING',sprintf('%s\nLast completed frame remains visible.',ME.message)); try, if guiAlive() && isvalid(outFooter), outFooter.Text='LIVE update warning — last good frame kept'; end, catch, end
        end
    end

    function updatePipelineDiagnostics(fi,timing,wallProc,hardCount,tbdCount,objectCount)
        if nargin<4, hardCount=get_default_field(fi,'live_hard_point_count',0); end
        if nargin<5, tbdCount=get_default_field(fi,'live_tbd_object_count',0); end
        if nargin<6, objectCount=get_default_field(fi,'estimated_cardinality',0); end
        cfarState='DONE'; if isstruct(fi) && isfield(fi,'cfar_info') && isstruct(fi.cfar_info) && isfield(fi.cfar_info,'error'), cfarState='CFAR ERROR'; end
        paperState='DONE'; if isstruct(fi) && isfield(fi,'paper_processing') && isempty(fieldnames(fi.paper_processing)), paperState='NO PAPER DATA'; end
        detectState=cfarState; if ~strcmp(detectState,'DONE'), detectState='DEGRADED'; end
        rows={
            'RX / simulation','DONE',timeText(get_default_field(timing,'rx_generation_s',NaN));
            'Interference / preprocessing', 'DONE', timeText(get_default_field(timing,'preprocess_s',NaN));
            'RD + CFAR + GS + AMF + AoA + tracking/TBD',detectState,timeText(get_default_field(timing,'detection_s',NaN));
            'Paper / stationary path',paperState,'—';
            'GUI render','DONE',timeText(get_default_field(timing,'gui_s',NaN));
            'Total frame processing','DONE',timeText(wallProc)};
pipelineTable.Data=rows;
        pp=get_default_field(fi,'paper_processing',struct());
        if ~isstruct(pp), pp=struct(); end
        diagText.Value={sprintf('Frame: F%d',get_default_field(fi,'frame_index',S.live.frame)), ...
            sprintf('CFAR points: %d',get_default_field(fi,'hard_cfar_count',0)), ...
            sprintf('AMF verified: %d',get_default_field(fi,'amf_verified_count',0)), ...
            sprintf('Groups: %d',get_default_field(fi,'group_count',0)), ...
            sprintf('Accepted measurements: %d',get_default_field(fi,'accepted_measurement_count',0)), ...
            sprintf('TBD weak objects: %d',numel(get_default_field(fi,'live_tbd_objects',struct([])))), ...
            sprintf('TBD candidates / paths: %d / %d',get_default_field(fi,'live_tbd_candidate_count',0),get_default_field(fi,'live_tbd_path_count',0)), ...
            sprintf('TBD object error: %s',get_default_field(fi,'live_tbd_object_error','none')), ...
            sprintf('Paper moving/stationary available: %d / %d',double(isfield(pp,'moving_rd_power')),double(isfield(pp,'stationary_range_angle_power'))), ...
            sprintf('Frame processing: %s ms',timeText(wallProc)), ...
            sprintf('Pipeline: CFAR=%s | hard meas=%d | hard obj=%d | TBD obj=%d | radar obj=%d',cfarState,get_default_field(fi,'accepted_measurement_count',0),hardCount,tbdCount,objectCount) };
    end
    function t=timeText(v)
        if isempty(v) || ~isfinite(v), t='—'; else, t=sprintf('%.2f',1000*v); end
    end
    function updateDetectionDiagnostics(obs,frame)
        % Accept the current radar_object_evaluation schema (obs.per_target)
        % while remaining tolerant of the older flat per-target array schema.
        perTarget=struct([]);
        if isstruct(obs)
            if isfield(obs,'per_target')
perTarget=obs.per_target;
            else
perTarget=obs;
            end
        end
        if isempty(perTarget)
            detDiagTable.Data=cell(0,8); S.live.lastObs=struct([]);
            detDiagFooter.Text=sprintf('F%d | No stage observability data.',frame);
return;
        end
        S.live.lastObs=perTarget(:); rows=cell(numel(perTarget),8);
        for k=1:numel(perTarget)
            d=perTarget(k);
            rows(k,:)={ ...
                sprintf('T%d',getObsField(d,'id',getObsField(d,'target_index',k))), ...
                sprintf('%.2f',getObsField(d,'range',getObsField(d,'truth_range',NaN))), ...
                getObsField(d,'cfar_frames',getObsField(d,'cfar_hits',0)), ...
                getObsField(d,'amf_frames',getObsField(d,'amf_hits',0)), ...
                getObsField(d,'group_frames',getObsField(d,'group_hits',0)), ...
                sprintf('%.2f',getObsField(d,'max_pre_cfar_db',getObsField(d,'max_cfar_db',-Inf))), ...
                sprintf('%.2f',getObsField(d,'max_amf_db',NaN)), ...
                getObsField(d,'deepest_stage',getObsField(d,'status','ABSENT'))};
        end
        detDiagTable.Data=rows; detDiagFooter.Text=sprintf('F%d | Select a truth target row to inspect where the target disappeared.',frame);
    end
    function v=getObsField(s,name,defaultValue)
v=defaultValue;
        try
            if isstruct(s) && isfield(s,name), v=s.(name); end
        catch
        end
    end

    function updatePerformanceDiagnostics(timing,wallProc,framePeriod,frame)
        vals=[get_default_field(timing,'rx_generation_s',NaN),get_default_field(timing,'preprocess_s',NaN),get_default_field(timing,'detection_s',NaN),get_default_field(timing,'gui_s',NaN)]; total=wallProc; names={'RX / acquisition','Preprocess / interference','Detection + tracking','GUI render'}; rows=cell(5,4);
        for k=1:4, rows(k,:)={names{k},round(1000*vals(k),3),sprintf('%.1f%%',100*vals(k)/max(total,eps)),'DONE'}; end
        rows(5,:)={'TOTAL FRAME',round(1000*total,3),'100%','DONE'}; perfTable.Data=rows;
        rtf=framePeriod/max(total,eps); perfSummary.Value={sprintf('Frame F%d',frame),sprintf('Simulation frame period: %.3f ms',1000*framePeriod),sprintf('End-to-end processing: %.3f ms',1000*total),sprintf('Real-time factor: %.2fx',rtf),sprintf('Throughput if unconstrained: %.2f frames/s',1/max(total,eps))}; perfFooter.Text=sprintf('F%d | real-time factor %.2fx | %s',frame,rtf,ternary(rtf>=1,'CAN KEEP UP','PROCESSING SLOWER THAN PHYSICAL CADENCE'));
    end

    function pushHistorySnapshot(snap)
        try
            h=S.live.history; h{end+1}=snap;
            if numel(h)>S.live.historyMax, h=h(end-S.live.historyMax+1:end); end
            S.live.history=h; S.live.historyIndex=numel(h); histFrame.Limits=[1 max(1,numel(h))]; histFrame.Value=max(1,numel(h)); histInfo.Text=sprintf('History: %d frame(s)',numel(h));
        catch
        end
    end
    function historyBack(~,~)
        % Stepping is a display operation. It stays available whenever frames
        % have been stored, including immediately after a stop, which is when
        % it is most useful.
        if isempty(S.live.history), return; end
        showHistorySnapshot(max(1,S.live.historyIndex-1));
    end
    function historyForward(~,~)
        if isempty(S.live.history), return; end
        showHistorySnapshot(min(numel(S.live.history),S.live.historyIndex+1));
    end
    function historySelect(~,~)
        if isempty(S.live.history), return; end
        showHistorySnapshot(max(1,min(numel(S.live.history),round(histFrame.Value))));
    end
    function historyLive(~,~)
        if isempty(S.live.history), return; end
        S.live.historyIndex=numel(S.live.history); histFrame.Value=S.live.historyIndex; showHistorySnapshot(S.live.historyIndex); setStatus('History view: latest completed frame. Start LIVE to resume real-time display.');
    end
    function showHistorySnapshot(idx)
        % Every view is redrawn from the stored snapshot, so stepping through
        % history shows a complete frame rather than a partial overlay on the
        % last live one. The selected tab is left alone: stepping while looking
        % at the bird's-eye map should keep showing the bird's-eye map.
        if idx<1 || idx>numel(S.live.history), return; end
        snap=S.live.history{idx};
        S.live.historyIndex=idx;
        histFrame.Value=idx;
        histInfo.Text=sprintf('History: F%d (%d/%d)',snap.frame,idx,numel(S.live.history));

        truth=snap.truth;
        objects=snap.objects;
        displayObjects=get_default_field(snap,'display_objects',objects);
        p=snap.p; frame=snap.frame; fi=snap.frame_info;
        P=snap.Pmove; PP=snap.paperProc;
        cfarPts=get_default_field(snap,'cfar_points',struct([]));
        amfPts=get_default_field(snap,'amf_points',struct([]));
        groups=get_default_field(snap,'groups',struct([]));
        timing=get_default_field(snap,'timing',struct());
        wallProc=get_default_field(snap,'wallProc',NaN);
        framePeriod=get_default_field(snap,'framePeriod',NaN);
        payload=struct('clean_cube',get_default_field(snap,'clean_cube',[]), ...
                       'rx_cube',get_default_field(snap,'rx_cube',[]), ...
                       'cfar_points',cfarPts,'amf_points',amfPts,'groups',groups);

        renderStageSafely('History table',@() updateLiveTruthRadarTable(truth,objects,frame));
        renderStageSafely('History metrics',@() updateMetricsPanel(truth,objects,frame));
        renderStageSafely('History Range-Angle',@() plotRangeAngleLive(axRA,truth,displayObjects,frame,cfarPts,amfPts,groups));
        renderStageSafely('History Velocity',@() plotVelocityLive(axV,truth,displayObjects,frame));
        renderStageSafely('History RD',@() renderLiveHeatMap(P,p,fi,frame,displayObjects,payload));
        renderStageSafely('History Paper',@() renderLivePaper(PP,p,frame,payload));
        renderStageSafely('History BEV',@() renderTruthBEV(truth,displayObjects,p,frame,cfarPts,amfPts,groups));
        renderStageSafely('History diagnostics',@() updateDetectionDiagnostics(get_default_field(snap,'obs',struct([])),frame));
        renderStageSafely('History pipeline',@() updatePipelineDiagnostics(fi,timing,wallProc, ...
            numel(get_default_field(snap,'hard_objects',objects)), ...
            numel(get_default_field(snap,'tbd_objects',struct([]))),numel(objects)));
        if isfinite(wallProc) && isfinite(framePeriod)
            renderStageSafely('History performance',@() updatePerformanceDiagnostics(timing,wallProc,framePeriod,frame));
        end
        if ~isempty(get_default_field(snap,'rx_cube',[]))
            renderStageSafely('History RX',@() renderLiveRX(payload,p,frame));
        end

        simTimeLabel.Text=sprintf('Sim: %.3f ms',1000*snap.simTime);
        rateLabel.Text=sprintf('History F%d | %.2f ms',frame,1000*wallProc);
        stageTimeLabel.Text='History view';
        summary4.Text=sprintf('HISTORY F%d',frame);
        histPrev.Enable=ternary(idx>1,'on','off');
        histNext.Enable=ternary(idx<numel(S.live.history),'on','off');
        setUiState('HISTORY',sprintf('Viewing stored frame F%d of %d\nSim time: %.3f ms\nUse the arrows or the slider to step; LATEST returns to the newest frame.', ...
            frame,numel(S.live.history),1000*snap.simTime));
        drawnow;
    end
    function clearHistory(~,~)
if S.live.processingActive, return;
end
        S.live.history={}; S.live.historyIndex=0; histFrame.Limits=[1 1]; histFrame.Value=1; histInfo.Text='History: —'; histPrev.Enable='off'; histNext.Enable='off'; histLive.Enable='off'; histFrame.Enable='off'; clearHistBtn.Enable='off';
        setStatusDetails('HISTORY','Stored frame history cleared.'); pipelineLabel.Text='Pipeline: history cleared'; refreshHistoryControls();
    end

    function renderLiveHeatMap(P,p,fi,frame,dets,payload)
        if (isempty(P) || ~isnumeric(P)) && isstruct(payload) && isfield(payload,'clean_cube') && ~isempty(payload.clean_cube)
            try, [P,~,~,~]=localDisplayMapsFromCube(payload.clean_cube,p); catch, P=[]; end
        end
        if isempty(P)
            title(hmAx,sprintf('LIVE F%d | RD | unavailable',frame)); xlabel(hmAx,'Radial velocity (m/s)'); ylabel(hmAx,'Range (m)'); hmFooter.Text=sprintf('F%d | RD unavailable',frame); return; end
        P=double(real(P)); P(~isfinite(P))=0; P=max(P,0);
        peak=max(P(:)); if isempty(peak)||~isfinite(peak)||peak<=0, peak=1; end
        if isempty(S.live.refPeak), S.live.refPeak=peak; end
        if ~isempty(S.live.prevP) && isequal(size(S.live.prevP),size(P))
            deltaDb=10*log10(max(max(abs(P-S.live.prevP)),eps)/max(peak,eps));
        else
deltaDb=NaN;
        end
        showHeatmapSafeFixed(hmAx,P,p.vel_axis,p.range_axis,[-60 0],max(S.live.refPeak,eps));
        hold(hmAx,'on');
        if showRadarCB.Value && ~isempty(dets)
            dv=arrayfun(@(d)d.velocity,dets(:)); dr=arrayfun(@(d)d.range,dets(:));
            good=isfinite(dv)&isfinite(dr); if any(good), plot(hmAx,dv(good),dr(good),'wo','LineStyle','none','MarkerSize',6,'LineWidth',1.2); end
        end
        if showCFARCB.Value && isstruct(payload) && isfield(payload,'cfar_points') && ~isempty(payload.cfar_points)
            cdv=arrayfun(@(d)d.velocity,payload.cfar_points(:)); cdr=arrayfun(@(d)d.range,payload.cfar_points(:)); good=isfinite(cdv)&isfinite(cdr); if any(good), plot(hmAx,cdv(good),cdr(good),'m.','MarkerSize',8,'HandleVisibility','off'); end
        end
        if showAMFCB.Value && isstruct(payload) && isfield(payload,'amf_points') && ~isempty(payload.amf_points)
            adv=arrayfun(@(d)d.velocity,payload.amf_points(:)); adr=arrayfun(@(d)d.range,payload.amf_points(:)); good=isfinite(adv)&isfinite(adr); if any(good), plot(hmAx,adv(good),adr(good),'c.','MarkerSize',8,'HandleVisibility','off'); end
        end
        if showGroupsCB.Value && isstruct(payload) && isfield(payload,'groups') && ~isempty(payload.groups)
            gdv=arrayfun(@(g)g.velocity,payload.groups(:)); gdr=arrayfun(@(g)g.range,payload.groups(:)); good=isfinite(gdv)&isfinite(gdr); if any(good), plot(hmAx,gdv(good),gdr(good),'gd','LineStyle','none','MarkerSize',5,'HandleVisibility','off'); end
        end
        hold(hmAx,'off');
        title(hmAx,sprintf('F%d | RD',frame),'FontWeight','bold');
        xlabel(hmAx,'Radial velocity (m/s)'); ylabel(hmAx,'Range (m)');
        if isfinite(deltaDb)
            hmFooter.Text=sprintf('F%d | CFAR %d | AMF %d | Groups %d | dRD %.1f dB',frame,get_default_field(fi,'hard_cfar_count',0),get_default_field(fi,'amf_verified_count',0),get_default_field(fi,'group_count',0),deltaDb);
        else
            hmFooter.Text=sprintf('F%d | CFAR %d | AMF %d | Groups %d | first frame',frame,get_default_field(fi,'hard_cfar_count',0),get_default_field(fi,'amf_verified_count',0),get_default_field(fi,'group_count',0));
        end
S.live.prevP=P;
S.live.frame=frame;
    end

    function renderLivePaper(PP,p,frame,payload)
        if isempty(p), return; end
        if (~isfield(PP,'moving_rd_power') || isempty(PP.moving_rd_power)) && isstruct(payload) && isfield(payload,'clean_cube') && ~isempty(payload.clean_cube)
            try, [Pm,~,~,PPfb]=localDisplayMapsFromCube(payload.clean_cube,p); PP=mergePaperFallback(PP,PPfb,Pm); catch, end
        end
        if isfield(PP,'moving_rd_power') && ~isempty(PP.moving_rd_power)
            Q=double(real(PP.moving_rd_power)); Q(~isfinite(Q))=0; Q=max(Q,0);
            pk=max(Q(:)); if isempty(pk)||pk<=0, pk=1; end
            if isempty(S.live.refPeak), S.live.refPeak=pk; end
            showHeatmapSafeFixed(paperMoveAx,Q,p.vel_axis,p.range_axis,[-50 0],max(S.live.refPeak,eps));
            title(paperMoveAx,sprintf('F%d | Paper Moving',frame),'FontWeight','bold');
            xlabel(paperMoveAx,'Radial velocity (m/s)'); ylabel(paperMoveAx,'Range (m)');
        end
        if isfield(PP,'stationary_range_angle_power') && ~isempty(PP.stationary_range_angle_power)
            Q=double(real(PP.stationary_range_angle_power)); Q(~isfinite(Q))=0; Q=max(Q,0);
            pk=max(Q(:)); if isempty(pk)||pk<=0, pk=1; end
            showHeatmapSafeFixed(paperStatAx,Q,p.theta_axis,p.range_axis,[-50 0],max(S.live.refPeak,pk));
            title(paperStatAx,sprintf('F%d | Paper Stationary',frame),'FontWeight','bold');
            xlabel(paperStatAx,'Azimuth (deg)'); ylabel(paperStatAx,'Range (m)');
        end
        paperFooter.Text=sprintf('Frame %d | paper moving + stationary',frame);
    end

    function PP=mergePaperFallback(PP,FB,Pm)
        if ~isstruct(PP), PP=struct(); end
        if isstruct(FB)
            if ~isfield(PP,'moving_rd_power') || isempty(PP.moving_rd_power), PP.moving_rd_power=FB.moving_rd_power; end
            if ~isfield(PP,'stationary_range_angle_power') || isempty(PP.stationary_range_angle_power), PP.stationary_range_angle_power=FB.stationary_range_angle_power; end
        end
        if (~isfield(PP,'moving_rd_power') || isempty(PP.moving_rd_power)) && ~isempty(Pm), PP.moving_rd_power=Pm; end
    end

    function [Pmove,Pref,auxSum,PP]=localDisplayMapsFromCube(cleanCube,p)
        Pmove=zeros(p.Nr,p.Nd); Pref=zeros(p.Nr,p.Nd); auxSum=cell(1,p.n_rx);
        for rx=1:p.n_rx
            [~,~,Prd,~,aux]=range_doppler_processor(cleanCube(:,:,rx),p,struct('keystone',p.range_processing.keystone,'clutter_method','off'));
            Pmove=Pmove+max(real(Prd),0); Pref=Pref+max(real(Prd),0); auxSum{rx}=aux;
        end
        if isfield(p,'paper') && p.paper.enabled
            PP=moving_stationary_separator(cleanCube,p); Pmove=PP.moving_rd_power;
        else
            PP=struct('moving_rd_power',Pmove,'stationary_range_power',zeros(p.Nr,1),...
                'stationary_range_angle_power',zeros(p.Nr,numel(p.theta_axis)));
        end
        Pmove(~isfinite(Pmove))=0; Pref(~isfinite(Pref))=0;
    end

    function p=lastSafeParams(payload)
        p=[]; if isfield(payload,'pf') && isstruct(payload.pf), p=payload.pf; end
    end

    function renderLiveRX(payload,p,frame)
        try
            X=get_default_field(payload,'rx_cube',[]);
            if isempty(X) || ndims(X)~=3 || isempty(p), return; end
rx=1;
chirp=1;
            if size(X,3)<rx || size(X,2)<chirp, return; end
            z=X(:,chirp,rx); z=z(:); n=numel(z);
            cla(rxWaveAx,'reset'); grid(rxWaveAx,'on'); hold(rxWaveAx,'on');
            plot(rxWaveAx,1:n,abs(z),'LineWidth',1.0);
            title(rxWaveAx,sprintf('LIVE Frame %d | RX1 | dechirped ADC chirp 1',frame));
            xlabel(rxWaveAx,'ADC sample'); ylabel(rxWaveAx,'|x[n]|');
            cla(rxSpecAx,'reset'); grid(rxSpecAx,'on');
            N=numel(z); w=0.5-0.5*cos(2*pi*(0:N-1)'/max(N,1)); Y=fftshift(fft(z.*w));
            f=(-floor(N/2):ceil(N/2)-1)*(p.fs_ADC/N)/1e6;
            P=20*log10(abs(Y)/max(max(abs(Y)),eps));
            plot(rxSpecAx,f,P,'LineWidth',1.0);
            title(rxSpecAx,sprintf('LIVE F%d | RX1 | beat spectrum',frame));
            xlabel(rxSpecAx,'Beat frequency (MHz)'); ylabel(rxSpecAx,'Relative dB');
            rxFooter.Text=sprintf('Frame %d | Fs %.2f MHz | Nr %d | Nd %d | RX %d | noise %.2f dBm/RX',frame,p.fs_ADC/1e6,p.Nr,p.Nd,p.n_rx,10*log10(max(mean(p.rx_noise_power_W),realmin)/1e-3));
        catch ME
            rxFooter.Text=['RX view: ' ME.message];
        end
    end

    function plotRangeAngleLive(ax,truth,dets,frame,cfarPts,amfPts,groups)
        % LIVE comparison visualization only: current-frame truth (red o) and
        % current-frame radar detections (black +). No IDs, legends, lines, or
        % truth-to-radar association is shown.
        cla(ax,'reset'); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        truth=double(truth);
        if ~isempty(truth) && size(truth,2)~=4 && size(truth,1)==4, truth=truth.'; end
        if isempty(truth) || size(truth,2)~=4, truth=zeros(0,4); end

        trR=truth(:,1); trA=truth(:,4);
        [rdR,rdA]=objectRangeAngleVectors(dets);
        gt=isfinite(trR)&isfinite(trA);
        gr=isfinite(rdR)&isfinite(rdA);

        if any(gt)
            plot(ax,trR(gt),trA(gt),'ro','LineStyle','none','MarkerSize',7,'LineWidth',1.3);
        end
        if any(gr)
            plot(ax,rdR(gr),rdA(gr),'k+','LineStyle','none','MarkerSize',9,'LineWidth',1.5);
        end

maxR=10;
        if any(gt), maxR=max(maxR,max(trR(gt))); end
        if any(gr), maxR=max(maxR,max(rdR(gr))); end
        maxR=max(10,ceil(1.05*maxR));
        xlim(ax,[0 maxR]);
        angVals=[trA(isfinite(trA)); rdA(isfinite(rdA))];
        if isempty(angVals)
az=90;
        else
            az=max(10,1.10*max(abs(angVals)));
        end
        ylim(ax,[-az az]);
        xlabel(ax,'Range (m)');
        ylabel(ax,'Angle (deg)');
        title(ax,sprintf('F%d',frame),'FontWeight','bold');
        hold(ax,'off');
    end

    function plotVelocityLive(ax,truth,dets,frame)
        % LIVE velocity visualization: current-frame truth (red o) and
        % current-frame radar outputs (black +), both plotted against range.
        cla(ax,'reset'); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        truth=double(truth);
        if ~isempty(truth) && size(truth,2)~=4 && size(truth,1)==4, truth=truth.'; end
        if isempty(truth) || size(truth,2)~=4, truth=zeros(0,4); end

        trR=truth(:,1); trV=truth(:,2);
        [rdR,rdV]=objectRangeVelocityVectors(dets);
        gt=isfinite(trR)&isfinite(trV);
        gr=isfinite(rdR)&isfinite(rdV);

        if any(gt)
            plot(ax,trR(gt),trV(gt),'ro','LineStyle','none','MarkerSize',7,'LineWidth',1.3);
        end
        if any(gr)
            plot(ax,rdR(gr),rdV(gr),'k+','LineStyle','none','MarkerSize',9,'LineWidth',1.5);
        end

maxR=10;
        if any(gt), maxR=max(maxR,max(trR(gt))); end
        if any(gr), maxR=max(maxR,max(rdR(gr))); end
        xlim(ax,[0 max(10,ceil(1.05*maxR))]);

        vals=[];
        if any(gt), vals=[vals; trV(gt)]; end
        if any(gr), vals=[vals; rdV(gr)]; end
        if isempty(vals)
            ylim(ax,[-10 10]);
        else
            lo=min(vals); hi=max(vals);
            lo=min(lo,0); hi=max(hi,0);
            span=max(hi-lo,1);
            pad=max(0.10*span,0.75);
            ylim(ax,[lo-pad hi+pad]);
        end
        % Always show a numeric 0 tick for quick visual interpretation.
        yt=yticks(ax);
        yticks(ax,sort(unique([yt(:);0])));
        xlabel(ax,'Range (m)');
        ylabel(ax,'Velocity (m/s)');
        title(ax,sprintf('F%d',frame),'FontWeight','bold');
        hold(ax,'off');
    end

    function [r,a]=objectRangeAngleVectors(dets)
        dets=dets(:);
        r=nan(numel(dets),1); a=nan(numel(dets),1);
        for kk=1:numel(dets)
            r(kk)=getScalarFieldLocal(dets(kk),'range');
            a(kk)=getScalarFieldLocal(dets(kk),'angle_deg');
        end
    end

    function [r,v]=objectRangeVelocityVectors(dets)
        dets=dets(:);
        r=nan(numel(dets),1); v=nan(numel(dets),1);
        for kk=1:numel(dets)
            r(kk)=getScalarFieldLocal(dets(kk),'range');
            v(kk)=getScalarFieldLocal(dets(kk),'velocity');
        end
    end

    function [r,a,v]=objectRAVVectors(dets)
        dets=dets(:);
        r=nan(numel(dets),1); a=nan(numel(dets),1); v=nan(numel(dets),1);
        for kk=1:numel(dets)
            r(kk)=getScalarFieldLocal(dets(kk),'range');
            a(kk)=getScalarFieldLocal(dets(kk),'angle_deg');
            v(kk)=getScalarFieldLocal(dets(kk),'velocity');
        end
    end

    function v=getScalarFieldLocal(s,name)
v=NaN;
        if ~isstruct(s) || ~isfield(s,name), return; end
        q=s.(name);
        if isempty(q) || ~isnumeric(q), return; end
        if isscalar(q), v=double(q); else, v=double(q(1)); end
    end

    function n=countLiveMatches(truth,objects)
n=0;
        if isempty(truth), return; end
        try
            E=radar_object_evaluation(objects,truth);
            if isfield(E,'matched_count'), n=double(E.matched_count); end
        catch
        end
    end


    function updateMetricsPanel(truth,objects,frame)
        % The Metrics tab reports the same scored result the offline evaluator
        % produces, so a number shown here and a number in a campaign summary
        % mean the same thing. Scoring is a one-to-one assignment under the
        % configured gate, not a nearest-neighbour count, which is why a radar
        % object can be present and still be reported as a false output.
        truth=double(truth);
        if isempty(truth) || size(truth,2)~=4
            for i=1:numel(metricFields), metricFields{i}.Text='—'; end
            metricFields{2}.Text=sprintf('%d',numel(objects));
            return;
        end
        try
            E=radar_object_evaluation(objects,truth);
        catch
            for i=1:numel(metricFields), metricFields{i}.Text='—'; end
            return;
        end
        nT=size(truth,1); nR=numel(objects);
        nM=get_default_field(E,'matched',0);
        vals={ sprintf('%d',nT), ...
               sprintf('%d',nR), ...
               sprintf('%d',nM), ...
               sprintf('%d',max(0,nR-nM)), ...
               sprintf('%d',max(0,nT-nM)), ...
               sprintf('%.1f %%',100*get_default_field(E,'pd',0)), ...
               fmtMetric(get_default_field(E,'range_rmse',NaN),'%.3f m'), ...
               fmtMetric(get_default_field(E,'velocity_rmse',NaN),'%.3f m/s'), ...
               fmtMetric(get_default_field(E,'angle_rmse',NaN),'%.2f deg') };
        for i=1:min(numel(metricFields),numel(vals))
            metricFields{i}.Text=vals{i};
        end
        % A false output with no missed target usually means one physical
        % response was reported twice; a false output alongside a missed
        % target usually means the match gate was exceeded. Colour the row so
        % the difference is visible without reading the table.
        if max(0,nR-nM)>0
            metricFields{4}.FontColor=[0.72 0.15 0.12];
        else
            metricFields{4}.FontColor=[0.10 0.45 0.20];
        end
        S.live.lastMetricsFrame=frame;
    end
    function s=fmtMetric(v,fmt)
        if isempty(v) || ~isfinite(v), s='—'; else, s=sprintf(fmt,v); end
    end
    function updateLiveTruthRadarTable(truth,objects,frame)
        % Truth rows first. Unmatched radar objects are explicitly labeled R#.
        truth=double(truth);
        if isempty(truth) || size(truth,2)~=4, return; end
        try, E=radar_object_evaluation(objects,truth); catch, E=struct('assignment',zeros(size(truth,1),1)); end
        assign=zeros(size(truth,1),1);
        if isfield(E,'assignment') && numel(E.assignment)>=size(truth,1), assign=E.assignment(:); end
        used=false(1,numel(objects)); rows=cell(size(truth,1)+numel(objects),12); rr=0;
        for t=1:size(truth,1)
            rr=rr+1; d=assign(t);
            if d>0 && d<=numel(objects)
                used(d)=true; z=objects(d); er=z.range-truth(t,1); ev=z.velocity-truth(t,2); ea=wrap180(z.angle_deg-truth(t,4));
                rows(rr,:)={sprintf('T%d',t),sprintf('R%d',d),truth(t,1),z.range,er,truth(t,2),z.velocity,ev,truth(t,4),z.angle_deg,ea,'MATCH'};
            else
                rows(rr,:)={sprintf('T%d',t),'—',truth(t,1),NaN,NaN,truth(t,2),NaN,NaN,truth(t,4),NaN,NaN,'MISSED'};
            end
        end
        for d=1:numel(objects)
            if used(d), continue; end
            rr=rr+1; z=objects(d);
            rows(rr,:)={sprintf('R%d',d),sprintf('R%d',d),NaN,z.range,NaN,NaN,z.velocity,NaN,NaN,z.angle_deg,NaN,'RADAR-ONLY'};
        end
        resultTable.Data=rows(1:rr,:);
        nMatched=sum(used); nRadar=numel(objects); nFalse=sum(~used); nMiss=max(0,size(truth,1)-nMatched);
        summary1.Text=sprintf('Truth: %d',size(truth,1)); summary2.Text=sprintf('Radar objects: %d',nRadar); summary3.Text=sprintf('Matched: %d',nMatched);
        summary4.Text=sprintf('LIVE F%d | displayed',frame); summary4.BackgroundColor=C.good;
        outFooter.Text=sprintf('LIVE F%d | Truth %d | Radar %d | matched %d | missed %d | radar-only %d',frame,size(truth,1),nRadar,nMatched,nMiss,nFalse);
    end

    function renderTruthBEV(truth,dets,p,frame,cfarPts,amfPts,groups)
        % Robust BEV renderer. Uses scalar-safe extraction so malformed/empty
        % diagnostic structs cannot create array concatenation errors.
        if nargin<5, cfarPts=struct([]); end; if nargin<6, amfPts=struct([]); end; if nargin<7, groups=struct([]); end
        cla(bevAx,'reset'); bevAx.Visible='on'; hold(bevAx,'on'); grid(bevAx,'on'); axis(bevAx,'equal'); box(bevAx,'on');
        truth=double(truth);
        if ~isempty(truth) && size(truth,2)~=4 && size(truth,1)==4, truth=truth.'; end
        if isempty(truth) || size(truth,2)~=4, truth=zeros(0,4); end

        if ~isempty(S.live.history) && showTrailsCB.Value && numel(S.live.history)>1
            hist=S.live.history(1:max(0,numel(S.live.history)-1));
            for hh=1:numel(hist)
                od=hist{hh}.objects;
                [hr,ha,~]=objectRAVVectors(od);
                good=isfinite(hr)&isfinite(ha);
                if any(good)
                    hx=hr(good).*sind(ha(good)); hy=hr(good).*cosd(ha(good));
                    plot(bevAx,hx,hy,'.','Color',[0.75 0.75 0.75],'MarkerSize',6,'HandleVisibility','off');
                end
            end
        end

        trR=truth(:,1); trA=truth(:,4); gt=isfinite(trR)&isfinite(trA);
        if showTruthCB.Value && any(gt)
            tx=trR(gt).*sind(trA(gt)); ty=trR(gt).*cosd(trA(gt));
            plot(bevAx,tx,ty,'ro','LineStyle','none','MarkerSize',6,'LineWidth',1.2);
        end

        [rdR,rdA,~]=objectRAVVectors(dets); gr=isfinite(rdR)&isfinite(rdA);
        if showRadarCB.Value && any(gr)
            dx=rdR(gr).*sind(rdA(gr)); dy=rdR(gr).*cosd(rdA(gr));
            plot(bevAx,dx,dy,'k+','LineStyle','none','MarkerSize',8,'LineWidth',1.4);
        end

        if showTracksCB.Value && any(gr)
            dx=rdR(gr).*sind(rdA(gr)); dy=rdR(gr).*cosd(rdA(gr));
            plot(bevAx,dx,dy,'bs','LineStyle','none','MarkerSize',5,'LineWidth',1,'HandleVisibility','off');
        end
        if showCFARCB.Value && ~isempty(cfarPts)
            [cr,ca]=objectRangeAngleVectors(cfarPts); good=isfinite(cr)&isfinite(ca);
            if any(good), plot(bevAx,cr(good).*sind(ca(good)),cr(good).*cosd(ca(good)),'m.','MarkerSize',8,'HandleVisibility','off'); end
        end
        if showAMFCB.Value && ~isempty(amfPts)
            [ar,aa]=objectRangeAngleVectors(amfPts); good=isfinite(ar)&isfinite(aa);
            if any(good), plot(bevAx,ar(good).*sind(aa(good)),ar(good).*cosd(aa(good)),'c.','MarkerSize',8,'HandleVisibility','off'); end
        end
        if showGroupsCB.Value && ~isempty(groups)
            groups=groups(:); gx=nan(numel(groups),1); gy=nan(numel(groups),1);
            for kk=1:numel(groups)
                gx(kk)=getScalarFieldLocal(groups(kk),'x_pos'); gy(kk)=getScalarFieldLocal(groups(kk),'y_pos');
            end
            good=isfinite(gx)&isfinite(gy);
            if any(good), plot(bevAx,gx(good),gy(good),'gd','LineStyle','none','MarkerSize',5,'HandleVisibility','off'); end
        end
        plot(bevAx,0,0,'ks','MarkerFaceColor','k','MarkerSize',7,'HandleVisibility','off');

        vals=[];
        if showTruthCB.Value && any(isfinite(trR)), vals=[vals; trR(isfinite(trR))]; end
        if showRadarCB.Value && any(isfinite(rdR)), vals=[vals; rdR(isfinite(rdR))]; end
        if isempty(vals), lim=10; else, lim=1.05*max([vals;10]); end
        xlim(bevAx,[-lim lim]); ylim(bevAx,[0 lim]);
        title(bevAx,sprintf('F%d | BEV',frame),'FontWeight','bold'); xlabel(bevAx,'Cross-range x (m)'); ylabel(bevAx,'Forward range y (m)'); hold(bevAx,'off');
    end


    function renderFinal(R)
        last=R.last; truth=last.pf.targets; objects=last.objects; nT=size(truth,1); nOut=numel(objects);
        if isfield(last,'object_eval'), E=last.object_eval; else, E=radar_object_evaluation(objects,truth); end
        rows=cell(nT,12); assign=E.assignment; matched=E.matched_count;
        for t=1:nT
            d=assign(t);
            if d>0 && d<=numel(objects)
                z=objects(d); er=z.range-truth(t,1); ev=z.velocity-truth(t,2); ea=wrap180(z.angle_deg-truth(t,4));
                rows(t,:)={sprintf('T%d',t),sprintf('R%d',d),truth(t,1),z.range,er,truth(t,2),z.velocity,ev,truth(t,4),z.angle_deg,ea,'PASS'};
            else
                rows(t,:)={sprintf('T%d',t),'—',truth(t,1),NaN,NaN,truth(t,2),NaN,NaN,truth(t,4),NaN,NaN,'MISSED'};
            end
        end
resultTable.Data=rows;
        summary1.Text=sprintf('Truth: %d',nT);
        summary2.Text=sprintf('Radar objects: %d',nOut);
        summary3.Text=sprintf('Matched: %d',matched);
        summary4.Text=sprintf('Status: %s',ternary(E.missed_count==0 && E.false_object_count==0,'PASS','CHECK OUTPUTS'));
        summary4.BackgroundColor=ternaryColor(E.missed_count==0 && E.false_object_count==0);
        metricFields{1}.Text=sprintf('%d',nT); metricFields{2}.Text=sprintf('%d',nOut); metricFields{3}.Text=sprintf('%d',matched); metricFields{4}.Text=sprintf('%d',E.false_object_count); metricFields{5}.Text=sprintf('%d',E.missed_count); metricFields{6}.Text=sprintf('%.1f %%',E.object_pd); metricFields{7}.Text=rmseValue(E.range_rmse); metricFields{8}.Text=rmseValue(E.velocity_rmse); metricFields{9}.Text=rmseValue(E.angle_rmse);
        renderStageSafely('Range-Angle plot',@() plotRangeAngleFinal(axRA,truth,objects,assign,S.live.nframes));
        renderStageSafely('Velocity plot',@() plotVelocityFinal(axV,truth(:,2),objects,assign,S.live.nframes));
        renderStageSafely('Detection heat map',@() renderDetectionHeatMap(last));
        renderStageSafely('Paper detection',@() renderPaperDetection(last));
        renderStageSafely('BEV map',@() renderBEVMap(truth,objects,E));
points=0;
formationCount=0;
        if isfield(last,'measurement_point_count'), points=last.measurement_point_count; elseif isfield(last,'final_point_dets'), points=numel(last.final_point_dets); end
        if isfield(last,'formation_point_count'), formationCount=last.formation_point_count; elseif isfield(last,'formation_points'), formationCount=numel(last.formation_points); end
        outFooter.Text=sprintf('Final | Radar %d | Matched %d/%d | False %d | Missed %d',nOut,matched,nT,E.false_object_count,E.missed_count); setStatusDetails('EVALUATION COMPLETE',sprintf('Truth: %d | Radar: %d | Matched: %d | False: %d | Missed: %d',nT,nOut,matched,E.false_object_count,E.missed_count)); pipelineLabel.Text='Pipeline: evaluation complete';
tabs.SelectedTab=tabHM;
drawnow;
    end
    function renderStageSafely(name,fh)
        try
            fh();
        catch ME
            fprintf(2,'[FMCW GUI] %s render warning: %s\n',name,ME.message);
            try
                outFooter.Text=sprintf('%s failed: %s',name,ME.message); setStatusDetails('RENDER WARNING',sprintf('%s\n%s',name,ME.message)); pipelineLabel.Text='Pipeline: display warning';
            catch
            end
        end
    end

    function renderDetectionHeatMap(last)
        cla(hmAx,'reset'); hmAx.Visible='on'; hold(hmAx,'on'); grid(hmAx,'on');
        p=last.pf; P=[];
        if isfield(last,'detection_heatmap') && isstruct(last.detection_heatmap) && isfield(last.detection_heatmap,'Pclean') && ~isempty(last.detection_heatmap.Pclean)
P=last.detection_heatmap.Pclean;
        elseif isfield(last,'object_info') && isfield(last.object_info,'final_rd_power_clean')
P=last.object_info.final_rd_power_clean;
        elseif isfield(last,'object_info') && isfield(last.object_info,'frame_info') && ~isempty(last.object_info.frame_info)
            fi=last.object_info.frame_info{end}; if isfield(fi,'rd_power_clean'), P=fi.rd_power_clean; end
        end
        % Final fallback: recompute the detector map directly from the stored
        % final-frame clean RX cube. This makes the GUI visualization robust
        % even when a saved result lacks the cached heat map.
        if isempty(P) && isfield(last,'final_clean_cube') && ~isempty(last.final_clean_cube)
            try
                P=zeros(numel(p.range_axis),numel(p.vel_axis));
X=last.final_clean_cube;
                for rx=1:min(size(X,3),p.n_rx)
                    [~,~,Prd]=range_doppler_processor(X(:,:,rx),p,struct('keystone',true,'clutter_method','dc_cancel'));
P=P+Prd;
                end
            catch
                P=[];
            end
        end
        if isempty(P)
            % Last-resort recomputation directly from the final clean cube.
            if isfield(last,'final_clean_cube') && ~isempty(last.final_clean_cube)
                try
X=last.final_clean_cube;
                    P=zeros(numel(p.range_axis),numel(p.vel_axis));
                    for rx=1:min(size(X,3),p.n_rx)
                        [~,~,Prd]=range_doppler_processor(X(:,:,rx),p,struct('keystone',true,'clutter_method','dc_cancel'));
                        P=P+max(real(Prd),0);
                    end
                catch ME
                    hmFooter.Text=['Heat-map recompute failed: ' ME.message];
                    P=[];
                end
            end
        end
        if isempty(P) || ~isequal(size(P),[numel(p.range_axis),numel(p.vel_axis)])
            hmFooter.Text=sprintf('Final RD unavailable | expected %dx%d',numel(p.range_axis),numel(p.vel_axis));
            title(hmAx,'Moving-Target Detection Heat Map — unavailable');
            xlim(hmAx,[min(p.vel_axis) max(p.vel_axis)]); ylim(hmAx,[min(p.range_axis) max(p.range_axis)]); hold(hmAx,'off'); return;
        end
        P=real(P); P(~isfinite(P))=0; P=max(P,0); peak=max(P(:)); if isempty(peak) || ~isfinite(peak) || peak<=0, peak=1; end; Pn=max(P/peak,1e-15);
        showHeatmapSafe(hmAx,P,p.vel_axis,p.range_axis,[-60 0]); colormap(hmAx,parula(256));
        hc=0; if isfield(last,'object_info') && ~isempty(last.object_info.frame_info), hc=get_default_field(last.object_info.frame_info{end},'hard_cfar_count',0); end
        title(hmAx,sprintf('FINAL Frame %d/%d | Moving RD | CFAR %d',max(1,getfield_default_local_safe(S.live,'nframes',1)),max(1,getfield_default_local_safe(S.live,'nframes',1)),hc)); xlabel(hmAx,'Radial velocity (m/s)'); ylabel(hmAx,'Range (m)');
        dets=[]; if isfield(last,'object_info') && isfield(last.object_info,'frame_detections') && ~isempty(last.object_info.frame_detections), dets=last.object_info.frame_detections{end}; end
        if ~isempty(dets), rr=arrayfun(@(d)d.range,dets); vv=arrayfun(@(d)d.velocity,dets); plot(hmAx,vv,rr,'wo','MarkerSize',7,'LineWidth',1.4,'DisplayName','Verified point'); end
        if isfield(last,'truth') && ~isempty(last.truth), tv=last.truth; plot(hmAx,tv(:,2),tv(:,1),'k+','MarkerSize',7,'LineWidth',1.2,'DisplayName','Truth R/V'); end
        if ~isempty(dets) || (isfield(last,'truth') && ~isempty(last.truth)), legend(hmAx,'Location','northeastoutside'); end
        hmFooter.Text=sprintf('Final RD | CFAR %d | dR %.3f m | dV %.3f m/s',hc,p.range_resolution_actual,p.velocity_resolution_actual); hold(hmAx,'off');
    end

    function showHeatmapSafe(ax,P,xv,yv,clims)
        P=real(P); P(~isfinite(P))=0; P=max(P,0);
        peak=max(P(:)); if isempty(peak) || peak<=0, peak=1; end
        showHeatmapSafeFixed(ax,P,xv,yv,clims,peak);
    end

    function showHeatmapSafeFixed(ax,P,xv,yv,clims,refPeak)
        if isempty(P), error('Empty heatmap'); end
        P=double(real(P)); P(~isfinite(P))=0; P=max(P,0);
        if ndims(P)>2, P=squeeze(P); end
        if ndims(P)~=2, error('Heatmap must be 2-D'); end
        xv=double(xv(:)); yv=double(yv(:)); refPeak=max(double(real(refPeak)),eps);
        Z=10*log10(max(P/refPeak,1e-15));
        nx=numel(xv); ny=numel(yv); sz=size(Z);
        if isequal(sz,[nx ny])
            Z=Z.';
        elseif ~isequal(sz,[ny nx])
            % Graceful fallback: interpolate onto the requested physical grid.
            xi=linspace(1,size(Z,2),nx); yi=linspace(1,size(Z,1),ny);
            [Xq,Yq]=meshgrid(xi,yi); [X,Y]=meshgrid(1:size(Z,2),1:size(Z,1));
            Z=interp2(X,Y,Z,Xq,Yq,'linear',min(Z(:)));
        end
        if any(~isfinite(Z(:))), finiteZ=Z(isfinite(Z)); if isempty(finiteZ), finiteZ=-60; end; Z(~isfinite(Z))=min(finiteZ); end
        % Force exact physical-grid dimensions before plotting.
        xv=xv(:).'; yv=yv(:);
        if ~isequal(size(Z),[numel(yv) numel(xv)])
            error('Heatmap dimensions %dx%d do not match grid %dx%d.',size(Z,1),size(Z,2),numel(yv),numel(xv));
        end
        cla(ax,'reset'); ax.Visible='on';
        imagesc(ax,xv,yv,Z);
        set(ax,'YDir','normal'); colormap(ax,parula(256)); caxis(ax,clims); grid(ax,'on'); colorbar(ax);
        xlim(ax,[xv(1) xv(end)]); ylim(ax,[yv(1) yv(end)]);
    end

    function renderPaperDetection(last)
        cla(paperMoveAx,'reset'); cla(paperStatAx,'reset'); paperMoveAx.Visible='on'; paperStatAx.Visible='on';
p=last.pf;
hasMove=false;
hasStat=false;
        if isfield(last,'paper_processing') && isstruct(last.paper_processing)
PP=last.paper_processing;
            if isfield(PP,'moving_rd_power') && ~isempty(PP.moving_rd_power)
                P=double(real(PP.moving_rd_power)); P(~isfinite(P))=0;
                showHeatmapSafe(paperMoveAx,P,p.vel_axis,p.range_axis,[-50 0]);
                title(paperMoveAx,sprintf('FINAL Frame %d/%d | Paper Moving',max(1,getfield_default_local_safe(S.live,'nframes',1)),max(1,getfield_default_local_safe(S.live,'nframes',1))));
                xlabel(paperMoveAx,'Radial velocity (m/s)'); ylabel(paperMoveAx,'Range (m)'); hasMove=true;
            end
            if isfield(PP,'stationary_range_angle_power') && ~isempty(PP.stationary_range_angle_power)
                Q=double(real(PP.stationary_range_angle_power)); Q(~isfinite(Q))=0;
                showHeatmapSafe(paperStatAx,Q,p.theta_axis,p.range_axis,[-50 0]);
                title(paperStatAx,sprintf('FINAL Frame %d/%d | Paper Stationary',max(1,getfield_default_local_safe(S.live,'nframes',1)),max(1,getfield_default_local_safe(S.live,'nframes',1))));
                xlabel(paperStatAx,'Azimuth (deg)'); ylabel(paperStatAx,'Range (m)'); hasStat=true;
            end
        end
        if ~hasMove, title(paperMoveAx,'Paper Moving | no data'); xlabel(paperMoveAx,'Radial velocity (m/s)'); ylabel(paperMoveAx,'Range (m)'); end
        if ~hasStat, title(paperStatAx,'Paper Stationary | no data'); xlabel(paperStatAx,'Azimuth (deg)'); ylabel(paperStatAx,'Range (m)'); end
        paperFooter.Text='Paper | moving RD + stationary range-angle';
    end

    function renderBEVMap(truth,objects,E)
        cla(bevAx,'reset'); bevAx.Visible='on'; hold(bevAx,'on'); grid(bevAx,'on'); axis(bevAx,'equal');
        nT=size(truth,1); tx=truth(:,1).*sind(truth(:,4)); ty=truth(:,1).*cosd(truth(:,4));
        if nT>0
            plot(bevAx,tx,ty,'b+','LineWidth',1.6,'MarkerSize',8,'DisplayName','Truth');
        end
        used=false(1,numel(objects)); a=E.assignment(:);
        for t=1:min(nT,numel(a))
            d=a(t);
            if isfinite(d) && d>=1 && d<=numel(objects)
                used(d)=true; ox=objects(d).range*sind(objects(d).angle_deg); oy=objects(d).range*cosd(objects(d).angle_deg);
                if isfinite(ox)&&isfinite(oy), plot(bevAx,[tx(t) ox],[ty(t) oy],'k--','LineWidth',0.7,'HandleVisibility','off'); end
            end
        end
        if ~isempty(objects)
            ox=arrayfun(@(o)o.range*sind(o.angle_deg),objects); oy=arrayfun(@(o)o.range*cosd(o.angle_deg),objects);
            good=isfinite(ox)&isfinite(oy);
            plot(bevAx,ox(used&good),oy(used&good),'ro','MarkerSize',6,'LineWidth',1.2,'DisplayName','Radar');
            if any((~used)&good), plot(bevAx,ox((~used)&good),oy((~used)&good),'mx','MarkerSize',7,'LineWidth',1.5,'DisplayName','Radar-only'); end
        end
        plot(bevAx,0,0,'ks','MarkerFaceColor','k','MarkerSize',7,'DisplayName','Radar origin');
        vals=[truth(:,1); arrayfun(@(o)o.range,objects(:))]; vals=vals(isfinite(vals)); if isempty(vals), vals=10; end
        lim=1.05*max([vals;10]); xlim(bevAx,[-lim lim]); ylim(bevAx,[0 lim]);
        title(bevAx,sprintf('FINAL | %d-frame run | BEV | truth vs radar',max(1,getfield_default_local_safe(S.live,'nframes',1))));
        xlabel(bevAx,'Cross-range x (m)'); ylabel(bevAx,'Forward range y (m)'); legend(bevAx,'Location','northeastoutside'); hold(bevAx,'off');
    end

    function plotRangeAngleFinal(ax,truth,objects,assigned,nframes)
        % Final spatial comparison. Red o = truth, black + = all radar objects.
        cla(ax,'reset'); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
        truth=double(truth); nT=size(truth,1);
        if nT>0
            tx=truth(:,1).*sind(truth(:,4)); ty=truth(:,1).*cosd(truth(:,4)); good=isfinite(tx)&isfinite(ty);
            plot(ax,tx(good),ty(good),'ro','LineStyle','none','MarkerSize',6,'LineWidth',1.3);
            ids=find(good); for k=1:numel(ids), t=ids(k); text(ax,tx(t),ty(t),sprintf(' T%d',t),'Color',[0.75 0 0],'FontWeight','bold','VerticalAlignment','bottom'); end
        end
        objects=objects(:); nR=numel(objects);
        if nR>0
            rx=arrayfun(@(o)o.range*sind(o.angle_deg),objects); ry=arrayfun(@(o)o.range*cosd(o.angle_deg),objects); good=isfinite(rx)&isfinite(ry);
            plot(ax,rx(good),ry(good),'k+','LineStyle','none','MarkerSize',8,'LineWidth',1.5);
            ids=find(good); for k=1:numel(ids), r=ids(k); text(ax,rx(r),ry(r),sprintf(' R%d',r),'Color',[0.1 0.1 0.1],'FontWeight','bold','VerticalAlignment','top'); end
        end
        plot(ax,0,0,'bs','MarkerFaceColor','b','MarkerSize',7);
        vals=[]; if nT>0, vals=[vals;truth(:,1)]; end; if nR>0, vals=[vals;arrayfun(@(o)o.range,objects)]; end; vals=vals(isfinite(vals));
        if isempty(vals), lim=10; else, lim=max(10,1.08*max(vals)); end
        xlim(ax,[-lim lim]); ylim(ax,[0 lim]);
        xlabel(ax,'Cross-range x (m)'); ylabel(ax,'Forward range y (m)'); title(ax,sprintf('FINAL F%d | Range + Angle',max(1,nframes)),'FontWeight','bold');
        hold(ax,'off');
    end

    function plotVelocityFinal(ax,truthVel,objects,assigned,nframes)
        % Final velocity comparison using truth/radar matched identity through assignment.
        cla(ax,'reset'); hold(ax,'on'); grid(ax,'on');
        tv=double(truthVel(:)); nT=numel(tv); assigned=assigned(:); est=nan(nT,1);
        for t=1:min(nT,numel(assigned)), d=assigned(t); if isfinite(d)&&d>=1&&d<=numel(objects), est(t)=objects(d).velocity; end; end
        x=(1:nT).';
        gt=isfinite(tv); ge=isfinite(est);
        if any(gt), plot(ax,x(gt),tv(gt),'ro','LineStyle','none','MarkerSize',6,'LineWidth',1.2); end
        if any(ge), plot(ax,x(ge),est(ge),'k+','LineStyle','none','MarkerSize',8,'LineWidth',1.5); end
        % Radar-only velocities are placed after the truth slots.
        used=false(1,numel(objects)); for t=1:min(nT,numel(assigned)), d=assigned(t); if isfinite(d)&&d>=1&&d<=numel(objects), used(d)=true; end; end
        ro=find(~used); if ~isempty(ro), rv=arrayfun(@(o)o.velocity,objects(ro)); plot(ax,(nT+(1:numel(ro))).',rv(:),'k+','LineStyle','none','MarkerSize',8,'LineWidth',1.5); end
        vals=[tv(gt);est(ge)]; if ~isempty(ro), vals=[vals;arrayfun(@(o)o.velocity,objects(ro)).']; end;
        if ~isempty(vals), lo=min(vals); hi=max(vals); span=max(hi-lo,1); pad=0.08*span; ylim(ax,[lo-pad hi+pad]); end
        xlim(ax,[0 max(1,nT+numel(ro))+1]);
        xlabel(ax,'Range (m)'); ylabel(ax,'Velocity (m/s)');
        yt=yticks(ax); yticks(ax,sort(unique([yt(:);0])));
        title(ax,sprintf('FINAL F%d | Velocity',max(1,nframes)),'FontWeight','bold');
        hold(ax,'off');
    end

    function safeRmAppdata(rootObj,key)
        try, if isappdata(rootObj,key), rmappdata(rootObj,key); end, catch, end
    end
    function v=getfield_default_local_safe(s,f,d)
v=d;
        try, if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), v=s.(f); end, catch, end
    end

    function s=rmseValue(x), if isempty(x) || ~isfinite(x), s='—'; else, s=sprintf('%.4f',x); end, end
    function col=ternaryColor(c),if c,col=C.good;else,col=C.warn;end,end
    function a=wrap180(a),a=mod(a+180,360)-180;end
end

% --- Merged local helper: get_default_field ---
