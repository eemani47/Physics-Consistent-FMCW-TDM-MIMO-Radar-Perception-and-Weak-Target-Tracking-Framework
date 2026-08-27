function fig = fmcw_radar_gui
%FMCW_RADAR_GUI Robust launcher for the FMCW Automotive Radar GUI.
root=fileparts(mfilename('fullpath'));
project_root=fileparts(root);
addpath(genpath(project_root));
if ~contains(path,root), addpath(root); end
try
    fig=fmcw_radar_gui_impl();
    if isempty(fig) || ~isvalid(fig), error('FMCW_GUI:NoFigure','GUI implementation returned no valid figure.'); end
    fig.Visible='on'; drawnow;
catch ME
    fprintf(2,'[FMCW GUI] STARTUP ERROR:\n%s\n',getReport(ME,'extended','hyperlinks','off'));
    try
        f=uifigure('Name','FMCW GUI Startup Error','Position',[300 300 760 500]);
        uialert(f,getReport(ME,'basic','hyperlinks','off'),'FMCW GUI failed to start','Icon','error');
    catch
        errordlg(getReport(ME,'basic','hyperlinks','off'),'FMCW GUI failed to start');
    end
    fig=[];
end
end

% ---- merged public launcher implementation ----
function fig = local_launch_radar_gui
%LAUNCH_RADAR_GUI Launch the FMCW Automotive Radar GUI.
fprintf('[FMCW Radar] Starting GUI...\n');
root = fileparts(mfilename('fullpath'));
if ~contains(path,root), addpath(root); end
rehash toolboxcache;
try
    fig = fmcw_radar_gui();
    if isempty(fig) || ~isvalid(fig)
        error('FMCW_GUI:NoFigure','GUI did not return a valid figure.');
    end
    fig.Visible='on'; drawnow;
catch ME
    fprintf(2,'[FMCW Radar] GUI STARTUP ERROR:\n%s\n',getReport(ME,'extended','hyperlinks','off'));
    try
        f=uifigure('Name','FMCW Radar GUI Error','Position',[300 300 760 500]);
        uialert(f,getReport(ME,'basic','hyperlinks','off'),'FMCW Radar GUI failed to start','Icon','error');
    catch
        errordlg(getReport(ME,'basic','hyperlinks','off'),'FMCW Radar GUI failed to start');
    end
    fig=[];
end
end
