function fig = launch_radar_gui
%LAUNCH_RADAR_GUI Launch the FMCW Automotive Radar GUI EMANI.
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
