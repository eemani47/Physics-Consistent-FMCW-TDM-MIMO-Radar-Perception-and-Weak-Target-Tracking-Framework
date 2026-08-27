function report = contract_selftest(verbose)
%CONTRACT_SELFTEST  Structural contracts across configuration, GUI and code.
%
%   Three properties are checked mechanically, because each has been violated
%   in this codebase before and none is visible from any single file:
%
%     * every configuration field the model publishes is read by at least one
%       consumer, so a parameter cannot look tunable while being inert;
%     * no algorithm exists in more than one place, so a caller cannot silently
%       reach a stale copy through MATLAB's local-function precedence;
%     * every control the interface exposes writes a key the pipeline reads.
%
%   These run by source inspection deliberately: they are statements about the
%   shape of the package, not about the value of any computation.

if nargin < 1, verbose = true;
end
report = struct('name','contracts','ok',true,'checks',repmat(struct('name','','pass',false,'detail',''),0,1));
root = fileparts(fileparts(mfilename('fullpath')));

files = collect_m_files(root);
src = containers.Map('KeyType','char','ValueType','char');
for i = 1:numel(files)
    src(files{i}) = fileread(files{i});
end

% --- 1. every published configuration field has a consumer ---------------
cfgFile = fullfile(root,'core','config','radar_configuration.m');
cfgText = fileread(cfgFile);
tok = regexp(cfgText,'\n\s*p\.([a-z_]+)\.([a-z_0-9]+)\s*=','tokens');
unused = {};
for i = 1:numel(tok)
    sec = tok{i}{1}; fld = tok{i}{2};
    if any(strcmp(fld,{'q'})), continue; end     % derived inside the model
hits = 0;
keys_ = src.keys;
    for k = 1:numel(keys_)
        if strcmp(keys_{k},cfgFile)
            % The model derives some quantities from its own fields; count
            % those, but only where the name is read rather than assigned.
            reads = numel(regexp(cfgText,['[^\n]*' regexprep(fld,'_','_') '[^\n]*'],'match')) - ...
                    numel(regexp(cfgText,['\n\s*p\.' sec '\.' fld '\s*='],'match'));
            hits = hits + max(reads,0);
continue;
        end
        if contains(src(keys_{k}),fld), hits = hits + 1; end
    end
    if hits == 0, unused{end+1} = sprintf('p.%s.%s',sec,fld); end
end
report = add(report,'no configuration field is published without a consumer', ...
    isempty(unused), describe_list(unused));

% --- 2. no algorithm is defined twice ------------------------------------
% A function whose name matches a file in the package must not also appear as
% a local definition inside a different file: MATLAB resolves the local copy
% first, so the canonical file would never execute.
publicNames = cell(1,numel(files));
for i = 1:numel(files)
    [~,publicNames{i}] = fileparts(files{i});
end
shadowed = {};
for i = 1:numel(files)
    text = src(files{i});
    [~,selfName] = fileparts(files{i});
    defs = regexp(text,'\n\s*function\s+[^\n]*?([a-zA-Z_]\w*)\s*\(','tokens');
    for d = 1:numel(defs)
        nm = defs{d}{1};
        if strcmp(nm,selfName), continue; end
        if any(strcmp(nm,publicNames))
            shadowed{end+1} = sprintf('%s defines %s',selfName,nm);
        end
    end
end
report = add(report,'no package function is shadowed by a local copy', ...
    isempty(shadowed), describe_list(shadowed));

% --- 3. every interface control writes a key the pipeline reads ----------
guiFile = fullfile(root,'gui','fmcw_radar_gui_impl.m');
inert = {};
if exist(guiFile,'file')
    guiText = fileread(guiFile);
    ctl = regexp(guiText,'addDet(?:Spinner|Dropdown|Checkbox)\([^\n]*?,''([a-z_0-9]+)'',''([a-z_0-9]+)''','tokens');
keys_ = src.keys;
    for i = 1:numel(ctl)
        grp = ctl{i}{1}; key = ctl{i}{2};
hits = 0;
        for k = 1:numel(keys_)
            f = keys_{k};
            if strcmp(f,cfgFile) || startsWith(f,fullfile(root,'gui')) || ...
               startsWith(f,fullfile(root,'validation'))
continue;
            end
            if contains(src(f),key), hits = hits + 1; end
        end
        if hits == 0, inert{end+1} = sprintf('%s.%s',grp,key); end
    end
end
report = add(report,'every interface control binds to a live parameter', ...
    isempty(inert), describe_list(inert));

% --- 3b. the interface writes where the pipeline reads -------------------
% A path mismatch here detaches the interface from the pipeline silently: the
% scene table keeps showing the edited targets while every run consumes a
% stale file. This is a structural check because no single file reveals it.
guiWrites = regexp(fileread(guiFile),'save\(fullfile\([^)]*?\)\s*,\s*''config''','match');
readers = {fullfile(root,'run_radar_project.m'), ...
           fullfile(root,'run_radar_realtime.m'), ...
           fullfile(root,'experiments','radar_experiment_common.m')};
readsGuiDir = true;
for i = 1:numel(readers)
    if exist(readers{i},'file')
        txt = fileread(readers{i});
        if contains(txt,'gui_config.mat') && ~contains(txt,'''gui'',''gui_config.mat''')
            readsGuiDir = false;
        end
    end
end
writesGuiDir = ~isempty(guiWrites) && any(contains(guiWrites,'cfgDir')) ;
report = add(report,'the interface writes its configuration where the pipeline reads it', ...
    writesGuiDir && readsGuiDir, ...
    'gui/gui_config.mat is the single canonical location');
report = add(report,'no stale configuration file shadows the canonical one', ...
    exist(fullfile(root,'gui_config.mat'),'file') ~= 2, ...
    'a copy in the project root would be written but never read');

% --- 3c. every result panel has an updater ------------------------------
% A panel that is constructed but never written to shows placeholder text for
% the life of the session and looks like a rendering failure.
guiText = fileread(guiFile);
panels = {'metricFields','resultTable','detDiagTable','perfTable'};
noUpdater = {};
for i = 1:numel(panels)
    n = numel(regexp(guiText,[panels{i} '\s*[\{\(][^=]*=\s*'],'match')) + ...
        numel(regexp(guiText,[panels{i} '\.Data\s*='],'match')) + ...
        numel(regexp(guiText,[panels{i} '\{i\}\.Text\s*='],'match'));
    if n == 0, noUpdater{end+1} = panels{i}; end
end
report = add(report,'every result panel is written to somewhere', ...
    isempty(noUpdater),describe_list(noUpdater));

% --- 3d. history stepping is not gated on the run flag ------------------
gated = ~isempty(regexp(guiText, ...
    'function history(Back|Forward|Select)[^\n]*\n\s*if S\.live\.processingActive','once'));
report = add(report,'history stepping stays available after a run stops', ...
    ~gated,'stepping is a display operation, not a run operation');

% --- 4. the configuration validates and is self-consistent ---------------
ok = true; msg = '';
try
    p = radar_configuration(struct());
    validate_radar_config(p);
catch ME
ok = false;
msg = ME.message;
end
report = add(report,'default configuration passes its own validator',ok,msg);

% --- 5. deliberately inconsistent configurations are rejected ------------
report = add(report,'an impossible CFAR window is rejected', ...
    rejects(struct('cfar',struct('Tr',1,'Td',1,'Gr',0,'Gd',0,'min_reference_cells',5000))));
report = add(report,'an out-of-range Pfa is rejected', ...
    rejects(struct('cfar',struct('Pfa',1.5))));
report = add(report,'a degenerate azimuth span is rejected', ...
    rejects(struct('az_span',0.0)));

if verbose, print_report(report); end
end

function tf = rejects(cfg)
tf = false;
try
    radar_configuration(cfg);
catch
tf = true;
end
end

function files = collect_m_files(root)
files = {};
sub = {'core/config','core/simulation','core/processing','core/detection', ...
       'core/estimation','core/interference','core/tbd','core/tracking', ...
       'core/evaluation','experiments','gui','validation',''};
for i = 1:numel(sub)
    d = dir(fullfile(root,sub{i},'*.m'));
    for k = 1:numel(d)
        files{end+1} = fullfile(d(k).folder,d(k).name);
    end
end
files = unique(files);
end

function s = describe_list(c)
if isempty(c), s = ''; return; end
n = min(numel(c),6);
s = sprintf('%d found: %s',numel(c),strjoin(c(1:n),', '));
if numel(c) > n
    s = sprintf('%s and %d more',s,numel(c)-n);
end
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
