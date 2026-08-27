function [paths,info] = coherent_tbd_detector(frame_data,p,frame_info)
%COHERENT_TBD_DETECTOR  Motion-compensated long-time coherent integration.
%
%   The dynamic-programming branch integrates detected power, which is a
%   non-coherent operation: doubling the observation time buys roughly 1.5 dB.
%   This branch integrates the complex signal instead. Along a hypothesised
%   trajectory the range track is predicted chirp by chirp, the sample at each
%   predicted range bin is de-rotated by the Doppler phase of the hypothesis,
%   and the results are summed coherently. Signal amplitudes then add linearly
%   while noise adds in power, so the gain scales with the number of samples
%   rather than its square root. That is what makes a far, sub-threshold target
%   recoverable.
%
%   Per frame, for transmit chain m and receive chain n,
%
%       z_k = R( r_hat(k), k, n ) exp( -j 2 pi f_d t_k ) ,   f_d = 2 v / lambda
%       T_mn = | sum_k z_k |^2 / ( K sigma^2 )
%
%   With unitary range compression each z_k has noise variance sigma^2, so
%   under H0 each T_mn is unit-mean exponential and their average over the
%   n_tx n_rx chain pairs is Gamma(n_tx n_rx, 1/(n_tx n_rx)). Averaging further
%   over the L frames of the path gives Gamma(L n_tx n_rx, .) and the detection
%   threshold follows from the inverse incomplete Gamma at the configured
%   per-candidate false-alarm probability:
%
%       T_thr = Q^-1( P_fa ; L n_tx n_rx ) / ( L n_tx n_rx )
%
%   so the operating point is a probability, not a hand-set score. The
%   configured floor in decibels is retained and the stricter of the two
%   applies.
%
%   Seeding is deliberately restricted to the far-range window where the
%   single-frame detector is expected to fail, and every accepted path must
%   also survive an angular-consistency test: the bearing measured at the
%   path's own state must not fan out across frames, which rejects sidelobe and
%   multipath replicas that persist in range and Doppler but not in angle.

paths = repmat(empty_path(),0,1);
info = struct('enabled',false,'candidate_counts',zeros(1,numel(frame_data)), ...
    'total_candidates',0,'path_count',0,'accepted_path_count',0,'best_score_db',-Inf, ...
    'threshold_db',NaN,'threshold_source','','coherent_scores_db',zeros(1,0), ...
    'path_support',zeros(1,0),'angle_std_deg',zeros(1,0), ...
    'method','motion-compensated coherent integration over weak RD seeds');
if ~isfield(p,'tbd') || ~isfield(p.tbd,'coherent') || ~p.tbd.coherent.enabled || isempty(frame_data)
return;
end
info.enabled = true;

Nf = numel(frame_data);
cand = cell(Nf,1);
for f = 1:Nf
    P = []; N = [];
    if nargin >= 3 && numel(frame_info) >= f && isstruct(frame_info{f})
        Pmove = get_default_field(frame_info{f},'rd_power_moving', ...
                get_default_field(frame_info{f},'rd_power_clean',[]));
        Pref  = get_default_field(frame_info{f},'rd_power_reference',[]);
        if ~isempty(Pref) && ~isempty(Pmove)
            % Retain a bounded fraction of the unsubtracted energy. Coherent
            % stationary subtraction attenuates targets whose Doppler is close
            % to zero, and the RX-domain test below decides whether what
            % survives is real.
            P = max(Pmove,0.25*Pref);
        else
P = Pmove;
        end
        N = get_default_field(frame_info{f},'noise_map',[]);
    end
    if isempty(P), cand{f} = repmat(seed_template(),0,1); continue; end
    if isempty(N) || isscalar(N) || ~isequal(size(N),size(P))
        N = repmat(map_noise_floor(P),size(P));
    else
        N = max(N,map_noise_floor(P));
    end
    cand{f} = seed_weak_cells(P,N,p);
    info.candidate_counts(f) = numel(cand{f});
end
info.total_candidates = sum(info.candidate_counts);
if info.total_candidates == 0, return;
end

% One range transform per frame, shared by every candidate path in this call.
% The transform of a frame does not depend on the trajectory being tested, so
% recomputing it per candidate made live cost grow with the number of
% hypotheses instead of the number of frames.
fftCache = containers.Map('KeyType','double','ValueType','any');

for pass = 1:p.tbd.coherent.max_paths
    [path,best] = best_motion_path(cand,p);
    if isempty(path) || best < p.tbd.coherent.path_score_threshold, break; end
    path = decorate_path(path,best,cand);
    if path.length < p.tbd.coherent.min_path_frames || ...
       path.support_fraction < p.tbd.coherent.min_support_fraction
        cand = suppress_path(cand,path,p,0.5);
continue;
    end

    [thrDb,thrSrc] = coherent_threshold_db(path.length,p);
info.threshold_db = thrDb;
info.threshold_source = thrSrc;

    [ok,cscore,theta,angStd] = coherent_validate_path(path,frame_data,p,thrDb,fftCache);
    info.best_score_db = max(info.best_score_db,cscore);
    if ok
path.coherent_score_db = cscore;
path.angle_deg = theta;
path.final_angle = theta;
path.angle_std_deg = angStd;
        paths(end+1) = path;
info.accepted_path_count = info.accepted_path_count + 1;
        info.coherent_scores_db(end+1) = cscore;
        info.path_support(end+1) = path.support_fraction;
        info.angle_std_deg(end+1) = angStd;
    end
    cand = suppress_path(cand,path,p,1.0);
if info.accepted_path_count >= p.tbd.coherent.max_paths, break;
end
end
% Report the operating point even when nothing was accepted. Leaving it NaN
% made a branch that ran and found nothing indistinguishable from one that
% never ran at all.
if ~isfinite(info.threshold_db)
    [info.threshold_db,info.threshold_source] = ...
        coherent_threshold_db(max(p.tbd.coherent.min_path_frames,1),p);
end
info.path_count = numel(paths);
end

% =========================================================================
function [thrDb,src] = coherent_threshold_db(L,p)
%COHERENT_THRESHOLD_DB  Gamma-exact threshold for the averaged coherent test.
floorDb = p.tbd.coherent.coherent_score_threshold_db;
if ~logical(get_default_field(p.tbd.coherent,'use_gamma_threshold',true))
    thrDb = floorDb; src = 'configured floor'; return;
end
nObs = max(1,p.n_tx*p.n_rx*max(L,1));
Pfa  = min(max(p.tbd.coherent.Pfa,1e-12),0.5);
try
    T = gammaincinv(Pfa,nObs,'upper')/nObs;
    gammaDb = 10*log10(max(T,realmin));
    src = 'Gamma inverse at configured Pfa';
catch
    gammaDb = floorDb; src = 'configured floor (inverse unavailable)';
end
thrDb = max(gammaDb,floorDb);
if thrDb > gammaDb, src = [src ', floored by configuration']; end
end

function nf = map_noise_floor(P)
%MAP_NOISE_FLOOR  Estimate the noise scale from cells that actually contain data.
v = P(isfinite(P) & P > 0);
if isempty(v)
    nf = realmin;
else
    nf = median(v);
    if ~isfinite(nf) || nf <= 0, nf = min(v); end
end
end

function c = seed_weak_cells(P,N,p)
%SEED_WEAK_CELLS  Sub-threshold cells in the far-range window, as seeds.
%   The mask and the local-maximum test are formed over the whole map at once.
%   Walking the map cell by cell cost one iteration per range-Doppler bin, the
%   overwhelming majority of which fail the first test immediately.
[Nr,Nd] = size(P);
c = repmat(seed_template(),0,1);
r0 = p.tbd.coherent.start_range_fraction*p.R_max;
thr = p.tbd.coherent.seed_threshold_db;

rows = p.range_axis(:) >= r0 & p.range_axis(:) <= p.R_max*0.995;
if ~any(rows), return; end

Z = P./max(N,realmin);
mask = false(Nr,Nd);
mask(rows,:) = 10*log10(max(Z(rows,:),realmin)) >= thr;
if ~any(mask(:)), return; end

% Three-by-three local maximum, computed by shifting rather than by
% re-slicing a window around every candidate.
Pmax = P;
for dr = -1:1
    for dd = -1:1
        if dr == 0 && dd == 0, continue; end
        Pmax = max(Pmax,circshift(P,[dr dd]));
    end
end
mask = mask & (P >= Pmax);

idx = find(mask);
if isempty(idx), return; end
[rIdx,dIdx] = ind2sub([Nr Nd],idx);
zv = Z(idx);
llr = log1p(max(zv-1,0));

% Keep the strongest seeds within the configured budget.
budget = p.tbd.coherent.max_seeds_per_frame;
if numel(idx) > budget
    [~,ord] = sort(llr,'descend');
    keep = ord(1:budget);
    rIdx = rIdx(keep); dIdx = dIdx(keep); zv = zv(keep); llr = llr(keep);
end

c = repmat(seed_template(),1,numel(rIdx));
for k = 1:numel(rIdx)
    c(k).range = p.range_axis(rIdx(k));
    c(k).velocity = p.vel_axis(dIdx(k));
    c(k).r_bin = rIdx(k);
    c(k).d_bin = dIdx(k);
    c(k).z = zv(k);
    c(k).llr = llr(k);
end
end

function [path,best] = best_motion_path(cand,p)
%BEST_MOTION_PATH  Forward search over motion-compatible seed cells.
%
%   Same recursion as before, evaluated as matrix operations rather than one
%   predecessor-successor pair at a time. With a few dozen seeds per frame the
%   scalar form spent almost all of its time on MATLAB call dispatch instead
%   of on arithmetic.

Nf = numel(cand); best = -Inf; path = [];
scores = cell(Nf,1); prevF = cell(Nf,1); prevI = cell(Nf,1);

R = cell(Nf,1); V = cell(Nf,1); LLR = cell(Nf,1);
for f = 1:Nf
    c = cand{f};
    if isempty(c)
        R{f} = zeros(1,0); V{f} = zeros(1,0); LLR{f} = zeros(1,0);
    else
        R{f} = [c.range]; V{f} = [c.velocity]; LLR{f} = [c.llr];
    end
end

maxRstep = p.tbd.coherent.max_range_step_m;
maxVstep = p.tbd.coherent.max_velocity_step_mps;
maxAcc   = p.tbd.coherent.max_acceleration_mps2;
sR = max(p.tbd.range_sigma_m,0.4);
sV = max(p.tbd.velocity_sigma_mps,0.5);

for f = 1:Nf
    K = numel(R{f});
    scores{f} = -Inf(1,K); prevF{f} = zeros(1,K); prevI{f} = zeros(1,K);
    if K == 0, continue; end
    scores{f} = LLR{f} - 1.0;
    if f == 1, continue; end

    for pf = max(1,f-2):f-1
        M = numel(R{pf});
        if M == 0, continue; end
        gap = f - pf; dt = p.track.dt*gap;

        dr = R{f}(ones(M,1),:) - (R{pf}(:) + V{pf}(:)*dt);
        dv = V{f}(ones(M,1),:) - V{pf}(:);
        cost = 0.5*(dr/sR).^2 + 0.5*(dv/sV).^2;
        forbid = abs(dr) > maxRstep*gap | abs(dv) > maxVstep*gap | ...
                 abs(dv)/max(dt,eps) > maxAcc;
        cost(forbid) = Inf;

        s = scores{pf}(:) + LLR{f}(ones(M,1),:) - cost + 0.25/gap;
        s(forbid) = -Inf;
        [sBest,iBest] = max(s,[],1);
        take = sBest > scores{f};
        scores{f}(take) = sBest(take);
        prevF{f}(take)  = pf;
        prevI{f}(take)  = iBest(take);
    end
end
endF = 0;
endI = 0;
for f = 1:Nf
    if isempty(scores{f}), continue; end
    [s,j] = max(scores{f});
if s > best, best = s;
endF = f;
endI = j;
end
end
if ~isfinite(best) || endF == 0, return; end
ff = []; ii = []; f = endF; i = endI;
while f > 0 && i > 0
    ff(end+1) = f; ii(end+1) = i;
    pf = prevF{f}(i); pi = prevI{f}(i);
if pf <= 0 || pi <= 0, break;
end
f = pf;
i = pi;
end
[ff,ord] = sort(ff); ii = ii(ord);
path = empty_path();
path.frame_indices = ff;
path.candidate_indices = ii;
path.length = numel(ff); path.score = best;
end

function pth = decorate_path(pth,best,cand)
L = pth.length;
pth.score = best;
span = max(pth.frame_indices) - min(pth.frame_indices) + 1;
pth.support_fraction = L/max(span,1);
pth.ranges = zeros(1,L); pth.velocities = zeros(1,L); pth.z = zeros(1,L);
for k = 1:L
    q = cand{pth.frame_indices(k)}(pth.candidate_indices(k));
    pth.ranges(k) = q.range; pth.velocities(k) = q.velocity; pth.z(k) = q.z;
end
pth.final_range = pth.ranges(end);
pth.final_velocity = pth.velocities(end);
pth.mean_z = mean(pth.z);
pth.coherent_score_db = -Inf;
pth.angle_deg = NaN;
pth.final_angle = NaN;
pth.angle_std_deg = NaN;
end

function [ok,score_db,theta,angStd] = coherent_validate_path(path,frame_data,p,thresholdDb,fftCache)
ok = false;
score_db = -Inf;
theta = NaN;
angStd = Inf;
frame_scores = []; angle_votes = [];
if nargin < 5, fftCache = []; end

for kk = 1:path.length
    fi = path.frame_indices(kk);
    if fi < 1 || fi > numel(frame_data), continue; end
    [Rfft,cube] = cached_range_fft(fftCache,fi,frame_data,p);
    if isempty(Rfft), continue; end

    v = path.velocities(kk); R0 = path.ranges(kk);
fd = 2*v/p.lambda;
powSum = 0;
nObs = 0;
    for tx = 1:p.n_tx
chirps = tx:p.n_tx:p.Nd;
        if isempty(chirps), continue; end
        t = (chirps-1)*p.Tchirp;
        Rpred = R0 + v*(t - mean(t));
        rb = zeros(size(chirps));
        for cIdx = 1:numel(chirps)
            [~,rb(cIdx)] = min(abs(p.range_axis - Rpred(cIdx)));
        end
        deRot = exp(-1j*2*pi*fd*t);
        for rx = 1:p.n_rx
            lin = (chirps-1)*p.Nr + rb;
            z = reshape(Rfft(:,:,rx),[],1);
            z = z(lin).';
z = z.*deRot;
            sigNoise = max(p.rx_noise_power_W(rx),realmin);
            powSum = powSum + abs(sum(z))^2/max(numel(chirps)*sigNoise,realmin);
nObs = nObs + 1;
        end
    end
if nObs == 0, continue;
end
    frame_scores(end+1) = 10*log10(max(powSum/nObs,realmin));

    % Bearing measured at the path's own motion-compensated state.
    try
        va = tdm_virtual_aperture(cube,p,nearest_index(p.range_axis,R0),v,false);
        [~,~,~,~,ai] = music_aoa_estimator(va,p,R0);
        % Only a prominent spectral peak is admitted as a bearing vote; a flat
        % spectrum carries no angular information and would otherwise dilute
        % the consistency test toward always passing.
        promFloor = get_default_field(p.tbd,'path_angle_min_prominence_db',0);
        if isfinite(ai.music_peak_angle_deg) && ai.music_peak_prominence_db >= promFloor
            angle_votes(end+1) = ai.music_peak_angle_deg;
        end
    catch
    end
end

if isempty(frame_scores), return; end
score_db = 10*log10(max(mean(10.^(frame_scores/10)),realmin));
if numel(frame_scores) < p.tbd.coherent.min_path_frames, return; end
ok = score_db >= thresholdDb;

if ~isempty(angle_votes)
    theta = atan2d(mean(sind(angle_votes)),mean(cosd(angle_votes)));
    resid = mod(angle_votes - theta + 180,360) - 180;
    angStd = sqrt(mean(resid.^2));
    if numel(angle_votes) >= 2 && angStd > get_default_field(p.tbd,'path_angle_max_std_deg',8.0)
ok = false;
    end
else
angStd = Inf;
end
end

function [Rfft,cube] = cached_range_fft(cache,idx,frame_data,p)
%CACHED_RANGE_FFT  Unitary range compression of one frame, computed once.
%   Unitary normalisation keeps the per-sample noise variance equal to the
%   configured chain noise, which is what the Gamma threshold assumes.
Rfft = []; cube = [];
if ~isempty(cache) && isKey(cache,idx)
    v = cache(idx); Rfft = v{1}; cube = v{2}; return;
end
cube = unpack_frame(frame_data{idx});
if isempty(cube), return; end
[wr,wrms] = rd_window(p.Nr,get_default_field(p.range_processing,'window','hann'));
Rfft = fft(cube.*wr,p.Nr,1)/(sqrt(p.Nr)*wrms);
if ~isempty(cache), cache(idx) = {Rfft,cube}; end
end

function out = suppress_path(cand,path,p,scale)
%SUPPRESS_PATH  Withdraw the seeds an accepted trajectory consumed.
out = cand;
gR = scale*p.tbd.coherent.max_range_step_m;
gV = scale*p.tbd.coherent.max_velocity_step_mps;
for k = 1:path.length
    f = path.frame_indices(k);
    if f < 1 || f > numel(out) || isempty(out{f}), continue; end
    near = abs([out{f}.range] - path.ranges(k)) <= gR & ...
           abs([out{f}.velocity] - path.velocities(k)) <= gV;
    out{f} = out{f}(~near);
end
end

function cube = unpack_frame(f)
cube = [];
if isstruct(f)
    if isfield(f,'clean'), cube = f.clean; end
    if isempty(cube) && isfield(f,'raw'), cube = f.raw; end
else
cube = f;
end
end

function q = seed_template()
q = struct('range',0,'velocity',0,'r_bin',1,'d_bin',1,'z',1,'llr',0);
end

function q = empty_path()
q = struct('frame_indices',zeros(1,0),'candidate_indices',zeros(1,0),'length',0, ...
    'score',-Inf,'support_fraction',0,'ranges',zeros(1,0),'velocities',zeros(1,0), ...
    'z',zeros(1,0),'mean_z',0,'coherent_score_db',-Inf,'angle_deg',NaN, ...
    'angle_std_deg',NaN,'final_angle',NaN,'final_range',NaN,'final_velocity',NaN);
end

function k = nearest_index(x,v), [~,k] = min(abs(x-v)); end
