function [detections,info] = tdm_velocity_resolver(detections,rx_cube,p)
%TDM_VELOCITY_RESOLVER  Joint TDM Doppler-ambiguity resolution.
%
%   With n_tx transmitters interleaved in time, a given transmitter revisits
%   the scene every n_tx*T_chirp seconds. The velocity that can be measured
%   without ambiguity from that revisit rate is
%
%       v_amb = lambda / ( 4 n_tx T_chirp )
%
%   and the Doppler spectrum repeats every
%
%       dv = lambda / ( 2 n_tx T_chirp )
%
%   Crucially, a target whose true velocity lies outside +/- v_amb still lands
%   inside the principal Doppler interval, so an alias cannot be detected from
%   the Doppler measurement alone. This resolver therefore tests each physically
%   admissible alias hypothesis against evidence the alias does not control:
%
%     * spatial consistency - only the correct hypothesis de-rotates the TDM
%       phase properly, so only that hypothesis produces a sharp MUSIC and
%       conventional-beamformer peak;
%     * slow-time coherence - after correct de-rotation the snapshots of a
%       given virtual channel add coherently.
%
%   The combined score is
%
%       J(v) = prom_MUSIC(v) + 0.25 prom_FFT(v) + w * 10 log10 coh(v)
%
%   The raw hypothesis is retained unless a competitor beats it by more than
%   the configured margin, which keeps the resolver conservative. No truth and
%   no target count are used.

info = struct('alias_spacing_mps',0,'changed',0,'tested_hypotheses',0, ...
    'principal_range_preserved',0,'ambiguous_candidates',0,'alias_span',0, ...
    'method','joint TDM slow-time coherence and spatial consistency');
if isempty(detections), return; end

alias_spacing = p.lambda/(2*p.n_tx*p.Tchirp);
info.alias_spacing_mps = alias_spacing;
principal_v = p.unambiguous_velocity_nominal;
Vmax   = max(abs(p.v_max),principal_v);
margin = get_default_field(p.detector,'tdm_alias_score_margin_db',1.0);
wCoh   = get_default_field(p.detector,'tdm_coherence_weight',4.0);
kspan  = max(0,round(get_default_field(p.detector,'tdm_alias_span',2)));
info.alias_span = kspan;

for i = 1:numel(detections)
    v0 = detections(i).velocity;
    detections(i).velocity_raw     = v0;
    detections(i).tdm_alias_offset = 0;
    detections(i).tdm_alias_score  = NaN;
    detections(i).tdm_alias_mode   = 'principal-preserved';

    candV = v0 + (-kspan:kspan)*alias_spacing;
    candV = unique(candV(abs(candV) <= Vmax + 1e-9));
    if isempty(candV), continue; end
    if numel(candV) > 1, info.ambiguous_candidates = info.ambiguous_candidates + 1; end

    scores = -inf(size(candV));
    mus = nan(size(candV)); ffts = nan(size(candV)); coh = nan(size(candV));
    for k = 1:numel(candV)
        try
            va = tdm_virtual_aperture(rx_cube,p,detections(i).r_bin,candV(k),false);
            [~,~,~,~,ai] = music_aoa_estimator(va,p,detections(i).range);
            mus(k)  = ai.music_peak_prominence_db;
            ffts(k) = ai.beamformer_peak_prominence_db;
            coh(k)  = tdm_slow_time_coherence(va);
            scores(k) = mus(k) + 0.25*ffts(k) + wCoh*10*log10(max(coh(k),1e-6));
        catch
            scores(k) = -inf;
        end
info.tested_hypotheses = info.tested_hypotheses + 1;
    end

    [bestScore,best] = max(scores);
    if ~isfinite(bestScore), continue; end
    [~,rawIdx] = min(abs(candV - v0));
    rawScore = scores(rawIdx);
    if candV(best) ~= v0 && isfinite(rawScore) && bestScore < rawScore + margin
best = rawIdx;
bestScore = rawScore;
    end

    chosen = candV(best);
    detections(i).tdm_alias_coherence     = coh(best);
    detections(i).tdm_alias_angle_preview = mus(best);
    if abs(chosen - v0) > 1e-9
        detections(i).velocity          = chosen;
        detections(i).tdm_alias_offset  = chosen - v0;
        detections(i).tdm_alias_score   = bestScore;
        detections(i).tdm_alias_mode    = 'resolved-by-coherence-and-spatial-consistency';
info.changed = info.changed + 1;
    elseif abs(v0) > principal_v + 1e-9
        detections(i).tdm_alias_mode = 'principal-fallback';
    else
        detections(i).tdm_alias_mode = 'principal-selected';
info.principal_range_preserved = info.principal_range_preserved + 1;
    end
end
end

function c = tdm_slow_time_coherence(va)
%TDM_SLOW_TIME_COHERENCE  Normalised coherent gain across de-rotated snapshots.
% Returns 1 when every snapshot of a channel is in phase and 1/Ns when they
% are mutually incoherent.
if isempty(va), c = 0; return; end
num = abs(sum(va,2)).^2;
den = max(size(va,2)*sum(abs(va).^2,2),eps);
c = mean(min(max(num./den,0),1),'omitnan');
if ~isfinite(c), c = 0; end
end
