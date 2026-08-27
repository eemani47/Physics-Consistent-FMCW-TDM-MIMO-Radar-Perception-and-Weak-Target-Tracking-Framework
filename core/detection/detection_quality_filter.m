function [out,info] = detection_quality_filter(detections,p)
%DETECTION_QUALITY_FILTER  Late soft-evidence scoring of verified detections.
%
%   By this point a detection has already passed two independent hard tests: a
%   calibrated energy threshold and a spatial template test. This stage does
%   not repeat either. It combines the evidence that each earlier stage
%   produced into a single interpretable quality figure, and removes only the
%   detections that are inadmissible on physical grounds.
%
%       Q = w_c ( SNR_CFAR - SNR_min ) + w_a ( AMF - AMF_min )
%           + w_m prominence_MUSIC + w_g agreement
%
%   where agreement rewards a small disparity between the matched-filter and
%   high-resolution bearings, a disagreement being the signature of an
%   unresolved pair or a poorly conditioned covariance. The score is mapped to
%   a bounded confidence through a logistic so that downstream weighting is
%   well behaved, and it feeds the grouping weights.
%
%   Only two rejections happen here: a bearing outside the configured azimuth
%   field of view, which is not a physically observable direction for this
%   array, and a CFAR statistic below the absolute floor. Anything stricter
%   belongs in the object-existence gate, where persistence is available as
%   additional evidence and a single marginal frame cannot destroy a real
%   target.

out = detections([]);
info = struct('input_count',numel(detections),'output_count',0,'removed',0, ...
    'rejected_low_snr',0,'rejected_invalid_angle',0,'mean_quality_db',NaN, ...
    'weights',struct('cfar',0.45,'amf',0.35,'music',0.15,'agreement',0.05));
if isempty(detections), return; end

w = info.weights;
minSnr = get_default_field(p.cfar,'min_snr_db',-20);
minAmf = get_default_field(p.detector,'min_amf_db',7);
fuseGate = get_default_field(p.est,'amf_music_fuse_deg',5);

q = nan(1,numel(detections));
keep = true(1,numel(detections));
for i = 1:numel(detections)
    d = detections(i);
    theta = get_default_field(d,'angle_deg',NaN);
    if ~isfinite(theta) || abs(theta) > p.az_span
        keep(i) = false; info.rejected_invalid_angle = info.rejected_invalid_angle + 1;
continue;
    end
    cfarDb = get_default_field(d,'cfar_snr_db',-Inf);
    if ~isfinite(cfarDb) || cfarDb < minSnr
        keep(i) = false; info.rejected_low_snr = info.rejected_low_snr + 1;
continue;
    end

    amfDb = get_default_field(d,'amf_db',-Inf);
    prom  = get_default_field(d,'music_peak_prominence_db',0);
    diff  = get_default_field(d,'angle_difference_deg',NaN);
    if isfinite(diff)
        agree = max(0,1 - abs(diff)/max(fuseGate,eps))*10;
    else
agree = 0;
    end

    score = w.cfar*(cfarDb - minSnr) + ...
            w.amf*(max(amfDb,-30) - minAmf) + ...
            w.music*max(prom,0) + ...
w.agreement*agree;

    q(i) = score;
    detections(i).quality_score_db = score;
    detections(i).confidence = 1/(1+exp(-score/6));
    detections(i).aoa_quality_db = max(prom,0);
    detections(i).is_cluster_supported = false;
end

out = detections(keep);
info.output_count = numel(out);
info.removed = info.input_count - info.output_count;
qq = q(keep); qq = qq(isfinite(qq));
if ~isempty(qq), info.mean_quality_db = mean(qq); end
end
