function detections = angle_refinement(detections,rx_cube,p)
%ANGLE_REFINEMENT  Final per-detection TDM-MIMO angle estimate.
%
%   Each surviving detection already carries an angle from the adaptive matched
%   filter, obtained by maximising a whitened template response over the
%   steering manifold. This stage forms the virtual aperture at the detection's
%   range bin and velocity, runs the high-resolution estimator, and fuses the
%   two answers under an explicit rule:
%
%     * the matched-filter solution is authoritative, because it was produced
%       by the same hypothesis test that admitted the detection;
%     * the high-resolution peak is accepted as a refinement only when it is
%       prominent enough, supported by enough snapshots, and lies within a
%       configured angular distance of the matched-filter solution;
%     * when both conditions hold the two are combined as unit vectors, which
%       averages correctly across the +/-180 degree wrap.
%
%   A high-resolution peak is never allowed to silently overwrite a verified
%   template solution, so a spurious MUSIC null cannot move a confirmed
%   detection across the beam.

fuseGate = get_default_field(p.est,'amf_music_fuse_deg',5.0);
promGate = get_default_field(p.est,'music_min_prom_db',3.0);

for i = 1:numel(detections)
    if ~isfinite(detections(i).range), continue; end
    rb = nearest_index(p.range_axis,detections(i).range);
    [va,rb] = tdm_virtual_aperture(rx_cube,p,rb,detections(i).velocity,false);
    [thb,Pb,thm,Pm,info] = music_aoa_estimator(va,p,detections(i).range);

    theta_amf   = get_default_field(detections(i),'angle_deg',NaN);
theta_music = info.music_peak_angle_deg;
    musicUsable = isfinite(theta_music) && ...
        info.music_peak_prominence_db >= promGate && ...
info.snapshot_sufficient;
info.music_usable = musicUsable;

    if isfinite(theta_amf)
theta = theta_amf;
        info.final_angle_source = 'AMF';
        if musicUsable && abs(wrap_angle(theta_music-theta_amf)) <= fuseGate
            theta = atan2d(sind(theta_amf)+sind(theta_music), cosd(theta_amf)+cosd(theta_music));
            info.final_angle_source = 'AMF+MUSIC';
        end
    elseif musicUsable
theta = theta_music;
        info.final_angle_source = 'MUSIC';
    else
theta = info.beamformer_peak_angle_deg;
        info.final_angle_source = 'BEAMFORMER';
    end
    % A bearing that did not resolve is left as NaN. Substituting boresight
    % would emit a confident report pointing straight ahead, which is how an
    % unresolved detection becomes a false object at zero degrees. The quality
    % filter rejects a non-finite bearing outright.
    if ~isfinite(theta), theta = NaN; info.final_angle_source = 'UNRESOLVED'; end
    % A bearing sitting on the edge of the search grid is the estimator
    % running out of grid, not finding a source there. Reporting it as a
    % measurement places an object hard against the field-of-view boundary
    % with full confidence. The quality filter rejects a non-finite bearing.
    edgeTol = max(2*get_default_field(p.detector,'angle_refine_step_deg',0.25),0.5);
    if isfinite(theta) && (p.az_span - abs(theta)) <= edgeTol
        theta = NaN;
        info.final_angle_source = 'GRID-EDGE-REJECTED';
    end

    detections(i).angle_deg   = theta;
    detections(i).x_pos       = detections(i).range*sind(theta);
    detections(i).y_pos       = detections(i).range*cosd(theta);
    detections(i).angle_fft_deg = info.beamformer_peak_angle_deg;
    detections(i).aoa_fft_deg   = info.beamformer_peak_angle_deg;
    detections(i).angle_music_deg = theta_music;
    detections(i).music_peak_db = info.music_peak_db;
    detections(i).music_peak_prominence_db = info.music_peak_prominence_db;
    detections(i).fft_peak_prominence_db   = info.beamformer_peak_prominence_db;
    detections(i).angle_difference_deg     = info.angle_difference_deg;
    detections(i).music_theta    = thm;
    detections(i).music_spectrum = Pm;
    detections(i).fft_theta      = thb;
    detections(i).fft_spectrum   = Pb;
    detections(i).music_info     = info;
    detections(i).r_bin          = rb;
end
end

function k = nearest_index(x,v), [~,k] = min(abs(x-v)); end
function a = wrap_angle(a), a = mod(a+180,360)-180; end
