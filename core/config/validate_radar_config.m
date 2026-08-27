function validate_radar_config(p)
%VALIDATE_RADAR_CONFIG  Fail-fast consistency checks for the full radar model.
%
%   Verifies that the coupled radar relationships actually hold in the
%   assembled parameter struct, that array geometry and axes are physically
%   realisable, and that every detector section carries the fields the
%   downstream code will dereference. Contradictory settings fail here rather
%   than propagating silently into the signal chain.

required = {'c','fc','lambda','B','Tchirp','slope','fs_ADC','Nr','Nd','n_tx','n_rx', ...
    'range_axis','vel_axis','noise_power_W','cfar','detector','est','tbd','track', ...
    'group','paper','interference','clutter','obs','eval','valid_range_mask'};
for k = 1:numel(required)
    if ~isfield(p,required{k})
        error('validate_radar_config:MissingField','Missing p.%s.',required{k});
    end
end

% ---- scalar integrity ----------------------------------------------------
intFields = {'Nr','Nd','n_tx','n_rx','N_angle'};
for k = 1:numel(intFields)
    f = intFields{k};
    v = p.(f);
    if ~(isnumeric(v) && isscalar(v) && isfinite(v) && v == round(v) && v > 0)
        error('validate_radar_config:IntegerField','p.%s must be a positive integer scalar.',f);
    end
end

% ---- coupled waveform relationships -------------------------------------
if abs(p.lambda - p.c/p.fc) > 1e-12*max(p.lambda,1)
    error('validate_radar_config:LambdaMismatch','lambda is inconsistent with c/fc.');
end
if abs(p.slope - p.B/p.Tchirp) > 1e-9*max(abs(p.slope),1)
    error('validate_radar_config:SlopeMismatch','slope is inconsistent with B/Tchirp.');
end
if abs(p.B - p.c/(2*p.R_res)) > 1e-6*p.B
    error('validate_radar_config:BandwidthMismatch','Bandwidth is inconsistent with the requested range resolution.');
end
if abs(p.Tchirp - p.lambda/(4*p.v_max)) > 1e-15
    error('validate_radar_config:ChirpMismatch','Chirp duration is inconsistent with the velocity design point.');
end
if p.fs_ADC <= p.fs_ADC_required_real
    error('validate_radar_config:Sampling','ADC rate does not clear the conservative real-ADC Nyquist requirement.');
end
if abs(p.tdm_unambiguous_velocity - p.lambda/(4*p.n_tx*p.Tchirp)) > 1e-12
    error('validate_radar_config:TDMAmbiguity','TDM same-TX unambiguous velocity is inconsistent with n_tx.');
end

% ---- axes ---------------------------------------------------------------
if numel(p.range_axis) ~= p.Nr || numel(p.vel_axis) ~= p.Nd
    error('validate_radar_config:AxisSize','Range/Doppler axes do not match Nr/Nd.');
end
if any(diff(p.range_axis) <= 0) || any(diff(p.vel_axis) <= 0)
    error('validate_radar_config:Axes','Range and velocity axes must be strictly increasing.');
end
if numel(p.valid_range_mask) ~= p.Nr || ~any(p.valid_range_mask)
    error('validate_radar_config:ValidMask','Valid range mask is malformed or empty.');
end

% ---- scene --------------------------------------------------------------
if size(p.targets,2) ~= 4
    error('validate_radar_config:TargetShape','Targets must be N-by-4 [R,V,RCS,Az].');
end
if any(~isfinite(p.targets(:)))
    error('validate_radar_config:TargetFinite','Target table contains non-finite values.');
end

% ---- array geometry -----------------------------------------------------
if numel(p.tx_x) ~= p.n_tx || numel(p.rx_x) ~= p.n_rx || numel(p.virtual_x) ~= p.n_virt
    error('validate_radar_config:ArrayGeometry','TX/RX/virtual geometry sizes are inconsistent.');
end
if any(~isfinite(p.tx_x)) || any(~isfinite(p.rx_x)) || any(~isfinite(p.virtual_x))
    error('validate_radar_config:ArrayFinite','Array coordinates must be finite.');
end
if numel(unique(round(p.virtual_x/max(p.lambda,eps),12))) ~= p.n_virt
    error('validate_radar_config:ArrayCollision','Virtual aperture contains duplicate phase-center positions.');
end
if p.az_span <= 0 || p.az_span >= 90
    error('validate_radar_config:AzimuthSpan','Azimuth span must lie in (0,90) degrees.');
end

% ---- detector sections --------------------------------------------------
reqCfar = {'Tr','Td','Gr','Gd','Pfa','os_fraction','local_max_r','local_max_d', ...
    'min_reference_cells','power_shape','max_detections','weak_snr_db','min_snr_db'};
for k = 1:numel(reqCfar)
    if ~isfield(p.cfar,reqCfar{k})
        error('validate_radar_config:CFARField','Missing p.cfar.%s.',reqCfar{k});
    end
end
if p.cfar.Tr < 1 || p.cfar.Td < 1 || p.cfar.Gr < 0 || p.cfar.Gd < 0
    error('validate_radar_config:CFARGeometry','Invalid CFAR training/guard sizes.');
end
if ~(p.cfar.Pfa > 0 && p.cfar.Pfa < 1)
    error('validate_radar_config:CFARPfa','CFAR Pfa must lie strictly between 0 and 1.');
end
if ~(p.cfar.os_fraction > 0 && p.cfar.os_fraction < 1)
    error('validate_radar_config:CFAROS','OS-CFAR fraction must lie strictly between 0 and 1.');
end
nTrain = (2*(p.cfar.Tr+p.cfar.Gr)+1)*(2*(p.cfar.Td+p.cfar.Gd)+1) - (2*p.cfar.Gr+1)*(2*p.cfar.Gd+1);
if nTrain < p.cfar.min_reference_cells
    error('validate_radar_config:CFARReference', ...
        'CFAR window yields %d reference cells but %d are required.',nTrain,p.cfar.min_reference_cells);
end

reqDet = {'amf_threshold_pfa','angle_coarse_step_deg','angle_refine_halfspan_deg', ...
    'angle_refine_step_deg','min_amf_db','max_candidates','gs'};
for k = 1:numel(reqDet)
    if ~isfield(p.detector,reqDet{k})
        error('validate_radar_config:DetectorField','Missing p.detector.%s.',reqDet{k});
    end
end
if ~(p.detector.amf_threshold_pfa > 0 && p.detector.amf_threshold_pfa < 1)
    error('validate_radar_config:AMFPfa','AMF Pfa must lie strictly between 0 and 1.');
end
if ~(p.detector.gs.pfa > 0 && p.detector.gs.pfa < 1)
    error('validate_radar_config:GSPfa','GS detector Pfa must lie strictly between 0 and 1.');
end
if p.detector.angle_coarse_step_deg <= 0 || p.detector.angle_refine_step_deg <= 0
    error('validate_radar_config:AngleStep','Angle search steps must be positive.');
end

if ~any(strcmpi(char(p.est.model_order),{'mdl','fixed'}))
    error('validate_radar_config:ModelOrder','Estimator model_order must be mdl or fixed.');
end
if p.est.n_src_max < 1 || p.est.n_src_max > max(1,p.n_virt-1)
    error('validate_radar_config:SourceCount','Estimator maximum source count is invalid.');
end
if p.est.fft_zero_pad < 1 || p.est.max_aoa_peaks < 1 || p.est.min_snapshots < 2
    error('validate_radar_config:EstimatorBounds','Estimator padding, peak count and snapshot minimum must be positive.');
end

% ---- TBD / tracker ------------------------------------------------------
if p.tbd.min_path_frames < 2
    error('validate_radar_config:TBDPath','TBD minimum path length must be at least 2 frames.');
end
if ~(p.tbd.min_path_support_fraction > 0 && p.tbd.min_path_support_fraction <= 1)
    error('validate_radar_config:TBDSupport','TBD support fraction must lie in (0,1].');
end
if ~(p.tbd.coherent.Pfa > 0 && p.tbd.coherent.Pfa < 1)
    error('validate_radar_config:CoherentPfa','Coherent TBD Pfa must lie strictly between 0 and 1.');
end
if p.track.gate_nis <= 0
    error('validate_radar_config:TrackGate','Track NIS gate must be positive.');
end
if p.track.measurement_sigma_range <= 0 || p.track.measurement_sigma_velocity <= 0 || p.track.measurement_sigma_angle <= 0
    error('validate_radar_config:TrackNoise','Track measurement standard deviations must be positive.');
end

% ---- noise --------------------------------------------------------------
if p.noise_power_W <= 0 || ~isfinite(p.noise_power_W)
    error('validate_radar_config:Noise','Receiver thermal-noise power must be finite and positive.');
end
if numel(p.rx_noise_power_W) ~= p.n_rx
    error('validate_radar_config:NoisePerRx','Per-receiver noise power vector length must equal n_rx.');
end

if p.N_angle < 128
    warning('validate_radar_config:AngleGrid','N_angle is small for a research-grade AoA grid.');
end
end
