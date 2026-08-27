function p = radar_configuration(config)
%RADAR_CONFIGURATION  Physics-consistent FMCW/TDM-MIMO radar system model.
%
%   p = RADAR_CONFIGURATION(config) turns a small set of user-level radar
%   design choices into the complete, dimensionally consistent parameter set
%   consumed by the simulator, the signal-processing chain, the detector, the
%   estimators, the track-before-detect branch, the tracker and the evaluator.
%
%   Design rules enforced here:
%     * Every derived quantity follows from a coupled radar relationship.
%       Range resolution fixes bandwidth, the velocity design point fixes the
%       chirp duration, and the chirp slope follows from both.
%     * Detection thresholds are expressed as false-alarm probabilities,
%       physical SNR margins or geometric gates, never as opaque scores.
%     * Receiver noise is a band-limited thermal process derived from kTBF.
%     * Every field written here is consumed somewhere in the pipeline. There
%       are no vestigial knobs.
%
%   Nested sections (cfar, paper, detector, est, tbd, track, group,
%   interference, clutter, range_processing, obs, noise) may be overridden by
%   supplying matching nested structs in CONFIG. Overrides are merged
%   recursively, so partial sections are legal.

if nargin < 1 || isempty(config), config = struct(); end
if ~isstruct(config)
    error('radar_configuration:Type','config must be a struct, not %s.',class(config));
end

% Normalize stored integer-class configuration values before derived arithmetic.
config = coerce_numeric_double(config);

% -------------------------------------------------------------------------
% Learned detector/tracker defaults. These are a persistent default layer:
% explicit caller fields remain higher priority, while the learned values are
% automatically used by every later run after feedback learning completes.
% -------------------------------------------------------------------------
% Only the algorithm sections are adopted. The learned file also records the
% full parameter set it was measured on, which carries the training scene,
% array geometry and waveform; adopting those would silently replace the
% caller's radar with the one the tuning happened to use.
config = apply_learned_defaults(config);

% -------------------------------------------------------------------------
% Top-level user-facing defaults.
% -------------------------------------------------------------------------
defaults = struct();
defaults.fc        = 77e9;
defaults.R_max     = 300;
defaults.R_res     = 0.50;
defaults.v_max     = 60;
defaults.v_res     = 0.234375;
defaults.N_angle   = 1024;
defaults.az_span   = 70;
defaults.n_tx      = 2;
defaults.n_rx      = 4;
defaults.P_tx_dBm  = 12;
defaults.G_dB      = 15;
defaults.NF_dB     = 10;
defaults.temp_K    = 290;
defaults.random_seed = 47;
defaults.targets = [ ...
30,   5,  10, -50;
...
45, -12,  12, -18;
...
65,  -8,  11,  35;
...
82,   7,   8,  10;
...
105,  14,  12,  -5;
...
125,  20,  16,  28;
...
155,  -2,  13, -36;
...
   175,   4,  11,  50];

defaults.noise_enabled          = true;
defaults.noise_figure_dB        = defaults.NF_dB;
defaults.receiver_temperature_K = defaults.temp_K;
defaults.swerling_model         = 'Swerling0';
defaults.interference_enabled   = true;
defaults.clutter_enabled        = true;
defaults.rx_noise_figure_dB     = [];
defaults.rx_noise_temperature_K = [];
defaults.snr_override_enabled   = false;
defaults.noise_model            = 'Physical thermal AWGN';
defaults.noise_level            = 20;
defaults.noise_fixed_power_dBm  = -92;

fn = fieldnames(defaults);
for k = 1:numel(fn)
    if ~isfield(config,fn{k}) || isempty(config.(fn{k}))
        config.(fn{k}) = defaults.(fn{k});
    end
end

% -------------------------------------------------------------------------
% Constants and primary design point.
% -------------------------------------------------------------------------
p       = struct();
p.c     = 299792458;
p.k_B   = 1.380649e-23;
p.fc    = scalar_pos(config.fc,'fc');
p.lambda= p.c/p.fc;
p.temp  = scalar_pos(config.temp_K,'temp_K');
p.R_max = scalar_pos(config.R_max,'R_max');
p.R_res = scalar_pos(config.R_res,'R_res');
p.v_max = scalar_pos(config.v_max,'v_max');
p.v_res = scalar_pos(config.v_res,'v_res');

validateattributes(config.targets,{'numeric'},{'2d','ncols',4,'real','finite'});
p.targets   = double(config.targets);
p.n_targets = size(p.targets,1);
if any(p.targets(:,1) <= 0 | p.targets(:,1) >= p.R_max)
    error('radar_configuration:TargetsRange','All target ranges must satisfy 0 < R < R_max.');
end
if any(abs(p.targets(:,2)) >= p.v_max)
    warning('radar_configuration:TargetsVelocity', ...
        'At least one target is at or beyond the configured principal velocity limit.');
end

% Waveform description carried into simulator metadata.
p.waveform = struct( ...
    'model','equivalent_complex_baseband_dechirp', ...
    'dechirp_enabled',true, ...
    'carrier_sampled_directly',false, ...
    'dechirp_sign','tx_times_conj_rx', ...
    'beat_frequency_sign','positive');

% -------------------------------------------------------------------------
% Coupled waveform design.
%   B      = c / (2 dR)                 range resolution -> bandwidth
%   Tchirp = lambda / (4 v_max)         velocity design point -> chirp time
%   S      = B / Tchirp                 chirp slope
%   Nd     = 2^ceil(log2(lambda/(2 dV Tchirp)))
% v_max is the consecutive-chirp Doppler bound. For n_tx > 1 the same-TX TDM
% ambiguity is narrower by exactly n_tx and is reported separately.
% -------------------------------------------------------------------------
p.B      = p.c/(2*p.R_res);
p.Tchirp = p.lambda/(4*p.v_max);
p.PRF    = 1/p.Tchirp;
p.slope  = p.B/p.Tchirp;
p.Nd     = 2^nextpow2(max(32,ceil(p.lambda/(2*p.v_res*p.Tchirp))));

p.f_doppler_max = 2*p.v_max/p.lambda;
p.f_beat_max    = 2*p.slope*p.R_max/p.c + p.f_doppler_max;

% Sampling. The dechirped interface is complex I/Q, so the information-theoretic
% requirement is fs > f_beat_max. A conservative real-ADC requirement (2 f_beat_max)
% is retained as the hard floor, and a fast-time length floor guarantees enough
% range bins for a research-grade range FFT.
p.fs_ADC_required_complex = p.f_beat_max;
p.fs_ADC_required_real    = 2*p.f_beat_max;
p.fast_time_samples_min   = 2048;
p.fs_ADC = max(2.5*p.f_beat_max, p.fast_time_samples_min/p.Tchirp);
if p.fs_ADC <= p.fs_ADC_required_real
    error('radar_configuration:ADCSampling', ...
        'ADC sample rate %.3f MHz does not clear the real-ADC Nyquist requirement %.3f MHz.', ...
        p.fs_ADC/1e6,p.fs_ADC_required_real/1e6);
end
p.fs_ADC_margin_over_real_nyquist = p.fs_ADC/p.fs_ADC_required_real;
p.Nr = 2^nextpow2(max(p.fast_time_samples_min,ceil(p.fs_ADC*p.Tchirp)));

% Physical axes.
p.f_range_axis = (0:p.Nr-1)*(p.fs_ADC/p.Nr);
p.range_axis   = p.c*p.f_range_axis/(2*p.slope);
p.fd_axis      = (-floor(p.Nd/2):ceil(p.Nd/2)-1)*(p.PRF/p.Nd);
p.vel_axis     = p.lambda*p.fd_axis/2;

% -------------------------------------------------------------------------
% Array geometry. TX and RX phase centres are physical coordinates; the
% virtual aperture is their pairwise sum, not a synthetic phase model.
% -------------------------------------------------------------------------
p.n_tx   = scalar_int(config.n_tx,'n_tx',1);
p.n_rx   = scalar_int(config.n_rx,'n_rx',1);
p.n_virt = p.n_tx*p.n_rx;
p.d_rx   = p.lambda/2;
p.d_tx   = p.n_rx*p.d_rx;
p.rx_x   = (0:p.n_rx-1)*p.d_rx;
p.tx_x   = (0:p.n_tx-1)*p.d_tx;
p.virtual_x = zeros(1,p.n_virt);
vi = 1;
for itx = 1:p.n_tx
    for irx = 1:p.n_rx
        p.virtual_x(vi) = p.tx_x(itx) + p.rx_x(irx);
vi = vi + 1;
    end
end
p.array_geometry_units = 'm';
p.array_geometry = struct('tx_x',p.tx_x,'rx_x',p.rx_x,'virtual_x',p.virtual_x);
p.aperture_length_m = max(p.virtual_x) - min(p.virtual_x);
% Angular resolution of the virtual aperture, from the standard broadside
% half-power beamwidth of a uniform line array, theta_3dB ~ 0.886 lambda / L.
% Two reports inside one beamwidth are not separable by this array, so keeping
% them as distinct objects claims a resolution the radar does not have.
p.beamwidth_deg = rad2deg(0.886*p.lambda/max(p.aperture_length_m,eps));
p.N_angle    = scalar_int(config.N_angle,'N_angle',128);
p.az_span    = min(scalar_pos(config.az_span,'az_span'),89);
p.theta_axis = linspace(-p.az_span,p.az_span,p.N_angle);

% -------------------------------------------------------------------------
% Transmit power, antenna gain and band-limited receiver noise.
%   N = k T B F, with B the one-sided complex IF noise bandwidth.
% The simulator shapes the injected AWGN to exactly this band so that the
% declared noise power and the sampled process agree.
% -------------------------------------------------------------------------
p.P_tx_dBm = scalar_num(config.P_tx_dBm,'P_tx_dBm');
p.G_dB     = scalar_num(config.G_dB,'G_dB');
p.NF_dB    = scalar_num(config.NF_dB,'NF_dB');
p.P_tx_W   = 1e-3*10^(p.P_tx_dBm/10);
p.G_lin    = 10^(p.G_dB/10);
p.noise_figure_linear = 10^(p.NF_dB/10);
p.noise_bandwidth       = p.f_beat_max;
p.noise_bandwidth_basis = 'complex_baseband_one_sided_IF';

p.rx_noise_figure_dB     = expand_per_rx(config.rx_noise_figure_dB,p.NF_dB,p.n_rx,'rx_noise_figure_dB',false);
p.rx_noise_temperature_K = expand_per_rx(config.rx_noise_temperature_K,p.temp,p.n_rx,'rx_noise_temperature_K',true);
p.rx_noise_power_W = p.k_B .* p.rx_noise_temperature_K .* p.noise_bandwidth .* 10.^(p.rx_noise_figure_dB/10);
p.noise_power_W    = mean(p.rx_noise_power_W);

switch lower(strrep(char(config.noise_model),' ','_'))
    case {'physical_thermal_awgn','physical_thermal','thermal_awgn'}, noiseMode = 'physical_rx_chain';
    case {'snr-controlled_awgn','snr_controlled_awgn','snr_awgn','snr'}, noiseMode = 'snr_awgn';
    case {'fixed_awgn_power','fixed_awgn','fixed_power_awgn','fixed_power'}, noiseMode = 'fixed_awgn';
    case {'no_noise','none','off'}, noiseMode = 'none';
    otherwise
        error('radar_configuration:NoiseModel','Unsupported noise model: %s',char(config.noise_model));
end
p.noise = struct();
p.noise.mode              = noiseMode;
p.noise.level             = scalar_num(config.noise_level,'noise_level');
p.noise.fixed_power_dBm   = scalar_num(config.noise_fixed_power_dBm,'noise_fixed_power_dBm');
p.noise.fixed_power_W     = 1e-3*10^(p.noise.fixed_power_dBm/10);
p.noise.snr_db            = p.noise.level;
p.noise.enabled           = logical(config.noise_enabled) && ~strcmp(noiseMode,'none');
p.noise.snr_override_enabled = logical(config.snr_override_enabled);
p.noise.power_W           = p.noise_power_W;
p.noise.power_per_rx_W    = p.rx_noise_power_W;
p.noise.temperature_K     = p.temp;
p.noise.temperature_per_rx_K = p.rx_noise_temperature_K;
p.noise.noise_figure_dB   = p.NF_dB;
p.noise.noise_figure_per_rx_dB = p.rx_noise_figure_dB;
p.noise.bandwidth_Hz      = p.noise_bandwidth;
p.noise.injection_point   = 'post_dechirp_pre_ADC';
p.noise.independent_rx_chains = true;
% Anti-alias shaping: white noise is band-limited to the IF passband so that
% total injected power equals kTBF exactly. Set false for an unshaped white model.
p.noise.band_limited      = true;

p.target = struct('swerling_model',string(config.swerling_model),'rcs_units','m^2');

% -------------------------------------------------------------------------
% Realised resolution and ambiguity.
% -------------------------------------------------------------------------
p.range_resolution_actual    = p.c/(2*p.slope)*(p.fs_ADC/p.Nr);
p.range_resolution_nominal   = p.c/(2*p.B);
p.velocity_resolution_actual = p.lambda/(2*p.Nd*p.Tchirp);
p.velocity_resolution_nominal= p.velocity_resolution_actual;
p.unambiguous_velocity_nominal = p.lambda/(4*p.Tchirp);
p.tdm_same_tx_spacing        = p.n_tx*p.Tchirp;
p.tdm_unambiguous_velocity   = p.lambda/(4*p.tdm_same_tx_spacing);
p.tdm_alias_spacing_mps      = p.lambda/(2*p.tdm_same_tx_spacing);

% -------------------------------------------------------------------------
% Calibrated 2-D CFAR. Pfa is a null probability; alpha is solved numerically
% from the assumed Gamma null unless an explicit alpha override is supplied.
% -------------------------------------------------------------------------
p.cfar = struct();
p.cfar.mode        = 'adaptive';
p.cfar.Tr = 12;
p.cfar.Td = 16;
p.cfar.Gr = 2;
p.cfar.Gd = 2;
p.cfar.Pfa         = 1e-5;
p.cfar.os_fraction = 0.75;
p.cfar.local_max_r = 1;
p.cfar.local_max_d = 1;
p.cfar.group_r_bins = 2;
p.cfar.group_d_bins = 2;
% Trimmed by the full CFAR window half-width. A cell closer to the band edge
% than that draws part of its reference from cells the valid mask holds at
% zero, which drags the estimate down and, when a whole region is zero,
% collapses it to realmin and reports an impossible signal-to-noise ratio.
p.cfar.valid_margin_bins   = 1;
% Exclude unresolved Doppler-edge bins from declarations and weak candidates.
p.cfar.valid_doppler_margin_bins = 3;
p.cfar.weak_snr_db         = -6.6148;
p.cfar.min_snr_db          = -20.7377;
p.cfar.min_reference_cells = 32;
p.cfar.edge_ratio_low      = 0.65;
p.cfar.edge_ratio_high     = 1.55;
p.cfar.heterogeneity_cv    = 0.8;
p.cfar.os_contamination_ratio = 0.75;
p.cfar.max_detections      = 256;
p.cfar.subbin_interpolation= true;
% Point-spread rejection. A windowed transform spreads a point target over a
% mainlobe of a few cells surrounded by sidelobes at a known level, so a strong
% target manufactures secondary local maxima that are real energy but not
% separate objects. Left in, they become their own tracks, and their number
% grows with target strength - which is why a false-object count that rises
% with signal-to-noise ratio is the signature of this and not of noise.
% A candidate is rejected when it lies inside the response width of a stronger
% candidate and sits below it by more than the window can produce.
p.cfar.psf_rejection        = true;
p.cfar.psf_range_bins       = 6;      % half-width of the range response
p.cfar.psf_doppler_bins     = 4;      % half-width of the Doppler response
p.cfar.psf_margin_db        = 26.0;   % below the 31.5 dB Hann sidelobe ratio
% Explicit multiplier overrides. NaN keeps the numerically calibrated value.
p.cfar.ca_alpha = NaN;
p.cfar.os_alpha = NaN;
p.cfar.go_alpha = NaN;
p.cfar.so_alpha = NaN;
% Gamma shape of the cell-under-test under H0: non-coherent sum of n_rx
% independent complex channel powers.
p.cfar.power_shape = p.n_rx;

% -------------------------------------------------------------------------
% Coherent stationary/moving decomposition (Hyun, Jin & Lee, 2017).
% -------------------------------------------------------------------------
p.paper = struct();
p.paper.enabled                       = true;
p.paper.coherent_subtraction          = true;
p.paper.moving_coherent_mean          = true;
p.paper.moving_use_cfar               = true;
p.paper.coarse_grouping_before_tracking = true;
p.paper.separate_track_processes      = true;
p.paper.stationary = struct();
p.paper.stationary.enabled            = true;
p.paper.stationary.use_calibrated_cfar= true;
p.paper.stationary.Pfa                = 1e-4;
p.paper.stationary.threshold_scale    = 10.0;   % used when calibration disabled
p.paper.stationary.min_reference_cells= 48;
p.paper.stationary.range_ref_bins     = 6;
p.paper.stationary.angle_ref_bins     = 8;
p.paper.stationary.range_guard_bins   = 1;
p.paper.stationary.angle_guard_bins   = 1;
p.paper.stationary.local_max_range    = 1;
p.paper.stationary.local_max_angle    = 1;
p.paper.stationary.max_detections     = 64;

% -------------------------------------------------------------------------
% Second-stage verification: adaptive matched filter and spatial GS detector.
% -------------------------------------------------------------------------
p.detector = struct();
p.detector.angle_coarse_step_deg   = 2.0;
p.detector.angle_refine_halfspan_deg = 3.0;
p.detector.angle_refine_step_deg   = 0.25;
p.detector.max_candidates          = 512;
p.detector.amf_threshold_pfa       = 6.1003e-04;
p.detector.min_amf_db              = 8.0303;
p.detector.hard_point_only         = true;
p.detector.use_whitened_amf        = true;
p.detector.noise_estimation_guard_ratio = 0.90;
p.detector.min_template_energy     = 1e-12;
p.detector.max_angle_hypotheses    = 361;
p.detector.gs = struct();
p.detector.gs.enabled              = true;
p.detector.gs.pfa                  = 1e-3;
p.detector.gs.min_snapshots        = max(4,p.n_tx+2);
p.detector.gs.angle_step_deg       = 1.0;
p.detector.gs.min_interference_angle_sep_deg = 8.0;
p.detector.gs.interference_inr_threshold_db  = 3.0;
p.detector.gs.max_inr_linear       = 1e4;
p.detector.gs.rescue_enabled       = true;
p.detector.gs.rescue_min_db        = 0.0;
p.detector.gs.use_measured_noise   = true;
p.detector.tdm_alias_span          = 2;
p.detector.tdm_alias_score_margin_db = 1.0;
p.detector.tdm_coherence_weight    = 4.0;

% -------------------------------------------------------------------------
% Angle-of-arrival estimation.
% -------------------------------------------------------------------------
p.est = struct();
p.est.music_grid   = 2048;
p.est.music_min_prom_db = 3;
p.est.n_src_max    = min(3,max(1,p.n_virt-1));
p.est.music_cov_diagonal_loading = 1e-3;
p.est.model_order  = 'mdl';
p.est.fft_zero_pad = max(8,round(p.N_angle/p.n_virt));
p.est.max_aoa_peaks= 3;
p.est.min_snapshots= max(8,p.n_tx+2);
p.est.forward_backward = true;
p.est.amf_music_fuse_deg = 5.0;

% -------------------------------------------------------------------------
% Track-before-detect.
% -------------------------------------------------------------------------
p.tbd = struct();
p.tbd.enabled                  = true;
p.tbd.max_candidates_per_frame = 256;
p.tbd.weak_snr_db              = -6.0;
p.tbd.local_max_range_radius   = 1;
p.tbd.local_max_velocity_radius= 1;
p.tbd.max_paths                = 20;
p.tbd.max_confirmed_paths      = 16;
p.tbd.max_cell_llr             = 50.0;
p.tbd.min_path_frames          = 5;
% Sliding window for the live weak-target search. Cost is linear in the number
% of frames searched, so searching the whole display buffer made per-frame time
% climb until the buffer filled. A window slightly longer than the shortest
% admissible path carries all the evidence a path can use.
p.tbd.live_window_frames       = 8;
p.tbd.min_path_support_fraction= 0.9165;
p.tbd.path_min_score           = 7.0;
p.tbd.path_promotion_score     = 17.4950;
p.tbd.birth_log_prior          = -1.5;
p.tbd.birth_min_cfar_db        = -7.0;
p.tbd.continuation_log_bonus   = 0.25;
p.tbd.gap_penalty              = 1.5;
p.tbd.miss_penalty_db          = 1.0;
p.tbd.max_gap_frames           = 2;
p.tbd.range_sigma_m            = max(1.5*p.range_resolution_actual,0.40);
p.tbd.velocity_sigma_mps       = max(2.0*p.velocity_resolution_actual,0.50);
p.tbd.angle_sigma_deg          = 4.0;
p.tbd.max_transition_range_m   = max(6.0,4.5*p.range_resolution_actual);
p.tbd.max_transition_velocity_mps = max(2.0,4.0*p.velocity_resolution_actual);
p.tbd.max_path_fit_residual_m  = max(2.5,3*p.range_resolution_actual);
p.tbd.max_path_fit_velocity_error_mps = 4.0;
p.tbd.path_exclusion_range_m   = max(2.0,2.5*p.range_resolution_actual);
p.tbd.path_exclusion_velocity_mps = max(1.0,2.5*p.velocity_resolution_actual);
p.tbd.path_mean_llr_bonus      = 0.75;
p.tbd.path_angle_bonus         = 0.50;
p.tbd.path_amf_pfa             = 0.10;
p.tbd.path_min_amf_db          = 7.3300;
p.tbd.path_promotion_min_amf_db= 7.3300;
p.tbd.path_angle_max_std_deg   = 6.8350;
p.tbd.path_angle_min_prominence_db = 0.5;
% Angular support required of a path that votes on bearing frame by frame, as
% the coherent branch does. The dynamic-programming branch forms no spatial
% estimate of its own; its bearing is measured once at promotion, and a path
% whose bearing does not resolve is not promoted at all.
p.tbd.min_promotion_angle_support  = 0.50;
% Local reference window used to normalise weak-cell evidence independently of
% the CFAR reference map.
p.tbd.reference_range_radius   = 8;
p.tbd.reference_velocity_radius= 8;
p.tbd.reference_guard_range    = 2;
p.tbd.reference_guard_velocity = 2;
% Cost-based ownership test against already-verified hard points.
p.tbd.suppress_near_hard       = true;
p.tbd.near_hard_suppress_amf_db= 0.0;
p.tbd.match_range_sigma_m      = max(1.5*p.range_resolution_actual,0.5);
p.tbd.match_velocity_sigma_mps = max(2.0*p.velocity_resolution_actual,0.5);
p.tbd.max_verified_match_cost  = 9.0;
% Object-level fusion and exclusion.
p.tbd.object_fusion_range_m      = max(3.0,6.0*p.range_resolution_actual);
p.tbd.object_fusion_velocity_mps = max(1.0,2.0*p.velocity_resolution_actual);
p.tbd.object_fusion_angle_deg    = max(3.0,1.00*p.beamwidth_deg);
p.tbd.object_fusion_gate       = 2.5;
p.tbd.exclusion_track_min_hits = 2;
% Range-dependent prior and long-range recovery.
p.tbd.range_compensation_enabled = false;
p.tbd.range_compensation_start_fraction = 0.70;
p.tbd.range_compensation_max_db = 6.0;
p.tbd.far_range_recovery_start_m = 0.75*p.R_max;
p.tbd.far_range_recovery_min_integrated_amf_db = 5.0;
p.tbd.trajectory_recovery_enabled = true;
p.tbd.trajectory_recovery_min_integrated_amf_db = 7.0;
p.tbd.trajectory_recovery_min_support_fraction  = 0.80;
% Motion-compensated coherent integration branch.
p.tbd.coherent = struct();
p.tbd.coherent.enabled              = true;
p.tbd.coherent.start_range_fraction = 0.72;
p.tbd.coherent.seed_threshold_db    = -12.0;
p.tbd.coherent.max_seeds_per_frame  = 48;
p.tbd.coherent.max_paths            = 12;
p.tbd.coherent.min_path_frames      = 5;
p.tbd.coherent.min_support_fraction = 0.80;
p.tbd.coherent.max_range_step_m     = max(6.0,5.0*p.range_resolution_actual);
p.tbd.coherent.max_velocity_step_mps= max(2.0,4.0*p.velocity_resolution_actual);
p.tbd.coherent.max_acceleration_mps2= 8.0;
p.tbd.coherent.path_score_threshold = 8.0;
p.tbd.coherent.coherent_score_threshold_db = 7.0;
p.tbd.coherent.path_promotion_score_db     = 10.0;
p.tbd.coherent.window               = 'hann';
p.tbd.coherent.use_tdm_tx_separation= true;
p.tbd.coherent.noise_normalized     = true;
p.tbd.coherent.use_gamma_threshold  = true;
p.tbd.coherent.Pfa                  = 1e-3;

% -------------------------------------------------------------------------
% Group tracker and final-object existence logic.
% -------------------------------------------------------------------------
p.track = struct();
p.track.dt = p.Nd*p.Tchirp;
p.track.process_sigma_a = 3.0;
p.track.q = p.track.process_sigma_a^2;
p.track.measurement_sigma_range    = max(p.range_resolution_actual,0.25);
p.track.measurement_sigma_velocity = max(p.velocity_resolution_actual,0.10);
p.track.measurement_sigma_angle    = 3.0;
p.track.process_sigma_angle_deg    = 1.5;   % per-frame angular process noise
p.track.gate_range_sigma    = 2.9673;
p.track.gate_velocity_sigma = 2.4720;
p.track.gate_angle_sigma    = 1.4813;
p.track.gate_nis            = 16.0;   % chi-square(2) gate on the R/V innovation
p.track.birth_min_amf_db    = 4.0;
% Provisional confirmation.
p.track.group_confirm_hits  = 3;
p.track.group_confirm_window= 4;
p.track.group_max_missed    = 2;
p.track.provisional_max_missed = 1;
p.track.min_confirmation_mean_amf_db  = 6.0;
p.track.min_confirmation_mean_cfar_db = 3.0;
p.track.min_confirmation_angle_support= 0.50;
p.track.max_confirmation_angle_std_deg= 8.0;
p.track.min_strong_amf_db   = 8.5;
p.track.min_strong_cfar_db  = 6.0;
p.track.strong_evidence_bonus = 2.0;  % strong hits count extra toward confirmation
% Final-object existence gate (moving branch).
p.track.group_final_min_hits      = 3;
p.track.group_final_min_support   = 0.7453;
p.track.group_final_mean_amf_db   = 10.4731;
p.track.group_final_mean_cfar_db  = 6.3548;
p.track.group_final_angle_support = 0.70;
p.track.group_final_angle_std_deg = 5.7635;
% Final-object existence gate (stationary branch).
p.track.stationary_group_final_min_hits      = 3;
p.track.stationary_group_final_min_support   = 0.70;
p.track.stationary_group_final_mean_amf_db   = 10.0;
p.track.stationary_group_final_mean_cfar_db  = 6.0;
p.track.stationary_group_final_angle_support = 0.70;
p.track.stationary_group_final_angle_std_deg = 6.0;
% Alternative promotion route for well-supported but marginal tracks.
p.track.group_recovery_min_hits    = 5;
p.track.group_recovery_min_support = 0.9377;
p.track.group_recovery_mean_amf_db = 10.0;
p.track.group_recovery_mean_cfar_db= 6.0;
p.track.group_recovery_angle_support = 0.90;
p.track.group_recovery_angle_std_deg = 6.0;
% Persistent strong-track recovery across a temporary miss.
p.track.persistent_recovery_min_hits      = 6;
p.track.persistent_recovery_min_support   = 0.85;
p.track.persistent_recovery_min_hard_hits = 5;
p.track.persistent_recovery_mean_amf_db   = 15.6790;
p.track.persistent_recovery_angle_support = 0.85;
p.track.persistent_recovery_angle_std_deg = 7.0;
p.track.persistent_recovery_max_missed    = 1;
% End-of-range region: stricter evidence, no recovery route.
p.track.range_edge_fraction = 0.92;
p.track.edge_min_hits        = 3;
p.track.edge_min_strong_hits = 2;
p.track.edge_min_mean_amf_db = 10.0;
p.track.edge_min_mean_cfar_db= 6.0;
% Duplicate suppression and history depth.
% Duplicate suppression is set by what the sensor can actually resolve.
% Reporting two objects closer together than one range cell and one beamwidth
% manufactures a false object out of a single physical response.
% Sized to the width of a point response, not to bare resolution: two reports
% inside one response width are the same physical return however they arose.
p.track.duplicate_range_m      = max(3.0,6.0*p.range_resolution_actual);
p.track.duplicate_velocity_mps = max(1.0,2.0*p.velocity_resolution_actual);
p.track.duplicate_angle_deg    = max(3.0,1.00*p.beamwidth_deg);
p.track.max_history_frames   = 16;

% -------------------------------------------------------------------------
% Point-cloud grouping.
% -------------------------------------------------------------------------
p.group = struct();
p.group.position_gate_m   = 1.3156;
p.group.velocity_gate_mps = 2.3437;
p.group.angle_gate_deg    = 2.7541;
p.group.max_cluster_cost  = 1.0;
p.group.min_points        = 1;
p.group.max_points_per_group = 32;
p.group.weight_floor_db   = 2.0;
p.group.adaptive_position_gain = 0.0;
p.group.enable_centroid_growth = true;
p.group.use_cartesian          = true;
p.group.enforce_common_radial_velocity = true;
p.group.moving_velocity_gate_mps     = 2.3437;
p.group.stationary_velocity_gate_mps = max(0.75,2*p.velocity_resolution_actual);

% Offline observability gates (diagnostics only).
p.obs = struct('range_gate_m',1.5,'velocity_gate_mps',1.25,'angle_gate_deg',5.0, ...
    'group_range_gate_m',2.0,'group_velocity_gate_mps',1.75,'group_angle_gate_deg',5.0, ...
    'pre_cfar_min_db',3.0);

% Offline evaluation match gate.
p.eval = struct('match_range_m',4.0,'match_velocity_mps',2.0,'match_angle_deg',8.0);

% -------------------------------------------------------------------------
% Mutual FMCW interference: injection model and two mitigation front ends.
% -------------------------------------------------------------------------
p.interference = struct();
p.interference.enabled              = logical(config.interference_enabled);
p.interference.f0_Hz                = 1e6;
p.interference.bandwidth_Hz         = 3.2e6;
p.interference.amplitude_vs_weakest = 2.5;
p.interference.angle_deg            = 0.0;   % DOA of the interfering emitter
% Median-background STFT ridge masking (front end, applied per RX chain).
p.interference.mask_threshold       = 7;
p.interference.mask_radius          = 2;
p.interference.median_min_output_fraction = 0.45;
p.interference.median_max_mask_fraction   = 0.18;
p.interference.median_safety_blend        = 0.35;
% Power-weighted Hough time-frequency ridge suppression (in-tracker).
p.interference.hough_tf_enabled     = true;
p.interference.hough_candidate_ratio= 6.0;
p.interference.hough_min_points     = 8;
p.interference.hough_theta_deg      = [15:5:75 105:5:165];
p.interference.hough_rho_bins       = 96;
p.interference.hough_weight_cap     = 40;
p.interference.hough_min_score      = 35;
p.interference.hough_min_angle_deg  = 12;
p.interference.hough_mask_width     = 0.035;
p.interference.hough_max_mask_fraction  = 0.12;
p.interference.hough_min_output_fraction= 0.45;
p.interference.hough_safety_blend   = 0.35;
% Iterative suppression control (shared by both front ends).
p.interference.iterative_enabled    = true;
p.interference.iterative_passes     = 2;
p.interference.iterative_min_power_reduction_db = 1.0;

p.clutter = struct();
p.clutter.enabled = logical(config.clutter_enabled);
p.clutter.power_fraction_of_weakest = 0.05;
p.clutter.slow_variation_std = 0.05;
p.clutter.range_decay_power  = 1.5;

p.range_processing = struct('window','hann','keystone',true, ...
    'clutter_method','dc_cancel','highpass_alpha',0.92, ...
    'power_normalization','unitary_noise_preserving');

p.random_seed = scalar_int(config.random_seed,'random_seed',0);
p.seed = p.random_seed;

% -------------------------------------------------------------------------
% Recursive override merge for every nested section.
% -------------------------------------------------------------------------
overrideSections = {'cfar','paper','detector','est','tbd','track','group', ...
    'interference','clutter','range_processing','obs','eval','noise','target','waveform'};
for oi = 1:numel(overrideSections)
    key = overrideSections{oi};
    if isfield(config,key) && isstruct(config.(key))
        p.(key) = merge_struct_local(p.(key),config.(key));
    end
end

% Post-merge coercions.
p.tbd.max_candidates_per_frame = max(1,round(p.tbd.max_candidates_per_frame));
p.tbd.max_confirmed_paths      = max(1,round(p.tbd.max_confirmed_paths));
p.cfar.power_shape             = max(1,round(p.cfar.power_shape));
p.track.q                      = p.track.process_sigma_a^2;

% Valid range support, shrunk by the configured guard margin so that CFAR
% reference windows never straddle the edge of the processed band.
% The margin must cover the reference window, otherwise a threshold near the
% edge is computed partly from cells that carry no data.
marginBins = max(0,round(p.cfar.valid_margin_bins));
mask = p.range_axis <= 0.98*p.R_max;
lastValid = find(mask,1,'last');
if isempty(lastValid), lastValid = 1; end
lastValid = max(1,lastValid-marginBins);
mask(:) = false; mask(1+marginBins:lastValid) = true;
p.valid_range_mask = mask;
p.valid_range_bins = find(mask);
p.unambiguous_range_nominal = max(p.range_axis(p.valid_range_bins));
if isempty(p.valid_range_bins)
    error('radar_configuration:ValidRange','Configured margin removes every valid range bin.');
end

validate_radar_config(p);
end

% =========================================================================
% Local helpers
% =========================================================================
function out = merge_config_struct(base,override)
%MERGE_CONFIG_STRUCT Recursive merge: OVERRIDE always has precedence.
out = base;
if isempty(override), return; end
fn = fieldnames(override);
for ii=1:numel(fn)
    f = fn{ii};
    if isfield(out,f) && isstruct(out.(f)) && isstruct(override.(f))
        out.(f) = merge_config_struct(out.(f),override.(f));
    else
        out.(f) = override.(f);
    end
end
end

function config = apply_learned_defaults(config)
%APPLY_LEARNED_DEFAULTS  Adopt persisted tuning as a default layer.
%   Precedence, lowest to highest: shipped defaults, learned values, explicit
%   caller fields. A caller that passes a section explicitly always wins, so a
%   deliberate experiment is never overridden by a previous tuning run.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
learnedPath = fullfile(root,'core','config','learned_defaults.mat');
if ~exist(learnedPath,'file'), return; end
try
    S = load(learnedPath);
catch
return;
end
tuned = struct();
if isfield(S,'tuned') && isstruct(S.tuned)
tuned = S.tuned;
elseif isfield(S,'best') && isstruct(S.best)
    % Legacy file: keep only the algorithm sections.
    keep = {'cfar','detector','track','group','tbd','paper','est'};
    for i = 1:numel(keep)
        if isfield(S.best,keep{i}) && isstruct(S.best.(keep{i}))
            tuned.(keep{i}) = S.best.(keep{i});
        end
    end
end
if isempty(fieldnames(tuned)), return; end
sections = fieldnames(tuned);
for i = 1:numel(sections)
    s = sections{i};
    if isfield(config,s) && isstruct(config.(s))
        config.(s) = merge_struct_local(tuned.(s),config.(s));
    else
        config.(s) = tuned.(s);
    end
end
end

function s = coerce_numeric_double(s)
%COERCE_NUMERIC_DOUBLE  Convert numeric fields in a nested configuration to double.
if ~isstruct(s), return; end
f = fieldnames(s);
for i = 1:numel(f)
    v = s.(f{i});
    if isstruct(v)
        s.(f{i}) = coerce_numeric_double(v);
    elseif isnumeric(v) && ~isa(v,'double')
        s.(f{i}) = double(v);
    end
end
end

function q = expand_per_rx(value,defaultScalar,n,name,mustBePositive)
if isempty(value)
    q = repmat(defaultScalar,1,n); return;
end
q = double(value);
if isscalar(q), q = repmat(q,1,n); end
if numel(q) ~= n || any(~isfinite(q))
    error('radar_configuration:PerRxVector','%s must be scalar or a finite vector with n_rx elements.',name);
end
if mustBePositive && any(q <= 0)
    error('radar_configuration:PerRxVector','%s must be strictly positive.',name);
end
q = reshape(q,1,[]);
end

function out = merge_struct_local(base,ov)
out = base;
if isempty(ov) || ~isstruct(ov), return; end
f = fieldnames(ov);
for k = 1:numel(f)
    key = f{k}; val = ov.(key);
    if isstruct(val) && isfield(out,key) && isstruct(out.(key))
        out.(key) = merge_struct_local(out.(key),val);
    else
        out.(key) = val;
    end
end
end

function v = scalar_pos(x,name)
validateattributes(x,{'numeric'},{'scalar','real','finite','positive'},mfilename,name);
v = double(x);
end
function v = scalar_num(x,name)
validateattributes(x,{'numeric'},{'scalar','real','finite'},mfilename,name);
v = double(x);
end
function v = scalar_int(x,name,minv)
validateattributes(x,{'numeric'},{'scalar','real','finite','integer','>=',minv},mfilename,name);
v = double(x);
end
