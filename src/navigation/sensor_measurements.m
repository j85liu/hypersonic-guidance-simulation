function measurements = sensor_measurements(sensor_type, true_state, target_position, sensor_config, environment_effects, current_time)
%SENSOR_MEASUREMENTS Individual sensor measurement generation
%
% This function generates realistic measurements from individual navigation
% sensors extracted from the main simulation comprehensive code. Provides
% modular sensor modeling for GPS, TERCOM, laser, IR, and communication
% systems with realistic noise and environmental effects.
%
% Inputs:
%   sensor_type - Sensor type: 'gps', 'tercom', 'laser', 'ir', 'comm'
%   true_state - True vehicle state [pos(3), vel(3), att(3)]
%   target_position - Target position [3x1] (m)
%   sensor_config - Sensor-specific configuration structure
%   environment_effects - Environmental effects structure
%   current_time - Current simulation time (s)
%
% Outputs:
%   measurements - Sensor measurement structure with data and metadata
%
% Supported Sensors:
%   - GPS: Position measurements with jamming and atmospheric effects
%   - TERCOM: Terrain correlation altitude measurements
%   - Laser: High-accuracy target-relative measurements
%   - IR: Infrared target detection and tracking
%   - Comm: Communication system position/velocity updates
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Extract state components
    position = true_state(1:3);
    velocity = true_state(4:6);
    if length(true_state) >= 9
        attitude = true_state(7:9);
    else
        attitude = [0; 0; 0];  % Default attitude if not provided
    end
    
    % Initialize measurement structure
    measurements = struct();
    measurements.sensor_type = sensor_type;
    measurements.timestamp = current_time;
    measurements.valid = false;
    
    try
        switch lower(sensor_type)
            case 'gps'
                measurements = generate_gps_measurements(position, sensor_config, environment_effects, measurements);
                
            case 'tercom'
                measurements = generate_tercom_measurements(position, sensor_config, environment_effects, measurements);
                
            case 'laser'
                measurements = generate_laser_measurements(position, target_position, sensor_config, environment_effects, measurements);
                
            case 'ir'
                measurements = generate_ir_measurements(position, target_position, sensor_config, environment_effects, measurements);
                
            case 'comm'
                measurements = generate_comm_measurements(position, velocity, sensor_config, environment_effects, measurements);
                
            otherwise
                error('Unknown sensor type: %s', sensor_type);
        end
        
        % Add common metadata
        measurements = add_measurement_metadata(measurements, environment_effects, current_time);
        
    catch ME
        fprintf('Warning: %s sensor measurement error: %s\n', sensor_type, ME.message);
        measurements.error_message = ME.message;
        measurements.valid = false;
    end
end

%% Individual Sensor Measurement Functions

function measurements = generate_gps_measurements(position, gps_config, environment_effects, measurements)
    %GENERATE_GPS_MEASUREMENTS GPS position measurements with realistic errors
    %
    % Implements GPS measurement generation from the original simulation
    % with plasma interference, atmospheric effects, and multipath errors
    
    % Base GPS measurement noise
    plasma_factor = 1 + environment_effects.plasma_interference;
    base_accuracy = gps_config.accuracy * plasma_factor;
    
    % Generate primary GPS noise
    gps_noise = base_accuracy * randn(3,1);
    
    % Add realistic GPS error sources
    multipath_error = zeros(3,1);
    if isfield(gps_config, 'multipath_error')
        multipath_error = gps_config.multipath_error * randn(3,1);
    end
    
    atmospheric_error = zeros(3,1);
    if isfield(gps_config, 'ionospheric_error') && isfield(gps_config, 'tropospheric_error')
        iono_error = gps_config.ionospheric_error * randn(3,1);
        tropo_error = gps_config.tropospheric_error * randn(3,1);
        atmospheric_error = iono_error + tropo_error;
    end
    
    % Satellite geometry effects (simplified)
    geometry_factor = 1;
    if isfield(gps_config, 'pdop_threshold')
        % Simulate varying satellite geometry
        pdop = 1 + 0.5 * abs(randn);  % Simplified PDOP variation
        if pdop > gps_config.pdop_threshold
            geometry_factor = pdop / gps_config.pdop_threshold;
        end
    end
    
    % Total GPS measurement
    total_error = (gps_noise + multipath_error + atmospheric_error) * geometry_factor;
    measurements.position = position + total_error;
    
    % Measurement uncertainty
    measurements.position_std = base_accuracy * geometry_factor * ones(3,1);
    
    % GPS-specific metadata
    measurements.satellite_count = max(4, round(8 - 2*environment_effects.plasma_interference));
    measurements.pdop = 1 + environment_effects.plasma_interference;
    measurements.hdop = measurements.pdop * 0.7;
    measurements.vdop = measurements.pdop * 1.2;
    
    % Signal quality metrics
    measurements.carrier_to_noise = gps_config.carrier_to_noise_threshold * ...
                                   (1 - 0.5 * environment_effects.plasma_interference);
    measurements.signal_strength = 1 / plasma_factor;
    
    measurements.valid = true;
end

function measurements = generate_tercom_measurements(position, tercom_config, environment_effects, measurements)
    %GENERATE_TERCOM_MEASUREMENTS Terrain-aided navigation measurements
    %
    % Implements TERCOM measurement generation matching the original
    % simulation terrain model with correlation-based position fixes
    
    % Terrain reference model (matching original simulation)
    terrain_height = 100 + 50*sin(position(1)/15000) + 30*cos(position(2)/12000);
    
    % Weather effects on terrain correlation
    weather_factor = 1;
    if isfield(environment_effects, 'visibility_factor')
        weather_degradation = tercom_config.weather_degradation * (1 - environment_effects.visibility_factor);
        weather_factor = 1 + weather_degradation;
    end
    
    % Terrain variation assessment (affects correlation quality)
    terrain_variation = calculate_terrain_variation(position);
    correlation_quality = min(1.0, terrain_variation / tercom_config.min_terrain_variation);
    
    % TERCOM measurement: altitude above terrain
    measurement_noise = tercom_config.accuracy * weather_factor * randn;
    measurements.altitude_above_terrain = position(3) - terrain_height + measurement_noise;
    
    % Reference terrain information
    measurements.terrain_height = terrain_height;
    measurements.terrain_variation = terrain_variation;
    measurements.correlation_quality = correlation_quality * (1 / weather_factor);
    
    % Measurement uncertainty
    measurements.measurement_std = tercom_config.accuracy * weather_factor;
    
    % TERCOM-specific metadata
    measurements.map_resolution = tercom_config.reference_map_resolution;
    measurements.search_window = tercom_config.search_window_size;
    measurements.correlation_threshold = tercom_config.correlation_threshold;
    
    % Position fix quality
    if correlation_quality > tercom_config.correlation_threshold
        measurements.position_fix_quality = 'good';
        measurements.valid = true;
    elseif correlation_quality > 0.5
        measurements.position_fix_quality = 'fair';
        measurements.valid = true;
        measurements.measurement_std = measurements.measurement_std * 2;  % Increased uncertainty
    else
        measurements.position_fix_quality = 'poor';
        measurements.valid = false;
    end
end

function measurements = generate_laser_measurements(position, target_position, laser_config, environment_effects, measurements)
    %GENERATE_LASER_MEASUREMENTS Laser designator measurements
    %
    % Implements high-accuracy laser measurements for terminal guidance
    % with atmospheric transmission and weather effects
    
    % Range to target
    relative_position = target_position - position;
    range_to_target = norm(relative_position);
    
    % Atmospheric transmission effects
    atmospheric_transmission = laser_config.atmospheric_transmission;
    if isfield(environment_effects, 'visibility_factor')
        atmospheric_transmission = atmospheric_transmission * environment_effects.visibility_factor;
    end
    
    % Weather-specific degradation
    weather_factor = laser_config.weather_factor;
    if isfield(environment_effects, 'weather') && isfield(environment_effects.weather, 'type')
        switch environment_effects.weather.type
            case 'fog'
                weather_factor = weather_factor * laser_config.fog_degradation;
            case 'rain'
                weather_factor = weather_factor * laser_config.rain_degradation;
            case 'snow'
                weather_factor = weather_factor * laser_config.snow_degradation;
        end
    end
    
    % Total degradation factor
    total_degradation = weather_factor / atmospheric_transmission;
    
    % Laser measurement accuracy
    laser_accuracy = laser_config.accuracy * total_degradation;
    laser_noise = laser_accuracy * randn(3,1);
    
    % Laser measurements
    measurements.target_position = target_position + laser_noise;
    measurements.range_to_target = range_to_target + laser_accuracy * randn;
    
    % Line-of-sight vector (unit vector from vehicle to target)
    if range_to_target > 0
        measurements.line_of_sight = relative_position / range_to_target;
    else
        measurements.line_of_sight = [1; 0; 0];  % Default LOS
    end
    
    % Measurement uncertainties
    measurements.position_std = laser_accuracy * ones(3,1);
    measurements.range_std = laser_accuracy;
    
    % Laser-specific metadata
    measurements.beam_divergence = laser_config.beam_width;
    measurements.pointing_accuracy = laser_config.pointing_accuracy;
    measurements.atmospheric_transmission = atmospheric_transmission;
    measurements.weather_factor = weather_factor;
    measurements.signal_return_strength = atmospheric_transmission / total_degradation;
    
    % Target characteristics
    measurements.target_reflectance = 0.3;  % Assumed target reflectance
    measurements.beam_quality = atmospheric_transmission;
    
    measurements.valid = atmospheric_transmission > 0.1;  % Minimum transmission threshold
end

function measurements = generate_ir_measurements(position, target_position, ir_config, environment_effects, measurements)
    %GENERATE_IR_MEASUREMENTS Infrared sensor measurements
    %
    % Implements passive IR detection and tracking with atmospheric
    % attenuation and environmental effects
    
    % Range and geometry calculations
    relative_position = target_position - position;
    range_to_target = norm(relative_position);
    
    % Angular measurements (IR sensor primary output)
    if range_to_target > 0
        line_of_sight = relative_position / range_to_target;
        azimuth = atan2(line_of_sight(2), line_of_sight(1));
        elevation = asin(line_of_sight(3));
    else
        azimuth = 0;
        elevation = 0;
        line_of_sight = [1; 0; 0];
    end
    
    % Angular noise based on sensor resolution
    angular_noise_std = ir_config.angular_resolution;
    azimuth_noise = angular_noise_std * randn;
    elevation_noise = angular_noise_std * randn;
    
    % Environmental effects on IR performance
    atmospheric_factor = ir_config.atmospheric_attenuation;
    if isfield(environment_effects, 'visibility_factor')
        atmospheric_factor = atmospheric_factor * environment_effects.visibility_factor;
    end
    
    % Temperature effects on IR sensor
    temperature_factor = 1;
    if isfield(environment_effects, 'temperature')
        temp_deviation = environment_effects.temperature - 288;  % Deviation from standard
        temperature_factor = 1 + ir_config.temperature_sensitivity * temp_deviation / 100;
    end
    
    % Cloud interference
    cloud_factor = 1;
    if isfield(environment_effects, 'weather') && strcmp(environment_effects.weather.type, 'cloudy')
        cloud_factor = ir_config.cloud_degradation;
    end
    
    % Total degradation
    total_degradation = temperature_factor * cloud_factor / atmospheric_factor;
    
    % Convert angular measurements back to position
    position_uncertainty = range_to_target * angular_noise_std * total_degradation;
    position_noise = position_uncertainty * randn(3,1);
    
    % IR measurements
    measurements.target_position = target_position + position_noise;
    measurements.azimuth = azimuth + azimuth_noise;
    measurements.elevation = elevation + elevation_noise;
    measurements.range_estimate = range_to_target + 0.1 * range_to_target * randn;
    
    % Measurement uncertainties
    measurements.position_std = position_uncertainty * ones(3,1);
    measurements.angular_std = angular_noise_std * total_degradation * ones(2,1);
    measurements.range_std = 0.1 * range_to_target;
    
    % IR-specific metadata
    measurements.signal_strength = 1 / total_degradation;
    measurements.atmospheric_transmission = atmospheric_factor;
    measurements.detector_temperature = 77;  % Typical IR detector temperature (K)
    measurements.field_of_view = ir_config.field_of_view;
    measurements.spectral_band = ir_config.spectral_band;
    
    % Target signature estimation
    measurements.target_temperature_estimate = 300 + 50*randn;  % Estimated target temperature
    measurements.target_signature_strength = 0.8 / total_degradation;
    
    % Detection quality assessment
    if total_degradation < 2 && range_to_target < ir_config.max_range
        measurements.detection_quality = 'good';
        measurements.valid = true;
    elseif total_degradation < 5
        measurements.detection_quality = 'fair';
        measurements.valid = true;
        measurements.position_std = measurements.position_std * 2;
    else
        measurements.detection_quality = 'poor';
        measurements.valid = false;
    end
end

function measurements = generate_comm_measurements(position, velocity, comm_config, environment_effects, measurements)
    %GENERATE_COMM_MEASUREMENTS Communication system measurements
    %
    % Implements external communication updates with realistic data link
    % characteristics and jamming effects
    
    % Communication link quality
    link_quality = 1 - comm_config.packet_loss_rate;
    
    % Jamming effects
    jamming_factor = 1;
    if isfield(environment_effects, 'jamming_interference')
        jamming_factor = 1 + environment_effects.jamming_interference;
        link_quality = link_quality / jamming_factor;
    end
    
    % Range-dependent signal degradation (simplified)
    range_factor = 1;  % Assume within communication range
    
    % Data packet characteristics
    if rand < link_quality
        % Successful data reception
        
        % Position update with communication system accuracy
        comm_position_noise = 5 * randn(3,1);  % 5m typical accuracy
        comm_velocity_noise = 1 * randn(3,1);  % 1 m/s typical accuracy
        
        measurements.position = position + comm_position_noise * jamming_factor;
        measurements.velocity = velocity + comm_velocity_noise * jamming_factor;
        
        % Measurement uncertainties
        measurements.position_std = 5 * jamming_factor * ones(3,1);
        measurements.velocity_std = 1 * jamming_factor * ones(3,1);
        
        measurements.valid = true;
    else
        % Communication failure
        measurements.position = position + 100 * randn(3,1);  % Large error
        measurements.velocity = velocity + 20 * randn(3,1);   % Large error
        measurements.position_std = 100 * ones(3,1);
        measurements.velocity_std = 20 * ones(3,1);
        measurements.valid = false;
    end
    
    % Communication-specific metadata
    measurements.link_quality = link_quality;
    measurements.signal_to_noise_ratio = 20 / jamming_factor;  % dB
    measurements.bit_error_rate = comm_config.packet_loss_rate * jamming_factor;
    measurements.data_rate = comm_config.data_rate / jamming_factor;
    measurements.encryption_status = comm_config.encryption_enabled;
    measurements.frequency_band = 'UHF';  % Typical military comm band
    
    % Jamming assessment
    if jamming_factor > 1.5
        measurements.jamming_detected = true;
        measurements.jamming_strength = (jamming_factor - 1) * 10;  % dB
    else
        measurements.jamming_detected = false;
        measurements.jamming_strength = 0;
    end
end

function measurements = add_measurement_metadata(measurements, environment_effects, current_time)
    %ADD_MEASUREMENT_METADATA Add common metadata to all measurements
    
    % Timing information
    measurements.measurement_time = current_time;
    measurements.processing_delay = 0.01 + 0.02*rand;  % 10-30ms processing delay
    
    % Environmental context
    measurements.environment = struct();
    measurements.environment.thermal_factor = environment_effects.thermal_factor;
    if isfield(environment_effects, 'plasma_interference')
        measurements.environment.plasma_interference = environment_effects.plasma_interference;
    end
    if isfield(environment_effects, 'visibility_factor')
        measurements.environment.visibility = environment_effects.visibility_factor;
    end
    
    % Data quality indicators
    if measurements.valid
        measurements.quality_score = min(1.0, 1 / environment_effects.thermal_factor);
    else
        measurements.quality_score = 0;
    end
    
    % Confidence level
    if measurements.quality_score > 0.8
        measurements.confidence = 'high';
    elseif measurements.quality_score > 0.5
        measurements.confidence = 'medium';
    else
        measurements.confidence = 'low';
    end
end

function terrain_var = calculate_terrain_variation(position)
    %CALCULATE_TERRAIN_VARIATION Calculate local terrain variation for TERCOM
    %
    % Estimates terrain relief in the local area for correlation assessment
    
    % Sample terrain at multiple points around current position
    sample_radius = 1000;  % 1km sampling radius
    sample_points = 8;     % Number of sample points
    
    heights = zeros(sample_points, 1);
    for i = 1:sample_points
        angle = 2*pi*i/sample_points;
        sample_pos = position + sample_radius * [cos(angle); sin(angle); 0];
        heights(i) = 100 + 50*sin(sample_pos(1)/15000) + 30*cos(sample_pos(2)/12000);
    end
    
    % Calculate terrain variation as standard deviation
    terrain_var = std(heights);
end