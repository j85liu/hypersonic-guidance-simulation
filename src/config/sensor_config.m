function sensors = sensor_config()
%SENSOR_CONFIG Multi-sensor navigation suite configuration
%
% This function contains all sensor parameters extracted from the main
% simulation comprehensive code (lines 67-95). Provides realistic sensor
% characteristics for GPS-denied navigation and terminal guidance.
%
% Outputs:
%   sensors - Structure containing all sensor configurations
%
% Sensor Suite:
%   - Inertial Navigation System (INS)
%   - Global Positioning System (GPS)
%   - Terrain-Aided Navigation (TERCOM)
%   - Laser Designation System
%   - Infrared/Electro-Optical (IR/EO)
%   - Communication Systems
%
% Each sensor includes accuracy, availability, and operational parameters
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    sensors = struct();
    
    %% Inertial Navigation System (INS) Configuration
    % INS is always available but drifts over time
    sensors.ins = struct();
    
    % Bias and drift characteristics (from original lines 70-74)
    sensors.ins.bias_accel = [0.08; 0.05; 0.12];         % Accelerometer bias (m/s²)
    sensors.ins.bias_gyro = [0.002; 0.003; 0.0025];      % Gyroscope bias (rad/s)
    sensors.ins.drift_rate = 3.0;                        % Position drift rate (m/hour)
    sensors.ins.random_walk_pos = 0.15;                  % Position random walk (m/s^0.5)
    sensors.ins.random_walk_att = 0.001;                 % Attitude random walk (rad/s^0.5)
    
    % Performance characteristics
    sensors.ins.update_rate = 200;                       % Update rate (Hz)
    sensors.ins.availability = 1.0;                      % Always available
    sensors.ins.initial_accuracy = [5; 5; 3];            % Initial position accuracy (m)
    sensors.ins.velocity_accuracy = [0.5; 0.5; 0.3];     % Velocity accuracy (m/s)
    sensors.ins.attitude_accuracy = [0.001; 0.001; 0.002]; % Attitude accuracy (rad)
    
    % Environmental sensitivity
    sensors.ins.temperature_sensitivity = 0.1;           % Performance degradation per 100K
    sensors.ins.vibration_sensitivity = 0.05;            % Performance degradation factor
    sensors.ins.alignment_time = 300;                    % Initial alignment time (s)
    
    %% Global Positioning System (GPS) Configuration  
    % GPS availability depends on altitude and jamming
    sensors.gps = struct();
    
    % Accuracy characteristics (from original lines 76-79)
    sensors.gps.accuracy = 1.0;                          % Standard deviation (m)
    sensors.gps.availability_altitude = 15000;           % Min altitude for availability (m)
    sensors.gps.jamming_range = 40000;                   % Jamming range from target (m)
    sensors.gps.update_rate = 1;                         % Update rate (Hz)
    
    % Signal characteristics
    sensors.gps.carrier_to_noise_threshold = 25;         % C/N0 threshold (dB-Hz)
    sensors.gps.multipath_error = 2.0;                   % Multipath error std dev (m)
    sensors.gps.ionospheric_error = 1.5;                 % Ionospheric delay error (m)
    sensors.gps.tropospheric_error = 1.0;                % Tropospheric delay error (m)
    
    % Availability factors
    sensors.gps.satellite_mask_angle = deg2rad(10);      % Minimum elevation angle
    sensors.gps.pdop_threshold = 6.0;                    % Position DOP threshold
    sensors.gps.jamming_threshold = 0.3;                 % Jamming interference threshold
    
    %% Terrain-Aided Navigation (TERCOM) Configuration
    % TERCOM uses terrain correlation for position fixes
    sensors.tercom = struct();
    
    % Operational parameters (from original lines 81-84)
    sensors.tercom.accuracy = 25.0;                      % Position accuracy (m)
    sensors.tercom.max_altitude = 8000;                  % Maximum operational altitude (m)
    sensors.tercom.update_rate = 0.2;                    % Update rate (Hz) - every 5 seconds
    sensors.tercom.min_speed = 100;                      % Minimum speed for operation (m/s)
    
    % Terrain requirements
    sensors.tercom.min_terrain_variation = 50;           % Minimum terrain relief (m)
    sensors.tercom.correlation_threshold = 0.7;          % Correlation coefficient threshold
    sensors.tercom.reference_map_resolution = 10;        % Reference map resolution (m)
    sensors.tercom.search_window_size = 500;             % Search window size (m)
    
    % Environmental limitations
    sensors.tercom.weather_degradation = 0.1;            % Performance loss in bad weather
    sensors.tercom.water_body_penalty = 0.8;             % Accuracy loss over water
    
    %% Laser Designation System Configuration
    % Laser provides high-accuracy terminal guidance when target is painted
    sensors.laser = struct();
    
    % Performance characteristics (from original lines 86-89)
    sensors.laser.accuracy = 0.5;                        % Position accuracy (m)
    sensors.laser.max_range = 60000;                     % Maximum range (m)
    sensors.laser.weather_factor = 1.0;                  % Weather degradation factor
    sensors.laser.update_rate = 10;                      % Update rate (Hz)
    
    % Operational requirements
    sensors.laser.beam_width = deg2rad(0.1);             % Laser beam width (rad)
    sensors.laser.target_reflectance_threshold = 0.1;    % Minimum target reflectance
    sensors.laser.atmospheric_transmission = 0.8;        % Atmospheric transmission factor
    sensors.laser.pointing_accuracy = deg2rad(0.01);     % Pointing accuracy (rad)
    
    % Environmental effects
    sensors.laser.fog_degradation = 5.0;                 % Performance loss in fog
    sensors.laser.rain_degradation = 3.0;                % Performance loss in rain
    sensors.laser.snow_degradation = 4.0;                % Performance loss in snow
    
    %% Infrared/Electro-Optical (IR/EO) Configuration
    % IR/EO provides passive target detection and tracking
    sensors.ir = struct();
    
    % Detection characteristics (from original lines 91-95)
    sensors.ir.accuracy = 10.0;                          % Position accuracy (m)
    sensors.ir.max_range = 80000;                        % Maximum detection range (m)
    sensors.ir.angular_resolution = 0.0001;              % Angular resolution (rad)
    sensors.ir.update_rate = 30;                         % Update rate (Hz)
    
    % Sensor characteristics
    sensors.ir.field_of_view = deg2rad(20);              % Field of view (rad)
    sensors.ir.sensitivity_threshold = 1e-12;            % Minimum detectable power (W)
    sensors.ir.spectral_band = [3, 5];                   % Spectral band (μm)
    sensors.ir.detector_noise = 1e-15;                   % Detector noise level (W)
    
    % Environmental sensitivity
    sensors.ir.atmospheric_attenuation = 0.9;            % Atmospheric transmission
    sensors.ir.temperature_sensitivity = 0.02;           % Performance vs temperature
    sensors.ir.cloud_degradation = 10.0;                 % Performance loss in clouds
    sensors.ir.sun_interference_angle = deg2rad(30);     % Sun interference cone
    
    %% Communication System Configuration
    % Communications for external updates and coordination
    sensors.comm = struct();
    
    % Communication parameters
    sensors.comm.data_rate = 1000000;                    % Data rate (bps)
    sensors.comm.max_range = 200000;                     % Maximum range (m)
    sensors.comm.update_rate = 0.1;                      % Update rate (Hz)
    sensors.comm.packet_loss_rate = 0.01;               % Expected packet loss
    
    % Jamming resistance
    sensors.comm.frequency_hopping = true;               % Frequency hopping enabled
    sensors.comm.encryption_enabled = true;              % Encryption enabled
    sensors.comm.jamming_resistance = 0.8;               % Resistance to jamming
    sensors.comm.backup_frequency_bands = 3;             % Number of backup bands
    
    %% Multi-Sensor Fusion Configuration
    sensors.fusion = struct();
    
    % Kalman filter parameters
    sensors.fusion.process_noise_scale = 1.0;            % Process noise scaling
    sensors.fusion.measurement_noise_scale = 1.0;        % Measurement noise scaling
    sensors.fusion.innovation_threshold = 3.0;           % Innovation rejection threshold
    sensors.fusion.divergence_threshold = 100.0;         % Filter divergence threshold
    
    % Sensor priority weights (higher = more trusted)
    sensors.fusion.sensor_weights = struct();
    sensors.fusion.sensor_weights.gps = 1.0;             % GPS weight when available
    sensors.fusion.sensor_weights.ins = 0.5;             % INS weight (biased)
    sensors.fusion.sensor_weights.tercom = 0.8;          % TERCOM weight
    sensors.fusion.sensor_weights.laser = 1.2;           % Laser weight (highest accuracy)
    sensors.fusion.sensor_weights.ir = 0.6;              % IR weight
    
    % Adaptive fusion parameters
    sensors.fusion.adaptive_scaling = true;              % Enable adaptive scaling
    sensors.fusion.learning_rate = 0.01;                 % Adaptation learning rate
    sensors.fusion.memory_length = 100;                  % Number of samples for adaptation
    
    %% Sensor Failure Modeling
    sensors.reliability = struct();
    
    % Mean time between failures (hours)
    sensors.reliability.mtbf_ins = 10000;                % INS MTBF
    sensors.reliability.mtbf_gps = 5000;                 % GPS MTBF  
    sensors.reliability.mtbf_tercom = 8000;              % TERCOM MTBF
    sensors.reliability.mtbf_laser = 3000;               % Laser MTBF
    sensors.reliability.mtbf_ir = 6000;                  % IR MTBF
    sensors.reliability.mtbf_comm = 4000;                % Comm MTBF
    
    % Failure recovery times (seconds)
    sensors.reliability.recovery_time_ins = 60;          % INS recovery time
    sensors.reliability.recovery_time_gps = 30;          % GPS recovery time
    sensors.reliability.recovery_time_tercom = 120;      % TERCOM recovery time
    sensors.reliability.recovery_time_laser = 45;        % Laser recovery time
    sensors.reliability.recovery_time_ir = 90;           % IR recovery time
    sensors.reliability.recovery_time_comm = 15;         % Comm recovery time
    
    %% Configuration Validation and Summary
    sensors = validate_sensor_config(sensors);
    
    fprintf('✅ Sensor configuration loaded:\n');
    fprintf('   - INS drift rate: %.1f m/hour\n', sensors.ins.drift_rate);
    fprintf('   - GPS accuracy: %.1f m\n', sensors.gps.accuracy);
    fprintf('   - TERCOM max altitude: %.0f m\n', sensors.tercom.max_altitude);
    fprintf('   - Laser max range: %.0f km\n', sensors.laser.max_range/1000);
    fprintf('   - IR max range: %.0f km\n', sensors.ir.max_range/1000);
end

%% Local Configuration Functions

function sensors = validate_sensor_config(sensors)
    %VALIDATE_SENSOR_CONFIG Validate sensor configuration parameters
    %
    % Ensures all sensor parameters are within reasonable bounds and
    % maintains consistency between related parameters
    
    % Validate INS parameters
    assert(sensors.ins.drift_rate > 0 && sensors.ins.drift_rate < 100, ...
           'INS drift rate must be between 0 and 100 m/hour');
    assert(all(sensors.ins.bias_accel >= 0) && all(sensors.ins.bias_accel < 1), ...
           'INS accelerometer bias must be between 0 and 1 m/s²');
    
    % Validate GPS parameters
    assert(sensors.gps.accuracy > 0 && sensors.gps.accuracy < 100, ...
           'GPS accuracy must be between 0 and 100 m');
    assert(sensors.gps.availability_altitude > 0, ...
           'GPS availability altitude must be positive');
    
    % Validate TERCOM parameters
    assert(sensors.tercom.accuracy > 0 && sensors.tercom.accuracy < 1000, ...
           'TERCOM accuracy must be between 0 and 1000 m');
    assert(sensors.tercom.max_altitude > 0, ...
           'TERCOM max altitude must be positive');
    
    % Validate laser parameters
    assert(sensors.laser.accuracy > 0 && sensors.laser.accuracy < 10, ...
           'Laser accuracy must be between 0 and 10 m');
    assert(sensors.laser.max_range > 0, ...
           'Laser max range must be positive');
    
    % Validate IR parameters
    assert(sensors.ir.accuracy > 0 && sensors.ir.accuracy < 1000, ...
           'IR accuracy must be between 0 and 1000 m');
    assert(sensors.ir.max_range > 0, ...
           'IR max range must be positive');
    
    % Validate fusion parameters
    assert(sensors.fusion.innovation_threshold > 0, ...
           'Innovation threshold must be positive');
    assert(sensors.fusion.divergence_threshold > sensors.fusion.innovation_threshold, ...
           'Divergence threshold must exceed innovation threshold');
    
    % Ensure sensor weights are positive
    weight_fields = fieldnames(sensors.fusion.sensor_weights);
    for i = 1:length(weight_fields)
        weight = sensors.fusion.sensor_weights.(weight_fields{i});
        assert(weight > 0 && weight <= 2.0, ...
               'Sensor weights must be between 0 and 2.0');
    end
    
    % Validate reliability parameters
    reliability_fields = fieldnames(sensors.reliability);
    mtbf_fields = reliability_fields(contains(reliability_fields, 'mtbf'));
    for i = 1:length(mtbf_fields)
        mtbf = sensors.reliability.(mtbf_fields{i});
        assert(mtbf > 0 && mtbf < 100000, ...
               'MTBF values must be between 0 and 100000 hours');
    end
end