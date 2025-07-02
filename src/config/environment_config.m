function [environment, guidance_config, target_config] = environment_config()
%ENVIRONMENT_CONFIG Environmental and operational configuration
%
% This function contains environmental parameters, guidance configuration,
% and target behavior extracted from the main simulation comprehensive code
% (lines 112-140). Provides atmospheric conditions, guidance law parameters,
% and target characteristics for realistic simulation scenarios.
%
% Outputs:
%   environment - Environmental conditions structure
%   guidance_config - Guidance law configuration
%   target_config - Target behavior and characteristics
%
% Configuration Categories:
%   - Atmospheric and weather conditions
%   - Wind and turbulence modeling
%   - Guidance law parameters
%   - Target movement and evasion
%   - Scenario-specific settings
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    %% Environmental Configuration (from original lines 112-118)
    environment = struct();
    
    % Atmospheric conditions
    environment.atmosphere_model = 'us_standard';       % Atmosphere model type
    environment.atmosphere_variations = true;           % Enable atmospheric variations
    environment.seasonal_effects = false;               % Seasonal atmospheric changes
    environment.diurnal_effects = false;                % Day/night atmospheric changes
    
    % Wind and turbulence (from original lines 115-117)
    environment.wind_speed = [10; 5; 2];                % Constant wind velocity (m/s)
    environment.turbulence_intensity = 0.1;             % Turbulence intensity (0-1 scale)
    environment.wind_shear_gradient = [0.01; 0.005; 0]; % Wind shear (s⁻¹)
    environment.gust_factor = 1.5;                       % Gust intensity multiplier
    
    % Advanced wind modeling
    environment.wind_layers = struct();
    environment.wind_layers.altitudes = [0, 5000, 15000, 30000, 50000]; % Wind layer altitudes (m)
    environment.wind_layers.speeds = [
        [8, 3, 1];     % Surface winds (m/s)
        [15, 8, 2];    % Low altitude winds
        [25, 12, 3];   % Mid altitude winds
        [35, 20, 5];   % High altitude winds
        [45, 30, 8];   % Very high altitude winds
    ];
    
    % Weather conditions (from original line 118)
    environment.weather_visibility = 8000;               % Visibility range (m)
    environment.cloud_ceiling = 3000;                   % Cloud ceiling height (m)
    environment.precipitation_rate = 0;                 % Precipitation rate (mm/h)
    environment.humidity = 0.6;                         % Relative humidity (0-1)
    environment.temperature_offset = 0;                 % Temperature offset from standard (K)
    
    % Advanced weather modeling
    environment.weather = struct();
    environment.weather.type = 'clear';                 % Weather type: clear/cloudy/storm/fog
    environment.weather.intensity = 0.1;                % Weather intensity (0-1)
    environment.weather.moving_weather = false;         % Moving weather systems
    environment.weather.weather_gradient = 0.05;        % Spatial weather variation
    
    % Environmental hazards
    environment.hazards = struct();
    environment.hazards.bird_strike_probability = 1e-6; % Bird strike probability per second
    environment.hazards.volcanic_ash = false;           % Volcanic ash presence
    environment.hazards.solar_activity = 'nominal';     % Solar activity level
    environment.hazards.ionospheric_disturbance = 0.1;  % Ionospheric disturbance level
    
    %% Guidance System Configuration (from original lines 140-145)
    guidance_config = struct();
    
    % Guidance law selection and parameters
    guidance_config.type = 'APN';                        % Augmented Proportional Navigation
    guidance_config.Nav = 4.0;                          % Navigation constant
    guidance_config.terminal_range = 15000;             % Terminal guidance threshold (m)
    guidance_config.proportional_gain = 3.0;            % Terminal proportional gain
    guidance_config.derivative_gain = 0.5;              % Derivative gain for stability
    
    % Phase-based guidance parameters
    guidance_config.phases = struct();
    guidance_config.phases.midcourse_Nav = 4.0;         % Mid-course navigation constant
    guidance_config.phases.terminal_Nav = 5.0;          % Terminal navigation constant
    guidance_config.phases.endgame_range = 2000;        % End-game phase threshold (m)
    guidance_config.phases.endgame_gain = 6.0;          % End-game guidance gain
    
    % Advanced guidance features
    guidance_config.adaptive = struct();
    guidance_config.adaptive.enabled = true;            % Enable adaptive guidance
    guidance_config.adaptive.learning_rate = 0.01;      % Adaptation learning rate
    guidance_config.adaptive.performance_threshold = 0.8; % Performance threshold for adaptation
    guidance_config.adaptive.memory_length = 50;        % Adaptation memory length
    
    % Constraint management
    guidance_config.constraints = struct();
    guidance_config.constraints.max_lateral_accel = 150;     % Maximum lateral acceleration (m/s²)
    guidance_config.constraints.max_normal_accel = 200;      % Maximum normal acceleration (m/s²)
    guidance_config.constraints.acceleration_rate_limit = 500; % Acceleration rate limit (m/s³)
    guidance_config.constraints.minimum_speed = 100;         % Minimum speed for guidance (m/s)
    
    % Evasive maneuvering
    guidance_config.evasion = struct();
    guidance_config.evasion.max_evasive_accel = 80;          % Maximum evasive acceleration (m/s²)
    guidance_config.evasion.evasion_duration = 5;           % Evasive maneuver duration (s)
    guidance_config.evasion.recovery_time = 3;              % Recovery time after evasion (s)
    guidance_config.evasion.threat_response_gain = 2.0;     % Threat response gain multiplier
    
    %% Target Configuration (from original lines 125-135)
    target_config = struct();
    
    % Initial target state
    target_config.initial_position = [0; 0; 0];         % Initial target position (m)
    target_config.initial_velocity = [25; 15; 0];       % Initial target velocity (m/s)
    target_config.target_type = 'moving_evasive';       % Target behavior type
    target_config.target_size = 'medium';               % Target size classification
    
    % Target movement characteristics
    target_config.max_speed = 50;                       % Maximum target speed (m/s)
    target_config.max_accel = 50;                       % Maximum target acceleration (m/s²)
    target_config.evasive_probability = 0.3;            % Evasive maneuver probability per second
    target_config.evasive_duration = 3;                 % Average evasive maneuver duration (s)
    target_config.course_change_probability = 0.1;      % Course change probability per second
    
    % Advanced target behavior
    target_config.behavior = struct();
    target_config.behavior.intelligence_level = 'high'; % Target intelligence: low/medium/high
    target_config.behavior.predictability = 0.3;        % Movement predictability (0-1)
    target_config.behavior.threat_awareness = true;     % Target aware of incoming threat
    target_config.behavior.defensive_systems = true;    % Target has defensive systems
    target_config.behavior.terrain_following = false;   % Target follows terrain
    
    % Target signatures
    target_config.signatures = struct();
    target_config.signatures.radar_cross_section = 5.0; % Radar cross section (m²)
    target_config.signatures.infrared_signature = 0.8;  % IR signature intensity
    target_config.signatures.acoustic_signature = 0.6;  % Acoustic signature level
    target_config.signatures.visual_signature = 0.9;    % Visual signature level
    
    % Target defensive capabilities
    target_config.defenses = struct();
    target_config.defenses.chaff_dispensers = 4;        % Number of chaff dispensers
    target_config.defenses.flare_dispensers = 6;        % Number of flare dispensers
    target_config.defenses.ecm_power = 50;              % ECM transmitter power (W)
    target_config.defenses.warning_systems = true;      % Missile warning systems
    target_config.defenses.automatic_countermeasures = true; % Automatic countermeasure deployment
    
    %% Scenario Configuration
    scenario_config = struct();
    scenario_config.mission_type = 'precision_strike';   % Mission type
    scenario_config.time_of_day = 'day';                % Time of day: day/night/dawn/dusk
    scenario_config.season = 'spring';                  % Season: spring/summer/fall/winter
    scenario_config.geography = 'temperate';            % Geographic region
    scenario_config.threat_level = 'high';              % Threat environment level
    
    % Mission constraints
    scenario_config.constraints = struct();
    scenario_config.constraints.max_flight_time = 300;   % Maximum flight time (s)
    scenario_config.constraints.fuel_limit = 150;        % Fuel limit (kg)
    scenario_config.constraints.no_fly_zones = [];       % No-fly zone coordinates
    scenario_config.constraints.minimum_altitude = 50;   % Minimum flight altitude (m)
    scenario_config.constraints.maximum_altitude = 50000; % Maximum flight altitude (m)
    
    % Add scenario config to environment for completeness
    environment.scenario = scenario_config;
    
    %% Configuration Validation and Summary
    [environment, guidance_config, target_config] = validate_environment_config(...
        environment, guidance_config, target_config);
    
    fprintf('✅ Environment configuration loaded:\n');
    fprintf('   - Wind speed: [%.1f, %.1f, %.1f] m/s\n', environment.wind_speed);
    fprintf('   - Turbulence intensity: %.1f\n', environment.turbulence_intensity);
    fprintf('   - Weather visibility: %.0f m\n', environment.weather_visibility);
    fprintf('   - Guidance type: %s (Nav=%.1f)\n', guidance_config.type, guidance_config.Nav);
    fprintf('   - Target max speed: %.0f m/s\n', target_config.max_speed);
    fprintf('   - Target evasion probability: %.1f%%\n', target_config.evasive_probability * 100);
end

%% Local Configuration Functions

function [env, guid, targ] = validate_environment_config(environment, guidance_config, target_config)
    %VALIDATE_ENVIRONMENT_CONFIG Validate environment configuration parameters
    %
    % Ensures all environment, guidance, and target parameters are within
    % reasonable bounds and maintains consistency between related parameters
    
    env = environment;
    guid = guidance_config;
    targ = target_config;
    
    % Validate environmental parameters
    assert(all(env.wind_speed >= 0) && all(env.wind_speed < 100), ...
           'Wind speeds must be between 0 and 100 m/s');
    assert(env.turbulence_intensity >= 0 && env.turbulence_intensity <= 1, ...
           'Turbulence intensity must be between 0 and 1');
    assert(env.weather_visibility > 0 && env.weather_visibility <= 50000, ...
           'Weather visibility must be between 0 and 50000 m');
    
    % Validate wind layer consistency
    if isfield(env, 'wind_layers')
        assert(length(env.wind_layers.altitudes) == size(env.wind_layers.speeds, 1), ...
               'Wind layer altitudes and speeds must have same number of entries');
        assert(all(diff(env.wind_layers.altitudes) > 0), ...
               'Wind layer altitudes must be in ascending order');
    end
    
    % Validate guidance parameters
    assert(guid.Nav > 0 && guid.Nav < 10, ...
           'Navigation constant must be between 0 and 10');
    assert(guid.terminal_range > 0 && guid.terminal_range < 100000, ...
           'Terminal range must be between 0 and 100000 m');
    assert(guid.proportional_gain > 0 && guid.proportional_gain < 20, ...
           'Proportional gain must be between 0 and 20');
    
    % Validate guidance constraints
    if isfield(guid, 'constraints')
        assert(guid.constraints.max_lateral_accel > 0 && guid.constraints.max_lateral_accel < 1000, ...
               'Max lateral acceleration must be between 0 and 1000 m/s²');
        assert(guid.constraints.max_normal_accel >= guid.constraints.max_lateral_accel, ...
               'Max normal acceleration must be >= max lateral acceleration');
    end
    
    % Validate target parameters
    assert(targ.max_speed > 0 && targ.max_speed < 200, ...
           'Target max speed must be between 0 and 200 m/s');
    assert(targ.max_accel > 0 && targ.max_accel < 500, ...
           'Target max acceleration must be between 0 and 500 m/s²');
    assert(targ.evasive_probability >= 0 && targ.evasive_probability <= 1, ...
           'Evasive probability must be between 0 and 1');
    
    % Ensure target initial velocity is within speed limits
    initial_speed = norm(targ.initial_velocity);
    if initial_speed > targ.max_speed
        warning('Target initial velocity exceeds max speed, scaling down');
        targ.initial_velocity = targ.initial_velocity * (targ.max_speed / initial_speed);
    end
    
    % Validate target signatures
    if isfield(targ, 'signatures')
        assert(targ.signatures.radar_cross_section > 0 && targ.signatures.radar_cross_section < 1000, ...
               'Radar cross section must be between 0 and 1000 m²');
        signature_fields = {'infrared_signature', 'acoustic_signature', 'visual_signature'};
        for i = 1:length(signature_fields)
            if isfield(targ.signatures, signature_fields{i})
                sig_val = targ.signatures.(signature_fields{i});
                assert(sig_val >= 0 && sig_val <= 1, ...
                       sprintf('%s must be between 0 and 1', signature_fields{i}));
            end
        end
    end
    
    % Validate weather parameters
    if isfield(env, 'weather')
        valid_weather_types = {'clear', 'cloudy', 'storm', 'fog', 'rain', 'snow'};
        if ~ismember(env.weather.type, valid_weather_types)
            warning('Invalid weather type, setting to clear');
            env.weather.type = 'clear';
        end
        
        assert(env.weather.intensity >= 0 && env.weather.intensity <= 1, ...
               'Weather intensity must be between 0 and 1');
    end
    
    % Validate scenario constraints
    if isfield(env, 'scenario') && isfield(env.scenario, 'constraints')
        sc = env.scenario.constraints;
        assert(sc.max_flight_time > 0 && sc.max_flight_time < 7200, ...
               'Max flight time must be between 0 and 7200 s (2 hours)');
        assert(sc.minimum_altitude >= 0, ...
               'Minimum altitude must be non-negative');
        assert(sc.maximum_altitude > sc.minimum_altitude, ...
               'Maximum altitude must exceed minimum altitude');
    end
    
    % Cross-validate guidance and target parameters
    % Ensure guidance can handle target maneuvers
    if targ.max_accel > guid.constraints.max_lateral_accel * 0.5
        warning('Target acceleration may exceed guidance system capability');
    end
    
    % Ensure environmental conditions are consistent
    if env.weather_visibility < 1000 && strcmp(env.weather.type, 'clear')
        warning('Low visibility inconsistent with clear weather, adjusting weather type');
        env.weather.type = 'fog';
    end
end