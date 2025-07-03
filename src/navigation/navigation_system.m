function [nav_estimates, sensor_measurements, sensor_availability] = navigation_system(vehicle_state, target_position, sensors_config, environment_effects, threats, current_time, dt)
%NAVIGATION_SYSTEM Multi-sensor navigation coordinator for hypersonic vehicle
%
% This function implements the complete navigation system extracted from the
% main simulation comprehensive code (lines 350-460). Coordinates INS, GPS,
% TERCOM, laser, and IR sensors with Extended Kalman Filter fusion for
% GPS-denied navigation capability.
%
% Inputs:
%   vehicle_state - Current vehicle state [12x1]: [pos, vel, att, rates]
%   target_position - Target position [3x1] (m)
%   sensors_config - Sensor configuration structure
%   environment_effects - Environmental effects structure
%   threats - Threat assessment structure
%   current_time - Current simulation time (s)
%   dt - Time step (s)
%
% Outputs:
%   nav_estimates - Navigation estimates structure
%   sensor_measurements - Individual sensor measurements structure
%   sensor_availability - Sensor availability status [6x1] boolean
%
% Navigation Features:
%   - Multi-sensor fusion with Extended Kalman Filter
%   - GPS-denied navigation capability
%   - INS drift modeling and compensation
%   - Sensor availability assessment
%   - Adaptive sensor weighting
%   - Plasma interference modeling
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Extract current vehicle state
    position = vehicle_state(1:3);
    velocity = vehicle_state(4:6);
    attitude = vehicle_state(7:9);
    
    % Initialize output structures
    nav_estimates = struct();
    sensor_measurements = struct();
    sensor_availability = false(6, 1);
    
    try
        % Assess sensor availability based on conditions
        sensor_availability = assess_sensor_availability(position, velocity, target_position, ...
                                                       environment_effects, threats, current_time, sensors_config);
        
        % Generate individual sensor measurements
        sensor_measurements = generate_sensor_measurements(position, velocity, attitude, ...
                                                         target_position, sensor_availability, ...
                                                         sensors_config, environment_effects, current_time);
        
        % Perform multi-sensor navigation fusion
        nav_estimates = perform_navigation_fusion(sensor_measurements, sensor_availability, ...
                                                sensors_config, environment_effects, dt);
        
        % Validate navigation estimates
        nav_estimates = validate_navigation_estimates(nav_estimates, position, velocity);
        
    catch ME
        fprintf('Warning: Navigation system error: %s\n', ME.message);
        % Fallback to INS-only navigation
        nav_estimates = create_ins_fallback(position, velocity, sensors_config, current_time);
        sensor_measurements = struct();
        sensor_availability = [true; false; false; false; false; false]; % INS only
    end
end

%% Local Navigation Implementation Functions

function availability = assess_sensor_availability(position, velocity, target_position, environment_effects, threats, current_time, sensors_config)
    %ASSESS_SENSOR_AVAILABILITY Determine which sensors are currently available
    %
    % This implements the sensor availability logic from the original
    % simulation (lines 360-390) with enhanced environmental considerations
    
    availability = false(6, 1);  % [INS, GPS, TERCOM, LASER, IR, COMM]
    
    % Calculate derived quantities
    range_to_target = norm(target_position - position);
    altitude = position(3);
    speed = norm(velocity);
    
    % INS (always available - from original line 365)
    availability(1) = true;
    
    % GPS availability (from original lines 367-375)
    gps_altitude_ok = altitude > sensors_config.gps.availability_altitude;
    gps_jamming_ok = range_to_target > sensors_config.gps.jamming_range;
    plasma_interference = environment_effects.plasma_interference;
    gps_plasma_ok = rand > plasma_interference;
    
    availability(2) = gps_altitude_ok && gps_jamming_ok && gps_plasma_ok;
    
    % TERCOM availability (from original lines 377-380)
    tercom_altitude_ok = altitude < sensors_config.tercom.max_altitude;
    tercom_speed_ok = speed > sensors_config.tercom.min_speed;
    tercom_update_ready = mod(current_time, 1/sensors_config.tercom.update_rate) < 0.1;
    
    availability(3) = tercom_altitude_ok && tercom_speed_ok && tercom_update_ready;
    
    % Laser designation availability (from original lines 382-386)
    laser_range_ok = range_to_target < sensors_config.laser.max_range;
    laser_target_painted = current_time > 60;  % Assume painting starts after 60s
    laser_weather_ok = rand > (1 - environment_effects.visibility_factor);
    
    availability(4) = laser_range_ok && laser_target_painted && laser_weather_ok;
    
    % IR/EO sensor availability (from original lines 388-390)
    ir_range_ok = range_to_target < sensors_config.ir.max_range;
    ir_operational = current_time > 30;  % Available after 30s
    ir_weather_ok = environment_effects.visibility_factor > 0.3;
    
    availability(5) = ir_range_ok && ir_operational && ir_weather_ok;
    
    % Communication availability
    comm_plasma_ok = rand > plasma_interference;
    comm_jamming_ok = ~threats.detected || rand > 0.5;  % 50% availability under threat
    
    availability(6) = comm_plasma_ok && comm_jamming_ok;
end

function measurements = generate_sensor_measurements(position, velocity, attitude, target_position, availability, sensors_config, environment_effects, current_time)
    %GENERATE_SENSOR_MEASUREMENTS Create measurements from available sensors
    %
    % This implements the sensor measurement generation from the original
    % simulation with realistic noise characteristics and error modeling
    
    measurements = struct();
    
    % INS measurements (always available but drifting)
    if availability(1)
        measurements.ins = generate_ins_measurements(position, velocity, attitude, ...
                                                   sensors_config.ins, environment_effects, current_time);
    end
    
    % GPS measurements (high accuracy when available)
    if availability(2)
        measurements.gps = generate_gps_measurements(position, sensors_config.gps, environment_effects);
    end
    
    % TERCOM measurements (terrain correlation)
    if availability(3)
        measurements.tercom = generate_tercom_measurements(position, sensors_config.tercom, environment_effects);
    end
    
    % Laser measurements (high accuracy terminal guidance)
    if availability(4)
        measurements.laser = generate_laser_measurements(target_position, position, ...
                                                       sensors_config.laser, environment_effects);
    end
    
    % IR/EO measurements (passive target tracking)
    if availability(5)
        measurements.ir = generate_ir_measurements(target_position, position, ...
                                                 sensors_config.ir, environment_effects);
    end
    
    % Communication updates
    if availability(6)
        measurements.comm = generate_comm_measurements(position, velocity, sensors_config.comm);
    end
end

function ins_meas = generate_ins_measurements(position, velocity, attitude, ins_config, environment_effects, current_time)
    %GENERATE_INS_MEASUREMENTS Generate INS measurements with realistic drift
    %
    % This implements the INS error modeling from the original simulation
    % (lines 395-410) with time-varying drift and bias effects
    
    ins_meas = struct();
    
    % Time-dependent drift calculation (from original lines 400-405)
    flight_time_hours = current_time / 3600;
    position_drift_rate = ins_config.drift_rate * flight_time_hours;
    
    % Bias evolution (persistent errors)
    persistent ins_bias_drift;
    if isempty(ins_bias_drift)
        ins_bias_drift = zeros(3,1);
    end
    ins_bias_drift = ins_bias_drift + 0.001 * 0.05 * randn(3,1);  % Slow bias evolution
    
    % Random walk errors
    persistent ins_random_walk;
    if isempty(ins_random_walk)
        ins_random_walk = zeros(3,1);
    end
    ins_random_walk = ins_random_walk + ins_config.random_walk_pos * sqrt(0.05) * randn(3,1);
    
    % Total INS position error
    ins_position_error = position_drift_rate * [1; 0.6; 0.9] + ins_random_walk + ins_bias_drift;
    ins_velocity_error = 0.1 * ins_position_error + ins_config.velocity_accuracy .* randn(3,1);
    
    % Apply thermal effects
    thermal_factor = environment_effects.thermal_factor;
    
    % INS reported measurements (what INS thinks position/velocity is)
    ins_meas.position = position + ins_position_error * thermal_factor;
    ins_meas.velocity = velocity + ins_velocity_error * thermal_factor;
    ins_meas.attitude = attitude + ins_config.attitude_accuracy .* randn(3,1);
    
    % Measurement uncertainties
    ins_meas.position_std = (5 + position_drift_rate/10) * thermal_factor * ones(3,1);
    ins_meas.velocity_std = (2 + position_drift_rate/50) * thermal_factor * ones(3,1);
    ins_meas.attitude_std = ins_config.attitude_accuracy;
end

function gps_meas = generate_gps_measurements(position, gps_config, environment_effects)
    %GENERATE_GPS_MEASUREMENTS Generate GPS position measurements
    
    gps_meas = struct();
    
    % GPS measurement noise (affected by plasma interference)
    plasma_factor = 1 + environment_effects.plasma_interference;
    gps_noise_std = gps_config.accuracy * plasma_factor;
    gps_noise = gps_noise_std * randn(3,1);
    
    % Add multipath and atmospheric errors
    multipath_error = gps_config.multipath_error * randn(3,1);
    atmospheric_error = (gps_config.ionospheric_error + gps_config.tropospheric_error) * randn(3,1);
    
    % GPS measurement
    gps_meas.position = position + gps_noise + multipath_error + atmospheric_error;
    gps_meas.position_std = gps_noise_std * ones(3,1);
    gps_meas.quality_factor = 1 / plasma_factor;  % Quality degrades with plasma
end

function tercom_meas = generate_tercom_measurements(position, tercom_config, environment_effects)
    %GENERATE_TERCOM_MEASUREMENTS Generate terrain-aided navigation measurements
    
    tercom_meas = struct();
    
    % Terrain reference (simplified terrain model matching original)
    terrain_height = 100 + 50*sin(position(1)/15000) + 30*cos(position(2)/12000);
    
    % TERCOM measurement: altitude above terrain
    tercom_noise = tercom_config.accuracy * randn;
    weather_degradation = 1 + tercom_config.weather_degradation * (1 - environment_effects.visibility_factor);
    
    tercom_meas.altitude_above_terrain = position(3) - terrain_height + tercom_noise * weather_degradation;
    tercom_meas.terrain_height = terrain_height;
    tercom_meas.correlation_quality = 0.8 - 0.2 * weather_degradation;
    tercom_meas.measurement_std = tercom_config.accuracy * weather_degradation;
end

function laser_meas = generate_laser_measurements(target_position, vehicle_position, laser_config, environment_effects)
    %GENERATE_LASER_MEASUREMENTS Generate laser designator measurements
    
    laser_meas = struct();
    
    % Laser measurement of target position (high accuracy)
    weather_factor = laser_config.weather_factor * environment_effects.visibility_factor;
    laser_noise_std = laser_config.accuracy / weather_factor;
    laser_noise = laser_noise_std * randn(3,1);
    
    laser_meas.target_position = target_position + laser_noise;
    laser_meas.range_to_target = norm(target_position - vehicle_position) + laser_noise_std * randn;
    laser_meas.measurement_std = laser_noise_std * ones(3,1);
    laser_meas.beam_quality = weather_factor;
end

function ir_meas = generate_ir_measurements(target_position, vehicle_position, ir_config, environment_effects)
    %GENERATE_IR_MEASUREMENTS Generate infrared sensor measurements
    
    ir_meas = struct();
    
    % IR sensor angular measurements
    relative_position = target_position - vehicle_position;
    range_to_target = norm(relative_position);
    
    % Angular noise based on sensor resolution
    angular_noise = ir_config.angular_resolution * randn(2,1);
    
    % Convert to position uncertainty
    position_uncertainty = range_to_target * ir_config.angular_resolution;
    weather_degradation = 2 - environment_effects.visibility_factor;
    
    ir_noise = position_uncertainty * weather_degradation * randn(3,1);
    
    ir_meas.target_position = target_position + ir_noise;
    ir_meas.range_estimate = range_to_target + 0.1 * range_to_target * randn;
    ir_meas.measurement_std = position_uncertainty * weather_degradation * ones(3,1);
    ir_meas.signal_strength = 1 / weather_degradation;
end

function comm_meas = generate_comm_measurements(position, velocity, comm_config)
    %GENERATE_COMM_MEASUREMENTS Generate communication system updates
    
    comm_meas = struct();
    
    % Simulated external position/velocity updates (when available)
    comm_noise_pos = 5 * randn(3,1);  % 5m position uncertainty
    comm_noise_vel = 1 * randn(3,1);  % 1 m/s velocity uncertainty
    
    comm_meas.position = position + comm_noise_pos;
    comm_meas.velocity = velocity + comm_noise_vel;
    comm_meas.position_std = 5 * ones(3,1);
    comm_meas.velocity_std = 1 * ones(3,1);
    comm_meas.link_quality = 1 - comm_config.packet_loss_rate;
end

function nav_estimates = perform_navigation_fusion(measurements, availability, sensors_config, environment_effects, dt)
    %PERFORM_NAVIGATION_FUSION Multi-sensor Extended Kalman Filter fusion
    %
    % This implements the FIXED EKF from the original simulation
    % (lines 420-460) with proper 6-state implementation
    
    persistent ekf_state ekf_P;
    
    % Initialize EKF state if first run
    if isempty(ekf_state)
        if availability(1) && isfield(measurements, 'ins')
            ekf_state = [measurements.ins.position; measurements.ins.velocity];
            ekf_P = diag([50^2, 50^2, 30^2, 10^2, 10^2, 5^2]);  % Initial uncertainty
        else
            ekf_state = zeros(6,1);
            ekf_P = diag([100^2, 100^2, 50^2, 20^2, 20^2, 10^2]);
        end
    end
    
    % EKF Process Model (6-state: position + velocity)
    F = [eye(3), dt*eye(3);
         zeros(3), eye(3)];
    
    % Process noise
    Q = diag([1^2, 1^2, 1^2, 0.5^2, 0.5^2, 0.3^2]);
    
    % Prediction step
    ekf_state = F * ekf_state;
    ekf_P = F * ekf_P * F' + Q;
    
    % Measurement updates for each available sensor
    if availability(1) && isfield(measurements, 'ins')
        [ekf_state, ekf_P] = update_ekf_ins(ekf_state, ekf_P, measurements.ins, environment_effects);
    end
    
    if availability(2) && isfield(measurements, 'gps')
        [ekf_state, ekf_P] = update_ekf_gps(ekf_state, ekf_P, measurements.gps);
    end
    
    if availability(3) && isfield(measurements, 'tercom')
        [ekf_state, ekf_P] = update_ekf_tercom(ekf_state, ekf_P, measurements.tercom);
    end
    
    if availability(4) && isfield(measurements, 'laser')
        [ekf_state, ekf_P] = update_ekf_laser(ekf_state, ekf_P, measurements.laser);
    end
    
    if availability(5) && isfield(measurements, 'ir')
        [ekf_state, ekf_P] = update_ekf_ir(ekf_state, ekf_P, measurements.ir);
    end
    
    % Create navigation estimates output
    nav_estimates = struct();
    nav_estimates.position = ekf_state(1:3);
    nav_estimates.velocity = ekf_state(4:6);
    nav_estimates.position_std = sqrt(diag(ekf_P(1:3, 1:3)));
    nav_estimates.velocity_std = sqrt(diag(ekf_P(4:6, 4:6)));
    nav_estimates.covariance = ekf_P;
    nav_estimates.state_vector = ekf_state;
end

function [state, P] = update_ekf_ins(state, P, ins_meas, environment_effects)
    %UPDATE_EKF_INS Update EKF with INS measurements
    
    % INS measures both position and velocity (biased)
    H = eye(6);
    ins_measurement = [ins_meas.position; ins_meas.velocity];
    
    % Measurement noise (time and thermal dependent)
    R = diag([ins_meas.position_std.^2; ins_meas.velocity_std.^2]);
    
    % Kalman update
    S = H * P * H' + R;
    K = P * H' / S;
    innovation = ins_measurement - H * state;
    
    state = state + K * innovation;
    P = (eye(6) - K * H) * P;
end

function [state, P] = update_ekf_gps(state, P, gps_meas)
    %UPDATE_EKF_GPS Update EKF with GPS measurements
    
    % GPS measures position only
    H = [eye(3), zeros(3)];
    R = diag(gps_meas.position_std.^2);
    
    % Kalman update
    S = H * P * H' + R;
    K = P * H' / S;
    innovation = gps_meas.position - H * state;
    
    state = state + K * innovation;
    P = (eye(6) - K * H) * P;
end

function [state, P] = update_ekf_tercom(state, P, tercom_meas)
    %UPDATE_EKF_TERCOM Update EKF with TERCOM measurements
    
    % TERCOM measures altitude above terrain
    H = [0, 0, 1, 0, 0, 0];  % Altitude only
    R = tercom_meas.measurement_std^2;
    
    % Expected measurement: altitude above terrain
    expected_altitude_above_terrain = state(3) - tercom_meas.terrain_height;
    innovation = tercom_meas.altitude_above_terrain - expected_altitude_above_terrain;
    
    % Kalman update
    S = H * P * H' + R;
    K = P * H' / S;
    
    state = state + K * innovation;
    P = (eye(6) - K * H) * P;
end

function [state, P] = update_ekf_laser(state, P, laser_meas)
    %UPDATE_EKF_LASER Update EKF with laser measurements
    
    % Laser provides target position - use for position correction
    H = [eye(3), zeros(3)];  % Position measurement
    R = diag(laser_meas.measurement_std.^2);
    
    % Use laser target measurement to infer vehicle position accuracy
    % (This is simplified - in reality would use geometry)
    laser_position_estimate = laser_meas.target_position;  % Simplified
    
    % Kalman update (simplified implementation)
    S = H * P * H' + R;
    K = P * H' / S;
    
    % Note: This is a simplified laser update. In practice, would use
    % target-relative geometry for position estimation
    state = state + K * 0.1 * (laser_position_estimate - state(1:3));  % Small correction
    P = (eye(6) - K * H) * P;
end

function [state, P] = update_ekf_ir(state, P, ir_meas)
    %UPDATE_EKF_IR Update EKF with IR measurements
    
    % IR provides target position estimate - similar to laser but lower accuracy
    H = [eye(3), zeros(3)];  % Position measurement
    R = diag(ir_meas.measurement_std.^2);
    
    % Simplified IR update similar to laser
    S = H * P * H' + R;
    K = P * H' / S;
    
    state = state + K * 0.05 * (ir_meas.target_position - state(1:3));  % Smaller correction
    P = (eye(6) - K * H) * P;
end

function nav_estimates = validate_navigation_estimates(nav_estimates, true_position, true_velocity)
    %VALIDATE_NAVIGATION_ESTIMATES Ensure navigation estimates are reasonable
    
    % Check for NaN or Inf values
    if any(isnan(nav_estimates.position)) || any(isinf(nav_estimates.position))
        nav_estimates.position = true_position;
        nav_estimates.position_std = 100 * ones(3,1);  % Large uncertainty
    end
    
    if any(isnan(nav_estimates.velocity)) || any(isinf(nav_estimates.velocity))
        nav_estimates.velocity = true_velocity;
        nav_estimates.velocity_std = 50 * ones(3,1);  % Large uncertainty
    end
    
    % Ensure reasonable bounds
    nav_estimates.position(3) = max(0, nav_estimates.position(3));  % Non-negative altitude
    
    % Limit uncertainty bounds
    nav_estimates.position_std = max(0.1, min(1000, nav_estimates.position_std));
    nav_estimates.velocity_std = max(0.1, min(100, nav_estimates.velocity_std));
end

function nav_estimates = create_ins_fallback(position, velocity, sensors_config, current_time)
    %CREATE_INS_FALLBACK Create INS-only navigation when other sensors fail
    
    nav_estimates = struct();
    
    % Use INS with increasing uncertainty over time
    flight_time_hours = current_time / 3600;
    ins_uncertainty = sensors_config.ins.drift_rate * flight_time_hours;
    
    nav_estimates.position = position;
    nav_estimates.velocity = velocity;
    nav_estimates.position_std = max(5, ins_uncertainty) * ones(3,1);
    nav_estimates.velocity_std = 2 * ones(3,1);
    nav_estimates.covariance = diag([nav_estimates.position_std.^2; nav_estimates.velocity_std.^2]);
    nav_estimates.state_vector = [position; velocity];
end