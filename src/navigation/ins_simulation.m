function ins_data = ins_simulation(true_position, true_velocity, true_attitude, ins_config, environment_effects, current_time, dt)
%INS_SIMULATION Realistic Inertial Navigation System error modeling
%
% This function simulates realistic INS behavior extracted from the main
% simulation comprehensive code (lines 395-410). Models time-dependent
% drift, bias evolution, random walk errors, and environmental effects
% for accurate GPS-denied navigation simulation.
%
% Inputs:
%   true_position - True vehicle position [3x1] (m)
%   true_velocity - True vehicle velocity [3x1] (m/s)
%   true_attitude - True vehicle attitude [3x1] (rad)
%   ins_config - INS configuration structure
%   environment_effects - Environmental effects structure
%   current_time - Current simulation time (s)
%   dt - Time step (s)
%
% Outputs:
%   ins_data - INS measurements and error characteristics structure
%
% INS Error Sources:
%   - Systematic bias drift
%   - Random walk errors
%   - Time-dependent position drift
%   - Thermal effects
%   - Vibration and acceleration effects
%   - Initial alignment errors
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Initialize persistent variables for error accumulation
    persistent ins_bias_drift ins_random_walk ins_alignment_error
    persistent ins_initialization_time ins_last_update_time
    
    % Initialize persistent variables on first call
    if isempty(ins_initialization_time)
        ins_initialization_time = current_time;
        ins_last_update_time = current_time;
        ins_bias_drift = ins_config.bias_accel + 0.1 * ins_config.bias_accel .* randn(3,1);
        ins_random_walk = zeros(3,1);
        ins_alignment_error = ins_config.initial_accuracy .* randn(3,1);
    end
    
    % Calculate time-dependent parameters
    flight_time_hours = (current_time - ins_initialization_time) / 3600;
    time_since_last_update = current_time - ins_last_update_time;
    
    try
        % Calculate INS error components
        [position_error, velocity_error, attitude_error] = calculate_ins_errors(...
            true_position, true_velocity, true_attitude, ins_config, ...
            environment_effects, flight_time_hours, time_since_last_update);
        
        % Update persistent error states
        [ins_bias_drift, ins_random_walk] = update_ins_error_states(...
            ins_bias_drift, ins_random_walk, ins_config, environment_effects, dt);
        
        % Generate INS measurements
        ins_data = generate_ins_output(true_position, true_velocity, true_attitude, ...
                                     position_error, velocity_error, attitude_error, ...
                                     ins_config, environment_effects, flight_time_hours);
        
        % Update time tracking
        ins_last_update_time = current_time;
        
    catch ME
        fprintf('Warning: INS simulation error: %s\n', ME.message);
        % Fallback to basic INS model
        ins_data = create_basic_ins_output(true_position, true_velocity, true_attitude, ins_config);
    end
end

%% Local INS Error Modeling Functions

function [pos_error, vel_error, att_error] = calculate_ins_errors(true_position, true_velocity, true_attitude, ins_config, environment_effects, flight_time_hours, dt)
    %CALCULATE_INS_ERRORS Compute comprehensive INS error components
    %
    % This implements the INS error modeling from the original simulation
    % (lines 400-410) with enhanced realism and environmental effects
    
    % Access persistent error states
    persistent ins_bias_drift ins_random_walk ins_alignment_error
    
    % 1. Time-dependent position drift (from original lines 400-405)
    position_drift_rate = ins_config.drift_rate * flight_time_hours;
    systematic_drift = position_drift_rate * [1; 0.6; 0.9];  % Different drift rates per axis
    
    % 2. Bias drift evolution (from original)
    if isempty(ins_bias_drift)
        ins_bias_drift = ins_config.bias_accel;
    end
    
    % 3. Random walk accumulation
    if isempty(ins_random_walk)
        ins_random_walk = zeros(3,1);
    end
    
    % 4. Initial alignment errors
    if isempty(ins_alignment_error)
        ins_alignment_error = ins_config.initial_accuracy .* randn(3,1);
    end
    
    % 5. Environmental effects on INS performance
    thermal_factor = environment_effects.thermal_factor;
    vibration_factor = 1 + 0.1 * norm(true_velocity) / 1000;  % High-speed vibration
    acceleration_factor = 1 + 0.05 * (norm(true_velocity) / 100 - 1);  % Acceleration effects
    
    % Combined environmental degradation
    environmental_factor = thermal_factor * vibration_factor * acceleration_factor;
    
    % Total position error
    pos_error = systematic_drift + ins_bias_drift + ins_random_walk + ins_alignment_error;
    pos_error = pos_error * environmental_factor;
    
    % Velocity error (derivative of position error + additional noise)
    vel_error = 0.1 * pos_error + ins_config.velocity_accuracy .* randn(3,1);
    vel_error = vel_error * sqrt(environmental_factor);
    
    % Attitude error (gyroscope drift and noise)
    gyro_drift = ins_config.bias_gyro * flight_time_hours;
    attitude_noise = ins_config.attitude_accuracy .* randn(3,1);
    att_error = gyro_drift + attitude_noise + 0.01 * ins_alignment_error;
    att_error = att_error * sqrt(environmental_factor);
end

function [new_bias_drift, new_random_walk] = update_ins_error_states(bias_drift, random_walk, ins_config, environment_effects, dt)
    %UPDATE_INS_ERROR_STATES Update persistent INS error states
    %
    % Models the evolution of INS errors over time including bias drift
    % and random walk accumulation
    
    % Bias drift evolution (slow random walk in bias)
    bias_evolution_rate = 0.001 * sqrt(dt);  % Bias changes slowly
    bias_noise = bias_evolution_rate * randn(3,1);
    new_bias_drift = bias_drift + bias_noise;
    
    % Limit bias drift to reasonable bounds
    max_bias = 5 * ins_config.bias_accel;
    new_bias_drift = max(-max_bias, min(max_bias, new_bias_drift));
    
    % Random walk accumulation
    random_walk_increment = ins_config.random_walk_pos * sqrt(dt) * randn(3,1);
    new_random_walk = random_walk + random_walk_increment;
    
    % Apply environmental effects to error growth
    thermal_growth = (environment_effects.thermal_factor - 1) * 0.1;
    new_random_walk = new_random_walk * (1 + thermal_growth);
    
    % Limit random walk to prevent unrealistic error growth
    max_random_walk = 1000;  % Maximum 1km random walk error
    if norm(new_random_walk) > max_random_walk
        new_random_walk = new_random_walk * (max_random_walk / norm(new_random_walk));
    end
end

function ins_data = generate_ins_output(true_position, true_velocity, true_attitude, position_error, velocity_error, attitude_error, ins_config, environment_effects, flight_time_hours)
    %GENERATE_INS_OUTPUT Create realistic INS measurement output
    %
    % Generates what the INS would actually report, including all error
    % sources and realistic measurement characteristics
    
    ins_data = struct();
    
    % INS reported measurements (what INS thinks the state is)
    ins_data.position = true_position + position_error;
    ins_data.velocity = true_velocity + velocity_error;
    ins_data.attitude = true_attitude + attitude_error;
    
    % Wrap attitude angles to appropriate ranges
    ins_data.attitude(1) = wrap_to_pi(ins_data.attitude(1));  % Roll
    ins_data.attitude(2) = wrap_to_pi(ins_data.attitude(2));  % Pitch
    ins_data.attitude(3) = wrap_to_pi(ins_data.attitude(3));  % Yaw
    
    % Measurement uncertainties (time and environment dependent)
    base_pos_std = ins_config.initial_accuracy;
    drift_uncertainty = ins_config.drift_rate * flight_time_hours / 3;  % Grows with time
    thermal_uncertainty = (environment_effects.thermal_factor - 1) * base_pos_std;
    
    ins_data.position_std = base_pos_std + drift_uncertainty + thermal_uncertainty;
    ins_data.velocity_std = ins_config.velocity_accuracy + 0.1 * drift_uncertainty;
    ins_data.attitude_std = ins_config.attitude_accuracy * environment_effects.thermal_factor;
    
    % Additional INS performance metrics
    ins_data.quality_factor = 1 / environment_effects.thermal_factor;
    ins_data.navigation_ready = flight_time_hours * 3600 > ins_config.alignment_time;
    ins_data.drift_rate_estimate = ins_config.drift_rate * flight_time_hours;
    
    % INS status flags
    ins_data.status = struct();
    ins_data.status.alignment_complete = ins_data.navigation_ready;
    ins_data.status.gyro_health = environment_effects.thermal_factor < 2;
    ins_data.status.accelerometer_health = environment_effects.thermal_factor < 2;
    ins_data.status.navigation_mode = determine_navigation_mode(flight_time_hours, environment_effects);
    
    % Error analysis
    ins_data.errors = struct();
    ins_data.errors.position_error = position_error;
    ins_data.errors.velocity_error = velocity_error;
    ins_data.errors.attitude_error = attitude_error;
    ins_data.errors.total_position_error = norm(position_error);
    ins_data.errors.total_velocity_error = norm(velocity_error);
    ins_data.errors.total_attitude_error = norm(attitude_error);
end

function nav_mode = determine_navigation_mode(flight_time_hours, environment_effects)
    %DETERMINE_NAVIGATION_MODE Determine INS navigation mode based on conditions
    
    if flight_time_hours < 0.083  % Less than 5 minutes
        if environment_effects.thermal_factor > 1.5
            nav_mode = 'degraded';
        else
            nav_mode = 'normal';
        end
    elseif flight_time_hours < 0.5  % Less than 30 minutes
        if environment_effects.thermal_factor > 2
            nav_mode = 'degraded';
        else
            nav_mode = 'normal';
        end
    else  % Long duration flight
        if environment_effects.thermal_factor > 1.2
            nav_mode = 'degraded';
        else
            nav_mode = 'drift_compensation';
        end
    end
end

function ins_data = create_basic_ins_output(true_position, true_velocity, true_attitude, ins_config)
    %CREATE_BASIC_INS_OUTPUT Fallback INS model for error conditions
    
    ins_data = struct();
    
    % Basic noise model
    position_noise = ins_config.initial_accuracy .* randn(3,1);
    velocity_noise = ins_config.velocity_accuracy .* randn(3,1);
    attitude_noise = ins_config.attitude_accuracy .* randn(3,1);
    
    ins_data.position = true_position + position_noise;
    ins_data.velocity = true_velocity + velocity_noise;
    ins_data.attitude = true_attitude + attitude_noise;
    
    ins_data.position_std = ins_config.initial_accuracy;
    ins_data.velocity_std = ins_config.velocity_accuracy;
    ins_data.attitude_std = ins_config.attitude_accuracy;
    
    ins_data.quality_factor = 0.8;  % Reduced quality for fallback
    ins_data.navigation_ready = true;
    ins_data.drift_rate_estimate = ins_config.drift_rate;
    
    % Basic status
    ins_data.status = struct();
    ins_data.status.alignment_complete = true;
    ins_data.status.gyro_health = true;
    ins_data.status.accelerometer_health = true;
    ins_data.status.navigation_mode = 'basic';
    
    % Error information
    ins_data.errors = struct();
    ins_data.errors.position_error = position_noise;
    ins_data.errors.velocity_error = velocity_noise;
    ins_data.errors.attitude_error = attitude_noise;
    ins_data.errors.total_position_error = norm(position_noise);
    ins_data.errors.total_velocity_error = norm(velocity_noise);
    ins_data.errors.total_attitude_error = norm(attitude_noise);
end

function wrapped_angle = wrap_to_pi(angle)
    %WRAP_TO_PI Wrap angle to [-pi, pi] range
    wrapped_angle = atan2(sin(angle), cos(angle));
end