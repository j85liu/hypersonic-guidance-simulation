function guidance_command = guidance_system(guidance_position, guidance_velocity, target_position, target_velocity, guidance_config, threats_detected, active_threats, current_time)
%GUIDANCE_SYSTEM Advanced guidance law coordinator for hypersonic vehicle
%
% This function implements the guidance algorithms extracted from the main
% simulation comprehensive code (lines 460-520). Provides Augmented 
% Proportional Navigation (APN) for mid-course and terminal homing guidance
% with integrated threat evasion capabilities.
%
% Inputs:
%   guidance_position - Current vehicle position estimate [3x1] (m)
%   guidance_velocity - Current vehicle velocity estimate [3x1] (m/s)
%   target_position - Target position [3x1] (m)
%   target_velocity - Target velocity [3x1] (m/s)
%   guidance_config - Guidance configuration structure
%   threats_detected - Boolean flag indicating active threats
%   active_threats - Number of active threat sites
%   current_time - Current simulation time (s)
%
% Outputs:
%   guidance_command - Commanded acceleration [3x1] (m/s²)
%
% Features:
%   - Augmented Proportional Navigation (APN) for mid-course
%   - Terminal homing with lead angle prediction
%   - Integrated threat evasion maneuvers
%   - Acceleration limiting for vehicle constraints
%   - Phase-based guidance law switching
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Input validation
    if nargin < 8
        error('guidance_system requires 8 input arguments');
    end
    
    % Initialize output
    guidance_command = zeros(3, 1);
    
    try
        % Calculate primary guidance command based on flight phase
        relative_position = target_position - guidance_position;
        range_to_target = norm(relative_position);
        
        if range_to_target > guidance_config.terminal_range
            % Mid-course guidance: Augmented Proportional Navigation
            guidance_command = calculate_apn_guidance(guidance_position, guidance_velocity, ...
                                                    target_position, target_velocity, ...
                                                    guidance_config, relative_position, range_to_target);
        else
            % Terminal guidance: Pure pursuit with lead angle
            guidance_command = calculate_terminal_guidance(guidance_position, guidance_velocity, ...
                                                         target_position, target_velocity, ...
                                                         guidance_config, relative_position);
        end
        
        % Add evasive maneuvers if under threat
        if threats_detected
            evasive_command = calculate_evasive_maneuvers(active_threats, guidance_config);
            guidance_command = guidance_command + evasive_command;
        end
        
        % Apply vehicle acceleration limits
        guidance_command = apply_acceleration_limits(guidance_command, guidance_config);
        
    catch ME
        fprintf('Warning: Guidance system error: %s\n', ME.message);
        guidance_command = zeros(3, 1);
    end
end

%% Local Guidance Implementation Functions

function guidance_command = calculate_apn_guidance(guidance_position, guidance_velocity, ...
                                                 target_position, target_velocity, ...
                                                 guidance_config, relative_position, range_to_target)
    %CALCULATE_APN_GUIDANCE Augmented Proportional Navigation implementation
    %
    % This implements the APN algorithm from lines 470-485 of the original
    % main simulation comprehensive code.
    
    relative_velocity = target_velocity - guidance_velocity;
    
    if norm(relative_velocity) > 0.1
        % Line-of-sight rate calculation (core of APN)
        unit_los = relative_position / range_to_target;
        
        % Project relative velocity perpendicular to line of sight
        los_rate = (relative_velocity - dot(relative_velocity, unit_los) * unit_los) / range_to_target;
        
        % APN guidance command: Nav * Vc * ωlos
        closing_velocity = -dot(relative_velocity, unit_los);
        guidance_command = guidance_config.Nav * closing_velocity * los_rate;
        
        % Enhanced APN: Add bias term for target acceleration compensation
        if norm(target_velocity) > 50  % Only if target is maneuvering significantly
            % Estimate target acceleration (simplified)
            estimated_target_accel = target_velocity * 0.1;  % Rough estimate
            guidance_command = guidance_command + 0.5 * estimated_target_accel;
        end
        
    else
        % Fallback to proportional navigation for very low relative velocity
        guidance_command = guidance_config.proportional_gain * relative_position / range_to_target;
    end
end

function guidance_command = calculate_terminal_guidance(guidance_position, guidance_velocity, ...
                                                      target_position, target_velocity, ...
                                                      guidance_config, relative_position)
    %CALCULATE_TERMINAL_GUIDANCE Terminal homing with lead angle prediction
    %
    % This implements the terminal guidance from lines 487-495 of the 
    % original main simulation comprehensive code.
    
    % Time-to-intercept estimation
    vehicle_speed = max(norm(guidance_velocity), 100);  % Minimum speed assumption
    time_to_intercept = max(1, norm(relative_position) / vehicle_speed);
    
    % Predict target position at intercept
    predicted_target_pos = target_position + target_velocity * time_to_intercept;
    
    % Enhanced prediction: account for target evasive capability
    if norm(target_velocity) > 25  % Target is moving
        % Add uncertainty cone for evasive maneuvers
        evasion_factor = min(0.3, norm(target_velocity) / 100);
        evasion_uncertainty = evasion_factor * randn(3,1) * norm(relative_position) * 0.1;
        predicted_target_pos = predicted_target_pos + evasion_uncertainty;
    end
    
    % Pure pursuit guidance vector
    guidance_vector = predicted_target_pos - guidance_position;
    
    % Proportional navigation with enhanced gain for terminal phase
    terminal_gain = guidance_config.proportional_gain * 1.5;  % Increased aggressiveness
    guidance_command = terminal_gain * guidance_vector;
    
    % Add velocity matching component for terminal accuracy
    velocity_matching = 0.2 * (target_velocity - guidance_velocity);
    guidance_command = guidance_command + velocity_matching;
end

function evasive_command = calculate_evasive_maneuvers(active_threats, guidance_config)
    %CALCULATE_EVASIVE_MANEUVERS Generate threat evasion commands
    %
    % This implements the evasive maneuver logic from lines 497-502 of
    % the original main simulation comprehensive code.
    
    % Evasive intensity based on threat level
    evasive_intensity = min(1.0, active_threats * 0.2);  % Max 100% evasion
    
    % Generate random evasive acceleration (reduced from original for stability)
    base_evasive_accel = 80;  % m/s² base evasive acceleration
    evasive_command = evasive_intensity * base_evasive_accel * randn(3,1);
    
    % Ensure evasive maneuvers don't exceed reasonable bounds
    max_evasive = guidance_config.max_lateral_accel * 0.3;  % Limit evasion to 30% of max
    if norm(evasive_command) > max_evasive
        evasive_command = evasive_command * (max_evasive / norm(evasive_command));
    end
    
    % Prefer lateral evasion (X-Y plane) over altitude changes
    evasive_command(3) = evasive_command(3) * 0.5;  % Reduce vertical evasion
end

function limited_command = apply_acceleration_limits(guidance_command, guidance_config)
    %APPLY_ACCELERATION_LIMITS Enforce vehicle acceleration constraints
    %
    % This implements the acceleration limiting from lines 503-508 of
    % the original main simulation comprehensive code.
    
    % Apply maximum lateral acceleration limit
    if norm(guidance_command) > guidance_config.max_lateral_accel
        limited_command = guidance_command * (guidance_config.max_lateral_accel / norm(guidance_command));
    else
        limited_command = guidance_command;
    end
    
    % Additional constraints for realistic hypersonic vehicle
    % Limit individual axis accelerations
    max_axis_accel = guidance_config.max_lateral_accel * 0.8;
    limited_command(1) = max(-max_axis_accel, min(max_axis_accel, limited_command(1)));
    limited_command(2) = max(-max_axis_accel, min(max_axis_accel, limited_command(2)));
    limited_command(3) = max(-max_axis_accel, min(max_axis_accel, limited_command(3)));
    
    % Ensure reasonable command magnitudes
    if any(isnan(limited_command)) || any(isinf(limited_command))
        limited_command = zeros(3, 1);
    end
end