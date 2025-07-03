function new_vehicle_state = vehicle_dynamics(vehicle_state, guidance_command, vehicle_config, environment, dt)
%VEHICLE_DYNAMICS 6-DOF hypersonic vehicle dynamics integration
%
% This function implements the complete 6-DOF vehicle dynamics extracted
% from the main simulation comprehensive code (lines 520-620). Provides
% realistic hypersonic glide vehicle physics with aerodynamic effects,
% thermal constraints, and attitude dynamics.
%
% Inputs:
%   vehicle_state - Current 12-DOF state [pos(3), vel(3), att(3), rates(3)]
%   guidance_command - Commanded acceleration [3x1] (m/s²)
%   vehicle_config - Vehicle configuration structure
%   environment - Environmental conditions structure
%   dt - Integration time step (s)
%
% Outputs:
%   new_vehicle_state - Updated 12-DOF state vector
%
% State Vector Format:
%   [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
%   Position (m), Velocity (m/s), Euler angles (rad), Angular rates (rad/s)
%
% Features:
%   - 6-DOF translational and rotational dynamics
%   - Realistic aerodynamic drag modeling
%   - Mach-dependent drag coefficients
%   - Thermal effects on vehicle performance
%   - Attitude dynamics with control authority
%   - Wind and turbulence effects
%   - Ground collision detection
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Input validation
    if length(vehicle_state) ~= 12
        error('vehicle_state must be 12 elements: [pos(3), vel(3), att(3), rates(3)]');
    end
    
    % Extract current state components
    position = vehicle_state(1:3);
    velocity = vehicle_state(4:6);
    attitude = vehicle_state(7:9);
    ang_rates = vehicle_state(10:12);
    
    try
        % Calculate atmospheric properties at current altitude
        [atmosphere.rho, atmosphere.temp, atmosphere.pressure] = ...
            get_atmosphere_properties(position(3));
        
        % Apply environmental effects
        [relative_velocity, thermal_factor] = apply_environmental_effects(...
            velocity, position, environment, atmosphere);
        
        % Calculate forces and accelerations
        total_acceleration = calculate_total_acceleration(...
            guidance_command, relative_velocity, atmosphere, vehicle_config, thermal_factor);
        
        % Update translational motion
        new_velocity = velocity + total_acceleration * dt;
        new_position = position + new_velocity * dt;
        
        % Update attitude dynamics
        [new_attitude, new_ang_rates] = update_attitude_dynamics(...
            attitude, ang_rates, velocity, new_velocity, vehicle_config, dt);
        
        % Assemble new state vector
        new_vehicle_state = [new_position; new_velocity; new_attitude; new_ang_rates];
        
        % Apply physical constraints
        new_vehicle_state = apply_physical_constraints(new_vehicle_state, vehicle_config);
        
    catch ME
        fprintf('Warning: Vehicle dynamics error: %s\n', ME.message);
        new_vehicle_state = vehicle_state;  % Return unchanged state on error
    end
end

%% Local Dynamics Implementation Functions

function [atmosphere_rho, atmosphere_temp, atmosphere_pressure] = get_atmosphere_properties(altitude)
    %GET_ATMOSPHERE_PROPERTIES Calculate atmospheric properties
    %
    % Uses the same atmosphere model as the original simulation but
    % extracted for modularity (matching helper_functions implementation)
    
    % US Standard Atmosphere constants
    rho_0 = 1.225; T_0 = 288.15; P_0 = 101325; L = 0.0065; R = 287; g = 9.81;
    
    if altitude <= 11000
        % Troposphere (0-11 km)
        atmosphere_temp = T_0 - L * altitude;
        atmosphere_pressure = P_0 * (atmosphere_temp / T_0)^(g / (R * L));
        atmosphere_rho = atmosphere_pressure / (R * atmosphere_temp);
    else
        % Lower Stratosphere (11+ km)
        atmosphere_temp = 216.65;
        atmosphere_pressure = P_0 * 0.2234 * exp(-g * (altitude - 11000) / (R * atmosphere_temp));
        atmosphere_rho = atmosphere_pressure / (R * atmosphere_temp);
    end
    
    % Add realistic variations (reduced for stability)
    atmosphere_rho = atmosphere_rho * (1 + 0.02 * randn);
    atmosphere_temp = atmosphere_temp * (1 + 0.01 * randn);
    
    % Ensure physical constraints
    atmosphere_rho = max(atmosphere_rho, 1e-6);
    atmosphere_temp = max(atmosphere_temp, 180);
    atmosphere_pressure = max(atmosphere_pressure, 1);
end

function [relative_velocity, thermal_factor] = apply_environmental_effects(velocity, position, environment, atmosphere)
    %APPLY_ENVIRONMENTAL_EFFECTS Calculate wind and thermal effects
    %
    % This implements the environmental effects from the original simulation
    % (lines 300-320) including wind, turbulence, and thermal effects
    
    % Wind effects (from original lines 305-310)
    wind_velocity = environment.wind_speed + ...
                   environment.turbulence_intensity * randn(3,1) * 10;
    relative_velocity = velocity - wind_velocity;
    
    % Thermal effects calculation (from original lines 315-325)
    mach_number = norm(relative_velocity) / sqrt(1.4 * 287 * atmosphere.temp);
    
    % Heating effects reduce vehicle performance (matching original)
    thermal_factor = 1 + 0.3 * max(0, mach_number - 3);  % Reduced thermal effects
    
    % Ensure reasonable thermal factor bounds
    thermal_factor = max(1.0, min(3.0, thermal_factor));
end

function total_acceleration = calculate_total_acceleration(guidance_command, relative_velocity, atmosphere, vehicle_config, thermal_factor)
    %CALCULATE_TOTAL_ACCELERATION Compute all forces acting on vehicle
    %
    % This implements the force calculation from the original simulation
    % (lines 525-560) with realistic aerodynamic modeling
    
    % Gravitational acceleration (constant for simulation timeframe)
    gravity = [0; 0; -9.81];
    
    % Aerodynamic drag calculation (from original lines 535-550)
    speed = norm(relative_velocity);
    if speed > 0
        % Realistic drag coefficient progression (matching original)
        mach = speed / sqrt(1.4 * 287 * atmosphere.temp);
        
        if mach < 1
            drag_coeff = 0.3;
        elseif mach < 3
            drag_coeff = 0.5 + 0.1 * (mach - 1);  % Transonic/supersonic drag rise
        else
            drag_coeff = 0.4 + 0.05 * min(mach - 3, 2);  % Hypersonic drag
        end
        
        % Drag force calculation
        drag_force = -0.5 * atmosphere.rho * speed^2 * vehicle_config.reference_area * ...
                    drag_coeff * (relative_velocity / speed);
        drag_acceleration = drag_force / vehicle_config.mass;
    else
        drag_acceleration = zeros(3,1);
    end
    
    % Thermal effects on guidance authority (from original)
    thermal_guidance_command = guidance_command / thermal_factor;
    
    % Total acceleration
    total_acceleration = thermal_guidance_command + gravity + drag_acceleration;
    
    % Realistic acceleration limits for hypersonic vehicle (from original lines 555-560)
    max_total_accel = 200;  % m/s² (20g total limit)
    if norm(total_acceleration) > max_total_accel
        total_acceleration = total_acceleration * (max_total_accel / norm(total_acceleration));
    end
end

function [new_attitude, new_ang_rates] = update_attitude_dynamics(attitude, ang_rates, velocity, new_velocity, vehicle_config, dt)
    %UPDATE_ATTITUDE_DYNAMICS Update vehicle attitude and angular motion
    %
    % This implements the attitude dynamics from the original simulation
    % (lines 565-580) with simplified but realistic attitude control
    
    % Extract current attitude components
    roll = attitude(1);
    pitch = attitude(2);
    yaw = attitude(3);
    
    p = ang_rates(1);  % Roll rate
    q = ang_rates(2);  % Pitch rate
    r = ang_rates(3);  % Yaw rate
    
    % Simplified attitude dynamics (matching original approach)
    if norm(new_velocity) > 0
        % Desired attitude: vehicle points toward velocity vector
        desired_pitch = atan2(new_velocity(3), norm(new_velocity(1:2)));
        desired_yaw = atan2(new_velocity(2), new_velocity(1));
        
        % Attitude error calculation
        pitch_error = desired_pitch - pitch;
        yaw_error = desired_yaw - yaw;
        
        % Simple attitude control (from original lines 575-580)
        q_command = -2.0 * pitch_error;  % Pitch rate command
        r_command = -1.0 * yaw_error;    % Yaw rate command (reduced gain)
        
        % Update angular rates with control authority limits
        max_rate = deg2rad(30);  % Maximum angular rate
        q = max(-max_rate, min(max_rate, q_command));
        r = max(-max_rate, min(max_rate, r_command));
        
        % Maintain roll stability (assume roll control)
        p = -0.5 * roll;  % Roll damping
    end
    
    % Integrate attitude (simple Euler integration)
    new_roll = roll + p * dt;
    new_pitch = pitch + q * dt;
    new_yaw = yaw + r * dt;
    
    % Wrap angles to reasonable bounds
    new_roll = wrap_angle(new_roll);
    new_pitch = max(-pi/2, min(pi/2, new_pitch));  % Limit pitch
    new_yaw = wrap_angle(new_yaw);
    
    new_attitude = [new_roll; new_pitch; new_yaw];
    new_ang_rates = [p; q; r];
end

function new_state = apply_physical_constraints(vehicle_state, vehicle_config)
    %APPLY_PHYSICAL_CONSTRAINTS Enforce physical and vehicle constraints
    %
    % Ensures the vehicle state remains within realistic bounds
    
    new_state = vehicle_state;
    
    % Extract components
    position = new_state(1:3);
    velocity = new_state(4:6);
    attitude = new_state(7:9);
    ang_rates = new_state(10:12);
    
    % Altitude constraint (prevent ground penetration)
    if position(3) < 50  % Minimum altitude above ground
        position(3) = 50;
        velocity(3) = max(0, velocity(3));  % Prevent downward velocity
    end
    
    % Maximum altitude constraint (for simulation realism)
    if position(3) > 50000  % Maximum operational altitude
        position(3) = 50000;
        velocity(3) = min(0, velocity(3));  % Prevent further climb
    end
    
    % Speed constraints
    max_speed = 2000;  % m/s maximum speed (Mach 6+)
    speed = norm(velocity);
    if speed > max_speed
        velocity = velocity * (max_speed / speed);
    end
    
    % Angular rate constraints
    max_ang_rate = deg2rad(45);  % Maximum angular rate
    ang_rates = max(-max_ang_rate, min(max_ang_rate, ang_rates));
    
    % Attitude constraints (prevent unrealistic attitudes)
    attitude(2) = max(-deg2rad(60), min(deg2rad(60), attitude(2)));  % Pitch limits
    
    % Update state vector
    new_state = [position; velocity; attitude; ang_rates];
end

function wrapped_angle = wrap_angle(angle)
    %WRAP_ANGLE Wrap angle to [-pi, pi] range
    wrapped_angle = atan2(sin(angle), cos(angle));
end