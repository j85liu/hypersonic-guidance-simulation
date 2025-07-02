function [vehicle, initial_state] = vehicle_config()
%VEHICLE_CONFIG Hypersonic glide vehicle configuration and initial conditions
%
% This function contains all vehicle parameters and initial state extracted
% from the main simulation comprehensive code (lines 47-65). Provides
% realistic hypersonic glide vehicle properties and starting conditions.
%
% Outputs:
%   vehicle - Structure containing vehicle physical properties
%   initial_state - Initial 12-DOF vehicle state vector
%
% Vehicle Properties:
%   - Mass and geometric properties
%   - Aerodynamic characteristics
%   - Control authority and limits
%   - Performance constraints
%
% Initial State Format:
%   [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    %% Vehicle Physical Properties (from original lines 50-58)
    vehicle = struct();
    
    % Mass properties
    vehicle.mass = 2000;                    % Total vehicle mass (kg)
    vehicle.dry_mass = 1800;                % Dry mass without fuel (kg)
    vehicle.fuel_capacity = 200;            % Maximum fuel capacity (kg)
    vehicle.fuel_initial = 150;             % Initial fuel load (kg)
    
    % Geometric properties
    vehicle.length = 6.5;                   % Overall length (m)
    vehicle.diameter = 0.6;                 % Maximum diameter (m)
    vehicle.span = 2.0;                     % Wing/fin span (m)
    vehicle.reference_area = 0.28;          % Reference area for drag (m²)
    vehicle.wetted_area = 12.5;             % Total wetted area (m²)
    
    % Moments of inertia (kg⋅m²) - realistic for elongated missile
    vehicle.Ixx = 500;                      % Roll inertia
    vehicle.Iyy = 2000;                     % Pitch inertia  
    vehicle.Izz = 2000;                     % Yaw inertia
    vehicle.Ixz = 50;                       % Cross-coupling (typically small)
    
    %% Control System Properties
    vehicle.control = struct();
    
    % Control surface limits
    vehicle.control.max_fin_deflection = deg2rad(20);     % Maximum fin deflection (rad)
    vehicle.control.max_deflection_rate = deg2rad(60);    % Maximum deflection rate (rad/s)
    vehicle.control.control_effectiveness = 0.8;          % Control effectiveness factor
    
    % Control authority (from original lines 55-56)
    vehicle.max_lateral_accel = 150;        % Maximum lateral acceleration (m/s²) - 15g
    vehicle.max_normal_accel = 200;         % Maximum normal acceleration (m/s²) - 20g
    vehicle.max_axial_accel = 50;           % Maximum axial acceleration (m/s²) - 5g
    
    % Angular rate limits
    vehicle.max_roll_rate = deg2rad(90);    % Maximum roll rate (rad/s)
    vehicle.max_pitch_rate = deg2rad(45);   % Maximum pitch rate (rad/s)
    vehicle.max_yaw_rate = deg2rad(45);     % Maximum yaw rate (rad/s)
    
    %% Aerodynamic Properties
    vehicle.aero = struct();
    
    % Drag characteristics
    vehicle.aero.cd_subsonic = 0.3;         % Subsonic drag coefficient
    vehicle.aero.cd_supersonic = 0.5;       % Supersonic drag coefficient
    vehicle.aero.cd_hypersonic = 0.4;       % Hypersonic drag coefficient
    vehicle.aero.drag_area = vehicle.reference_area;  % Effective drag area
    
    % Lift characteristics (simplified)
    vehicle.aero.cl_alpha = 2.5;            % Lift curve slope (per radian)
    vehicle.aero.cl_max = 1.2;              % Maximum lift coefficient
    vehicle.aero.alpha_stall = deg2rad(25); % Stall angle of attack
    
    % Stability derivatives (simplified)
    vehicle.aero.cm_alpha = -0.15;          % Pitch moment derivative
    vehicle.aero.cn_beta = 0.12;            % Yaw moment derivative
    vehicle.aero.cl_beta = -0.08;           % Roll moment derivative
    
    %% Performance Limits
    vehicle.performance = struct();
    
    % Speed limits
    vehicle.performance.max_speed = 2000;            % Maximum speed (m/s) - Mach 6+
    vehicle.performance.min_speed = 100;             % Minimum controllable speed (m/s)
    vehicle.performance.cruise_speed = 600;          % Typical cruise speed (m/s)
    vehicle.performance.terminal_speed = 800;        % Terminal phase speed (m/s)
    
    % Altitude limits
    vehicle.performance.max_altitude = 50000;        % Maximum operational altitude (m)
    vehicle.performance.min_altitude = 50;           % Minimum safe altitude (m)
    vehicle.performance.cruise_altitude = 25000;     % Typical cruise altitude (m)
    
    % Load factor limits
    vehicle.performance.max_load_factor = 20;        % Maximum load factor (g)
    vehicle.performance.min_load_factor = -10;       % Minimum load factor (g)
    
    %% Thermal Properties
    vehicle.thermal = struct();
    
    % Temperature limits
    vehicle.thermal.max_skin_temp = 1500;            % Maximum skin temperature (K)
    vehicle.thermal.max_structure_temp = 800;        % Maximum structure temperature (K)
    vehicle.thermal.operating_temp_range = [200, 350]; % Operating temperature range (K)
    
    % Thermal effects
    vehicle.thermal.thermal_degradation_threshold = 1200;  % Temp for performance loss (K)
    vehicle.thermal.thermal_failure_threshold = 2000;      % Temp for failure (K)
    vehicle.thermal.heat_capacity = 800;                   % Specific heat capacity (J/kg⋅K)
    
    %% Initial Conditions (from original lines 47-53)
    initial_state = struct();
    
    % Initial position (m) - 80km range, 25km altitude
    initial_state.position = [80000; 3000; 25000];
    
    % Initial velocity (m/s) - Mach 1.8 initial (realistic hypersonic entry)
    initial_state.velocity = [-600; -15; -40];
    
    % Initial attitude (rad) - slight nose down for realistic trajectory
    initial_state.attitude = [0; 0.05; 0];  % [roll, pitch, yaw]
    
    % Initial angular rates (rad/s) - starting at rest
    initial_state.angular_rates = [0; 0; 0];  % [p, q, r]
    
    % Combine into 12-DOF state vector
    initial_state.vector = [initial_state.position; 
                           initial_state.velocity; 
                           initial_state.attitude; 
                           initial_state.angular_rates];
    
    %% Sensor Mount Locations (for future sensor modeling)
    vehicle.sensors = struct();
    vehicle.sensors.ins_location = [2.0; 0; 0];         % INS location (m from nose)
    vehicle.sensors.gps_antenna = [1.5; 0; 0.2];        % GPS antenna location
    vehicle.sensors.laser_receiver = [0.5; 0; 0];       % Laser designator receiver
    vehicle.sensors.ir_seeker = [0.2; 0; 0];            % IR seeker location
    
    %% Fuel and Propulsion (for future expansion)
    vehicle.propulsion = struct();
    vehicle.propulsion.fuel_consumption_rate = 2.0;      % Fuel consumption (kg/s)
    vehicle.propulsion.specific_impulse = 300;           % Specific impulse (s)
    vehicle.propulsion.thrust_vectoring = false;         % Thrust vectoring capability
    vehicle.propulsion.max_thrust = 5000;               % Maximum thrust (N)
    
    %% Configuration Validation
    [vehicle, initial_state] = validate_vehicle_config(vehicle, initial_state);
    
    % Display configuration summary
    fprintf('✅ Vehicle configuration loaded:\n');
    fprintf('   - Mass: %.0f kg\n', vehicle.mass);
    fprintf('   - Length: %.1f m\n', vehicle.length);
    fprintf('   - Max acceleration: %.0f m/s² (%.1fg)\n', ...
            vehicle.max_lateral_accel, vehicle.max_lateral_accel/9.81);
    fprintf('   - Initial position: [%.0f, %.0f, %.0f] m\n', initial_state.position);
    fprintf('   - Initial speed: %.0f m/s (Mach %.2f)\n', ...
            norm(initial_state.velocity), norm(initial_state.velocity)/343);
end

%% Local Configuration Functions

function [vehicle, initial_state] = validate_vehicle_config(vehicle, initial_state)
    %VALIDATE_VEHICLE_CONFIG Validate vehicle parameters and initial conditions
    %
    % Ensures all vehicle parameters are physically reasonable and
    % initial conditions are within operational limits
    
    % Validate mass properties
    assert(vehicle.mass > 0 && vehicle.mass < 50000, ...
           'Vehicle mass must be between 0 and 50000 kg');
    assert(vehicle.dry_mass < vehicle.mass, ...
           'Dry mass must be less than total mass');
    
    % Validate geometric properties
    assert(vehicle.length > 0 && vehicle.length < 50, ...
           'Vehicle length must be between 0 and 50 m');
    assert(vehicle.reference_area > 0 && vehicle.reference_area < 100, ...
           'Reference area must be between 0 and 100 m²');
    
    % Validate performance limits
    assert(vehicle.max_lateral_accel > 0 && vehicle.max_lateral_accel < 1000, ...
           'Maximum lateral acceleration must be between 0 and 1000 m/s²');
    
    % Validate initial conditions
    assert(initial_state.position(3) > 0, 'Initial altitude must be positive');
    assert(norm(initial_state.velocity) > 50, 'Initial speed must be > 50 m/s');
    
    % Ensure moments of inertia are positive definite
    assert(vehicle.Ixx > 0 && vehicle.Iyy > 0 && vehicle.Izz > 0, ...
           'All moments of inertia must be positive');
    
    % Check initial state is within vehicle limits
    initial_speed = norm(initial_state.velocity);
    if initial_speed > vehicle.performance.max_speed
        warning('Initial speed exceeds maximum vehicle speed');
        % Scale down velocity to maximum speed
        initial_state.velocity = initial_state.velocity * ...
                                (vehicle.performance.max_speed / initial_speed);
        initial_state.vector(4:6) = initial_state.velocity;
    end
    
    % Ensure initial altitude is within limits
    if initial_state.position(3) > vehicle.performance.max_altitude
        warning('Initial altitude exceeds maximum operational altitude');
        initial_state.position(3) = vehicle.performance.max_altitude;
        initial_state.vector(3) = initial_state.position(3);
    end
    
    % Validate thermal limits
    assert(vehicle.thermal.max_skin_temp > vehicle.thermal.max_structure_temp, ...
           'Skin temperature limit must exceed structure temperature limit');
end