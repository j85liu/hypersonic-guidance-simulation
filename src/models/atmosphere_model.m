function varargout = atmosphere_model(action, varargin)
%ATMOSPHERE_MODEL Comprehensive atmospheric modeling for hypersonic simulation
%
% This function provides atmospheric property calculations extracted and
% enhanced from the main simulation comprehensive code. Includes US Standard
% Atmosphere, environmental effects, and atmospheric variations for realistic
% hypersonic vehicle simulation.
%
% Usage:
%   [rho, temp, pressure] = atmosphere_model('standard', altitude)
%   [rho, temp, pressure] = atmosphere_model('with_variations', altitude)
%   mach = atmosphere_model('mach_number', velocity, altitude)
%   effects = atmosphere_model('environmental_effects', position, velocity, environment)
%   plasma_factor = atmosphere_model('plasma_effects', velocity, altitude)
%
% Actions:
%   'standard' - Basic US Standard Atmosphere
%   'with_variations' - Standard atmosphere with realistic variations
%   'mach_number' - Calculate Mach number at given conditions
%   'environmental_effects' - Comprehensive environmental effects
%   'plasma_effects' - Plasma interference calculation for hypersonic flight
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    switch action
        case 'standard'
            [varargout{1}, varargout{2}, varargout{3}] = standard_atmosphere(varargin{1});
            
        case 'with_variations'
            [varargout{1}, varargout{2}, varargout{3}] = atmosphere_with_variations(varargin{1});
            
        case 'mach_number'
            varargout{1} = calculate_mach_number(varargin{1}, varargin{2});
            
        case 'environmental_effects'
            varargout{1} = calculate_environmental_effects(varargin{1}, varargin{2}, varargin{3});
            
        case 'plasma_effects'
            varargout{1} = calculate_plasma_effects(varargin{1}, varargin{2});
            
        otherwise
            error('Unknown action: %s', action);
    end
end

%% Local Implementation Functions

function [rho, temperature, pressure] = standard_atmosphere(altitude)
    %STANDARD_ATMOSPHERE US Standard Atmosphere implementation
    %
    % This is the clean, base atmospheric model extracted from the original
    % simulation code, without variations for deterministic calculations
    
    % US Standard Atmosphere constants
    rho_0 = 1.225;          % Sea level density (kg/m³)
    T_0 = 288.15;           % Sea level temperature (K)
    P_0 = 101325;           % Sea level pressure (Pa)
    L = 0.0065;             % Temperature lapse rate (K/m)
    R = 287;                % Specific gas constant for air (J/kg·K)
    g = 9.81;               % Gravitational acceleration (m/s²)
    
    % Ensure altitude is within reasonable bounds
    altitude = max(0, min(altitude, 80000));  % 0 to 80 km
    
    if altitude <= 11000
        % Troposphere (0-11 km): Linear temperature decrease
        temperature = T_0 - L * altitude;
        pressure = P_0 * (temperature / T_0)^(g / (R * L));
        rho = pressure / (R * temperature);
        
    elseif altitude <= 20000
        % Lower Stratosphere (11-20 km): Isothermal region
        temperature = 216.65;  % Constant temperature
        h_tropo = 11000;       % Tropopause height
        
        % Pressure at tropopause
        T_tropo = T_0 - L * h_tropo;
        P_tropo = P_0 * (T_tropo / T_0)^(g / (R * L));
        
        % Exponential pressure decrease in isothermal layer
        pressure = P_tropo * exp(-g * (altitude - h_tropo) / (R * temperature));
        rho = pressure / (R * temperature);
        
    else
        % Upper Stratosphere (20+ km): Linear temperature increase
        temperature = 216.65 + 0.001 * (altitude - 20000);  % Simplified heating
        
        % Approximate pressure calculation for upper atmosphere
        pressure = P_0 * 0.054 * exp(-g * (altitude - 20000) / (R * temperature));
        rho = pressure / (R * temperature);
    end
    
    % Ensure physical constraints
    rho = max(rho, 1e-8);           % Minimum density
    temperature = max(temperature, 150);  % Minimum temperature
    pressure = max(pressure, 0.1);  % Minimum pressure
end

function [rho, temperature, pressure] = atmosphere_with_variations(altitude)
    %ATMOSPHERE_WITH_VARIATIONS Standard atmosphere with realistic variations
    %
    % This matches the atmospheric model used in the original simulation
    % (from helper_functions and vehicle_dynamics) with random variations
    
    % Get base standard atmosphere
    [rho_base, temp_base, press_base] = standard_atmosphere(altitude);
    
    % Add realistic atmospheric variations (matching original implementation)
    rho = rho_base * (1 + 0.02 * randn);         % ±2% density variation
    temperature = temp_base * (1 + 0.01 * randn); % ±1% temperature variation
    pressure = press_base * (1 + 0.015 * randn);  % ±1.5% pressure variation
    
    % Ensure physical constraints after variations
    rho = max(rho, 1e-8);
    temperature = max(temperature, 150);
    pressure = max(pressure, 0.1);
end

function mach_number = calculate_mach_number(velocity, altitude)
    %CALCULATE_MACH_NUMBER Compute Mach number at given conditions
    %
    % Calculates Mach number using standard atmosphere conditions
    % This function is used throughout the simulation for aerodynamic calculations
    
    % Get atmospheric temperature at altitude
    [~, temperature, ~] = standard_atmosphere(altitude);
    
    % Calculate speed of sound
    gamma = 1.4;  % Specific heat ratio for air
    R = 287;      % Specific gas constant for air
    sound_speed = sqrt(gamma * R * temperature);
    
    % Calculate Mach number
    speed = norm(velocity);  % Handle vector input
    mach_number = speed / sound_speed;
    
    % Ensure reasonable bounds
    mach_number = max(0, min(mach_number, 20));  % Cap at Mach 20
end

function effects = calculate_environmental_effects(position, velocity, environment)
    %CALCULATE_ENVIRONMENTAL_EFFECTS Comprehensive environmental impact calculation
    %
    % This implements the environmental effects from the original simulation
    % (lines 300-330) in a structured, reusable format
    
    effects = struct();
    
    % Get atmospheric properties at current position
    altitude = position(3);
    [effects.rho, effects.temperature, effects.pressure] = atmosphere_with_variations(altitude);
    
    % Wind effects (from original simulation lines 305-310)
    wind_base = environment.wind_speed;
    turbulence = environment.turbulence_intensity * randn(3,1) * 10;
    effects.wind_velocity = wind_base + turbulence;
    effects.relative_velocity = velocity - effects.wind_velocity;
    
    % Mach number calculation
    effects.mach_number = calculate_mach_number(effects.relative_velocity, altitude);
    
    % Thermal effects (from original simulation lines 315-325)
    if effects.mach_number > 3
        effects.thermal_factor = 1 + 0.3 * (effects.mach_number - 3);
    else
        effects.thermal_factor = 1.0;
    end
    
    % Weather visibility effects (for sensor performance)
    if isfield(environment, 'weather_visibility')
        effects.visibility_factor = min(1.0, environment.weather_visibility / 10000);
    else
        effects.visibility_factor = 1.0;
    end
    
    % Temperature effects on vehicle systems
    if effects.temperature < 220  % Very cold conditions
        effects.system_performance_factor = 0.9;
    elseif effects.temperature > 320  % Very hot conditions  
        effects.system_performance_factor = 0.95;
    else
        effects.system_performance_factor = 1.0;
    end
    
    % Altitude effects on engine/system performance
    if altitude > 30000  % Very high altitude
        effects.altitude_performance_factor = 0.8;
    elseif altitude < 1000  % Very low altitude
        effects.altitude_performance_factor = 0.95;
    else
        effects.altitude_performance_factor = 1.0;
    end
    
    % Combined environmental factor
    effects.overall_performance_factor = effects.system_performance_factor * ...
                                       effects.altitude_performance_factor * ...
                                       effects.visibility_factor;
end

function plasma_interference = calculate_plasma_effects(velocity, altitude)
    %CALCULATE_PLASMA_EFFECTS Calculate plasma interference for hypersonic flight
    %
    % This implements the plasma effects calculation from the original
    % simulation (lines 320-330) for GPS/communication interference
    
    % Calculate Mach number
    mach_number = calculate_mach_number(velocity, altitude);
    
    % Plasma effects occur at high Mach numbers and specific altitudes
    if mach_number > 4 && altitude < 40000
        % Plasma interference increases with Mach number
        plasma_interference = min(0.7, (mach_number - 4) * 0.2);
        
        % Altitude dependency (peak interference around 20-30 km)
        if altitude > 20000 && altitude < 30000
            altitude_factor = 1.0;  % Peak interference
        elseif altitude < 20000
            altitude_factor = altitude / 20000;  % Reduced at low altitude
        else
            altitude_factor = max(0.3, 1 - (altitude - 30000) / 10000);  % Reduced at high altitude
        end
        
        plasma_interference = plasma_interference * altitude_factor;
    else
        plasma_interference = 0;
    end
    
    % Ensure reasonable bounds
    plasma_interference = max(0, min(plasma_interference, 0.9));
end

function drag_coefficient = calculate_drag_coefficient(mach_number, altitude)
    %CALCULATE_DRAG_COEFFICIENT Mach and altitude dependent drag calculation
    %
    % This implements the drag coefficient calculation from the vehicle
    % dynamics with additional altitude effects for enhanced realism
    
    % Base drag coefficient progression (from original simulation)
    if mach_number < 1
        cd_base = 0.3;  % Subsonic
    elseif mach_number < 3
        cd_base = 0.5 + 0.1 * (mach_number - 1);  % Transonic/supersonic drag rise
    else
        cd_base = 0.4 + 0.05 * min(mach_number - 3, 2);  % Hypersonic
    end
    
    % Altitude effects on drag (rarefied gas effects at high altitude)
    if altitude > 40000
        altitude_factor = max(0.7, 1 - (altitude - 40000) / 40000);
    else
        altitude_factor = 1.0;
    end
    
    drag_coefficient = cd_base * altitude_factor;
    
    % Ensure reasonable bounds
    drag_coefficient = max(0.1, min(drag_coefficient, 2.0));
end

function thermal_properties = calculate_thermal_properties(mach_number, altitude)
    %CALCULATE_THERMAL_PROPERTIES Calculate thermal environment effects
    %
    % Enhanced thermal modeling for hypersonic vehicle performance analysis
    
    thermal_properties = struct();
    
    % Stagnation temperature calculation
    [~, static_temp, ~] = standard_atmosphere(altitude);
    gamma = 1.4;
    
    % Stagnation temperature rise
    thermal_properties.stagnation_temp = static_temp * (1 + (gamma - 1) / 2 * mach_number^2);
    
    % Heat transfer coefficient (simplified)
    if mach_number > 3
        thermal_properties.heat_transfer_coeff = 0.1 * mach_number^2;
    else
        thermal_properties.heat_transfer_coeff = 0.05;
    end
    
    % Thermal stress factor (affects vehicle performance)
    if thermal_properties.stagnation_temp > 1000  % High heating
        thermal_properties.stress_factor = min(2.0, thermal_properties.stagnation_temp / 800);
    else
        thermal_properties.stress_factor = 1.0;
    end
    
    % Material property degradation
    if thermal_properties.stagnation_temp > 1500  % Extreme heating
        thermal_properties.material_factor = 0.8;
    elseif thermal_properties.stagnation_temp > 1000
        thermal_properties.material_factor = 0.9;
    else
        thermal_properties.material_factor = 1.0;
    end
end