function varargout = helper_functions(func_name, varargin)
%HELPER_FUNCTIONS Utility functions for hypersonic guidance simulation
%
% This file contains all the small utility functions extracted from the
% main simulation comprehensive code to improve modularity and reusability.
%
% Usage:
%   result = helper_functions('function_name', arg1, arg2, ...)
%
% Available Functions:
%   - bool_to_status(bool_val)
%   - guidance_mode_text(range, terminal_range)
%   - get_improvement_percent(ins_error, ekf_error)
%   - get_atmosphere(altitude)
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    switch func_name
        case 'bool_to_status'
            varargout{1} = bool_to_status_impl(varargin{1});
            
        case 'guidance_mode_text'
            varargout{1} = guidance_mode_text_impl(varargin{1}, varargin{2});
            
        case 'get_improvement_percent'
            varargout{1} = get_improvement_percent_impl(varargin{1}, varargin{2});
            
        case 'get_atmosphere'
            [varargout{1}, varargout{2}, varargout{3}] = get_atmosphere_impl(varargin{1});
            
        otherwise
            error('Unknown function: %s', func_name);
    end
end

%% Local Helper Function Implementations

function status = bool_to_status_impl(bool_val)
    %BOOL_TO_STATUS Convert boolean value to status string
    %
    % Inputs:
    %   bool_val - Boolean value (true/false or 1/0)
    %
    % Outputs:
    %   status - String 'ON' or 'OFF'
    
    if bool_val
        status = 'ON';
    else
        status = 'OFF';
    end
end

function mode_text = guidance_mode_text_impl(range, terminal_range)
    %GUIDANCE_MODE_TEXT Get guidance mode description based on range
    %
    % Inputs:
    %   range - Current range to target (m)
    %   terminal_range - Range threshold for terminal guidance (m)
    %
    % Outputs:
    %   mode_text - String description of current guidance mode
    
    if range > terminal_range
        mode_text = 'Mid-Course (APN)';
    else
        mode_text = 'Terminal (Lead)';
    end
end

function improvement = get_improvement_percent_impl(ins_error, ekf_error)
    %GET_IMPROVEMENT_PERCENT Calculate percentage improvement of EKF over INS
    %
    % Inputs:
    %   ins_error - INS position error (m)
    %   ekf_error - EKF position error (m)
    %
    % Outputs:
    %   improvement - Percentage improvement (positive = EKF better)
    %
    % Notes:
    %   - Handles division by small numbers gracefully
    %   - Clamps result to reasonable range [-100, 100]
    
    if ins_error > 0.1  % Avoid division by very small numbers
        improvement = (ins_error - ekf_error) / ins_error * 100;
        improvement = max(-100, min(100, improvement));  % Clamp to reasonable range
    else
        improvement = 0;
    end
end

function [rho, temperature, pressure] = get_atmosphere_impl(altitude)
    %GET_ATMOSPHERE Calculate atmospheric properties using US Standard Atmosphere
    %
    % Inputs:
    %   altitude - Altitude above sea level (m)
    %
    % Outputs:
    %   rho - Air density (kg/m³)
    %   temperature - Air temperature (K)
    %   pressure - Air pressure (Pa)
    %
    % Notes:
    %   - Uses US Standard Atmosphere model with realistic variations
    %   - Includes random atmospheric variations (±2% density, ±1% temperature)
    %   - Valid for altitudes up to stratosphere
    
    % US Standard Atmosphere constants
    rho_0 = 1.225;          % Sea level density (kg/m³)
    T_0 = 288.15;           % Sea level temperature (K)
    P_0 = 101325;           % Sea level pressure (Pa)
    L = 0.0065;             % Temperature lapse rate (K/m)
    R = 287;                % Specific gas constant for air (J/kg·K)
    g = 9.81;               % Gravitational acceleration (m/s²)
    
    if altitude <= 11000
        % Troposphere (0-11 km)
        temperature = T_0 - L * altitude;
        pressure = P_0 * (temperature / T_0)^(g / (R * L));
        rho = pressure / (R * temperature);
    else
        % Lower Stratosphere (11+ km) - isothermal region
        temperature = 216.65;  % Constant temperature in lower stratosphere
        pressure = P_0 * 0.2234 * exp(-g * (altitude - 11000) / (R * temperature));
        rho = pressure / (R * temperature);
    end
    
    % Add realistic atmospheric variations (reduced from original for stability)
    rho = rho * (1 + 0.02 * randn);         % ±2% atmospheric density variation
    temperature = temperature * (1 + 0.01 * randn);  % ±1% temperature variation
    
    % Ensure physical constraints
    rho = max(rho, 1e-6);           % Minimum density
    temperature = max(temperature, 180);  % Minimum temperature
    pressure = max(pressure, 1);    % Minimum pressure
end