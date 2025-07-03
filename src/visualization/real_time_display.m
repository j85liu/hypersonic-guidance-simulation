function varargout = real_time_display(action, varargin)
%REAL_TIME_DISPLAY Real-time 3D visualization for hypersonic guidance simulation
%
% This function handles all real-time visualization functionality extracted
% from the main simulation comprehensive code (lines 150-250, 550-680).
% Provides professional 3D visualization with terrain, threats, and live data.
%
% Usage:
%   handles = real_time_display('initialize', enable_flag)
%   real_time_display('update', handles, vehicle_state, target_state, ...)
%   real_time_display('cleanup', handles)
%
% Actions:
%   'initialize' - Create and setup 3D visualization window
%   'update' - Update all visual elements with current simulation state
%   'cleanup' - Close visualization and cleanup resources
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    switch action
        case 'initialize'
            varargout{1} = initialize_display(varargin{:});
            
        case 'update'
            update_display(varargin{:});
            
        case 'cleanup'
            cleanup_display(varargin{:});
            
        otherwise
            error('Unknown action: %s', action);
    end
end

%% Local Implementation Functions

function viz_handles = initialize_display(enable_real_time_viz)
    %INITIALIZE_DISPLAY Setup 3D visualization window and all visual elements
    %
    % Inputs:
    %   enable_real_time_viz - Boolean flag to enable/disable visualization
    %
    % Outputs:
    %   viz_handles - Structure containing all visualization handles
    
    viz_handles = struct();
    
    if ~enable_real_time_viz
        viz_handles.enabled = false;
        fprintf('Real-time visualization disabled\n');
        return;
    end
    
    viz_handles.enabled = true;
    
    try
        % Create main figure (matching original lines 150-170)
        viz_handles.fig = figure('Name', 'Fixed Comprehensive Hypersonic Simulation', ...
                                'Position', [50, 50, 1400, 900], 'Color', 'black');
        
        % Main 3D plot setup (matching original lines 172-185)
        viz_handles.ax = axes('Position', [0.05, 0.05, 0.65, 0.9]);
        hold on; grid on; axis equal;
        view(45, 25);
        
        % Set viewing bounds (from original)
        xlim([-5000, 85000]); ylim([-15000, 20000]); zlim([0, 30000]);
        
        % Styling (matching original black theme)
        set(viz_handles.ax, 'Color', 'k', 'GridColor', 'w', 'GridAlpha', 0.3);
        xlabel('Range (m)', 'Color', 'w', 'FontSize', 12);
        ylabel('Cross-Range (m)', 'Color', 'w', 'FontSize', 12);
        zlabel('Altitude (m)', 'Color', 'w', 'FontSize', 12);
        title('Fixed Advanced Hypersonic Guidance Simulation', 'Color', 'w', 'FontSize', 14);
        
        % Create terrain (matching original lines 187-192)
        [X_terrain, Y_terrain] = meshgrid(-5000:8000:85000, -15000:6000:20000);
        Z_terrain = 100 + 50*sin(X_terrain/15000) + 30*cos(Y_terrain/12000);
        viz_handles.terrain = surf(X_terrain, Y_terrain, Z_terrain, ...
                                  'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
                                  'FaceColor', [0.3, 0.2, 0.1]);
        
        % Initialize vehicle plots (matching original lines 194-199)
        viz_handles.vehicle_plot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
        viz_handles.vehicle_trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 2);
        
        % Initialize target plots (matching original lines 201-204)
        viz_handles.target_plot = plot3(0, 0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'yellow');
        viz_handles.target_trail = plot3(nan, nan, nan, 'y--', 'LineWidth', 1.5);
        
        % Initialize threat visualization arrays
        viz_handles.sam_plots = [];
        viz_handles.threat_circles = [];
        
        % Info panel setup (matching original lines 212-220)
        viz_handles.info_panel = uipanel('Parent', viz_handles.fig, ...
                                        'Position', [0.72, 0.05, 0.27, 0.9], ...
                                        'BackgroundColor', 'black', 'ForegroundColor', 'white');
        
        viz_handles.info_text = uicontrol('Parent', viz_handles.info_panel, 'Style', 'text', ...
                                         'Position', [10, 10, 350, 750], ...
                                         'BackgroundColor', 'black', 'ForegroundColor', 'white', ...
                                         'FontSize', 9, 'HorizontalAlignment', 'left');
        
        fprintf('✅ Real-time 3D visualization initialized\n');
        
    catch ME
        fprintf('❌ Error initializing visualization: %s\n', ME.message);
        viz_handles.enabled = false;
    end
end

function update_display(viz_handles, vehicle_state, target_state, sensor_status, ...
                       threat_history, threat_sites, guidance_command, current_time, ...
                       k, pos_history, target_history, guidance_source, ...
                       vel_history, ins_error_history, ekf_error_history, ...
                       range_history, fuel_consumption, plasma_interference, ...
                       gps_available, active_threats)
    %UPDATE_DISPLAY Update all visual elements with current simulation state
    %
    % This function contains the real-time update logic from the main
    % simulation loop (lines 550-680 in original code)
    
    if ~viz_handles.enabled || ~isvalid(viz_handles.fig)
        return;
    end
    
    % Only update every 5 steps for performance (matching original)
    if mod(k, 5) ~= 0
        return;
    end
    
    try
        % Extract current states
        position = vehicle_state(1:3);
        velocity = vehicle_state(4:6);
        target_position = target_state(1:3);
        
        % Update vehicle position (matching original lines 554-556)
        set(viz_handles.vehicle_plot, 'XData', position(1), 'YData', position(2), 'ZData', position(3));
        
        % Update target position (matching original lines 558-560)
        set(viz_handles.target_plot, 'XData', target_position(1), 'YData', target_position(2), ...
            'ZData', target_position(3));
        
        % Update trails (matching original lines 562-569)
        trail_length = min(100, k);
        trail_indices = max(1, k-trail_length+1):k;
        
        if length(trail_indices) > 1
            set(viz_handles.vehicle_trail, 'XData', pos_history(1, trail_indices), ...
                'YData', pos_history(2, trail_indices), 'ZData', pos_history(3, trail_indices));
            set(viz_handles.target_trail, 'XData', target_history(1, trail_indices), ...
                'YData', target_history(2, trail_indices), 'ZData', target_history(3, trail_indices));
        end
        
        % Update SAM sites and threat circles (matching original lines 571-586)
        update_threat_display(viz_handles, threat_sites, threat_history, k);
        
        % Update info panel with comprehensive data (matching original lines 588-640)
        update_info_panel(viz_handles, current_time, velocity, position, range_history, k, ...
                         guidance_source, ins_error_history, ekf_error_history, sensor_status, ...
                         target_position, active_threats, gps_available, plasma_interference, ...
                         guidance_command, fuel_consumption);
        
        % Update camera to follow action (matching original lines 642-647)
        if mod(k, 20) == 0
            midpoint = (position + target_position) / 2;
            campos(midpoint' + [-15000, -10000, 8000]);
            camtarget(midpoint');
        end
        
        % Force graphics update
        drawnow limitrate;
        
    catch ME
        fprintf('Warning: Visualization update error: %s\n', ME.message);
    end
end

function update_threat_display(viz_handles, threat_sites, threat_history, k)
    %UPDATE_THREAT_DISPLAY Update SAM sites and threat circles visualization
    
    % Ensure we have enough plot handles for all threats
    num_threats = size(threat_sites, 1);
    while length(viz_handles.sam_plots) < num_threats
        % Add new SAM site plot
        viz_handles.sam_plots(end+1) = plot3(viz_handles.ax, nan, nan, nan, '^g', ...
                                            'MarkerSize', 10, 'MarkerFaceColor', 'green');
        % Add new threat circle plot
        viz_handles.threat_circles(end+1) = plot3(viz_handles.ax, nan, nan, nan, 'g-', 'LineWidth', 2);
    end
    
    % Update each threat site
    for i = 1:num_threats
        site = threat_sites(i, :);
        
        % Update SAM site position
        set(viz_handles.sam_plots(i), 'XData', site(1), 'YData', site(2), 'ZData', site(3));
        
        % Update threat circle if active
        if i <= size(threat_history, 1) && threat_history(i, k) == 1
            theta = 0:0.2:2*pi;
            circle_x = site(1) + site(4) * cos(theta);
            circle_y = site(2) + site(4) * sin(theta);
            circle_z = ones(size(theta)) * site(3);
            set(viz_handles.threat_circles(i), 'XData', circle_x, 'YData', circle_y, 'ZData', circle_z);
        else
            set(viz_handles.threat_circles(i), 'XData', nan, 'YData', nan, 'ZData', nan);
        end
    end
end

function update_info_panel(viz_handles, current_time, velocity, position, range_history, k, ...
                          guidance_source, ins_error_history, ekf_error_history, sensor_status, ...
                          target_position, active_threats, gps_available, plasma_interference, ...
                          guidance_command, fuel_consumption)
    %UPDATE_INFO_PANEL Update the information panel with current status
    %
    % This matches the original info panel update from lines 588-640
    
    % Calculate atmospheric properties for Mach number
    [~, temperature, ~] = get_atmosphere_simple(position(3));
    mach_number = norm(velocity) / sqrt(1.4 * 287 * temperature);
    
    % Create sensor status string (matching original format)
    if size(sensor_status, 1) >= 5 && k <= size(sensor_status, 2)
        sensor_status_text = sprintf('INS:%s GPS:%s TER:%s LAS:%s IR:%s', ...
            bool_to_status_simple(sensor_status(1,k)), bool_to_status_simple(sensor_status(2,k)), ...
            bool_to_status_simple(sensor_status(3,k)), bool_to_status_simple(sensor_status(4,k)), ...
            bool_to_status_simple(sensor_status(5,k)));
    else
        sensor_status_text = 'Sensor data unavailable';
    end
    
    % Calculate navigation improvement
    if k <= length(ins_error_history) && k <= length(ekf_error_history)
        improvement = get_improvement_percent_simple(ins_error_history(k), ekf_error_history(k));
    else
        improvement = 0;
    end
    
    % Get guidance mode text
    if k <= length(range_history)
        guidance_mode = guidance_mode_text_simple(range_history(k), 15000);
    else
        guidance_mode = 'Unknown';
    end
    
    % Create comprehensive info string (matching original format exactly)
    info_string = sprintf([
        'FIXED COMPREHENSIVE HYPERSONIC SIM\n\n'...
        'FLIGHT STATUS:\n'...
        'Time: %.1f s (%.1f min)\n'...
        'Speed: %.0f m/s (Mach %.2f)\n'...
        'Altitude: %.1f km\n'...
        'Range: %.1f km\n\n'...
        'NAVIGATION:\n'...
        'Source: %s\n'...
        'INS Error: %.1f m\n'...
        'EKF Error: %.1f m\n'...
        'Improvement: %.1f%%\n\n'...
        'SENSORS:\n'...
        '%s\n\n'...
        'TARGET:\n'...
        'Position: [%.0f, %.0f, %.0f]\n'...
        'Velocity: %.1f m/s\n\n'...
        'THREATS:\n'...
        'Active: %d SAM sites\n'...
        'GPS Jamming: %s\n'...
        'Plasma Effects: %.1f%%\n\n'...
        'GUIDANCE:\n'...
        'Mode: %s\n'...
        'Command: [%.0f, %.0f, %.0f] m/s²\n'...
        'Fuel Used: %.1f kg'
        ], ...
        current_time, current_time/60, norm(velocity), mach_number, position(3)/1000, ...
        range_history(k)/1000, guidance_source, ...
        ins_error_history(k), ekf_error_history(k), improvement, ...
        sensor_status_text, target_position(1), target_position(2), target_position(3), ...
        norm([25; 15; 0]), active_threats, bool_to_status_simple(~gps_available), ...
        plasma_interference*100, guidance_mode, ...
        guidance_command(1), guidance_command(2), guidance_command(3), fuel_consumption);
    
    set(viz_handles.info_text, 'String', info_string);
end

function cleanup_display(viz_handles)
    %CLEANUP_DISPLAY Close visualization and cleanup resources
    
    if ~viz_handles.enabled
        return;
    end
    
    try
        if isfield(viz_handles, 'fig') && isvalid(viz_handles.fig)
            close(viz_handles.fig);
            fprintf('✅ Visualization window closed\n');
        end
    catch ME
        fprintf('Warning: Error closing visualization: %s\n', ME.message);
    end
end

%% Local Helper Functions (Simplified versions to avoid circular dependencies)

function status = bool_to_status_simple(bool_val)
    if bool_val
        status = 'ON';
    else
        status = 'OFF';
    end
end

function mode_text = guidance_mode_text_simple(range, terminal_range)
    if range > terminal_range
        mode_text = 'Mid-Course (APN)';
    else
        mode_text = 'Terminal (Lead)';
    end
end

function improvement = get_improvement_percent_simple(ins_error, ekf_error)
    if ins_error > 0.1
        improvement = (ins_error - ekf_error) / ins_error * 100;
        improvement = max(-100, min(100, improvement));
    else
        improvement = 0;
    end
end

function [rho, temperature, pressure] = get_atmosphere_simple(altitude)
    % Simplified atmosphere model for visualization
    rho_0 = 1.225; T_0 = 288.15; P_0 = 101325; L = 0.0065; R = 287; g = 9.81;
    
    if altitude <= 11000
        temperature = T_0 - L * altitude;
        pressure = P_0 * (temperature / T_0)^(g / (R * L));
        rho = pressure / (R * temperature);
    else
        temperature = 216.65;
        pressure = P_0 * 0.2234 * exp(-g * (altitude - 11000) / (R * temperature));
        rho = pressure / (R * temperature);
    end
end