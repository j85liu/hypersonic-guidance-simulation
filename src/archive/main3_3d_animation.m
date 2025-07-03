%% Real-Time 3D Hypersonic Missile Simulation with Animation
% Author: James Liu
% Interactive visualization of hypersonic missile guidance with moving target
% Features: Real-time 3D animation, missile trail, interceptor threats, terrain

clear; close all; clc;

%% Animation Parameters
fprintf('=== Real-Time 3D Hypersonic Missile Simulation ===\n');
fprintf('Features: Moving target, interceptor threats, terrain visualization\n\n');

% Animation settings
dt = 0.05;              % Time step for smooth animation (50ms)
t_final = 60;           % Simulation time
time = 0:dt:t_final;
N = length(time);

% Animation speed control
animation_speed = 2;    % 1 = real-time, 2 = 2x speed, 0.5 = slow motion
pause_time = dt / animation_speed;

%% Scenario Setup
% Missile initial conditions
missile_pos = [80000; 10000; 20000];    % Start 80km away, 20km altitude
missile_vel = [-600; -50; -30];         % Hypersonic speed toward target

% Moving target
target_pos = [0; 0; 0];                 % Initial target position
target_vel = [20; 15; 0];               % Target moving at 20 m/s

% Interceptor threat locations (multiple SAM sites)
interceptor_sites = [
    30000,  5000, 100;     % SAM Site 1
    45000, -8000, 150;     % SAM Site 2
    25000, 12000, 200;     % SAM Site 3
];

% Vehicle parameters
mass = 1500;
reference_area = 0.3;

%% Initialize Figure and 3D Environment
fig = figure('Name', 'Real-Time Hypersonic Missile Simulation', ...
             'Position', [100, 100, 1200, 800], ...
             'Color', 'black');

% Create 3D axis
ax = axes('Position', [0.05, 0.05, 0.7, 0.9]);
hold on; grid on;
axis equal;
view(45, 20);

% Set axis limits for good viewing
xlim([-5000, 85000]);
ylim([-15000, 20000]);
zlim([0, 25000]);

% Styling
set(ax, 'Color', 'k', 'GridColor', 'w', 'GridAlpha', 0.3);
xlabel('Range (m)', 'Color', 'w', 'FontSize', 12);
ylabel('Cross-Range (m)', 'Color', 'w', 'FontSize', 12);
zlabel('Altitude (m)', 'Color', 'w', 'FontSize', 12);
title('Hypersonic Missile Guidance Simulation', 'Color', 'w', 'FontSize', 14);

%% Create Terrain Surface
[X_terrain, Y_terrain] = meshgrid(-5000:5000:85000, -15000:5000:20000);
Z_terrain = 50 + 20*sin(X_terrain/10000) + 10*cos(Y_terrain/8000);  % Rolling hills
terrain_surface = surf(X_terrain, Y_terrain, Z_terrain, 'FaceAlpha', 0.3, ...
                      'EdgeColor', 'none', 'FaceColor', [0.4, 0.2, 0.1]);

%% Initialize Moving Objects
% Missile representation
missile_plot = plot3(missile_pos(1), missile_pos(2), missile_pos(3), ...
                    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');

% Target representation  
target_plot = plot3(target_pos(1), target_pos(2), target_pos(3), ...
                   'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'yellow');

% Missile trail
trail_length = min(200, N);  % Show last 200 points or all points
missile_trail = plot3(nan, nan, nan, 'r-', 'LineWidth', 2, 'Color', [1, 0.3, 0.3]);

% Target trail
target_trail = plot3(nan, nan, nan, 'y--', 'LineWidth', 1.5, 'Color', [1, 1, 0.5]);

% Interceptor sites
interceptor_plots = [];
for i = 1:size(interceptor_sites, 1)
    site = interceptor_sites(i, :);
    interceptor_plots(i) = plot3(site(1), site(2), site(3), ...
                                '^g', 'MarkerSize', 12, 'MarkerFaceColor', 'green');
end

% Threat engagement zones (will be activated when in range)
threat_circles = [];
for i = 1:size(interceptor_sites, 1)
    threat_circles(i) = plot3(nan, nan, nan, 'g-', 'LineWidth', 2);
end

%% Create Information Panel
info_panel = uipanel('Parent', fig, 'Position', [0.77, 0.05, 0.22, 0.9], ...
                    'BackgroundColor', 'black', 'ForegroundColor', 'white');

% Create text displays
info_text = uicontrol('Parent', info_panel, 'Style', 'text', ...
                     'Position', [10, 10, 250, 600], ...
                     'BackgroundColor', 'black', 'ForegroundColor', 'white', ...
                     'FontSize', 10, 'HorizontalAlignment', 'left');

%% Storage Arrays for Analysis
pos_history = zeros(3, N);
target_history = zeros(3, N);
vel_history = zeros(3, N);
range_history = zeros(1, N);
threat_status = zeros(size(interceptor_sites, 1), N);

%% Main Animation Loop
fprintf('Starting real-time animation...\n');
fprintf('Close figure window to end simulation\n\n');

for k = 1:N
    current_time = time(k);
    
    % Check if figure still exists
    if ~isvalid(fig)
        fprintf('Simulation ended by user\n');
        break;
    end
    
    %% Update Target Motion (Evasive Maneuvers)
    % Target performs evasive maneuvers every 10 seconds
    if mod(current_time, 10) < dt
        % Random course change
        target_vel = target_vel + [randn*5; randn*5; 0];
        target_vel = target_vel * (25 / norm(target_vel));  % Maintain speed ~25 m/s
    end
    
    % Update target position
    target_pos = target_pos + target_vel * dt;
    
    %% Calculate Missile Guidance
    relative_position = target_pos - missile_pos;
    range_to_target = norm(relative_position);
    
    if range_to_target > 100  % Continue guidance until close
        % Proportional navigation with lead angle
        los_vector = relative_position / range_to_target;
        
        % Advanced guidance: predict target position
        time_to_intercept = range_to_target / norm(missile_vel);
        predicted_target_pos = target_pos + target_vel * time_to_intercept;
        guidance_vector = (predicted_target_pos - missile_pos) / norm(predicted_target_pos - missile_pos);
        
        guidance_command = 150.0 * guidance_vector;
    else
        guidance_command = [0; 0; 0];
    end
    
    %% Check Interceptor Threats
    threat_engaged = false;
    for i = 1:size(interceptor_sites, 1)
        site_pos = interceptor_sites(i, :)';
        range_to_site = norm(missile_pos - site_pos);
        
        % SAM engagement range
        sam_range = 25000;  % 25km engagement range
        
        if range_to_site < sam_range
            threat_status(i, k) = 1;
            threat_engaged = true;
            
            % Draw threat engagement circle
            theta = 0:0.1:2*pi;
            circle_x = site_pos(1) + sam_range * cos(theta);
            circle_y = site_pos(2) + sam_range * sin(theta);
            circle_z = ones(size(theta)) * site_pos(3);
            
            set(threat_circles(i), 'XData', circle_x, 'YData', circle_y, 'ZData', circle_z);
        else
            set(threat_circles(i), 'XData', nan, 'YData', nan, 'ZData', nan);
        end
    end
    
    %% Physics Update
    if k < N
        % Environmental forces
        gravity = [0; 0; -9.81];
        
        % Atmospheric effects
        [rho, ~, ~] = atmosphere_model(missile_pos(3));
        speed = norm(missile_vel);
        if speed > 0
            drag_coeff = 0.4;
            drag_force = -0.5 * rho * speed^2 * reference_area * drag_coeff * (missile_vel/speed);
            drag_acceleration = drag_force / mass;
        else
            drag_acceleration = [0; 0; 0];
        end
        
        % Evasive maneuver if under threat
        if threat_engaged
            evasive_command = [randn*50; randn*50; randn*25];  % Random evasive maneuver
            guidance_command = guidance_command + evasive_command;
        end
        
        % Total acceleration
        total_acceleration = guidance_command + gravity + drag_acceleration;
        
        % Limit acceleration (realistic constraint)
        max_accel = 200;  % m/s²
        if norm(total_acceleration) > max_accel
            total_acceleration = total_acceleration * (max_accel / norm(total_acceleration));
        end
        
        % Update missile state
        missile_vel = missile_vel + total_acceleration * dt;
        missile_pos = missile_pos + missile_vel * dt;
        
        % Ground impact check
        if missile_pos(3) <= 100
            fprintf('Ground impact at t=%.1f s\n', current_time);
            break;
        end
        
        % Target hit check
        if range_to_target < 50
            fprintf('Target hit at t=%.1f s!\n', current_time);
            fprintf('Final miss distance: %.1f m\n', range_to_target);
            break;
        end
    end
    
    %% Store Data
    pos_history(:, k) = missile_pos;
    target_history(:, k) = target_pos;
    vel_history(:, k) = missile_vel;
    range_history(k) = range_to_target;
    
    %% Update Visualization
    % Update missile position
    set(missile_plot, 'XData', missile_pos(1), 'YData', missile_pos(2), 'ZData', missile_pos(3));
    
    % Update target position
    set(target_plot, 'XData', target_pos(1), 'YData', target_pos(2), 'ZData', target_pos(3));
    
    % Update missile trail
    trail_start = max(1, k - trail_length + 1);
    trail_x = pos_history(1, trail_start:k);
    trail_y = pos_history(2, trail_start:k);
    trail_z = pos_history(3, trail_start:k);
    set(missile_trail, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);
    
    % Update target trail
    target_trail_x = target_history(1, trail_start:k);
    target_trail_y = target_history(2, trail_start:k);
    target_trail_z = target_history(3, trail_start:k);
    set(target_trail, 'XData', target_trail_x, 'YData', target_trail_y, 'ZData', target_trail_z);
    
    %% Update Information Panel
    mach_number = norm(missile_vel) / 343;
    altitude_km = missile_pos(3) / 1000;
    range_km = range_to_target / 1000;
    
    info_string = sprintf([
        'HYPERSONIC MISSILE SIMULATION\n\n'...
        'Time: %.1f s\n'...
        'Missile Speed: %.0f m/s\n'...
        'Mach Number: %.2f\n'...
        'Altitude: %.1f km\n'...
        'Range to Target: %.1f km\n\n'...
        'Target Speed: %.1f m/s\n'...
        'Target Position:\n'...
        '  X: %.0f m\n'...
        '  Y: %.0f m\n\n'...
        'Threat Status:\n'...
        'SAM Site 1: %s\n'...
        'SAM Site 2: %s\n'...
        'SAM Site 3: %s\n\n'...
        'Guidance: %s\n'...
        'Evasive Action: %s'
        ], ...
        current_time, norm(missile_vel), mach_number, altitude_km, range_km, ...
        norm(target_vel), target_pos(1), target_pos(2), ...
        threat_status_text(threat_status(1, k)), ...
        threat_status_text(threat_status(2, k)), ...
        threat_status_text(threat_status(3, k)), ...
        guidance_status_text(range_to_target), ...
        evasive_status_text(threat_engaged));
    
    set(info_text, 'String', info_string);
    
    %% Camera Control (Optional - follow missile)
    if mod(k, 10) == 0  % Update camera every 10 frames for smooth motion
        % Camera follows missile with offset
        cam_pos = missile_pos + [-10000; -5000; 5000];
        campos(cam_pos');
        camtarget(missile_pos');
    end
    
    %% Refresh Display
    drawnow limitrate;  % Efficient drawing update
    
    % Control animation speed
    if pause_time > 0
        pause(pause_time);
    end
    
    %% Progress indicator
    if mod(k, round(N/10)) == 0
        fprintf('Animation progress: %.0f%% | Range: %.1f km | Mach: %.2f\n', ...
                100*k/N, range_km, mach_number);
    end
end

%% Post-Simulation Analysis
fprintf('\n=== SIMULATION COMPLETE ===\n');
if exist('range_to_target', 'var')
    fprintf('Final range to target: %.1f m\n', range_to_target);
    fprintf('Flight time: %.1f seconds\n', current_time);
    fprintf('Average speed: %.1f m/s (Mach %.2f)\n', ...
            mean(sqrt(sum(vel_history.^2, 1))), ...
            mean(sqrt(sum(vel_history.^2, 1)))/343);
end

% Keep final view
fprintf('Simulation window will remain open for analysis\n');
fprintf('Close window manually when finished\n');

%% Helper Functions
function status_text = threat_status_text(status)
    if status == 1
        status_text = 'ENGAGED';
    else
        status_text = 'Clear';
    end
end

function status_text = guidance_status_text(range)
    if range > 1000
        status_text = 'Terminal Guidance';
    elseif range > 100
        status_text = 'Final Approach';
    else
        status_text = 'Impact Imminent';
    end
end

function status_text = evasive_status_text(engaged)
    if engaged
        status_text = 'ACTIVE';
    else
        status_text = 'None';
    end
end

function [rho, temperature, pressure] = atmosphere_model(altitude)
    % US Standard Atmosphere
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