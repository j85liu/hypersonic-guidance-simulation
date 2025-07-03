%% Hypersonic Guidance Simulation - Enhanced Debugging Version
% Author: James Liu
% Detailed debugging to track down Y-direction drift issue
% Focus: Comprehensive logging of all calculations

clear; close all; clc;

%% Basic Parameters
fprintf('=== Enhanced Debug Hypersonic Guidance Simulation ===\n');

% Set random seed for repeatable results
rng(42);

% Time parameters
dt = 0.1;           % Time step (s)
t_final = 100;      % Total simulation time (s)
time = 0:dt:t_final;
N = length(time);

% Vehicle initial conditions (simple scenario)
position = [20000; 0; 10000];        % Start 20km away, 10km altitude
velocity = [-500; 0; -20];           % Flying toward target at Mach ~1.5
target_position = [0; 0; 0];         % Target at origin

% Vehicle parameters
mass = 1000;                         % kg
reference_area = 0.2;                % m^2

% Navigation system parameters (GPS-denied)
nav_position = position;             % INS estimated position (starts accurate)
nav_velocity = velocity;             % INS estimated velocity  
position_drift = [0; 0; 0];          % Accumulated position error
drift_rate = 0.01;                  % Reduced drift rate for debugging

% Storage arrays
pos_history = zeros(3, N);
vel_history = zeros(3, N);
cmd_history = zeros(3, N);
range_history = zeros(1, N);
nav_pos_history = zeros(3, N);
nav_error_history = zeros(1, N);
drift_history = zeros(3, N);
random_history = zeros(3, N);

% Debug storage
debug_data = struct();
debug_data.step = [];
debug_data.true_pos = [];
debug_data.nav_pos = [];
debug_data.drift = [];
debug_data.random_nums = [];
debug_data.los_vector = [];
debug_data.guidance_cmd = [];
debug_data.relative_pos = [];

%% Main Simulation Loop
fprintf('Running simulation with enhanced debugging...\n');
fprintf('First 15 steps will show detailed calculations:\n\n');

for k = 1:N
    current_time = time(k);
    
    % Store current state
    pos_history(:, k) = position;
    vel_history(:, k) = velocity;
    nav_pos_history(:, k) = nav_position;
    drift_history(:, k) = position_drift;
    
    % Calculate range to target using NAVIGATION estimate
    relative_position = target_position - nav_position;
    range = norm(relative_position);
    range_history(k) = range;
    
    % Simple guidance: accelerate toward target (based on nav estimate)
    if range > 10  % Continue until close
        los_vector = relative_position / range;  % Unit vector toward target
        guidance_command = 800.0 * los_vector;   % Strong guidance
    else
        guidance_command = [0; 0; 0];           % Stop guidance when close
    end
    
    cmd_history(:, k) = guidance_command;
    
    % Calculate navigation error for monitoring
    navigation_error = norm(nav_position - position);
    nav_error_history(k) = navigation_error;
    
    % Store debug data for detailed analysis
    if k <= 15
        debug_data.step(end+1) = k;
        debug_data.true_pos(:, end+1) = position;
        debug_data.nav_pos(:, end+1) = nav_position;
        debug_data.drift(:, end+1) = position_drift;
        debug_data.relative_pos(:, end+1) = relative_position;
        debug_data.los_vector(:, end+1) = los_vector;
        debug_data.guidance_cmd(:, end+1) = guidance_command;
    end
    
    % Enhanced debug output for first 15 steps
    if k <= 15
        fprintf('=== STEP %d (t=%.1fs) ===\n', k, current_time);
        fprintf('True position:     [%9.2f %9.2f %9.2f]\n', position);
        fprintf('Nav position:      [%9.2f %9.2f %9.2f]\n', nav_position);
        fprintf('Position drift:    [%9.4f %9.4f %9.4f]\n', position_drift);
        fprintf('Relative position: [%9.2f %9.2f %9.2f]\n', relative_position);
        fprintf('Range to target:   %9.2f m\n', range);
        
        if range > 10
            fprintf('LOS vector:        [%9.4f %9.4f %9.4f]\n', los_vector);
            fprintf('Guidance command:  [%9.2f %9.2f %9.2f]\n', guidance_command);
        else
            fprintf('LOS vector:        [   ---    ---    ---  ]\n');
            fprintf('Guidance command:  [   0.0    0.0    0.0  ]\n');
        end
        
        fprintf('Navigation error:  %9.4f m\n', navigation_error);
        fprintf('\n');
    end
    
    % Physics integration
    if k < N
        % Forces
        gravity = [0; 0; -9.81];
        total_acceleration = guidance_command + gravity;
        
        % Generate random drift for this step (with detailed logging)
        if k <= 15
            current_random = randn(3,1);
            random_history(:, k) = current_random;
            drift_increment = drift_rate * dt * current_random;
            
            fprintf('Random numbers:    [%9.4f %9.4f %9.4f]\n', current_random);
            fprintf('Drift increment:   [%9.6f %9.6f %9.6f]\n', drift_increment);
            fprintf('Total accel:       [%9.2f %9.2f %9.2f]\n', total_acceleration);
            
            % Update drift
            position_drift = position_drift + drift_increment;
            
            fprintf('New total drift:   [%9.4f %9.4f %9.4f]\n', position_drift);
            fprintf('---\n');
        else
            % Normal operation for later steps
            position_drift = position_drift + drift_rate * dt * randn(3,1);
        end
        
        % Update TRUE physics (what actually happens)
        velocity = velocity + total_acceleration * dt;
        position = position + velocity * dt;
        
        % Update NAVIGATION system separately (with drift)
        nav_velocity = nav_velocity + total_acceleration * dt;
        nav_position = nav_position + nav_velocity * dt + position_drift;
        
        % Debug the navigation update
        if k <= 15
            fprintf('After physics update:\n');
            fprintf('New true position: [%9.2f %9.2f %9.2f]\n', position);
            fprintf('New nav position:  [%9.2f %9.2f %9.2f]\n', nav_position);
            fprintf('New nav error:     %9.4f m\n', norm(nav_position - position));
            fprintf('=====================================\n\n');
        end
        
        % Stop if hit ground
        if position(3) <= 0
            fprintf('Ground impact at t=%.1f s, range=%.1f m\n', current_time, range);
            % Trim arrays
            pos_history = pos_history(:, 1:k);
            vel_history = vel_history(:, 1:k);
            cmd_history = cmd_history(:, 1:k);
            range_history = range_history(1:k);
            nav_pos_history = nav_pos_history(:, 1:k);
            nav_error_history = nav_error_history(1:k);
            drift_history = drift_history(:, 1:k);
            time = time(1:k);
            break;
        end
    end
    
    % Progress indicator (less frequent to reduce clutter)
    if mod(k, round(N/3)) == 0 && k > 15
        fprintf('Progress: %d%%, Navigation error: %.1f m\n', round(100*k/N), navigation_error);
    end
end

%% Detailed Analysis Section
fprintf('\n=== DETAILED DRIFT ANALYSIS ===\n');

% Analyze the first 15 steps in detail
if length(debug_data.step) >= 10
    fprintf('Analysis of first 10 steps:\n');
    
    % Check Y-direction behavior specifically
    y_drift_values = debug_data.drift(2, 1:min(10, end));
    y_guidance_values = debug_data.guidance_cmd(2, 1:min(10, end));
    y_nav_pos_values = debug_data.nav_pos(2, 1:min(10, end));
    y_true_pos_values = debug_data.true_pos(2, 1:min(10, end));
    
    fprintf('\nY-Direction Analysis:\n');
    fprintf('Step |  Y_drift  | Y_guidance | Y_nav_pos | Y_true_pos\n');
    fprintf('-----|-----------|------------|-----------|------------\n');
    for i = 1:min(10, length(debug_data.step))
        fprintf('%4d | %9.4f | %10.2f | %9.4f | %10.4f\n', ...
            i, y_drift_values(i), y_guidance_values(i), y_nav_pos_values(i), y_true_pos_values(i));
    end
    
    % Summary statistics
    fprintf('\nY-Direction Summary:\n');
    fprintf('Max |Y drift|:     %.4f m\n', max(abs(y_drift_values)));
    fprintf('Max |Y guidance|:  %.2f m/s²\n', max(abs(y_guidance_values)));
    fprintf('Max |Y nav error|: %.4f m\n', max(abs(y_nav_pos_values)));
    fprintf('Max |Y true pos|:  %.4f m\n', max(abs(y_true_pos_values)));
end

%% Results Analysis (same as before)
final_position = pos_history(:, end);
final_nav_position = nav_pos_history(:, end);
miss_distance = norm(final_position - target_position);
final_navigation_error = nav_error_history(end);
final_velocity = vel_history(:, end);
final_speed = norm(final_velocity);

fprintf('\n=== FINAL RESULTS ===\n');
fprintf('Final true position: [%.0f, %.0f, %.0f] m\n', final_position);
fprintf('Final nav position:  [%.0f, %.0f, %.0f] m\n', final_nav_position);
fprintf('Miss distance: %.1f m\n', miss_distance);
fprintf('Final navigation error: %.1f m\n', final_navigation_error);
fprintf('Final speed: %.1f m/s (Mach %.1f)\n', final_speed, final_speed/343);
fprintf('Flight time: %.1f s\n', time(end));

%% Enhanced Visualization with Y-focus
figure('Name', 'Enhanced Debug: GPS-Denied Navigation', 'Position', [50, 50, 1600, 1200]);

% 3D Trajectory Comparison
subplot(3, 3, 1);
plot3(pos_history(1, :)/1000, pos_history(2, :)/1000, pos_history(3, :)/1000, ...
      'b-', 'LineWidth', 3, 'DisplayName', 'True Trajectory');
hold on;
plot3(nav_pos_history(1, :)/1000, nav_pos_history(2, :)/1000, nav_pos_history(3, :)/1000, ...
      'r--', 'LineWidth', 2, 'DisplayName', 'Navigation Estimate');
plot3(target_position(1)/1000, target_position(2)/1000, target_position(3)/1000, ...
      'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', 'Target');
grid on;
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Altitude (km)');
title('3D Trajectory: True vs Navigation');
legend('Location', 'best');
view(45, 30);

% Y-Direction Focus Plot
subplot(3, 3, 2);
plot(time, pos_history(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'True Y Position');
hold on;
plot(time, nav_pos_history(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Nav Y Estimate');
plot(time, zeros(size(time)), 'k:', 'LineWidth', 1, 'DisplayName', 'Target Y=0');
grid on;
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y-Direction Behavior (Should be Near Zero)');
legend('Location', 'best');

% Y-Direction Drift Components
subplot(3, 3, 3);
plot(time, drift_history(2, :), 'g-', 'LineWidth', 2, 'DisplayName', 'Y Drift');
hold on;
plot(time, cmd_history(2, :)/100, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Y Command/100');
grid on;
xlabel('Time (s)'); ylabel('Y Drift (m) / Y Cmd/100');
title('Y-Direction Drift and Commands');
legend('Location', 'best');

% Range vs Time
subplot(3, 3, 4);
plot(time, range_history/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Range (km)');
title('Range to Target');

% Navigation Error Growth
subplot(3, 3, 5);
plot(time, nav_error_history, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Navigation Error (m)');
title('Total Navigation Error');

% X-Direction Comparison (Should Work Well)
subplot(3, 3, 6);
plot(time, pos_history(1, :)/1000, 'b-', 'LineWidth', 2, 'DisplayName', 'True X');
hold on;
plot(time, nav_pos_history(1, :)/1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Nav X');
grid on;
xlabel('Time (s)'); ylabel('X Position (km)');
title('X-Direction (Should Track Well)');
legend();

% Z-Direction Comparison (Should Work Well)
subplot(3, 3, 7);
plot(time, pos_history(3, :)/1000, 'b-', 'LineWidth', 2, 'DisplayName', 'True Z');
hold on;
plot(time, nav_pos_history(3, :)/1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Nav Z');
grid on;
xlabel('Time (s)'); ylabel('Z Position (km)');
title('Z-Direction (Altitude)');
legend();

% Guidance Commands All Directions
subplot(3, 3, 8);
plot(time, cmd_history(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X Command');
hold on;
plot(time, cmd_history(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y Command');
plot(time, cmd_history(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z Command');
grid on;
xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
title('All Guidance Commands');
legend();

% Drift History All Directions
subplot(3, 3, 9);
plot(time, drift_history(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X Drift');
hold on;
plot(time, drift_history(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y Drift');
plot(time, drift_history(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z Drift');
grid on;
xlabel('Time (s)'); ylabel('Drift (m)');
title('Position Drift Components');
legend();

%% Key Diagnostics
fprintf('\n=== KEY DIAGNOSTICS ===\n');
y_final_error = abs(final_position(2));
y_max_drift = max(abs(drift_history(2, :)));
y_max_command = max(abs(cmd_history(2, :)));

fprintf('Y-direction final error: %.4f m\n', y_final_error);
fprintf('Y-direction max drift: %.4f m\n', y_max_drift);
fprintf('Y-direction max guidance command: %.2f m/s²\n', y_max_command);

if y_final_error > 0.5
    fprintf('⚠️  WARNING: Significant Y-direction drift detected!\n');
    fprintf('   This indicates either:\n');
    fprintf('   1. Random drift is too large\n');
    fprintf('   2. Feedback coupling between navigation and guidance\n');
    fprintf('   3. Integration order issue\n');
else
    fprintf('✅ Y-direction behavior is reasonable\n');
end

fprintf('\nSimulation complete with enhanced debugging!\n');