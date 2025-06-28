%% Simple Hypersonic Guidance Simulation - Clean Start
% Author: James Liu
% A minimal, working hypersonic guidance simulation
% Focus: Get basic guidance working first, then add complexity

clear; close all; clc;

%% Basic Parameters
fprintf('=== Simple Hypersonic Guidance Simulation ===\n');

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

% Storage arrays
pos_history = zeros(3, N);
vel_history = zeros(3, N);
cmd_history = zeros(3, N);
range_history = zeros(1, N);

%% Main Simulation Loop
fprintf('Running simulation...\n');

for k = 1:N
    current_time = time(k);
    
    % Store current state
    pos_history(:, k) = position;
    vel_history(:, k) = velocity;
    
    % Calculate range to target
    relative_position = target_position - position;
    range = norm(relative_position);
    range_history(k) = range;
    
    % Simple guidance: accelerate toward target
    if range > 10  % Continue until close
        los_vector = relative_position / range;  % Unit vector toward target
        guidance_command = 50.0 * los_vector;   % 50 m/s^2 toward target
    else
        guidance_command = [0; 0; 0];           % Stop guidance when close
    end
    
    cmd_history(:, k) = guidance_command;
    
    % Debug output for first 10 steps
    if k <= 10
        fprintf('Step %d: pos=[%.0f %.0f %.0f], range=%.0f, cmd=[%.1f %.1f %.1f]\n', ...
            k, position, range, guidance_command);
    end
    
    % Physics integration (keep it simple)
    if k < N
        % Forces
        gravity = [0; 0; -9.81];
        total_acceleration = guidance_command + gravity;
        
        % Simple Euler integration
        velocity = velocity + total_acceleration * dt;
        position = position + velocity * dt;
        
        % Stop if hit ground
        if position(3) <= 0
            fprintf('Ground impact at t=%.1f s, range=%.1f m\n', current_time, range);
            % Trim arrays
            pos_history = pos_history(:, 1:k);
            vel_history = vel_history(:, 1:k);
            cmd_history = cmd_history(:, 1:k);
            range_history = range_history(1:k);
            time = time(1:k);
            break;
        end
    end
    
    % Progress
    if mod(k, round(N/5)) == 0
        fprintf('Progress: %d%%\n', round(100*k/N));
    end
end

%% Results
final_position = pos_history(:, end);
miss_distance = norm(final_position - target_position);
final_velocity = vel_history(:, end);
final_speed = norm(final_velocity);

fprintf('\n=== RESULTS ===\n');
fprintf('Final position: [%.0f, %.0f, %.0f] m\n', final_position);
fprintf('Miss distance: %.1f m\n', miss_distance);
fprintf('Final speed: %.1f m/s (Mach %.1f)\n', final_speed, final_speed/343);
fprintf('Flight time: %.1f s\n', time(end));

%% Simple Visualization
figure('Name', 'Simple Hypersonic Trajectory', 'Position', [100, 100, 1200, 800]);

% 3D Trajectory
subplot(2, 2, 1);
plot3(pos_history(1, :)/1000, pos_history(2, :)/1000, pos_history(3, :)/1000, ...
      'b-', 'LineWidth', 2);
hold on;
plot3(target_position(1)/1000, target_position(2)/1000, target_position(3)/1000, ...
      'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(pos_history(1, 1)/1000, pos_history(2, 1)/1000, pos_history(3, 1)/1000, ...
      'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
grid on;
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Altitude (km)');
title('3D Trajectory');
legend('Vehicle', 'Target', 'Start');
view(45, 30);

% Range vs Time
subplot(2, 2, 2);
plot(time, range_history/1000, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Range (km)');
title('Range to Target');

% Velocity Components
subplot(2, 2, 3);
plot(time, vel_history(1, :), 'r-', 'LineWidth', 1.5);
hold on;
plot(time, vel_history(2, :), 'g-', 'LineWidth', 1.5);
plot(time, vel_history(3, :), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Components');
legend('Vx', 'Vy', 'Vz');

% Guidance Commands
subplot(2, 2, 4);
plot(time, cmd_history(1, :), 'r-', 'LineWidth', 1.5);
hold on;
plot(time, cmd_history(2, :), 'g-', 'LineWidth', 1.5);
plot(time, cmd_history(3, :), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
title('Guidance Commands');
legend('Ax', 'Ay', 'Az');

%% Success Check
if miss_distance < 1000
    fprintf('\n✅ SUCCESS: Miss distance < 1km - Basic guidance is working!\n');
    fprintf('Next step: Add navigation errors, Kalman filter, etc.\n');
else
    fprintf('\n❌ ISSUE: Miss distance = %.1f m - Need to debug guidance\n', miss_distance);
    fprintf('Check if vehicle trajectory curves toward target in 3D plot\n');
end

fprintf('\nSimulation complete!\n');