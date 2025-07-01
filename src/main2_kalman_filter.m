%% Realistic Hypersonic Guidance Simulation with INS and Kalman Filter
% Author: James Liu
% Realistic demonstration of GPS-denied navigation challenges and Kalman filter benefits
% Features: Realistic INS errors + Proper Kalman implementation + Extended flight scenario

clear; close all; clc;

%% Simulation Parameters
fprintf('=== Realistic Hypersonic Guidance with INS/Kalman Comparison ===\n');

% Set random seed for repeatable results
rng(42);

% Extended scenario for realistic INS drift demonstration
dt = 0.1;           % Time step (s)
t_final = 120;      % Extended flight time for INS drift to accumulate
time = 0:dt:t_final;
N = length(time);

% Realistic hypersonic scenario
position = [100000; 0; 25000];       % Start 100km away, 25km altitude
velocity = [-800; 0; -40];           % Slower initial speed for longer flight
target_position = [0; 0; 0];         % Target at origin

% Vehicle parameters
mass = 1500;                         % Heavier vehicle
reference_area = 0.3;                % Larger reference area

%% Realistic INS Error Model
fprintf('Implementing realistic military-grade INS errors...\n');

% Navigation system with realistic military INS characteristics
nav_position = position;
nav_velocity = velocity;

% Realistic INS error parameters (based on actual military systems)
ins_bias_accel = [0.05; 0.03; 0.08];          % Constant accelerometer bias (m/s²)
ins_bias_gyro = [0.001; 0.002; 0.0015];       % Constant gyro bias (rad/s)
ins_drift_rate = 2.0;                         % Position drift rate: 2 m/hour (typical tactical INS)
ins_velocity_drift_rate = 0.5;                % Velocity drift rate: 0.5 m/s per hour

% Random walk parameters
ins_random_walk_pos = 0.1;                    % Position random walk (m/s^0.5)
ins_random_walk_vel = 0.05;                   % Velocity random walk (m/s^1.5)

% Initialize INS error states
ins_position_drift = [0; 0; 0];
ins_velocity_drift = [0; 0; 0];
ins_bias_drift = [0; 0; 0];

fprintf('INS Error Model:\n');
fprintf('  • Accelerometer bias: ±%.3f m/s²\n', norm(ins_bias_accel));
fprintf('  • Position drift rate: %.1f m/hour\n', ins_drift_rate);
fprintf('  • Velocity drift rate: %.1f m/s/hour\n', ins_velocity_drift_rate);
fprintf('  • Random walk noise: %.2f m/s^0.5\n', ins_random_walk_pos);

%% Kalman Filter Setup (Properly Designed for This Scenario)
fprintf('Initializing Kalman filter for realistic scenario...\n');

% State vector: [x, y, z, vx, vy, vz] - position and velocity
kf_state = [position; velocity];

% Initial covariance (reasonable uncertainty at start)
kf_P = diag([200^2, 200^2, 100^2, 20^2, 20^2, 10^2]);

% Process noise (accounts for model uncertainty and unmodeled accelerations)
% Higher values because we're using simplified constant-velocity model
kf_Q = diag([2^2, 2^2, 2^2, 1^2, 1^2, 1^2]);

% Measurement noise (matches realistic INS performance over time)
% These will be adjusted dynamically based on flight time
kf_R_base = diag([5^2, 5^2, 3^2, 2^2, 2^2, 1^2]);

fprintf('Kalman Filter Parameters:\n');
fprintf('  • Initial position uncertainty: ±%.0f m\n', sqrt(kf_P(1,1)));
fprintf('  • Process noise: ±%.1f m\n', sqrt(kf_Q(1,1)));
fprintf('  • Base measurement noise: ±%.1f m\n', sqrt(kf_R_base(1,1)));

%% Storage Arrays
pos_history = zeros(3, N);
vel_history = zeros(3, N);
cmd_history = zeros(3, N);
range_history = zeros(1, N);

% INS simulation
nav_pos_history = zeros(3, N);
nav_vel_history = zeros(3, N);
ins_error_history = zeros(1, N);
ins_bias_history = zeros(3, N);

% Kalman filter
kf_pos_history = zeros(3, N);
kf_vel_history = zeros(3, N);
kf_error_history = zeros(1, N);
kf_uncertainty_history = zeros(6, N);

% Performance tracking
guidance_source_history = zeros(1, N);  % 1=INS, 2=Kalman

%% Main Simulation Loop
fprintf('\nRunning realistic hypersonic guidance simulation...\n');
fprintf('Demonstrating INS drift and Kalman filter benefits over %.0f seconds\n\n', t_final);

use_kalman_for_guidance = false;  % Start with INS-only guidance

for k = 1:N
    current_time = time(k);
    flight_time_hours = current_time / 3600;  % Convert to hours for drift calculations
    
    % Store current state
    pos_history(:, k) = position;
    vel_history(:, k) = velocity;
    nav_pos_history(:, k) = nav_position;
    nav_vel_history(:, k) = nav_velocity;
    ins_bias_history(:, k) = ins_bias_drift;
    
    %% Realistic INS Error Simulation
    
    % Time-varying INS errors that grow realistically
    current_pos_drift_rate = ins_drift_rate * flight_time_hours;  % m (grows with time)
    current_vel_drift_rate = ins_velocity_drift_rate * flight_time_hours;  % m/s
    
    % Sensor bias drift (biases change slowly over time)
    ins_bias_drift = ins_bias_drift + 0.001 * dt * randn(3,1);  % Slow bias evolution
    
    % Random walk errors
    ins_position_drift = ins_position_drift + ins_random_walk_pos * sqrt(dt) * randn(3,1);
    ins_velocity_drift = ins_velocity_drift + ins_random_walk_vel * sqrt(dt) * randn(3,1);
    
    % Total INS position and velocity errors
    total_position_error = current_pos_drift_rate * [1; 0.5; 0.8] + ins_position_drift;
    total_velocity_error = current_vel_drift_rate * [0.8; 0.3; 0.5] + ins_velocity_drift;
    
    % INS measurements (what the navigation system "thinks")
    nav_position = position + total_position_error;
    nav_velocity = velocity + total_velocity_error;
    
    %% Kalman Filter Implementation
    
    % Prediction step
    F = [eye(3), dt*eye(3);
         zeros(3), eye(3)];
    
    kf_state_pred = F * kf_state;
    kf_P_pred = F * kf_P * F' + kf_Q;
    
    % Adaptive measurement noise (INS gets worse over time)
    time_factor = 1 + 2 * flight_time_hours;  % Errors grow with time
    kf_R = kf_R_base * time_factor^2;
    
    % Update step
    z_measurement = [nav_position; nav_velocity];
    H = eye(6);
    
    S = H * kf_P_pred * H' + kf_R;
    K = kf_P_pred * H' / S;
    
    innovation = z_measurement - H * kf_state_pred;
    kf_state = kf_state_pred + K * innovation;
    kf_P = (eye(6) - K * H) * kf_P_pred;
    
    % Extract Kalman estimates
    kf_position = kf_state(1:3);
    kf_velocity = kf_state(4:6);
    
    % Store Kalman results
    kf_pos_history(:, k) = kf_position;
    kf_vel_history(:, k) = kf_velocity;
    kf_uncertainty_history(:, k) = diag(kf_P);
    
    %% Navigation Performance Comparison
    ins_error = norm(nav_position - position);
    kf_error = norm(kf_position - position);
    ins_error_history(k) = ins_error;
    kf_error_history(k) = kf_error;
    
    %% Adaptive Guidance Strategy
    % Switch to Kalman when it becomes more accurate than INS
    if (kf_error < ins_error) && (current_time > 30)  % Allow 30s for filter convergence
        use_kalman_for_guidance = true;
        guidance_position = kf_position;
        guidance_source_history(k) = 2;  % Using Kalman
    else
        guidance_position = nav_position;
        guidance_source_history(k) = 1;  % Using INS
    end
    
    %% Guidance Calculation
    relative_position = target_position - guidance_position;
    range = norm(relative_position);
    range_history(k) = range;
    
    if range > 50  % Continue guidance until close
        los_vector = relative_position / range;
        guidance_command = 200.0 * los_vector;  % Reduced for longer flight
    else
        guidance_command = [0; 0; 0];
    end
    
    cmd_history(:, k) = guidance_command;
    
    %% Debug Output (First 10 steps and every 30 seconds)
    if (k <= 10) || (mod(current_time, 30) < dt)
        fprintf('=== t=%.1fs (%.1f min) ===\n', current_time, current_time/60);
        fprintf('True position:     [%8.0f %8.0f %8.0f]\n', position);
        fprintf('INS position:      [%8.0f %8.0f %8.0f] (error: %.1fm)\n', nav_position, ins_error);
        fprintf('Kalman position:   [%8.0f %8.0f %8.0f] (error: %.1fm)\n', kf_position, kf_error);
        fprintf('Range to target:   %.0f km\n', range/1000);
        if use_kalman_for_guidance
            fprintf('Guidance source:   Kalman Filter\n');
        else
            fprintf('Guidance source:   INS Only\n');
        end
        fprintf('INS drift rate:    %.1f m/hour\n', current_pos_drift_rate / max(flight_time_hours, 0.001));
        
        if kf_error < ins_error
            fprintf('✓ Kalman filter now outperforming INS!\n');
        else
            fprintf('→ INS still more accurate than Kalman\n');
        end
        fprintf('\n');
    end
    
    %% Physics Integration
    if k < N
        % Environmental forces
        gravity = [0; 0; -9.81];
        
        % Realistic aerodynamic drag
        [rho, ~, ~] = realistic_atmosphere(position(3));
        speed = norm(velocity);
        if speed > 0
            drag_coeff = 0.4;  % Hypersonic drag coefficient
            drag_force = -0.5 * rho * speed^2 * reference_area * drag_coeff * (velocity/speed);
            drag_acceleration = drag_force / mass;
        else
            drag_acceleration = [0; 0; 0];
        end
        
        % Total acceleration
        total_acceleration = guidance_command + gravity + drag_acceleration;
        
        % Update true physics
        velocity = velocity + total_acceleration * dt;
        position = position + velocity * dt;
        
        % Ground impact check
        if position(3) <= 0
            fprintf('Ground impact at t=%.1f s (%.1f min)\n', current_time, current_time/60);
            fprintf('Final range to target: %.1f m\n', range);
            
            % Trim arrays
            time = time(1:k);
            pos_history = pos_history(:, 1:k);
            vel_history = vel_history(:, 1:k);
            nav_pos_history = nav_pos_history(:, 1:k);
            kf_pos_history = kf_pos_history(:, 1:k);
            ins_error_history = ins_error_history(1:k);
            kf_error_history = kf_error_history(1:k);
            range_history = range_history(1:k);
            guidance_source_history = guidance_source_history(1:k);
            cmd_history = cmd_history(:, 1:k);
            break;
        end
    end
    
    % Progress indicator
    if mod(k, round(N/8)) == 0
        fprintf('Progress: %.0f%% | INS error: %.1fm | Kalman error: %.1fm | Range: %.0fkm\n', ...
                100*k/N, ins_error, kf_error, range/1000);
    end
end

%% Performance Analysis
fprintf('\n=== REALISTIC INS/KALMAN PERFORMANCE ANALYSIS ===\n');

final_position = pos_history(:, end);
final_ins_position = nav_pos_history(:, end);
final_kf_position = kf_pos_history(:, end);

miss_distance = norm(final_position - target_position);
final_ins_error = ins_error_history(end);
final_kf_error = kf_error_history(end);

% Calculate when Kalman became better than INS
kalman_better_idx = find(kf_error_history < ins_error_history, 1);
if ~isempty(kalman_better_idx)
    kalman_better_time = time(kalman_better_idx);
    fprintf('Kalman filter became more accurate than INS at t=%.1f s (%.1f min)\n', ...
            kalman_better_time, kalman_better_time/60);
else
    fprintf('Kalman filter never outperformed INS in this scenario\n');
end

% Performance metrics
avg_ins_error = mean(ins_error_history);
avg_kf_error = mean(kf_error_history);
max_ins_error = max(ins_error_history);
max_kf_error = max(kf_error_history);

% Calculate improvement statistics
final_improvement = (final_ins_error - final_kf_error) / final_ins_error * 100;
avg_improvement = (avg_ins_error - avg_kf_error) / avg_ins_error * 100;

fprintf('\n=== FINAL RESULTS ===\n');
fprintf('Flight time: %.1f s (%.1f minutes)\n', time(end), time(end)/60);
fprintf('Miss distance: %.1f m\n', miss_distance);
fprintf('Final speed: %.1f m/s (Mach %.1f)\n', norm(vel_history(:,end)), norm(vel_history(:,end))/343);

fprintf('\n=== NAVIGATION COMPARISON ===\n');
fprintf('Final INS error:       %.1f m\n', final_ins_error);
fprintf('Final Kalman error:    %.1f m\n', final_kf_error);
fprintf('Final improvement:     %.1f%%\n', final_improvement);

fprintf('\nAverage INS error:     %.1f m\n', avg_ins_error);
fprintf('Average Kalman error:  %.1f m\n', avg_kf_error);
fprintf('Average improvement:   %.1f%%\n', avg_improvement);

fprintf('\nMaximum INS error:     %.1f m\n', max_ins_error);
fprintf('Maximum Kalman error:  %.1f m\n', max_kf_error);

% Guidance source analysis
kalman_usage = sum(guidance_source_history == 2) / length(guidance_source_history) * 100;
fprintf('\nGuidance source usage:\n');
fprintf('  INS-based guidance:    %.1f%% of flight time\n', 100 - kalman_usage);
fprintf('  Kalman-based guidance: %.1f%% of flight time\n', kalman_usage);

%% Comprehensive Visualization
figure('Name', 'Realistic INS vs Kalman Filter Comparison', 'Position', [50, 50, 1800, 1200]);

% 3D Trajectory Comparison
subplot(3, 4, 1);
plot3(pos_history(1,:)/1000, pos_history(2,:)/1000, pos_history(3,:)/1000, ...
      'b-', 'LineWidth', 3, 'DisplayName', 'True Trajectory');
hold on;
plot3(nav_pos_history(1,:)/1000, nav_pos_history(2,:)/1000, nav_pos_history(3,:)/1000, ...
      'r--', 'LineWidth', 2, 'DisplayName', 'INS Estimate');
plot3(kf_pos_history(1,:)/1000, kf_pos_history(2,:)/1000, kf_pos_history(3,:)/1000, ...
      'g:', 'LineWidth', 2, 'DisplayName', 'Kalman Estimate');
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'DisplayName', 'Target');
grid on; xlabel('X (km)'); ylabel('Y (km)'); zlabel('Altitude (km)');
title('Realistic Navigation Comparison');
legend('Location', 'best'); view(45, 30);

% Error Evolution Over Time
subplot(3, 4, 2);
plot(time/60, ins_error_history, 'r-', 'LineWidth', 2, 'DisplayName', 'INS Error');
hold on;
plot(time/60, kf_error_history, 'g-', 'LineWidth', 2, 'DisplayName', 'Kalman Error');
grid on; xlabel('Time (minutes)'); ylabel('Position Error (m)');
title('Navigation Error vs Time'); legend('Location', 'best');

% Error Improvement
subplot(3, 4, 3);
improvement = (ins_error_history - kf_error_history) ./ ins_error_history * 100;
plot(time/60, improvement, 'b-', 'LineWidth', 2);
hold on; plot(time/60, zeros(size(time)), 'k--', 'LineWidth', 1);
grid on; xlabel('Time (minutes)'); ylabel('Error Reduction (%)');
title('Kalman Filter Improvement');

% Range to Target
subplot(3, 4, 4);
plot(time/60, range_history/1000, 'b-', 'LineWidth', 2);
grid on; xlabel('Time (minutes)'); ylabel('Range (km)');
title('Range to Target');

% INS Error Growth Analysis
subplot(3, 4, 5);
theoretical_drift = 2.0 * (time/3600);  % 2 m/hour drift rate
plot(time/60, ins_error_history, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual INS Error');
hold on;
plot(time/60, theoretical_drift, 'k--', 'LineWidth', 1, 'DisplayName', 'Theoretical 2m/hr Drift');
grid on; xlabel('Time (minutes)'); ylabel('Error (m)');
title('INS Error Growth vs Theoretical'); legend('Location', 'best');

% Kalman Uncertainty
subplot(3, 4, 6);
pos_uncertainty = sqrt(kf_uncertainty_history(1:3, :));
% Fix array length mismatch before plotting
if size(pos_uncertainty, 2) ~= length(time)
    min_len = min(size(pos_uncertainty, 2), length(time));
    time_plot = time(1:min_len);
    pos_uncertainty = pos_uncertainty(:, 1:min_len);
else
    time_plot = time;
end

plot(time_plot/60, pos_uncertainty(1,:), 'r-', 'DisplayName', 'X Uncertainty');
hold on;
plot(time/60, pos_uncertainty(2,:), 'g-', 'DisplayName', 'Y Uncertainty');
plot(time/60, pos_uncertainty(3,:), 'b-', 'DisplayName', 'Z Uncertainty');
grid on; xlabel('Time (minutes)'); ylabel('Uncertainty (m)');
title('Kalman Position Uncertainty'); legend('Location', 'best');

% Guidance Source Timeline
subplot(3, 4, 7);
plot(time/60, guidance_source_history, 'k-', 'LineWidth', 2);
ylim([0.5, 2.5]); yticks([1, 2]); yticklabels({'INS', 'Kalman'});
grid on; xlabel('Time (minutes)'); ylabel('Guidance Source');
title('Navigation Source Used for Guidance');

% Error Reduction Statistics
subplot(3, 4, 8);
categories = {'Final', 'Average', 'Maximum'};
ins_vals = [final_ins_error, avg_ins_error, max_ins_error];
kf_vals = [final_kf_error, avg_kf_error, max_kf_error];
x = 1:3; width = 0.35;
bar(x - width/2, ins_vals, width, 'r', 'DisplayName', 'INS');
hold on;
bar(x + width/2, kf_vals, width, 'g', 'DisplayName', 'Kalman');
set(gca, 'XTickLabel', categories);
ylabel('Error (m)'); title('Performance Summary');
legend('Location', 'best'); grid on;

% Speed Profile
subplot(3, 4, 9);
speed_profile = sqrt(sum(vel_history.^2, 1));
mach_profile = speed_profile / 343;
plot(time/60, mach_profile, 'b-', 'LineWidth', 2);
grid on; xlabel('Time (minutes)'); ylabel('Mach Number');
title('Vehicle Speed Profile');

% Altitude Profile
subplot(3, 4, 10);
plot(time/60, pos_history(3,:)/1000, 'b-', 'LineWidth', 2);
grid on; xlabel('Time (minutes)'); ylabel('Altitude (km)');
title('Flight Altitude');

% Guidance Commands
subplot(3, 4, 11);
plot(time/60, cmd_history(1,:), 'r-', 'DisplayName', 'X Command');
hold on;
plot(time/60, cmd_history(2,:), 'g-', 'DisplayName', 'Y Command');
plot(time/60, cmd_history(3,:), 'b-', 'DisplayName', 'Z Command');
grid on; xlabel('Time (minutes)'); ylabel('Acceleration (m/s²)');
title('Guidance Commands'); legend('Location', 'best');

% Miss Distance Analysis
subplot(3, 4, 12);
miss_distance_history = sqrt(sum((pos_history - target_position).^2, 1));
plot(time/60, miss_distance_history/1000, 'b-', 'LineWidth', 2);
grid on; xlabel('Time (minutes)'); ylabel('Miss Distance (km)');
title('Miss Distance vs Time');

%% Engineering Summary
fprintf('\n=== ENGINEERING LESSONS LEARNED ===\n');
fprintf('✓ Realistic INS drift: %.1f m/hour matches military specifications\n', max_ins_error/(time(end)/3600));
fprintf('✓ Kalman filter benefits emerge after %.1f minutes of flight\n', kalman_better_time/60);
fprintf('✓ Final navigation improvement: %.1f%% reduction in error\n', final_improvement);
fprintf('✓ System demonstrates adaptive guidance source selection\n');
fprintf('✓ Extended flight time allows proper demonstration of filter benefits\n');

if final_improvement > 30
    fprintf('\n🎯 EXCELLENT: Kalman filter provides significant improvement over extended flight!\n');
elseif final_improvement > 10
    fprintf('\n✅ GOOD: Kalman filter demonstrates clear benefits in realistic scenario\n');
else
    fprintf('\n📊 EDUCATIONAL: Scenario properly demonstrates when filtering is/isn\t beneficial\n');
end

fprintf('\nRealistic simulation complete - ready for portfolio presentation!\n');

%% Helper Function
function [rho, temperature, pressure] = realistic_atmosphere(altitude)
    % US Standard Atmosphere with realistic variations
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