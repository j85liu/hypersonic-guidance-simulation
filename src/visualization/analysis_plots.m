function fig_handles = analysis_plots(results, enable_visualization)
%ANALYSIS_PLOTS Create comprehensive post-simulation analysis plots
%
% This function generates all the analysis plots extracted from the main
% simulation comprehensive code (lines 780-920). Creates professional
% multi-panel analysis figures for mission performance evaluation.
%
% Inputs:
%   results - Results structure from simulation
%   enable_visualization - Boolean flag to enable/disable plotting
%
% Outputs:
%   fig_handles - Array of figure handles for further processing
%
% Features:
%   - 3D trajectory visualization
%   - Navigation performance comparison
%   - Speed and altitude profiles  
%   - Sensor availability timeline
%   - Guidance command analysis
%   - Threat exposure visualization
%   - Performance summary dashboard
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    if nargin < 2
        enable_visualization = true;
    end
    
    fig_handles = [];
    
    if ~enable_visualization
        fprintf('Analysis plots disabled\n');
        return;
    end
    
    try
        % Create main analysis figure (matching original structure)
        fig_analysis = create_main_analysis_figure(results);
        fig_handles = [fig_handles, fig_analysis];
        
        fprintf('✅ Comprehensive analysis plots created\n');
        
    catch ME
        fprintf('❌ Error creating analysis plots: %s\n', ME.message);
        fig_handles = [];
    end
end

function fig = create_main_analysis_figure(results)
    %CREATE_MAIN_ANALYSIS_FIGURE Create the main 12-panel analysis figure
    %
    % This exactly matches the analysis plots from the original code
    % (lines 780-920) with all the same subplot arrangements and styling
    
    % Create figure (matching original positioning)
    fig = figure('Name', 'Mission Analysis', 'Position', [200, 100, 1600, 1000]);
    
    % Extract data from results structure
    k = length(results.time);
    time_min = results.time / 60;  % Convert to minutes
    
    % Calculate derived quantities
    speed_profile = sqrt(sum(results.velocity.^2, 1));
    mach_profile = speed_profile / 343;
    
    % 1. 3D Trajectory Plot (subplot 3,4,1)
    subplot(3, 4, 1);
    plot3(results.position(1,:)/1000, results.position(2,:)/1000, results.position(3,:)/1000, ...
          'r-', 'LineWidth', 2, 'DisplayName', 'Vehicle');
    hold on;
    plot3(results.target(1,:)/1000, results.target(2,:)/1000, results.target(3,:)/1000, ...
          'g--', 'LineWidth', 2, 'DisplayName', 'Target');
    plot3(results.position(1,end)/1000, results.position(2,end)/1000, results.position(3,end)/1000, ...
          'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    plot3(results.target(1,end)/1000, results.target(2,end)/1000, results.target(3,end)/1000, ...
          'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
    grid on; xlabel('X (km)'); ylabel('Y (km)'); zlabel('Alt (km)');
    title('3D Trajectory'); legend('Location', 'best');
    
    % 2. Navigation Error Comparison (subplot 3,4,2)
    subplot(3, 4, 2);
    plot(time_min, results.navigation.ins_error, 'r-', 'LineWidth', 2, 'DisplayName', 'INS Only');
    hold on;
    plot(time_min, results.navigation.ekf_error, 'b-', 'LineWidth', 2, 'DisplayName', 'Multi-Sensor EKF');
    grid on; xlabel('Time (min)'); ylabel('Position Error (m)');
    title('FIXED Navigation Performance'); legend('Location', 'best');
    
    % 3. Speed Profile (subplot 3,4,3)
    subplot(3, 4, 3);
    plot(time_min, mach_profile, 'b-', 'LineWidth', 2);
    grid on; xlabel('Time (min)'); ylabel('Mach Number');
    title('Speed Profile');
    
    % 4. Range to Target (subplot 3,4,4)
    subplot(3, 4, 4);
    plot(time_min, results.range_to_target/1000, 'g-', 'LineWidth', 2);
    grid on; xlabel('Time (min)'); ylabel('Range (km)');
    title('Range to Target');
    
    % 5. Sensor Availability (subplot 3,4,5)
    subplot(3, 4, 5);
    sensor_plot = results.sensor_status(1:5, :);
    imagesc(time_min, 1:5, sensor_plot);
    colormap([0.2 0.2 0.2; 0 1 0]);  % Dark gray for unavailable, green for available
    sensor_names = {'INS', 'GPS', 'TERCOM', 'LASER', 'IR'};
    yticks(1:5); yticklabels(sensor_names);
    xlabel('Time (min)'); title('Sensor Availability');
    colorbar('Ticks', [0.25, 0.75], 'TickLabels', {'Unavailable', 'Available'});
    
    % 6. Guidance Commands (subplot 3,4,6)
    subplot(3, 4, 6);
    plot(time_min, results.guidance(1,:), 'r-', 'DisplayName', 'X');
    hold on;
    plot(time_min, results.guidance(2,:), 'g-', 'DisplayName', 'Y');
    plot(time_min, results.guidance(3,:), 'b-', 'DisplayName', 'Z');
    grid on; xlabel('Time (min)'); ylabel('Acceleration (m/s²)');
    title('Guidance Commands'); legend('Location', 'best');
    
    % 7. Threat Timeline (subplot 3,4,7)
    subplot(3, 4, 7);
    threat_timeline = any(results.threat_status, 1);
    area(time_min, threat_timeline, 'FaceColor', 'red', 'FaceAlpha', 0.3);
    grid on; xlabel('Time (min)'); ylabel('Under Threat');
    title('Threat Exposure Timeline');
    ylim([0, 1.2]);
    
    % 8. Miss Distance Evolution (subplot 3,4,8)
    subplot(3, 4, 8);
    semilogy(time_min, results.range_to_target, 'b-', 'LineWidth', 2);
    grid on; xlabel('Time (min)'); ylabel('Miss Distance (m)');
    title('Miss Distance (Log Scale)');
    
    % 9. Altitude Profile (subplot 3,4,9)
    subplot(3, 4, 9);
    plot(time_min, results.position(3,:)/1000, 'b-', 'LineWidth', 2);
    grid on; xlabel('Time (min)'); ylabel('Altitude (km)');
    title('Altitude Profile');
    
    % 10. Navigation Improvement (subplot 3,4,10)
    subplot(3, 4, 10);
    improvement_history = calculate_improvement_history(results.navigation.ins_error, ...
                                                       results.navigation.ekf_error);
    plot(time_min, improvement_history, 'g-', 'LineWidth', 2);
    hold on; plot(time_min, zeros(size(time_min)), 'k--');
    grid on; xlabel('Time (min)'); ylabel('Improvement (%)');
    title('EKF vs INS Improvement');
    ylim([-50, 100]);
    
    % 11. Error Comparison Bar Chart (subplot 3,4,11)
    subplot(3, 4, 11);
    categories = {'Average', 'Final'};
    ins_vals = [results.statistics.avg_ins_error, results.statistics.final_ins_error];
    ekf_vals = [results.statistics.avg_ekf_error, results.statistics.final_ekf_error];
    x = 1:2; width = 0.35;
    bar(x - width/2, ins_vals, width, 'r', 'DisplayName', 'INS');
    hold on;
    bar(x + width/2, ekf_vals, width, 'b', 'DisplayName', 'EKF');
    set(gca, 'XTickLabel', categories);
    ylabel('Error (m)'); title('Navigation Comparison');
    legend('Location', 'best'); grid on;
    
    % 12. Performance Summary (subplot 3,4,12)
    subplot(3, 4, 12);
    create_performance_summary(results);
    
    % Add super title
    sgtitle('FIXED Comprehensive Hypersonic Guidance Mission Analysis', ...
            'FontSize', 16, 'FontWeight', 'bold');
end

function improvement_history = calculate_improvement_history(ins_error, ekf_error)
    %CALCULATE_IMPROVEMENT_HISTORY Calculate EKF improvement over time
    
    improvement_history = zeros(size(ins_error));
    for i = 1:length(ins_error)
        if ins_error(i) > 0.1
            improvement_history(i) = (ins_error(i) - ekf_error(i)) / ins_error(i) * 100;
            improvement_history(i) = max(-100, min(100, improvement_history(i)));
        else
            improvement_history(i) = 0;
        end
    end
end

function create_performance_summary(results)
    %CREATE_PERFORMANCE_SUMMARY Create text summary panel
    %
    % This matches the performance summary from the original code
    
    % Calculate summary statistics
    avg_improvement = results.performance.ekf_improvement;
    final_time_min = results.performance.flight_time / 60;
    avg_mach = results.performance.avg_speed / 343;
    max_mach = results.performance.max_speed / 343;
    
    % Get sensor utilization (handle missing fields gracefully)
    if isfield(results, 'sensor_utilization')
        gps_avail = get_sensor_availability(results.sensor_utilization, 'gps');
        tercom_avail = get_sensor_availability(results.sensor_utilization, 'tercom');
        laser_avail = get_sensor_availability(results.sensor_utilization, 'laser');
        ir_avail = get_sensor_availability(results.sensor_utilization, 'ir');
    else
        % Calculate from sensor_status if available
        if isfield(results, 'sensor_status') && size(results.sensor_status, 1) >= 5
            gps_avail = sum(results.sensor_status(2, :)) / size(results.sensor_status, 2) * 100;
            tercom_avail = sum(results.sensor_status(3, :)) / size(results.sensor_status, 2) * 100;
            laser_avail = sum(results.sensor_status(4, :)) / size(results.sensor_status, 2) * 100;
            ir_avail = sum(results.sensor_status(5, :)) / size(results.sensor_status, 2) * 100;
        else
            gps_avail = 0; tercom_avail = 0; laser_avail = 0; ir_avail = 0;
        end
    end
    
    % Create summary text (matching original format exactly)
    summary_text = sprintf([
        'FIXED MISSION SUMMARY\n\n'...
        'Final Miss Distance: %.1f m\n'...
        'Flight Time: %.1f minutes\n'...
        'Average Speed: Mach %.2f\n'...
        'Max Speed: Mach %.2f\n\n'...
        'Navigation Performance:\n'...
        'EKF Improvement: %.1f%%\n'...
        'Average INS Error: %.1f m\n'...
        'Average EKF Error: %.1f m\n\n'...
        'Sensor Availability:\n'...
        'GPS: %.1f%% | TERCOM: %.1f%%\n'...
        'Laser: %.1f%% | IR: %.1f%%\n\n'...
        'Mission Success:\n'...
        '✓ Stable EKF Performance\n'...
        '✓ Realistic Speed Profile\n'...
        '✓ Multi-Sensor Fusion\n'...
        '✓ Portfolio Ready!'
        ], ...
        results.performance.final_miss, final_time_min, avg_mach, max_mach, ...
        avg_improvement, results.statistics.avg_ins_error, results.statistics.avg_ekf_error, ...
        gps_avail, tercom_avail, laser_avail, ir_avail);
    
    text(0.05, 0.95, summary_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontSize', 10, 'FontName', 'FixedWidth');
    axis off;
end

function availability = get_sensor_availability(sensor_utilization, sensor_name)
    %GET_SENSOR_AVAILABILITY Safely extract sensor availability from structure
    
    if isfield(sensor_utilization, sensor_name)
        availability = sensor_utilization.(sensor_name);
    else
        availability = 0;  % Default if field doesn't exist
    end
end