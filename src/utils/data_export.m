function data_export(action, varargin)
%DATA_EXPORT Handle results saving and export for hypersonic guidance simulation
%
% This function manages all data export functionality extracted from the
% main simulation comprehensive code, including results structuring,
% performance analysis, and file saving.
%
% Usage:
%   data_export('save_results', time, pos_history, vel_history, ...)
%   data_export('print_analysis', results_struct)
%   data_export('export_plots', figure_handles)
%
% Actions:
%   'save_results' - Create and save comprehensive results structure
%   'print_analysis' - Print detailed performance analysis to console
%   'export_plots' - Export analysis plots to files
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    switch action
        case 'save_results'
            save_results_impl(varargin{:});
            
        case 'print_analysis'
            print_analysis_impl(varargin{:});
            
        case 'export_plots'
            export_plots_impl(varargin{:});
            
        otherwise
            error('Unknown action: %s', action);
    end
end

%% Local Implementation Functions

function save_results_impl(time, pos_history, vel_history, target_history, ...
                          nav_pos_history, ekf_pos_history, ins_error_history, ...
                          ekf_error_history, sensor_status, threat_history, ...
                          guidance_history, range_history, k, final_range, ...
                          final_time, fuel_consumption, avg_improvement)
    %SAVE_RESULTS_IMPL Create and save comprehensive results structure
    %
    % This function packages all simulation data into a structured format
    % for analysis and creates the results file as done in the original
    % 922-line main simulation.
    
    fprintf('\n=== SAVING RESULTS ===\n');
    
    % Create comprehensive results structure
    results = struct();
    
    % Core simulation data
    results.time = time(1:k);
    results.position = pos_history(:, 1:k);
    results.velocity = vel_history(:, 1:k);
    results.target = target_history(:, 1:k);
    
    % Navigation performance data
    results.navigation = struct();
    results.navigation.ins_position = nav_pos_history(:, 1:k);
    results.navigation.ekf_position = ekf_pos_history(:, 1:k);
    results.navigation.ins_error = ins_error_history(1:k);
    results.navigation.ekf_error = ekf_error_history(1:k);
    results.navigation.improvement_percent = avg_improvement;
    
    % Sensor and threat data
    results.sensor_status = sensor_status(:, 1:k);
    results.threat_status = threat_history(:, 1:k);
    results.guidance = guidance_history(:, 1:k);
    results.range_to_target = range_history(1:k);
    
    % Performance metrics (matching original structure)
    results.performance = struct();
    results.performance.final_miss = final_range;
    results.performance.flight_time = final_time;
    results.performance.avg_speed = mean(sqrt(sum(vel_history(:,1:k).^2, 1)));
    results.performance.max_speed = max(sqrt(sum(vel_history(:,1:k).^2, 1)));
    results.performance.fuel_used = fuel_consumption;
    results.performance.ekf_improvement = avg_improvement;
    
    % Mission statistics
    results.statistics = struct();
    results.statistics.avg_ins_error = mean(ins_error_history(1:k));
    results.statistics.avg_ekf_error = mean(ekf_error_history(1:k));
    results.statistics.final_ins_error = ins_error_history(min(k, length(ins_error_history)));
    results.statistics.final_ekf_error = ekf_error_history(min(k, length(ekf_error_history)));
    
    % Sensor utilization statistics
    sensor_names = {'INS', 'GPS', 'TERCOM', 'LASER', 'IR', 'COMM'};
    results.sensor_utilization = struct();
    for i = 1:min(length(sensor_names), size(sensor_status, 1))
        field_name = lower(sensor_names{i});
        results.sensor_utilization.(field_name) = sum(sensor_status(i, 1:k)) / k * 100;
    end
    
    % Threat exposure analysis
    results.threat_analysis = struct();
    results.threat_analysis.total_exposures = sum(sum(threat_history(:, 1:k)));
    results.threat_analysis.time_under_threat = sum(any(threat_history(:, 1:k), 1)) * mean(diff(time(1:min(k,length(time)-1))));
    results.threat_analysis.percent_under_threat = sum(any(threat_history(:, 1:k), 1)) / k * 100;
    
    % Metadata
    results.metadata = struct();
    results.metadata.simulation_date = datestr(now);
    results.metadata.total_samples = k;
    results.metadata.simulation_version = 'Fixed Comprehensive v1.0';
    results.metadata.author = 'James Liu - Columbia University';
    
    % Save results with timestamp
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('fixed_hypersonic_mission_results_%s.mat', timestamp);
    
    try
        save(filename, 'results');
        fprintf('✅ Results saved to %s\n', filename);
        
        % Also save a backup with standard name for easy access
        save('latest_hypersonic_results.mat', 'results');
        fprintf('✅ Latest results saved to latest_hypersonic_results.mat\n');
        
    catch ME
        fprintf('❌ Error saving results: %s\n', ME.message);
        fprintf('Results structure created but not saved to file.\n');
    end
    
    % Return basic info about saved data
    fprintf('📊 Saved data summary:\n');
    fprintf('   - Time samples: %d\n', length(results.time));
    fprintf('   - Flight duration: %.1f seconds\n', results.performance.flight_time);
    fprintf('   - Final miss distance: %.1f meters\n', results.performance.final_miss);
    fprintf('   - EKF improvement: %.1f%%\n', results.performance.ekf_improvement);
end

function print_analysis_impl(results)
    %PRINT_ANALYSIS_IMPL Print comprehensive performance analysis
    %
    % This function prints the detailed analysis that was embedded in the
    % main simulation loop, now extracted for modularity.
    
    fprintf('\n=== FIXED COMPREHENSIVE SIMULATION ANALYSIS ===\n');
    
    % Flight performance analysis
    fprintf('\nFLIGHT PERFORMANCE:\n');
    fprintf('Flight duration: %.1f s (%.2f minutes)\n', ...
            results.performance.flight_time, results.performance.flight_time/60);
    fprintf('Final miss distance: %.1f m\n', results.performance.final_miss);
    fprintf('Average speed: %.1f m/s (Mach %.2f)\n', ...
            results.performance.avg_speed, results.performance.avg_speed/343);
    fprintf('Maximum speed: %.1f m/s (Mach %.2f)\n', ...
            results.performance.max_speed, results.performance.max_speed/343);
    fprintf('Fuel consumption: %.2f kg\n', results.performance.fuel_used);
    
    % Navigation performance analysis
    fprintf('\nNAVIGATION PERFORMANCE:\n');
    fprintf('Average INS error: %.1f m\n', results.statistics.avg_ins_error);
    fprintf('Average EKF error: %.1f m\n', results.statistics.avg_ekf_error);
    fprintf('Final INS error: %.1f m\n', results.statistics.final_ins_error);
    fprintf('Final EKF error: %.1f m\n', results.statistics.final_ekf_error);
    
    % Calculate improvement percentages
    avg_improvement = (results.statistics.avg_ins_error - results.statistics.avg_ekf_error) / ...
                     results.statistics.avg_ins_error * 100;
    final_improvement = (results.statistics.final_ins_error - results.statistics.final_ekf_error) / ...
                       results.statistics.final_ins_error * 100;
    
    fprintf('EKF improvement: %.1f%% average, %.1f%% final\n', avg_improvement, final_improvement);
    
    % Sensor utilization analysis
    fprintf('\nSENSOR UTILIZATION:\n');
    sensor_fields = fieldnames(results.sensor_utilization);
    for i = 1:length(sensor_fields)
        sensor_name = upper(sensor_fields{i});
        availability = results.sensor_utilization.(sensor_fields{i});
        fprintf('%s: %.1f%% availability\n', sensor_name, availability);
    end
    
    % Threat exposure analysis
    fprintf('\nTHREAT EXPOSURE:\n');
    fprintf('Total threat exposures: %d\n', results.threat_analysis.total_exposures);
    fprintf('Time under threat: %.1f s (%.1f%%)\n', ...
            results.threat_analysis.time_under_threat, ...
            results.threat_analysis.percent_under_threat);
    
    % Mission success summary
    fprintf('\n🎯 MISSION ANALYSIS SUMMARY:\n');
    if results.performance.final_miss < 50
        fprintf('✅ Target engagement: SUCCESS (miss < 50m)\n');
    elseif results.performance.final_miss < 200
        fprintf('🟡 Target engagement: PARTIAL SUCCESS (miss < 200m)\n');
    else
        fprintf('❌ Target engagement: MISS (miss > 200m)\n');
    end
    
    if avg_improvement > 20
        fprintf('✅ Navigation system: EXCELLENT (>20%% improvement)\n');
    elseif avg_improvement > 0
        fprintf('🟡 Navigation system: GOOD (positive improvement)\n');
    else
        fprintf('❌ Navigation system: NEEDS IMPROVEMENT (negative improvement)\n');
    end
    
    if results.performance.max_speed/343 > 3
        fprintf('✅ Speed profile: HYPERSONIC (Mach %.1f)\n', results.performance.max_speed/343);
    elseif results.performance.max_speed/343 > 1.5
        fprintf('🟡 Speed profile: SUPERSONIC (Mach %.1f)\n', results.performance.max_speed/343);
    else
        fprintf('❌ Speed profile: SUBSONIC (Mach %.1f)\n', results.performance.max_speed/343);
    end
end

function export_plots_impl(figure_handles)
    %EXPORT_PLOTS_IMPL Export analysis plots to image files
    %
    % Inputs:
    %   figure_handles - Array of figure handles to export
    
    if nargin < 1 || isempty(figure_handles)
        fprintf('No figures provided for export.\n');
        return;
    end
    
    fprintf('\n=== EXPORTING PLOTS ===\n');
    
    % Create export directory if it doesn't exist
    export_dir = 'simulation_plots';
    if ~exist(export_dir, 'dir')
        mkdir(export_dir);
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    for i = 1:length(figure_handles)
        if isvalid(figure_handles(i))
            try
                % Get figure name or create default
                fig_name = get(figure_handles(i), 'Name');
                if isempty(fig_name)
                    fig_name = sprintf('Figure_%d', i);
                end
                
                % Clean filename
                clean_name = regexprep(fig_name, '[^a-zA-Z0-9_-]', '_');
                filename = sprintf('%s/%s_%s', export_dir, clean_name, timestamp);
                
                % Export as high-quality PNG
                print(figure_handles(i), filename, '-dpng', '-r300');
                fprintf('✅ Exported: %s.png\n', filename);
                
            catch ME
                fprintf('❌ Error exporting figure %d: %s\n', i, ME.message);
            end
        end
    end
    
    fprintf('📁 Plots exported to: %s/\n', export_dir);
end