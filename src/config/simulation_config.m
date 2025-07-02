function config = simulation_config()
%SIMULATION_CONFIG Master simulation configuration for hypersonic guidance
%
% This function contains all simulation parameters extracted from the main
% simulation comprehensive code (lines 15-45). Provides centralized
% configuration management for the entire hypersonic guidance simulation.
%
% Outputs:
%   config - Structure containing all simulation configuration parameters
%
% Configuration Categories:
%   - Simulation timing and integration
%   - Mission scenario parameters  
%   - Feature enable/disable flags
%   - Performance and analysis settings
%   - Random seed and repeatability
%
% Usage:
%   sim_config = simulation_config();
%   dt = sim_config.timing.dt;
%   enable_viz = sim_config.features.enable_real_time_viz;
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    config = struct();
    
    %% Simulation Timing Configuration
    config.timing = struct();
    config.timing.dt = 0.05;              % Time step (s)
    config.timing.t_final = 180;          % Total simulation time (s) - 3 minutes
    config.timing.progress_update_steps = 20;  % Number of progress updates during sim
    
    % Derived timing parameters
    config.timing.time_vector = 0:config.timing.dt:config.timing.t_final;
    config.timing.N = length(config.timing.time_vector);
    
    %% Mission Scenario Configuration
    config.scenario = struct();
    config.scenario.mission_type = 'terminal_guidance';  % Mission type identifier
    config.scenario.target_type = 'moving_evasive';     % Target behavior type
    config.scenario.threat_level = 'high';              % Threat environment level
    config.scenario.weather_conditions = 'nominal';     % Weather conditions
    
    %% Feature Enable/Disable Flags (from original lines 25-30)
    config.features = struct();
    config.features.enable_gps_jamming = true;        % GPS denial simulation
    config.features.enable_plasma_effects = true;     % Plasma interference
    config.features.enable_thermal_effects = true;    % Thermal heating effects
    config.features.enable_moving_target = true;      % Target evasive maneuvers
    config.features.enable_interceptors = true;       % Multiple threat sites
    config.features.enable_real_time_viz = true;      % 3D visualization
    config.features.enable_detailed_logging = true;   % Comprehensive data logging
    
    %% Analysis and Output Configuration
    config.analysis = struct();
    config.analysis.create_plots = true;              % Generate analysis plots
    config.analysis.save_results = true;              % Save simulation results
    config.analysis.export_plots = false;             % Export plots to files
    config.analysis.print_progress = true;            % Console progress updates
    config.analysis.detailed_summary = true;          % Detailed performance summary
    
    %% Visualization Configuration
    config.visualization = struct();
    config.visualization.update_rate = 5;             % Update every N simulation steps
    config.visualization.trail_length = 100;          % Number of trail points
    config.visualization.camera_follow_rate = 20;     % Camera update every N steps
    config.visualization.info_panel_enabled = true;   % Real-time info display
    
    %% Performance and Accuracy Settings
    config.performance = struct();
    config.performance.integration_method = 'euler';   % Integration method
    config.performance.error_tolerance = 1e-6;        % Numerical tolerance
    config.performance.max_iterations = 1000;         % Maximum solver iterations
    config.performance.convergence_criteria = 1e-8;   % Convergence threshold
    
    %% Random Seed and Repeatability (from original line 35)
    config.random = struct();
    config.random.seed = 42;                          % Fixed seed for repeatability
    config.random.enable_variations = true;          % Enable random variations
    config.random.variation_intensity = 1.0;         % Scaling factor for variations
    
    % Set random seed for repeatable results
    rng(config.random.seed);
    
    %% File and Directory Configuration
    config.files = struct();
    config.files.results_directory = 'results';       % Results output directory
    config.files.plots_directory = 'plots';          % Plots output directory
    config.files.data_directory = 'data';            % Input data directory
    config.files.log_directory = 'logs';             % Log files directory
    config.files.base_filename = 'hypersonic_sim';   % Base filename for outputs
    
    %% Debug and Development Settings
    config.debug = struct();
    config.debug.verbose_output = false;             % Detailed console output
    config.debug.enable_assertions = true;          % Enable runtime checks
    config.debug.break_on_warnings = false;         % Stop on warnings
    config.debug.profile_performance = false;       % Enable performance profiling
    config.debug.validate_inputs = true;            % Input validation checks
    
    %% Simulation Metadata
    config.metadata = struct();
    config.metadata.version = '1.0.0';               % Simulation version
    config.metadata.author = 'James Liu';            % Author information
    config.metadata.organization = 'Columbia University';  % Organization
    config.metadata.description = 'Fixed Comprehensive Hypersonic Guidance Simulation';
    config.metadata.creation_date = datestr(now);    % Configuration creation time
    
    %% Validation
    config = validate_simulation_config(config);
    
    if config.debug.verbose_output
        fprintf('✅ Simulation configuration loaded successfully\n');
        fprintf('   - Simulation time: %.1f seconds\n', config.timing.t_final);
        fprintf('   - Time step: %.3f seconds\n', config.timing.dt);
        fprintf('   - Total steps: %d\n', config.timing.N);
        fprintf('   - Random seed: %d\n', config.random.seed);
    end
end

%% Local Configuration Functions

function config = validate_simulation_config(config)
    %VALIDATE_SIMULATION_CONFIG Validate configuration parameters
    %
    % Ensures all configuration parameters are within reasonable bounds
    % and maintains consistency between related parameters
    
    % Validate timing parameters
    assert(config.timing.dt > 0 && config.timing.dt < 1, ...
           'Time step must be between 0 and 1 second');
    assert(config.timing.t_final > 0 && config.timing.t_final < 3600, ...
           'Final time must be between 0 and 3600 seconds');
    
    % Validate visualization parameters
    config.visualization.update_rate = max(1, config.visualization.update_rate);
    config.visualization.trail_length = max(10, min(1000, config.visualization.trail_length));
    
    % Validate performance parameters
    config.performance.error_tolerance = max(1e-12, config.performance.error_tolerance);
    config.performance.max_iterations = max(100, config.performance.max_iterations);
    
    % Validate random parameters
    config.random.variation_intensity = max(0, min(5, config.random.variation_intensity));
    
    % Ensure directories exist if saving is enabled
    if config.analysis.save_results || config.analysis.export_plots
        create_output_directories(config);
    end
end

function create_output_directories(config)
    %CREATE_OUTPUT_DIRECTORIES Create necessary output directories
    
    directories = {config.files.results_directory, ...
                  config.files.plots_directory, ...
                  config.files.log_directory};
    
    for i = 1:length(directories)
        if ~exist(directories{i}, 'dir')
            try
                mkdir(directories{i});
                if config.debug.verbose_output
                    fprintf('Created directory: %s\n', directories{i});
                end
            catch ME
                warning('Could not create directory %s: %s', directories{i}, ME.message);
            end
        end
    end
end