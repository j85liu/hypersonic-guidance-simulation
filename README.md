# Hypersonic Glide Vehicle Guidance Simulation

A MATLAB/Simulink implementation of GPS-denied navigation and terminal guidance for hypersonic boost-glide vehicles.

## Overview

This project models the complete guidance, navigation, and control system for a hypersonic glide vehicle operating in a GPS-denied environment. The simulation includes:

- 6-DOF vehicle dynamics with thermal effects
- Multi-sensor navigation system
- Extended Kalman Filter for state estimation
- Terminal guidance algorithms
- Atmospheric modeling and uncertainties

## Requirements

- MATLAB R2020a or later
- Control Systems Toolbox
- Aerospace Toolbox
- System Identification Toolbox
- Simulink


## File Structure

```
hypersonic-guidance-simulation/
├── README.md
├── .gitignore
├── LICENSE
├── main_simulation.m                    % Main entry point
├── config/
│   ├── vehicle_config.m                 % Vehicle parameters
│   ├── sensor_config.m                  % Sensor suite configuration
│   ├── threat_config.m                  % SAM sites, EW parameters
│   └── environment_config.m             % Atmosphere, weather, terrain
├── src/
│   ├── models/
│   │   ├── vehicle_dynamics.m           % 6-DOF physics integration
│   │   ├── atmosphere_model.m           % US Standard Atmosphere
│   │   ├── sensor_models.m              % INS, GPS, TERCOM error models
│   │   └── threat_models.m              % SAM detection, plasma effects
│   ├── navigation/
│   │   ├── kalman_filter.m              % EKF implementation
│   │   ├── ins_simulation.m             % INS drift and bias modeling
│   │   └── sensor_fusion.m              % Multi-sensor integration
│   ├── guidance/
│   │   ├── guidance_laws.m              % APN, terminal guidance
│   │   ├── threat_assessment.m          % SAM detection, evasion
│   │   └── trajectory_planning.m        % Path optimization
│   ├── visualization/
│   │   ├── realtime_3d.m               % Live 3D animation
│   │   ├── analysis_plots.m            % Post-flight analysis
│   │   └── info_panel.m                % Telemetry display
│   └── utils/
│       ├── helper_functions.m          % bool_to_status, etc.
│       ├── data_export.m               % Results saving
│       └── performance_metrics.m        % Error calculations
├── simulink/                           % Future Simulink models
│   ├── hgv_complete_model.slx          
│   └── subsystems/
├── tests/                              % Unit tests
│   ├── test_kalman_filter.m
│   ├── test_vehicle_dynamics.m
│   └── test_guidance_laws.m
├── data/                               % Simulation results
│   ├── baseline_results.mat
│   └── monte_carlo_results/
├── docs/                               % Documentation
│   ├── project_report.pdf
│   ├── user_guide.md
│   └── figures/
└── scripts/                            % Batch processing
    ├── run_monte_carlo.m
    ├── parameter_sweep.m
    └── generate_all_plots.m

## Quick Start

```matlab
% Clone and navigate to repository
cd hypersonic-guidance-simulation

% Add all subdirectories to path
addpath(genpath('.'))

% Run main simulation
main_simulation

% Generate all plots
generate_plots
