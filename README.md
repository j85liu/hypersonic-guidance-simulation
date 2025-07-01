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
└── src/
    ├── config/
    │   ├── simulation_config.m          % Simulation parameters and settings
    │   ├── vehicle_config.m             % Vehicle properties and parameters
    │   ├── sensor_config.m              % Sensor suite configuration
    │   ├── threat_config.m              % Threat environment setup
    │   └── environment_config.m         % Atmospheric and environmental parameters
    ├── models/
    │   ├── vehicle_dynamics.m           % 6-DOF vehicle dynamics integration
    │   └── atmosphere_model.m           % Atmospheric properties calculation
    ├── navigation/
    │   ├── navigation_system.m          % Multi-sensor navigation manager
    │   ├── extended_kalman_filter.m     % EKF implementation
    │   ├── ins_simulation.m             % INS error modeling
    │   └── sensor_measurements.m        % Individual sensor measurement models
    ├── guidance/
    │   ├── guidance_system.m            % Main guidance law coordinator
    │   └── threat_assessment.m          % Threat detection and evasion
    ├── visualization/
    │   ├── real_time_display.m          % 3D real-time visualization
    │   └── analysis_plots.m             % Post-simulation analysis plots
    └── utils/
        ├── helper_functions.m           % Utility functions (bool_to_status, etc.)
        └── data_export.m                % Results saving and export

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
