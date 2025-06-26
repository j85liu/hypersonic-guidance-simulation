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
├── docs/
│   ├── project_report.pdf
│   ├── presentation.pptx
│   └── figures/
├── src/
│   ├── main_simulation.m
│   ├── models/
│   │   ├── vehicle_dynamics.m
│   │   ├── atmosphere_model.m
│   │   ├── navigation_system.m
│   │   └── kalman_filter.m
│   ├── simulink/
│   │   ├── hgv_complete_model.slx
│   │   └── subsystems/
│   ├── utils/
│   │   ├── plotting_functions.m
│   │   ├── data_analysis.m
│   │   └── validation_tools.m
│   └── tests/
│       ├── test_vehicle_dynamics.m
│       └── test_kalman_filter.m
├── data/
│   ├── flight_test_data.mat
│   ├── atmospheric_data.mat
│   └── results/
└── scripts/
    ├── run_all_simulations.m
    ├── generate_plots.m
    └── parameter_sweep.m

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
