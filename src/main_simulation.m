% main_simulation.m - Entry point
function main_simulation()
    % Clear workspace and add paths
    clear; clc; close all;
    addpath(genpath('.'));
    
    % Run simulation
    results = run_hgv_simulation();
    
    % Generate plots
    generate_plots(results);
    
    % Save results
    save('data/results/simulation_results.mat', 'results');
end