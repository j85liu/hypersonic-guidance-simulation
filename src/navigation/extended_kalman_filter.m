function varargout = extended_kalman_filter(action, varargin)
%EXTENDED_KALMAN_FILTER Multi-sensor Extended Kalman Filter implementation
%
% This function provides a clean, standalone EKF implementation extracted
% and enhanced from the main simulation comprehensive code. Designed for
% hypersonic vehicle navigation with multiple sensor types and GPS-denied
% operation capability.
%
% Usage:
%   ekf_handle = extended_kalman_filter('initialize', initial_state, initial_P, Q)
%   ekf_handle = extended_kalman_filter('predict', ekf_handle, F, dt)
%   ekf_handle = extended_kalman_filter('update', ekf_handle, measurement, H, R)
%   [state, P, innovation] = extended_kalman_filter('get_state', ekf_handle)
%   extended_kalman_filter('reset', ekf_handle, new_state, new_P)
%
% Actions:
%   'initialize' - Create new EKF instance
%   'predict' - Perform time update (prediction step)
%   'update' - Perform measurement update
%   'get_state' - Retrieve current state and covariance
%   'reset' - Reset filter state and covariance
%   'get_innovation' - Get latest innovation statistics
%   'set_process_noise' - Update process noise matrix
%   'validate' - Check filter health and divergence
%
% State Vector (6-DOF):
%   [position(3), velocity(3)] - Position and velocity in ENU coordinates
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    switch action
        case 'initialize'
            varargout{1} = initialize_ekf(varargin{:});
            
        case 'predict'
            varargout{1} = predict_ekf(varargin{:});
            
        case 'update'
            varargout{1} = update_ekf(varargin{:});
            
        case 'get_state'
            [varargout{1}, varargout{2}, varargout{3}] = get_ekf_state(varargin{:});
            
        case 'reset'
            varargout{1} = reset_ekf(varargin{:});
            
        case 'get_innovation'
            varargout{1} = get_innovation_stats(varargin{:});
            
        case 'set_process_noise'
            varargout{1} = set_process_noise(varargin{:});
            
        case 'validate'
            varargout{1} = validate_ekf_health(varargin{:});
            
        otherwise
            error('Unknown EKF action: %s', action);
    end
end

%% Local EKF Implementation Functions

function ekf = initialize_ekf(initial_state, initial_P, process_noise_Q)
    %INITIALIZE_EKF Create new Extended Kalman Filter instance
    %
    % Inputs:
    %   initial_state - Initial state vector [6x1]
    %   initial_P - Initial covariance matrix [6x6]
    %   process_noise_Q - Process noise matrix [6x6]
    %
    % Outputs:
    %   ekf - EKF structure handle
    
    if nargin < 3
        % Default process noise for hypersonic vehicle navigation
        process_noise_Q = diag([1^2, 1^2, 1^2, 0.5^2, 0.5^2, 0.3^2]);
    end
    
    if nargin < 2
        % Conservative initial uncertainty
        initial_P = diag([50^2, 50^2, 30^2, 10^2, 10^2, 5^2]);
    end
    
    if nargin < 1
        % Default initial state
        initial_state = zeros(6, 1);
    end
    
    % Validate inputs
    assert(length(initial_state) == 6, 'Initial state must be 6 elements');
    assert(all(size(initial_P) == [6, 6]), 'Initial covariance must be 6x6');
    assert(all(size(process_noise_Q) == [6, 6]), 'Process noise must be 6x6');
    
    % Create EKF structure
    ekf = struct();
    ekf.state = initial_state(:);  % Ensure column vector
    ekf.P = initial_P;
    ekf.Q = process_noise_Q;
    
    % Innovation statistics
    ekf.innovation = struct();
    ekf.innovation.value = [];
    ekf.innovation.covariance = [];
    ekf.innovation.normalized = [];
    ekf.innovation.history = [];
    
    % Filter health monitoring
    ekf.health = struct();
    ekf.health.iteration_count = 0;
    ekf.health.divergence_flag = false;
    ekf.health.last_update_time = 0;
    ekf.health.condition_number = cond(initial_P);
    
    % Adaptive parameters
    ekf.adaptive = struct();
    ekf.adaptive.enabled = false;
    ekf.adaptive.innovation_window = 10;
    ekf.adaptive.scaling_factor = 1.0;
    
    % Store creation timestamp
    ekf.timestamp = now;
end

function ekf = predict_ekf(ekf, F, dt, Q_override)
    %PREDICT_EKF Perform EKF prediction step (time update)
    %
    % Inputs:
    %   ekf - EKF structure
    %   F - State transition matrix [6x6] or [] for default
    %   dt - Time step (s)
    %   Q_override - Optional process noise override
    %
    % Outputs:
    %   ekf - Updated EKF structure
    
    if nargin < 4
        Q_override = [];
    end
    
    if nargin < 2 || isempty(F)
        % Default constant velocity model
        F = [eye(3), dt*eye(3);
             zeros(3), eye(3)];
    end
    
    assert(nargin >= 3, 'Time step dt is required');
    assert(all(size(F) == [6, 6]), 'State transition matrix must be 6x6');
    
    % Use override process noise if provided
    Q = isempty(Q_override) ? ekf.Q : Q_override;
    
    try
        % Prediction equations
        ekf.state = F * ekf.state;
        ekf.P = F * ekf.P * F' + Q;
        
        % Ensure positive definite covariance
        ekf.P = make_positive_definite(ekf.P);
        
        % Update health monitoring
        ekf.health.iteration_count = ekf.health.iteration_count + 1;
        ekf.health.condition_number = cond(ekf.P);
        
        % Check for divergence
        if ekf.health.condition_number > 1e12
            ekf.health.divergence_flag = true;
            warning('EKF covariance matrix is becoming ill-conditioned');
        end
        
    catch ME
        warning('EKF prediction step failed: %s', ME.message);
        % Maintain previous state if prediction fails
    end
end

function ekf = update_ekf(ekf, measurement, H, R, sensor_type)
    %UPDATE_EKF Perform EKF measurement update step
    %
    % Inputs:
    %   ekf - EKF structure
    %   measurement - Measurement vector
    %   H - Measurement matrix
    %   R - Measurement noise covariance
    %   sensor_type - Optional sensor identifier for adaptive weighting
    %
    % Outputs:
    %   ekf - Updated EKF structure
    
    if nargin < 5
        sensor_type = 'unknown';
    end
    
    % Validate inputs
    measurement = measurement(:);  % Ensure column vector
    m = length(measurement);
    
    assert(size(H, 1) == m, 'Measurement matrix rows must match measurement size');
    assert(size(H, 2) == 6, 'Measurement matrix must have 6 columns');
    assert(all(size(R) == [m, m]), 'Measurement noise must be square matrix matching measurement size');
    
    try
        % Predicted measurement
        predicted_measurement = H * ekf.state;
        
        % Innovation
        innovation = measurement - predicted_measurement;
        
        % Innovation covariance
        S = H * ekf.P * H' + R;
        
        % Ensure positive definite innovation covariance
        S = make_positive_definite(S);
        
        % Kalman gain
        K = ekf.P * H' / S;
        
        % State update
        ekf.state = ekf.state + K * innovation;
        
        % Covariance update (Joseph form for numerical stability)
        I_KH = eye(6) - K * H;
        ekf.P = I_KH * ekf.P * I_KH' + K * R * K';
        
        % Ensure positive definite covariance
        ekf.P = make_positive_definite(ekf.P);
        
        % Store innovation statistics
        ekf.innovation.value = innovation;
        ekf.innovation.covariance = S;
        ekf.innovation.normalized = innovation' / S * innovation;
        
        % Update innovation history for adaptive filtering
        if length(ekf.innovation.history) >= ekf.adaptive.innovation_window
            ekf.innovation.history = ekf.innovation.history(2:end);
        end
        ekf.innovation.history(end+1) = ekf.innovation.normalized;
        
        % Adaptive measurement noise scaling
        if ekf.adaptive.enabled
            ekf = apply_adaptive_scaling(ekf, sensor_type);
        end
        
        % Update health monitoring
        ekf.health.last_update_time = now;
        ekf.health.condition_number = cond(ekf.P);
        
        % Innovation-based divergence detection
        if ekf.innovation.normalized > 25  % Chi-square threshold
            ekf.health.divergence_flag = true;
            warning('EKF innovation indicates possible divergence');
        end
        
    catch ME
        warning('EKF measurement update failed: %s', ME.message);
        % Skip this measurement update if it fails
    end
end

function [state, P, innovation_stats] = get_ekf_state(ekf)
    %GET_EKF_STATE Retrieve current EKF state and statistics
    %
    % Outputs:
    %   state - Current state estimate [6x1]
    %   P - Current covariance matrix [6x6]
    %   innovation_stats - Innovation statistics structure
    
    state = ekf.state;
    P = ekf.P;
    innovation_stats = ekf.innovation;
end

function ekf = reset_ekf(ekf, new_state, new_P)
    %RESET_EKF Reset EKF state and covariance
    %
    % Inputs:
    %   ekf - EKF structure
    %   new_state - New state vector [6x1]
    %   new_P - New covariance matrix [6x6]
    
    if nargin >= 2
        assert(length(new_state) == 6, 'New state must be 6 elements');
        ekf.state = new_state(:);
    end
    
    if nargin >= 3
        assert(all(size(new_P) == [6, 6]), 'New covariance must be 6x6');
        ekf.P = make_positive_definite(new_P);
    end
    
    % Reset health monitoring
    ekf.health.divergence_flag = false;
    ekf.health.condition_number = cond(ekf.P);
    
    % Clear innovation history
    ekf.innovation.history = [];
end

function innovation_stats = get_innovation_stats(ekf)
    %GET_INNOVATION_STATS Get innovation-based filter performance statistics
    
    innovation_stats = ekf.innovation;
    
    if ~isempty(ekf.innovation.history)
        innovation_stats.mean_normalized = mean(ekf.innovation.history);
        innovation_stats.std_normalized = std(ekf.innovation.history);
        innovation_stats.consistency_ratio = innovation_stats.mean_normalized;
    else
        innovation_stats.mean_normalized = NaN;
        innovation_stats.std_normalized = NaN;
        innovation_stats.consistency_ratio = NaN;
    end
end

function ekf = set_process_noise(ekf, new_Q)
    %SET_PROCESS_NOISE Update process noise matrix
    
    assert(all(size(new_Q) == [6, 6]), 'Process noise must be 6x6');
    ekf.Q = new_Q;
end

function health_status = validate_ekf_health(ekf)
    %VALIDATE_EKF_HEALTH Check EKF health and detect potential issues
    
    health_status = struct();
    health_status.healthy = true;
    health_status.warnings = {};
    health_status.errors = {};
    
    % Check for NaN or Inf in state
    if any(isnan(ekf.state)) || any(isinf(ekf.state))
        health_status.healthy = false;
        health_status.errors{end+1} = 'State contains NaN or Inf values';
    end
    
    % Check covariance matrix properties
    try
        % Check positive definiteness
        [~, flag] = chol(ekf.P);
        if flag ~= 0
            health_status.healthy = false;
            health_status.errors{end+1} = 'Covariance matrix is not positive definite';
        end
        
        % Check condition number
        if cond(ekf.P) > 1e10
            health_status.warnings{end+1} = 'Covariance matrix is ill-conditioned';
        end
        
        % Check diagonal elements (variances must be positive)
        if any(diag(ekf.P) <= 0)
            health_status.healthy = false;
            health_status.errors{end+1} = 'Negative variance detected';
        end
        
    catch
        health_status.healthy = false;
        health_status.errors{end+1} = 'Covariance matrix validation failed';
    end
    
    % Check innovation consistency
    if ~isempty(ekf.innovation.history) && length(ekf.innovation.history) > 5
        mean_innovation = mean(ekf.innovation.history);
        if mean_innovation > 10  % Threshold for poor consistency
            health_status.warnings{end+1} = 'Poor innovation consistency detected';
        end
    end
    
    % Check for divergence flag
    if ekf.health.divergence_flag
        health_status.healthy = false;
        health_status.errors{end+1} = 'Filter divergence detected';
    end
    
    % Store overall health metrics
    health_status.condition_number = ekf.health.condition_number;
    health_status.iterations = ekf.health.iteration_count;
    health_status.last_update = ekf.health.last_update_time;
end

function ekf = apply_adaptive_scaling(ekf, sensor_type)
    %APPLY_ADAPTIVE_SCALING Apply adaptive measurement noise scaling
    %
    % Adjusts measurement noise based on innovation consistency
    
    if length(ekf.innovation.history) < 5
        return;  % Need sufficient history for adaptation
    end
    
    % Calculate innovation consistency ratio
    expected_innovation = 1.0;  % Expected normalized innovation for consistent filter
    actual_innovation = mean(ekf.innovation.history(end-4:end));  % Last 5 measurements
    
    % Adaptation based on consistency
    if actual_innovation > 2 * expected_innovation
        % Innovation too large - increase measurement noise
        ekf.adaptive.scaling_factor = min(2.0, ekf.adaptive.scaling_factor * 1.1);
    elseif actual_innovation < 0.5 * expected_innovation
        % Innovation too small - decrease measurement noise
        ekf.adaptive.scaling_factor = max(0.5, ekf.adaptive.scaling_factor * 0.95);
    end
    
    % Apply scaling gradually to avoid instability
    ekf.adaptive.scaling_factor = 0.9 * ekf.adaptive.scaling_factor + 0.1 * 1.0;
end

function P_pd = make_positive_definite(P)
    %MAKE_POSITIVE_DEFINITE Ensure matrix is positive definite
    %
    % Uses eigenvalue decomposition to enforce positive definiteness
    
    try
        % Check if already positive definite
        [~, flag] = chol(P);
        if flag == 0
            P_pd = P;
            return;
        end
        
        % Eigenvalue decomposition
        [V, D] = eig(P);
        
        % Ensure real eigenvalues (should be for covariance matrix)
        D = real(D);
        V = real(V);
        
        % Set minimum eigenvalue
        min_eigenvalue = 1e-12;
        D(D < min_eigenvalue) = min_eigenvalue;
        
        % Reconstruct positive definite matrix
        P_pd = V * D * V';
        
        % Ensure symmetry
        P_pd = (P_pd + P_pd') / 2;
        
    catch
        % Fallback: use diagonal matrix with small values
        warning('Failed to make matrix positive definite, using diagonal fallback');
        P_pd = eye(size(P)) * 1e-6;
    end
end