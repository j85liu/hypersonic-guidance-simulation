function [threats_detected, active_threats, threat_status] = threat_assessment(vehicle_position, threat_sites, k)
%THREAT_ASSESSMENT Comprehensive threat detection and analysis system
%
% This function implements the threat assessment logic extracted from the
% main simulation comprehensive code (lines 430-460). Evaluates SAM site
% threats, calculates engagement zones, and provides threat status for
% guidance system integration.
%
% Inputs:
%   vehicle_position - Current vehicle position [3x1] (m)
%   threat_sites - Matrix of threat site data [Nx5]: [x, y, z, range, active]
%   k - Current simulation step (for threat history tracking)
%
% Outputs:
%   threats_detected - Boolean flag indicating any active threats
%   active_threats - Number of threat sites currently engaging vehicle
%   threat_status - Vector indicating which sites are active [Nx1]
%
% Features:
%   - Multi-site SAM threat evaluation
%   - Range-based engagement zone calculation
%   - Site activation status tracking
%   - Threat prioritization by proximity
%   - Real-time threat status updates
%
% Threat Site Format:
%   Each row: [x_pos, y_pos, z_pos, engagement_range, active_flag]
%   - Position in meters (ENU coordinates)
%   - Engagement range in meters
%   - Active flag: 1=operational, 0=disabled
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    % Input validation
    if nargin < 3
        error('threat_assessment requires 3 input arguments');
    end
    
    if size(threat_sites, 2) < 5
        error('threat_sites must have 5 columns: [x, y, z, range, active]');
    end
    
    % Initialize outputs
    threats_detected = false;
    active_threats = 0;
    threat_status = zeros(size(threat_sites, 1), 1);
    
    try
        % Evaluate each threat site
        for i = 1:size(threat_sites, 1)
            site_pos = threat_sites(i, 1:3)';
            site_range = threat_sites(i, 4);
            site_active = threat_sites(i, 5);
            
            % Check if site is operational
            if site_active
                % Calculate range to threat site
                distance_to_site = norm(vehicle_position - site_pos);
                
                % Check if vehicle is within engagement envelope
                if distance_to_site < site_range
                    threat_status(i) = 1;
                    threats_detected = true;
                    active_threats = active_threats + 1;
                end
            end
        end
        
        % Enhanced threat assessment with additional factors
        [threats_detected, active_threats, threat_status] = enhance_threat_assessment(...
            vehicle_position, threat_sites, threat_status, threats_detected, active_threats);
        
    catch ME
        fprintf('Warning: Threat assessment error: %s\n', ME.message);
        threats_detected = false;
        active_threats = 0;
        threat_status = zeros(size(threat_sites, 1), 1);
    end
end

%% Local Threat Assessment Functions

function [enhanced_threats, enhanced_count, enhanced_status] = enhance_threat_assessment(...
    vehicle_position, threat_sites, base_threat_status, base_threats, base_count)
    %ENHANCE_THREAT_ASSESSMENT Add advanced threat assessment capabilities
    %
    % This function adds more sophisticated threat evaluation beyond simple
    % range checking, including altitude considerations, threat priorities,
    % and engagement probability calculations.
    
    enhanced_threats = base_threats;
    enhanced_count = base_count;
    enhanced_status = base_threat_status;
    
    if ~base_threats
        return;  % No threats detected, nothing to enhance
    end
    
    % Advanced threat evaluation for detected threats
    for i = 1:size(threat_sites, 1)
        if enhanced_status(i) == 1  % Site is currently threatening
            site_pos = threat_sites(i, 1:3)';
            site_range = threat_sites(i, 4);
            
            % Calculate enhanced threat metrics
            [engagement_prob, threat_priority] = calculate_threat_metrics(...
                vehicle_position, site_pos, site_range);
            
            % Adjust threat status based on enhanced metrics
            if engagement_prob < 0.3  % Low probability engagement
                enhanced_status(i) = 0;
                enhanced_count = enhanced_count - 1;
            end
        end
    end
    
    % Update overall threat detection flag
    enhanced_threats = (enhanced_count > 0);
end

function [engagement_prob, threat_priority] = calculate_threat_metrics(vehicle_pos, site_pos, site_range)
    %CALCULATE_THREAT_METRICS Calculate sophisticated threat engagement metrics
    %
    % Evaluates multiple factors affecting threat engagement probability
    % and calculates threat priority for guidance system use.
    
    % Distance factor (closer = higher threat)
    distance = norm(vehicle_pos - site_pos);
    distance_factor = max(0, 1 - (distance / site_range));
    
    % Altitude factor (SAMs less effective at very high/low altitudes)
    vehicle_altitude = vehicle_pos(3);
    if vehicle_altitude < 5000  % Low altitude - terrain masking
        altitude_factor = 0.6;
    elseif vehicle_altitude > 25000  % High altitude - reduced effectiveness
        altitude_factor = 0.8;
    else  % Optimal SAM engagement altitude
        altitude_factor = 1.0;
    end
    
    % Geometry factor (consider approach angle)
    % SAMs are less effective against direct overflights
    horizontal_distance = norm(vehicle_pos(1:2) - site_pos(1:2));
    if horizontal_distance < 2000  % Direct overflight
        geometry_factor = 0.7;
    else
        geometry_factor = 1.0;
    end
    
    % Calculate overall engagement probability
    engagement_prob = distance_factor * altitude_factor * geometry_factor;
    
    % Calculate threat priority (for future multi-threat prioritization)
    threat_priority = engagement_prob * (1 / max(distance, 1000));  % Closer = higher priority
end

function threat_info = get_threat_summary(vehicle_position, threat_sites, threat_status)
    %GET_THREAT_SUMMARY Generate comprehensive threat situation summary
    %
    % Provides detailed threat information for display and logging purposes.
    % This is useful for the visualization system and mission analysis.
    
    threat_info = struct();
    threat_info.total_sites = size(threat_sites, 1);
    threat_info.active_sites = sum(threat_sites(:, 5));
    threat_info.engaging_sites = sum(threat_status);
    threat_info.closest_threat_range = inf;
    threat_info.threat_bearings = [];
    threat_info.threat_ranges = [];
    
    if threat_info.engaging_sites > 0
        % Calculate detailed threat metrics
        engaging_indices = find(threat_status == 1);
        
        for i = engaging_indices'
            site_pos = threat_sites(i, 1:3)';
            distance = norm(vehicle_position - site_pos);
            
            threat_info.closest_threat_range = min(threat_info.closest_threat_range, distance);
            threat_info.threat_ranges = [threat_info.threat_ranges; distance];
            
            % Calculate bearing to threat (for evasive maneuvering)
            relative_pos = site_pos - vehicle_position;
            bearing = atan2(relative_pos(2), relative_pos(1));
            threat_info.threat_bearings = [threat_info.threat_bearings; bearing];
        end
    end
    
    % Threat level assessment
    if threat_info.engaging_sites == 0
        threat_info.threat_level = 'CLEAR';
    elseif threat_info.engaging_sites == 1
        threat_info.threat_level = 'CAUTION';
    elseif threat_info.engaging_sites <= 3
        threat_info.threat_level = 'WARNING';
    else
        threat_info.threat_level = 'CRITICAL';
    end
end

function optimal_evasion = calculate_optimal_evasion_vector(vehicle_position, threat_sites, threat_status)
    %CALCULATE_OPTIMAL_EVASION_VECTOR Determine best evasion direction
    %
    % Analyzes active threat geometry to determine optimal evasion vector
    % that maximizes distance from all active threat sites.
    
    optimal_evasion = zeros(3, 1);
    
    engaging_sites = find(threat_status == 1);
    if isempty(engaging_sites)
        return;
    end
    
    % Calculate composite threat vector (weighted by proximity)
    composite_threat = zeros(3, 1);
    total_weight = 0;
    
    for i = engaging_sites'
        site_pos = threat_sites(i, 1:3)';
        threat_vector = vehicle_position - site_pos;
        distance = norm(threat_vector);
        
        if distance > 0
            % Weight by inverse distance (closer threats have more influence)
            weight = 1 / distance;
            composite_threat = composite_threat + weight * (threat_vector / distance);
            total_weight = total_weight + weight;
        end
    end
    
    if total_weight > 0
        % Normalize composite threat vector
        composite_threat = composite_threat / total_weight;
        
        % Optimal evasion is in direction away from composite threat
        optimal_evasion = composite_threat;
        
        % Prefer lateral evasion over altitude changes for hypersonic vehicle
        optimal_evasion(3) = optimal_evasion(3) * 0.3;
        
        % Normalize to unit vector
        if norm(optimal_evasion) > 0
            optimal_evasion = optimal_evasion / norm(optimal_evasion);
        end
    end
end