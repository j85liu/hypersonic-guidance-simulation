function threats = threat_config()
%THREAT_CONFIG Threat environment configuration for hypersonic simulation
%
% This function contains all threat parameters extracted from the main
% simulation comprehensive code (lines 97-110). Provides realistic threat
% environment with SAM sites, electronic warfare, and interceptors.
%
% Outputs:
%   threats - Structure containing all threat configurations
%
% Threat Categories:
%   - Surface-to-Air Missile (SAM) sites
%   - Electronic Warfare (EW) systems
%   - Interceptor threats
%   - Radar and detection systems
%   - Countermeasures and jamming
%
% Author: James Liu - Columbia University
% Part of: Hypersonic Glide Vehicle Guidance Simulation

    threats = struct();
    
    %% SAM Site Configuration (from original lines 100-107)
    % Multiple SAM sites with different capabilities and positions
    
    % SAM site data: [x_pos, y_pos, z_pos, engagement_range, active_flag]
    threats.sam_sites = [
        45000,  8000, 150, 30000, 1;      % Site 1: Medium range SAM
        65000, -12000, 200, 35000, 1;     % Site 2: Long range SAM
        30000,  15000, 100, 25000, 1;     % Site 3: Short range SAM
        50000,  -5000, 300, 40000, 1;     % Site 4: High altitude SAM
        70000,   2000, 250, 32000, 1;     % Site 5: Additional coverage
        25000, -10000, 180, 28000, 1;     % Site 6: Layered defense
    ];
    
    % SAM site characteristics
    threats.sam_characteristics = struct();
    threats.sam_characteristics.reaction_time = [3, 5, 2, 4, 3.5, 2.5];  % Reaction time (s)
    threats.sam_characteristics.missile_speed = [800, 1200, 600, 1000, 900, 700];  % Missile speed (m/s)
    threats.sam_characteristics.max_intercept_altitude = [25000, 35000, 15000, 45000, 30000, 20000];  % Max altitude (m)
    threats.sam_characteristics.min_intercept_altitude = [500, 1000, 200, 2000, 800, 300];  % Min altitude (m)
    threats.sam_characteristics.engagement_duration = [20, 30, 15, 25, 22, 18];  % Engagement time (s)
    
    %% Electronic Warfare Configuration (from original lines 109-112)
    threats.ew = struct();
    
    % Jamming systems
    threats.ew.gps_jammer_power = 0.8;                 % GPS jamming effectiveness (0-1)
    threats.ew.comm_jammer_power = 0.6;                % Communication jamming effectiveness
    threats.ew.radar_jammer_power = 0.4;               % Radar jamming effectiveness
    threats.ew.laser_jammer_power = 0.3;               % Laser jamming effectiveness
    
    % Jammer locations and characteristics
    threats.ew.jammer_sites = [
        40000,  5000, 500, 50000, 1;      % GPS jammer site 1
        60000, -8000, 300, 45000, 1;      % GPS jammer site 2
        35000, 12000, 200, 40000, 1;      % Comm jammer site
        55000,  -2000, 400, 35000, 1;     % Multi-spectrum jammer
    ];
    
    % Jamming effectiveness vs range
    threats.ew.jamming_range_factor = 0.8;             % Effectiveness at maximum range
    threats.ew.jamming_frequency_bands = [1.2, 1.5];   % GPS L1/L2 frequencies (GHz)
    threats.ew.adaptive_jamming = true;                 % Adaptive jamming capability
    threats.ew.jamming_scan_rate = 10;                  % Jamming scan rate (Hz)
    
    %% Interceptor Threat Configuration
    threats.interceptors = struct();
    
    % Interceptor launch sites
    threats.interceptors.launch_sites = [
        20000,  20000, 100, 80000, 1;     % Interceptor base 1
        80000, -15000, 200, 75000, 1;     % Interceptor base 2
        45000,  -20000, 150, 70000, 1;    % Interceptor base 3
    ];
    
    % Interceptor characteristics
    threats.interceptors.max_speed = 1500;              % Maximum interceptor speed (m/s)
    threats.interceptors.max_acceleration = 300;        % Maximum acceleration (m/s²)
    threats.interceptors.max_range = 100000;            % Maximum engagement range (m)
    threats.interceptors.launch_detection_range = 60000; % Detection range for launch (m)
    threats.interceptors.time_to_launch = 15;           % Time from detection to launch (s)
    threats.interceptors.guidance_type = 'proportional'; % Guidance law type
    threats.interceptors.warhead_radius = 50;           % Lethal radius (m)
    
    %% Radar Detection Systems
    threats.radar = struct();
    
    % Radar site locations and characteristics
    threats.radar.sites = [
        30000,  10000, 300, 120000, 1;    % Long range surveillance radar
        70000,  -5000, 250, 80000, 1;     % Medium range tracking radar
        50000,  15000, 400, 100000, 1;    % Multi-function radar
        10000, -20000, 200, 60000, 1;     % Short range radar
    ];
    
    % Radar performance characteristics
    threats.radar.detection_threshold = -30;            % Detection threshold (dBsm)
    threats.radar.false_alarm_rate = 1e-6;             % False alarm rate
    threats.radar.scan_rate = [2, 6, 4, 10];           % Scan rates (rpm)
    threats.radar.frequency_bands = [3, 10, 16, 35];   % Operating frequencies (GHz)
    threats.radar.pulse_repetition_freq = [1000, 2000, 1500, 3000]; % PRF (Hz)
    
    % Radar cross section modeling
    threats.radar.target_rcs = struct();
    threats.radar.target_rcs.nose_on = 0.1;            % Nose-on RCS (m²)
    threats.radar.target_rcs.side_on = 2.0;            % Side-on RCS (m²)
    threats.radar.target_rcs.tail_on = 1.5;            % Tail-on RCS (m²)
    
    %% Countermeasures and Deception
    threats.countermeasures = struct();
    
    % Electronic countermeasures
    threats.countermeasures.chaff_effectiveness = 0.6;  % Chaff effectiveness against radar
    threats.countermeasures.flare_effectiveness = 0.7;  % Flare effectiveness against IR
    threats.countermeasures.ecm_power = 100;            % ECM transmitter power (W)
    threats.countermeasures.frequency_agility = true;   % Frequency agile capability
    
    % Deception techniques
    threats.countermeasures.false_target_generation = true;  % False target capability
    threats.countermeasures.range_gate_pull_off = true;      % RGPO capability
    threats.countermeasures.velocity_gate_pull_off = true;   % VGPO capability
    threats.countermeasures.barrage_jamming = true;          % Barrage jamming
    
    %% Threat Coordination and Command
    threats.command = struct();
    
    % Command and control
    threats.command.coordination_enabled = true;        % Multi-site coordination
    threats.command.data_link_range = 150000;          % C2 data link range (m)
    threats.command.reaction_time = 2.0;               % Command reaction time (s)
    threats.command.engagement_priority = [1, 2, 3, 1, 2, 3]; % SAM site priorities
    
    % Engagement doctrine
    threats.command.simultaneous_engagements = 2;       % Max simultaneous engagements per site
    threats.command.minimum_separation = 5000;          % Minimum engagement separation (m)
    threats.command.handoff_capability = true;          % Target handoff between sites
    threats.command.cooperative_engagement = true;      % Cooperative engagement capability
    
    %% Threat Environment Modeling
    threats.environment = struct();
    
    % Atmospheric effects on threats
    threats.environment.weather_impact = 0.1;           % Weather impact on effectiveness
    threats.environment.visibility_threshold = 5000;    % Minimum visibility for optical systems (m)
    threats.environment.wind_impact = 0.05;            % Wind impact on missile accuracy
    threats.environment.temperature_impact = 0.02;      % Temperature impact on electronics
    
    % Terrain effects
    threats.environment.terrain_masking = true;         % Terrain masking enabled
    threats.environment.multipath_effects = true;       % Radar multipath effects
    threats.environment.ducting_probability = 0.1;      % Atmospheric ducting probability
    
    %% Threat Learning and Adaptation
    threats.adaptation = struct();
    
    % Adaptive threat behavior
    threats.adaptation.learning_enabled = true;         % Enable adaptive behavior
    threats.adaptation.pattern_recognition = true;      % Pattern recognition capability
    threats.adaptation.counter_tactics = true;          % Counter-tactics implementation
    threats.adaptation.memory_length = 300;             % Memory for pattern analysis (s)
    
    % Threat escalation
    threats.adaptation.escalation_threshold = 0.7;      % Escalation probability threshold
    threats.adaptation.backup_site_activation = true;   % Backup site activation
    threats.adaptation.emergency_protocols = true;      % Emergency engagement protocols
    
    %% Configuration Validation and Summary
    threats = validate_threat_config(threats);
    
    fprintf('✅ Threat configuration loaded:\n');
    fprintf('   - SAM sites: %d active\n', sum(threats.sam_sites(:,5)));
    fprintf('   - EW sites: %d active\n', sum(threats.ew.jammer_sites(:,5)));
    fprintf('   - Interceptor bases: %d active\n', sum(threats.interceptors.launch_sites(:,5)));
    fprintf('   - Radar sites: %d active\n', sum(threats.radar.sites(:,5)));
    fprintf('   - GPS jamming power: %.1f%%\n', threats.ew.gps_jammer_power * 100);
end

%% Local Configuration Functions

function threats = validate_threat_config(threats)
    %VALIDATE_THREAT_CONFIG Validate threat configuration parameters
    %
    % Ensures all threat parameters are within reasonable bounds and
    % maintains consistency between related parameters
    
    % Validate SAM site data
    assert(size(threats.sam_sites, 2) == 5, ...
           'SAM sites must have 5 columns: [x, y, z, range, active]');
    assert(all(threats.sam_sites(:,4) > 0), ...
           'All SAM engagement ranges must be positive');
    assert(all(threats.sam_sites(:,5) >= 0 & threats.sam_sites(:,5) <= 1), ...
           'SAM active flags must be 0 or 1');
    
    % Validate EW parameters
    assert(threats.ew.gps_jammer_power >= 0 && threats.ew.gps_jammer_power <= 1, ...
           'GPS jammer power must be between 0 and 1');
    assert(threats.ew.comm_jammer_power >= 0 && threats.ew.comm_jammer_power <= 1, ...
           'Comm jammer power must be between 0 and 1');
    
    % Validate interceptor parameters
    assert(threats.interceptors.max_speed > 0 && threats.interceptors.max_speed < 5000, ...
           'Interceptor max speed must be between 0 and 5000 m/s');
    assert(threats.interceptors.max_acceleration > 0 && threats.interceptors.max_acceleration < 1000, ...
           'Interceptor max acceleration must be between 0 and 1000 m/s²');
    
    % Validate radar parameters
    assert(size(threats.radar.sites, 2) == 5, ...
           'Radar sites must have 5 columns: [x, y, z, range, active]');
    assert(length(threats.radar.scan_rate) == size(threats.radar.sites, 1), ...
           'Scan rate array must match number of radar sites');
    
    % Validate countermeasures
    assert(threats.countermeasures.chaff_effectiveness >= 0 && ...
           threats.countermeasures.chaff_effectiveness <= 1, ...
           'Chaff effectiveness must be between 0 and 1');
    assert(threats.countermeasures.flare_effectiveness >= 0 && ...
           threats.countermeasures.flare_effectiveness <= 1, ...
           'Flare effectiveness must be between 0 and 1');
    
    % Validate command and control
    assert(threats.command.simultaneous_engagements > 0 && ...
           threats.command.simultaneous_engagements <= 10, ...
           'Simultaneous engagements must be between 1 and 10');
    assert(threats.command.minimum_separation > 0, ...
           'Minimum separation must be positive');
    
    % Ensure threat site positions are reasonable
    all_sites = [threats.sam_sites(:,1:3); threats.ew.jammer_sites(:,1:3); ...
                 threats.interceptors.launch_sites(:,1:3); threats.radar.sites(:,1:3)];
    
    assert(all(all_sites(:,3) >= 0), 'All threat site altitudes must be non-negative');
    assert(all(abs(all_sites(:,1)) < 200000), 'All threat site X positions must be < 200 km');
    assert(all(abs(all_sites(:,2)) < 200000), 'All threat site Y positions must be < 200 km');
    assert(all(all_sites(:,3) < 5000), 'All threat site altitudes must be < 5 km');
end