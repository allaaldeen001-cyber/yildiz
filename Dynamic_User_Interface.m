% DYNAMIC_USER_INTERFACE.M
% User interface for dynamic analysis using Spatial Operator Algebra (SOA)
%
% INSTRUCTIONS:
% 1. Edit the CONFIGURATION section below
% 2. Run this script
% 3. The system will automatically:
%    - Generate trajectories
%    - Compute joint torques using recursive Newton-Euler
%    - Calculate power and energy
%    - Create visualizations
%    - Export all data
%
% NO MODIFICATIONS needed outside the CONFIGURATION section!

clear all;
close all;
clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║  DYNAMIC ANALYSIS SYSTEM - SOA FRAMEWORK              ║\n');
fprintf('║  Joint Torque Computation                             ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  CONFIGURATION - EDIT THIS SECTION ONLY
%  ========================================================================

%% ---------- NUMBER OF ROBOTS ----------
fprintf('Configuration Step 1: System Setup\n');

% How many robots? (1, 2, 3, 4, ... any number)
p_robots = 2;  % Example: 2 robots

% How many links per robot? (array of length p_robots)
n_links_per_robot = [6, 6];  % Example: both are 6-DOF

fprintf('  ✓ System: %d robots\n', p_robots);
fprintf('  ✓ DOF: ');
fprintf('%d ', n_links_per_robot);
fprintf('\n\n');

%% ---------- ROBOT 1 PARAMETERS ----------
fprintf('Configuration Step 2: Robot Parameters\n');

robot1 = struct();

% Link vectors (3xN) - vectors from joint i-1 to joint i
robot1.link_vectors = [
    0,    0,    0,    0,    0,    0;     % x components
    0,    0,    0,    0,    0,    0;     % y components
    0.3,  0.3,  0.25, 0.2,  0.15, 0.1    % z components
];

% Joint axes (3xN) - rotation/translation axes for each joint
robot1.joint_axes = [
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    1, 1, 1, 1, 1, 1
];

% Joint types (string) - 'R' for Revolute, 'P' for Prismatic
robot1.joint_types = 'RRRRRR';  % All revolute

% Base position [x; y; z] in meters
robot1.base_position = [-0.6; 0; 0];

% Base velocity (usually zero for fixed base)
robot1.V_base = zeros(6, 1);

% Initial joint configuration (radians for R, meters for P)
robot1.q_initial = [0; 0; 0; 0; 0; 0];

% Final joint configuration
robot1.q_final = [pi/4; pi/6; pi/4; pi/6; pi/4; pi/6];

fprintf('  ✓ Robot 1: %d-DOF configured\n', n_links_per_robot(1));

%% ---------- ROBOT 2 PARAMETERS (if p_robots >= 2) ----------

if p_robots >= 2
    robot2 = struct();
    
    robot2.link_vectors = [
        0,    0,    0,    0,    0,    0;
        0,    0,    0,    0,    0,    0;
        0.3,  0.3,  0.25, 0.2,  0.15, 0.1
    ];
    
    robot2.joint_axes = [
        0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0;
        1, 1, 1, 1, 1, 1
    ];
    
    robot2.joint_types = 'RRRRRR';
    robot2.base_position = [0.6; 0; 0];
    robot2.V_base = zeros(6, 1);
    robot2.q_initial = [0; 0; 0; 0; 0; 0];
    robot2.q_final = [-pi/4; pi/6; -pi/4; pi/6; -pi/4; pi/6];
    
    fprintf('  ✓ Robot 2: %d-DOF configured\n', n_links_per_robot(2));
end

%% Add more robots as needed (robot3, robot4, etc.)

fprintf('\n');

%% ---------- MASS AND INERTIA PARAMETERS ----------
fprintf('Configuration Step 3: Dynamic Parameters\n');

dynamic_params = struct();

% Gravity vector [gx; gy; gz] in m/s^2
dynamic_params.gravity = [0; 0; -9.81];

% Mass parameters for Robot 1
% Each link has: mass (kg), center of mass [x;y;z] (m), inertia tensor (3x3) (kg·m^2)
mass_params_1 = struct('m', {}, 'c', {}, 'I', {});

for i = 1:n_links_per_robot(1)
    mass_params_1(i).m = 2.0;  % 2 kg per link
    mass_params_1(i).c = [0; 0; 0.05];  % CoM at 5cm along link
    mass_params_1(i).I = diag([0.01, 0.01, 0.005]);  % Inertia tensor
end

% Mass parameters for Robot 2 (if multiple robots)
if p_robots >= 2
    mass_params_2 = struct('m', {}, 'c', {}, 'I', {});
    
    for i = 1:n_links_per_robot(2)
        mass_params_2(i).m = 2.0;
        mass_params_2(i).c = [0; 0; 0.05];
        mass_params_2(i).I = diag([0.01, 0.01, 0.005]);
    end
end

% Store mass parameters per robot
dynamic_params.mass_params_per_robot = {mass_params_1};
if p_robots >= 2
    dynamic_params.mass_params_per_robot{2} = mass_params_2;
end

fprintf('  ✓ Dynamic parameters configured\n');
fprintf('    Gravity: [%.2f, %.2f, %.2f] m/s²\n', dynamic_params.gravity);
fprintf('    Link masses: %.1f kg (nominal)\n\n', mass_params_1(1).m);

%% ---------- TRAJECTORY PARAMETERS ----------
fprintf('Configuration Step 4: Trajectory Setup\n');

trajectory_params = struct();

% Trajectory type:
%   1 = Cubic Polynomial
%   2 = Harmonic
%   3 = Cycloidal
%   4 = Gutman 1-3
%   5 = Freudenstein 1-3-5
trajectory_params.type = 2;  % Harmonic

% Duration (seconds)
trajectory_params.duration = 5.0;

% Number of samples
trajectory_params.n_samples = 100;

fprintf('  ✓ Trajectory: Type %d\n', trajectory_params.type);
fprintf('  ✓ Duration: %.1f s\n', trajectory_params.duration);
fprintf('  ✓ Samples: %d\n\n', trajectory_params.n_samples);

%% ---------- MOVING PLATFORM (OPTIONAL) ----------
% Uncomment to enable moving platform

% platform_params = struct();
% platform_params.velocity = [0; 0; 0.1; 0.5; 0; 0];  % [wx;wy;wz; vx;vy;vz]
% platform_params.acceleration = zeros(6, 1);
% enable_moving_platform = true;

enable_moving_platform = false;

%% ---------- COOPERATIVE CONSTRAINTS (OPTIONAL) ----------
% Uncomment to enable constraints

% % Kinematic constraints
% kinematic_constraints = struct();
% kinematic_constraints.type = 'common_load';
% kinematic_constraints.load_separation = 1.0;  % meters
% enable_kinematic_constraints = true;
%
% % Dynamic constraints
% dynamic_constraints = struct();
% dynamic_constraints.force_distribution = [0.5, 0.5];  % 50-50 split
% enable_dynamic_constraints = true;

enable_kinematic_constraints = false;
enable_dynamic_constraints = false;

%% ---------- OUTPUT OPTIONS ----------
fprintf('Configuration Step 5: Output Options\n');

% Export prefix for saved files
export_prefix = 'dynamic_analysis';

% Create plots (true/false)
create_plots = true;

% Export data to files (true/false)
export_data = true;

fprintf('  ✓ Export prefix: %s\n', export_prefix);
fprintf('  ✓ Plots: %s\n', string(create_plots));
fprintf('  ✓ Export: %s\n\n', string(export_data));

%% ========================================================================
%  AUTOMATIC EXECUTION - DO NOT MODIFY BELOW THIS LINE
%  ========================================================================

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('Starting dynamic analysis...\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% Collect robots into cell array
robots = cell(1, p_robots);
robots{1} = robot1;
if p_robots >= 2, robots{2} = robot2; end
% Add robot3, robot4, etc. if defined

%% Run Dynamic Analysis
try
    % Main analysis
    results = Dynamic_SOA_Functions.analyze_multi_robot_dynamics(...
        robots, trajectory_params, dynamic_params);
    
    fprintf('Dynamic analysis completed successfully!\n\n');
    
    %% Apply constraints if enabled
    if enable_kinematic_constraints
        results = Dynamic_SOA_Functions.apply_kinematic_constraints(...
            results, kinematic_constraints);
    end
    
    if enable_dynamic_constraints
        results = Dynamic_SOA_Functions.apply_dynamic_constraints(...
            results, dynamic_constraints);
    end
    
    %% Apply moving platform if enabled
    if enable_moving_platform
        for i = 1:p_robots
            results.robots{i} = Dynamic_SOA_Functions.compute_with_moving_platform(...
                results.robots{i}, robots{i}, platform_params);
        end
    end
    
    %% Print Summary
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  DYNAMIC ANALYSIS SUMMARY                  ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    for i = 1:p_robots
        fprintf('ROBOT %d (%d-DOF):\n', i, results.robots{i}.n_links);
        fprintf('  Peak torques (N·m): ');
        fprintf('%.2f ', results.statistics.robot{i}.peak_torques);
        fprintf('\n');
        fprintf('  Max torque: %.2f N·m\n', results.statistics.robot{i}.max_torque);
        fprintf('  RMS torques (N·m): ');
        fprintf('%.2f ', results.statistics.robot{i}.rms_torques);
        fprintf('\n');
        fprintf('  Peak power: %.2f W\n', results.statistics.robot{i}.peak_power);
        fprintf('  Avg power: %.2f W\n', results.statistics.robot{i}.avg_power);
        fprintf('  Total energy: %.2f J\n\n', results.statistics.robot{i}.total_energy);
    end
    
    %% Create Visualizations
    if create_plots
        fprintf('Creating visualizations...\n');
        Dynamic_SOA_Functions.visualize_dynamics(results, robots, results.time_vector);
        fprintf('✓ Plots created\n\n');
    end
    
    %% Export Data
    if export_data
        fprintf('Exporting data...\n');
        Dynamic_SOA_Functions.export_dynamics_data(results, robots, results.time_vector, export_prefix);
        fprintf('✓ Data exported\n\n');
    end
    
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  ALL OPERATIONS COMPLETE!                  ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    if export_data
        fprintf('Results saved to output/ directory:\n');
        for i = 1:p_robots
            fprintf('  - %s_robot%d_torques.csv\n', export_prefix, i);
            fprintf('  - %s_robot%d_power.csv\n', export_prefix, i);
            fprintf('  - %s_robot%d_energy.csv\n', export_prefix, i);
        end
        fprintf('  - %s_dynamic_summary.txt\n\n', export_prefix);
    end
    
    fprintf('Torque plots show joint torques throughout the motion.\n');
    fprintf('Power plots show instantaneous power and cumulative energy.\n');
    if p_robots > 1
        fprintf('Comparative plots show torques across all robots.\n');
    end
    fprintf('\n');
    
catch ME
    fprintf('\n❌ ERROR during dynamic analysis:\n');
    fprintf('   %s\n\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('   %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end
