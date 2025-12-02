% KINEMATIC_USER_INTERFACE.M
% User interface for kinematic analysis using Spatial Operator Algebra (SOA)
% 
% HOW TO USE:
% 1. Edit the CONFIGURATION section below
% 2. Run this script
% 3. View results and exported data in output/ folder
%
% SUPPORTS:
% - Any number of robots (p)
% - Any number of links per robot (n)
% - All revolute joints on fixed platform
% - 5 trajectory types
% - Complete SOA analysis

clear all;
close all;
clc;

%% ========================================================================
%  CONFIGURATION - EDIT THIS SECTION ONLY
%  ========================================================================

% HOW MANY ROBOTS? (1, 2, 3, 4, ... any number)
p_robots = 2;  % Example: 2 robots

% HOW MANY LINKS PER ROBOT? (array of length p_robots)
% Example: [6, 9] means Robot 1 has 6 links, Robot 2 has 9 links
n_links_per_robot = [6, 9];

%% ---------- ROBOT 1 PARAMETERS ----------

robot1 = struct();

% Link vectors from joint i-1 to joint i (3×n matrix, meters)
robot1.link_vectors = [
    0.3,  0.28, 0.25, 0.22, 0.18, 0.15;   % x components
    0,    0,    0,    0,    0,    0;       % y components
    0,    0,    0,    0,    0,    0        % z components
];

% Joint axes (3×n matrix, will be normalized automatically)
robot1.joint_axes = [
    0,  0,  0,  0,  0,  1;   % x components
    0,  0,  0,  0,  1,  0;   % y components
    1,  1,  1,  1,  0,  0    % z components
];

% Joint types (string of length n, all 'R' for revolute)
robot1.joint_types = 'RRRRRR';

% Base position in global frame (3×1 vector, meters)
robot1.base_position = [-0.5; 0; 0];

% Base velocity (6×1 vector: [angular; linear], fixed platform = zeros)
robot1.V_base = zeros(6, 1);

% Initial joint configuration (n×1 vector, radians)
robot1.q_initial = zeros(6, 1);

% Final joint configuration (n×1 vector, radians)
robot1.q_final = [pi/4; pi/6; pi/4; pi/6; pi/4; pi/6];

%% ---------- ROBOT 2 PARAMETERS ----------

robot2 = struct();

% Link vectors
robot2.link_vectors = [
    0.25, 0.24, 0.22, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10;   % x
    0,    0,    0,    0,    0,    0,    0,    0,    0;       % y
    0,    0,    0,    0,    0,    0,    0,    0,    0        % z
];

% Joint axes
robot2.joint_axes = [
    0,  0,  0,  0,  0,  0,  0,  1,  0;   % x
    0,  0,  0,  0,  0,  1,  0,  0,  1;   % y
    1,  1,  1,  1,  1,  0,  1,  0,  0    % z
];

% Joint types
robot2.joint_types = 'RRRRRRRRR';

% Base position
robot2.base_position = [0.5; 0; 0];

% Base velocity
robot2.V_base = zeros(6, 1);

% Initial configuration
robot2.q_initial = zeros(9, 1);

% Final configuration
robot2.q_final = ones(9, 1) * (-pi/6);

%% ---------- ADD MORE ROBOTS IF NEEDED ----------
% Copy the pattern above to add robot3, robot4, etc.

% robot3 = struct();
% robot3.link_vectors = [...];
% ...

%% ---------- TRAJECTORY PARAMETERS ----------

trajectory_params = struct();

% Duration of motion (seconds)
trajectory_params.duration = 5.0;

% Number of trajectory samples
trajectory_params.n_samples = 100;

% Trajectory type:
%   1 = Cubic Polynomial
%   2 = Harmonic (Sinusoidal)
%   3 = Cycloidal
%   4 = Gutman 1-3 (Fourier)
%   5 = Freudenstein 1-3-5 (Fourier)
trajectory_params.type = 2;

%% ---------- OUTPUT OPTIONS ----------

% Export filename prefix
export_prefix = 'kinematic_analysis';

% Create visualizations?
create_plots = true;

% Export data to files?
export_data = true;

%% ========================================================================
%  AUTOMATIC EXECUTION - DO NOT MODIFY BELOW THIS LINE
%  ========================================================================

fprintf('\n');
fprintf('╔════════════════════════════════════════╗\n');
fprintf('║  KINEMATIC ANALYSIS USING SOA          ║\n');
fprintf('║  Spatial Operator Algebra              ║\n');
fprintf('╚════════════════════════════════════════╝\n');
fprintf('\n');

%% COLLECT ALL ROBOTS INTO CELL ARRAY

robots = cell(1, p_robots);

% Automatically assign robots based on p_robots
switch p_robots
    case 1
        robots{1} = robot1;
    case 2
        robots{1} = robot1;
        robots{2} = robot2;
    case 3
        robots{1} = robot1;
        robots{2} = robot2;
        robots{3} = robot3;
    case 4
        robots{1} = robot1;
        robots{2} = robot2;
        robots{3} = robot3;
        robots{4} = robot4;
    otherwise
        % For more robots, manually assign here or modify the configuration section
        error('For more than 4 robots, please add robot definitions and assignments');
end

%% VALIDATE CONFIGURATION

fprintf('Configuration:\n');
fprintf('  Number of robots (p): %d\n', p_robots);
for i = 1:p_robots
    fprintf('  Robot %d: %d links\n', i, n_links_per_robot(i));
end
fprintf('  Total DOF: %d\n', sum(n_links_per_robot));
fprintf('  Trajectory: %s, %.1fs, %d samples\n\n', ...
        get_trajectory_name(trajectory_params.type), ...
        trajectory_params.duration, trajectory_params.n_samples);

% Verify n_links_per_robot matches actual robot configurations
for i = 1:p_robots
    actual_links = size(robots{i}.link_vectors, 2);
    if actual_links ~= n_links_per_robot(i)
        error('Robot %d: Expected %d links but configuration has %d links', ...
              i, n_links_per_robot(i), actual_links);
    end
end

%% RUN KINEMATIC ANALYSIS

results = Kinematic_SOA_Functions.analyze_multi_robot_system(robots, trajectory_params);

%% DISPLAY RESULTS SUMMARY

fprintf('Results Summary:\n');
fprintf('  Total robots analyzed: %d\n', results.n_robots);
for i = 1:results.n_robots
    fprintf('  Robot %d:\n', i);
    fprintf('    Links: %d\n', results.robots{i}.n_links);
    fprintf('    Avg manipulability: %.6f\n', mean(results.robots{i}.manipulability));
    fprintf('    Avg condition number: %.2f\n', mean(results.robots{i}.condition_number));
    
    ee_travel = norm(results.robots{i}.ee_position(:,end) - results.robots{i}.ee_position(:,1));
    fprintf('    EE travel distance: %.4f m\n', ee_travel);
end
fprintf('\n');

%% CREATE VISUALIZATIONS

if create_plots
    Kinematic_SOA_Functions.visualize_results(results, robots);
end

%% EXPORT DATA

if export_data
    Kinematic_SOA_Functions.export_results(results, robots, export_prefix);
    fprintf('✓ All data exported to output/ folder\n');
end

%% FINAL MESSAGE

fprintf('\n');
fprintf('╔════════════════════════════════════════╗\n');
fprintf('║  ANALYSIS COMPLETE!                    ║\n');
fprintf('╚════════════════════════════════════════╝\n');
fprintf('\n');

fprintf('What was computed:\n');
fprintf('  ✓ Joint trajectories for all robots\n');
fprintf('  ✓ Forward kinematics (position & velocity)\n');
fprintf('  ✓ Jacobian matrices (6×n for each robot)\n');
fprintf('  ✓ H matrices (joint maps)\n');
fprintf('  ✓ Phi matrices (propagation)\n');
fprintf('  ✓ Base trajectory verification\n');
fprintf('  ✓ End-effector trajectory verification\n');

if results.verification.all_correct
    fprintf('\n✓✓✓ ALL TRAJECTORIES FOLLOWED CORRECTLY ✓✓✓\n');
else
    fprintf('\n⚠ Some trajectory errors detected - check output\n');
end

fprintf('\nNext steps:\n');
fprintf('  - View plots for visual analysis\n');
fprintf('  - Check output/ folder for exported data\n');
fprintf('  - Modify CONFIGURATION section to run different scenarios\n');
fprintf('\n');

%% HELPER FUNCTIONS

function name = get_trajectory_name(type)
    names = {'Cubic Polynomial', 'Harmonic', 'Cycloidal', 'Gutman 1-3', 'Freudenstein 1-3-5'};
    name = names{type};
end
