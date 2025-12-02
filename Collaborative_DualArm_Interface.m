% COLLABORATIVE_DUALARM_INTERFACE.M
% User interface for collaborative dual-arm manipulation with common load
%
% INSTRUCTIONS:
% 1. Edit the CONFIGURATION section below with your robot and task parameters
% 2. Run this script
% 3. The system will automatically:
%    - Generate coordinated trajectories for both robots
%    - Compute forward kinematics and Jacobian analysis
%    - Track the common load throughout the operation
%    - Verify cooperative motion quality
%    - Create visualizations and animations
%    - Export all data
%
% NO MODIFICATIONS needed outside the CONFIGURATION section!

clear all;
close all;
clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║  COLLABORATIVE DUAL-ARM SYSTEM                        ║\n');
fprintf('║  Two Robots Working Together with Common Load         ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  CONFIGURATION - EDIT THIS SECTION ONLY
%  ========================================================================

%% ---------- ROBOT 1 PARAMETERS ----------
fprintf('Configuring Robot 1...\n');

robot1 = struct();

% Link vectors (3xN) - vectors from joint i-1 to joint i
robot1.link_vectors = [
    0,    0,    0,    0,    0,    0;     % x components
    0,    0,    0,    0,    0,    0;     % y components
    0.3,  0.3,  0.25, 0.2,  0.15, 0.1    % z components
];

% Joint axes (3xN) - rotation axes for each joint
robot1.joint_axes = [
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    1, 1, 1, 1, 1, 1
];

% Joint types (string) - 'R' for Revolute
robot1.joint_types = 'RRRRRR';  % All revolute joints

% Base position [x; y; z] in meters
robot1.base_position = [-0.6; 0; 0];

% Base velocity (usually zero for fixed base)
robot1.V_base = zeros(6, 1);

% Initial joint configuration (radians)
robot1.q_initial = [0; 0; 0; 0; 0; 0];

% Final joint configuration (radians)
robot1.q_final = [pi/6; pi/4; pi/6; pi/4; pi/6; pi/4];

fprintf('  ✓ Robot 1: %d-DOF at [%.2f, %.2f, %.2f]\n', ...
        size(robot1.link_vectors, 2), robot1.base_position);

%% ---------- ROBOT 2 PARAMETERS ----------
fprintf('Configuring Robot 2...\n');

robot2 = struct();

% Link vectors (3xN)
robot2.link_vectors = [
    0,    0,    0,    0,    0,    0;
    0,    0,    0,    0,    0,    0;
    0.3,  0.3,  0.25, 0.2,  0.15, 0.1
];

% Joint axes (3xN)
robot2.joint_axes = [
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    1, 1, 1, 1, 1, 1
];

% Joint types
robot2.joint_types = 'RRRRRR';

% Base position
robot2.base_position = [0.6; 0; 0];

% Base velocity
robot2.V_base = zeros(6, 1);

% Initial configuration
robot2.q_initial = [0; 0; 0; 0; 0; 0];

% Final configuration
robot2.q_final = [-pi/6; pi/4; -pi/6; pi/4; -pi/6; pi/4];

fprintf('  ✓ Robot 2: %d-DOF at [%.2f, %.2f, %.2f]\n', ...
        size(robot2.link_vectors, 2), robot2.base_position);

%% ---------- COMMON LOAD PARAMETERS ----------
fprintf('Configuring Common Load...\n');

load_params = struct();

% Load mass (kg)
load_params.mass = 5.0;

% Load size [width, depth, height] in meters
load_params.size = [0.3, 0.2, 0.15];

% Initial load position [x; y; z] - will be computed automatically
load_params.initial_position = [];  % Leave empty for automatic

fprintf('  ✓ Load: %.1f kg, [%.2f × %.2f × %.2f] m\n', ...
        load_params.mass, load_params.size);

%% ---------- TASK PARAMETERS ----------
fprintf('Configuring Task...\n');

task_params = struct();

% Task type: 'pick_and_place', 'transport', 'assembly'
task_params.task_type = 'pick_and_place';

% Trajectory type:
%   1 = Cubic Polynomial
%   2 = Harmonic
%   3 = Cycloidal
task_params.trajectory_type = 2;

% Duration (seconds)
task_params.duration = 10.0;

% Number of samples
task_params.n_samples = 200;

fprintf('  ✓ Task: %s, %.1f seconds, %d samples\n', ...
        task_params.task_type, task_params.duration, task_params.n_samples);

%% ---------- OUTPUT OPTIONS ----------
fprintf('Configuring Output...\n');

% Export prefix for saved files
export_prefix = 'collaborative_dualarm';

% Create plots (true/false)
create_plots = true;

% Export data to files (true/false)
export_data = true;

% Create animation (true/false) - WARNING: Can be slow!
create_animation = false;

fprintf('  ✓ Output: plots=%s, export=%s, animate=%s\n\n', ...
        string(create_plots), string(export_data), string(create_animation));

%% ========================================================================
%  AUTOMATIC EXECUTION - DO NOT MODIFY BELOW THIS LINE
%  ========================================================================

fprintf('Starting collaborative system analysis...\n\n');

%% Perform Analysis
try
    results = Collaborative_DualArm_Functions.analyze_multi_robot_system(...
        robot1, robot2, load_params, task_params);
    
    fprintf('Analysis completed successfully!\n\n');
    
    %% Print Summary
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  COLLABORATIVE SYSTEM SUMMARY              ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    fprintf('ROBOTS:\n');
    fprintf('  Robot 1: %d-DOF, manipulability avg = %.4f\n', ...
            results.robot1.n_links, mean(results.robot1.manipulability));
    fprintf('  Robot 2: %d-DOF, manipulability avg = %.4f\n', ...
            results.robot2.n_links, mean(results.robot2.manipulability));
    
    fprintf('\nLOAD:\n');
    fprintf('  Mass: %.1f kg\n', load_params.mass);
    fprintf('  Grasped time: %.2f%% of operation\n', ...
            100 * sum(results.grasp_states) / length(results.grasp_states));
    
    fprintf('\nCOORDINATION:\n');
    fprintf('  Avg EE separation (grasped): %.4f m\n', ...
            results.coordination.avg_separation_grasped);
    fprintf('  Avg force balance: %.4f\n', ...
            results.coordination.avg_force_balance);
    fprintf('  Avg velocity sync: %.4f m/s\n', ...
            results.coordination.avg_velocity_sync);
    
    fprintf('\nVERIFICATION:\n');
    fprintf('  Grasp stable: %s\n', ...
            tern_str(results.verification.grasp_stable, '✓ YES', '✗ NO'));
    fprintf('  Forces balanced: %s\n', ...
            tern_str(results.verification.forces_balanced, '✓ YES', '⚠ NO'));
    fprintf('  Velocities synced: %s\n', ...
            tern_str(results.verification.velocities_synced, '✓ YES', '⚠ NO'));
    
    if results.verification.cooperative_success
        fprintf('\n  ✓✓✓ EXCELLENT COOPERATION ✓✓✓\n\n');
    else
        fprintf('\n  ⚠ COOPERATION NEEDS IMPROVEMENT\n\n');
    end
    
    %% Create Visualizations
    if create_plots
        fprintf('Creating visualizations...\n');
        Collaborative_DualArm_Functions.visualize_collaborative_system(...
            results, robot1, robot2, load_params);
        fprintf('✓ Plots created\n\n');
    end
    
    %% Create Animation
    if create_animation
        fprintf('Creating animation (this may take a while)...\n');
        Collaborative_DualArm_Functions.animate_collaborative_motion(...
            results, robot1, robot2, load_params);
        fprintf('✓ Animation complete\n\n');
    end
    
    %% Export Data
    if export_data
        fprintf('Exporting data...\n');
        Collaborative_DualArm_Functions.export_collaborative_data(...
            results, robot1, robot2, load_params, export_prefix);
        fprintf('✓ Data exported\n\n');
    end
    
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  ALL OPERATIONS COMPLETE!                  ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    if export_data
        fprintf('Results saved to output/ directory\n');
        fprintf('Files:\n');
        fprintf('  - %s_robot1_joints.csv\n', export_prefix);
        fprintf('  - %s_robot2_joints.csv\n', export_prefix);
        fprintf('  - %s_robot1_ee.csv\n', export_prefix);
        fprintf('  - %s_robot2_ee.csv\n', export_prefix);
        fprintf('  - %s_load_trajectory.csv\n', export_prefix);
        fprintf('  - %s_coordination.csv\n', export_prefix);
        fprintf('  - %s_summary.txt\n\n', export_prefix);
    end
    
    if create_animation
        fprintf('TIP: You can save the animation figure as a video using:\n');
        fprintf('     File → Export Setup → Export...\n\n');
    end
    
catch ME
    fprintf('\n❌ ERROR during analysis:\n');
    fprintf('   %s\n\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('   %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end

%% Helper Function
function result = tern_str(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end
