% EXAMPLE_COLLABORATIVE_QUICK.M
% Quick example of two robots working together with a common load
%
% This is a pre-configured example demonstrating:
% - Two 3-DOF robots
% - Pick-and-place task with common load
% - Coordinated motion
% - Load tracking and visualization
%
% USAGE: Simply run this file
%   >> example_collaborative_quick

clear all;
close all;
clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║  QUICK COLLABORATIVE DUAL-ARM EXAMPLE                 ║\n');
fprintf('║  Two 3-DOF Robots with Common Load                    ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% Define Robot 1 (Left arm - 3-DOF)
fprintf('Configuring Robot 1 (Left Arm)...\n');

robot1 = struct();

% Simple 3-DOF vertical configuration
robot1.link_vectors = [
    0,    0,    0;     % x components
    0,    0,    0;     % y components
    0.4,  0.4,  0.3    % z components (1.1m total height)
];

% All joints rotate about Z-axis
robot1.joint_axes = [
    0, 0, 0;
    0, 0, 0;
    1, 1, 1
];

robot1.joint_types = 'RRR';
robot1.base_position = [-0.5; 0; 0];  % 0.5m to the left
robot1.V_base = zeros(6, 1);
robot1.q_initial = [0; 0; 0];  % Straight up
robot1.q_final = [pi/6; pi/4; pi/6];  % Bent configuration

fprintf('  ✓ Robot 1: 3-DOF at [-0.50, 0, 0]\n');

%% Define Robot 2 (Right arm - 3-DOF)
fprintf('Configuring Robot 2 (Right Arm)...\n');

robot2 = struct();

% Mirror configuration
robot2.link_vectors = [
    0,    0,    0;
    0,    0,    0;
    0.4,  0.4,  0.3
];

robot2.joint_axes = [
    0, 0, 0;
    0, 0, 0;
    1, 1, 1
];

robot2.joint_types = 'RRR';
robot2.base_position = [0.5; 0; 0];  % 0.5m to the right
robot2.V_base = zeros(6, 1);
robot2.q_initial = [0; 0; 0];
robot2.q_final = [-pi/6; pi/4; -pi/6];  % Mirror of robot 1

fprintf('  ✓ Robot 2: 3-DOF at [0.50, 0, 0]\n');

%% Define Common Load
fprintf('Configuring Common Load...\n');

load_params = struct();
load_params.mass = 3.0;  % 3 kg box
load_params.size = [0.4, 0.3, 0.15];  % 40cm × 30cm × 15cm

fprintf('  ✓ Load: %.1f kg, [%.2f × %.2f × %.2f] m\n\n', ...
        load_params.mass, load_params.size);

%% Define Task Parameters
fprintf('Configuring Task...\n');

task_params = struct();
task_params.task_type = 'pick_and_place';  % 8-phase cycle
task_params.trajectory_type = 2;  % Harmonic (smooth)
task_params.duration = 8.0;  % 8 seconds
task_params.n_samples = 150;  % 150 samples

fprintf('  ✓ Task: pick_and_place, 8.0 seconds\n\n');

%% Run Analysis
fprintf('═══════════════════════════════════════════════════════\n');
fprintf('Running collaborative analysis...\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

try
    results = Collaborative_DualArm_Functions.analyze_multi_robot_system(...
        robot1, robot2, load_params, task_params);
    
    fprintf('\n✓ Analysis completed successfully!\n\n');
    
    %% Print Summary
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  RESULTS SUMMARY                           ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    fprintf('ROBOTS:\n');
    fprintf('  Robot 1 avg manipulability: %.4f\n', mean(results.robot1.manipulability));
    fprintf('  Robot 2 avg manipulability: %.4f\n', mean(results.robot2.manipulability));
    
    fprintf('\nLOAD:\n');
    fprintf('  Mass: %.1f kg\n', load_params.mass);
    fprintf('  Grasped: %.1f%% of operation\n', ...
            100 * sum(results.grasp_states) / length(results.grasp_states));
    
    fprintf('\nCOORDINATION METRICS:\n');
    fprintf('  EE separation (grasped): %.4f m\n', results.coordination.avg_separation_grasped);
    fprintf('  Force balance: %.4f\n', results.coordination.avg_force_balance);
    fprintf('  Velocity sync: %.4f m/s\n', results.coordination.avg_velocity_sync);
    
    fprintf('\nCOOPERATIVE PERFORMANCE:\n');
    if results.verification.cooperative_success
        fprintf('  ✓✓✓ EXCELLENT COOPERATION ✓✓✓\n\n');
    else
        fprintf('  ⚠ Some metrics need improvement\n\n');
    end
    
    %% Create Visualizations
    fprintf('Creating visualizations...\n\n');
    
    % Use the built-in visualization
    Collaborative_DualArm_Functions.visualize_collaborative_system(...
        results, robot1, robot2, load_params);
    
    fprintf('✓ All visualizations created\n\n');
    
    %% Create Animation
    fprintf('Creating animation...\n');
    Collaborative_DualArm_Functions.animate_collaborative_motion(...
        results, robot1, robot2, load_params);
    
    %% Additional Workspace Figure
    fprintf('Creating detailed workspace view...\n');
    
    % Figure: Detailed workspace with robot configurations
    figure('Name', 'Collaborative Workspace - Detailed', 'Position', [50, 50, 1200, 800]);
    
    % Robot 1 path
    plot3(results.robot1.ee_position(1,:), results.robot1.ee_position(2,:), ...
          results.robot1.ee_position(3,:), 'b-', 'LineWidth', 2);
    hold on;
    
    % Robot 2 path
    plot3(results.robot2.ee_position(1,:), results.robot2.ee_position(2,:), ...
          results.robot2.ee_position(3,:), 'r-', 'LineWidth', 2);
    
    % Load trajectory (when grasped)
    grasped_idx = find(results.grasp_states);
    if ~isempty(grasped_idx)
        plot3(results.load.position(1, grasped_idx), ...
              results.load.position(2, grasped_idx), ...
              results.load.position(3, grasped_idx), ...
              'g-', 'LineWidth', 3);
    end
    
    % Bases
    scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
             300, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
             300, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    grid on;
    xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Collaborative Dual-Arm System - Workspace', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Robot 1 Path', 'Robot 2 Path', 'Load Path (Grasped)', 'Robot 1 Base', 'Robot 2 Base', ...
           'Location', 'best', 'FontSize', 10);
    axis equal;
    view(45, 30);
    hold off;
    
    % Figure 2: Coordination metrics
    figure('Name', 'Coordination Analysis', 'Position', [100, 100, 1400, 600]);
    
    subplot(1, 3, 1);
    plot(results.time_vector, results.coordination.ee_separation, 'k-', 'LineWidth', 2);
    hold on;
    if ~isempty(grasped_idx)
        plot(results.time_vector(grasped_idx), ...
             results.coordination.ee_separation(grasped_idx), 'g-', 'LineWidth', 3);
    end
    grid on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Distance (m)', 'FontSize', 11);
    title('End-Effector Separation', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Separation', 'Grasped Phase', 'Location', 'best');
    
    subplot(1, 3, 2);
    plot(results.time_vector, results.coordination.force_balance, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Imbalance Ratio', 'FontSize', 11);
    title('Force Balance (lower = better)', 'FontSize', 12, 'FontWeight', 'bold');
    
    subplot(1, 3, 3);
    plot(results.time_vector, results.coordination.velocity_sync, 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('Velocity Difference (m/s)', 'FontSize', 11);
    title('Velocity Synchronization', 'FontSize', 12, 'FontWeight', 'bold');
    
    fprintf('✓ Visualizations created\n\n');
    
    %% Export Data
    fprintf('Exporting data to output/...\n');
    
    if ~exist('output', 'dir')
        mkdir('output');
    end
    
    % Export main trajectories
    writematrix([results.time_vector', results.robot1.q_trajectory'], ...
                'output/example_collab_robot1_joints.csv');
    writematrix([results.time_vector', results.robot2.q_trajectory'], ...
                'output/example_collab_robot2_joints.csv');
    writematrix([results.time_vector', results.load.position', double(results.grasp_states)'], ...
                'output/example_collab_load_trajectory.csv');
    
    fprintf('✓ Data exported\n\n');
    
    %% Final Message
    fprintf('╔════════════════════════════════════════════╗\n');
    fprintf('║  EXAMPLE COMPLETE!                         ║\n');
    fprintf('╚════════════════════════════════════════════╝\n\n');
    
    fprintf('This example demonstrated:\n');
    fprintf('  ✓ Two 3-DOF robots working together\n');
    fprintf('  ✓ Pick-and-place task with 3kg load\n');
    fprintf('  ✓ Coordinated motion planning\n');
    fprintf('  ✓ Grasp state tracking\n');
    fprintf('  ✓ Force distribution\n');
    fprintf('  ✓ Cooperation verification\n\n');
    
    fprintf('To customize this example:\n');
    fprintf('  1. Edit robot parameters (link lengths, DOF)\n');
    fprintf('  2. Change load properties (mass, size)\n');
    fprintf('  3. Select different task type\n');
    fprintf('  4. Adjust trajectory parameters\n\n');
    
    fprintf('For full control, use: Collaborative_DualArm_Interface.m\n');
    fprintf('For documentation, see: COLLABORATIVE_DUALARM_GUIDE.md\n\n');
    
catch ME
    fprintf('\n❌ ERROR:\n');
    fprintf('   %s\n\n', ME.message);
    fprintf('Make sure all required files are in the MATLAB path:\n');
    fprintf('   - ARAT_Core.m\n');
    fprintf('   - RobotIK.m\n');
    fprintf('   - RobotVisualizer.m\n');
    fprintf('   - Collaborative_DualArm_Functions.m\n\n');
    rethrow(ME);
end
