% EXAMPLE_DYNAMICS_6DOF.M
% Dynamic analysis of a single 6-DOF serial robot
% Demonstrates Part 6: Dynamic Analysis of a 6-Link Serial Robot
%
% This script:
% - Defines a 6-DOF robot with mass and inertia properties
% - Generates trajectory using harmonic motion
% - Computes joint torques using recursive Newton-Euler
% - Plots torque curves for all joints
% - Calculates power and energy

clear all;
close all;
clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║  DYNAMIC ANALYSIS - 6-DOF SERIAL ROBOT               ║\n');
fprintf('║  Part 6: Fixed 6-Link Configuration                  ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');

%% Define 6-DOF Robot
fprintf('Step 1: Defining 6-DOF robot...\n');

robot = struct();

% Link vectors (vertical configuration)
robot.link_vectors = [
    0,    0,    0,    0,    0,    0;     % x
    0,    0,    0,    0,    0,    0;     % y
    0.4,  0.35, 0.3,  0.25, 0.2,  0.15   % z (total 1.65m height)
];

% All joints rotate about Z-axis
robot.joint_axes = [
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    1, 1, 1, 1, 1, 1
];

robot.joint_types = 'RRRRRR';
robot.base_position = [0; 0; 0];
robot.V_base = zeros(6, 1);

% Initial: straight up
robot.q_initial = [0; 0; 0; 0; 0; 0];

% Final: bent configuration
robot.q_final = [pi/3; pi/4; pi/6; -pi/6; pi/4; pi/3];

fprintf('  ✓ 6-DOF robot defined\n');
fprintf('    Total height: %.2f m\n', sum(robot.link_vectors(3, :)));
fprintf('    Joint range: %.2f to %.2f rad\n\n', min(robot.q_final), max(robot.q_final));

%% Define Mass Properties
fprintf('Step 2: Defining mass and inertia properties...\n');

dynamic_params = struct();
dynamic_params.gravity = [0; 0; -9.81];

% Mass parameters for each link
mass_params = struct('m', {}, 'c', {}, 'I', {});

% Typical values for each link
link_masses = [3.0, 2.5, 2.0, 1.5, 1.0, 0.5];  % kg (heavier at base)

for i = 1:6
    mass_params(i).m = link_masses(i);
    mass_params(i).c = [0; 0; robot.link_vectors(3, i)/2];  % CoM at link midpoint
    
    % Inertia tensor (simplified as diagonal)
    L = robot.link_vectors(3, i);
    m = link_masses(i);
    Ixx = (1/12) * m * L^2;
    Iyy = Ixx;
    Izz = 0.01 * m;  % Small rotation about link axis
    mass_params(i).I = diag([Ixx, Iyy, Izz]);
end

dynamic_params.mass_params = mass_params;

fprintf('  ✓ Mass properties defined\n');
fprintf('    Total mass: %.2f kg\n', sum(link_masses));
fprintf('    Link masses: ');
fprintf('%.1f ', link_masses);
fprintf('kg\n\n');

%% Define Trajectory
fprintf('Step 3: Generating trajectory...\n');

trajectory_params = struct();
trajectory_params.type = 2;  % Harmonic
trajectory_params.duration = 4.0;  % 4 seconds
trajectory_params.n_samples = 120;

fprintf('  ✓ Trajectory: Harmonic\n');
fprintf('  ✓ Duration: %.1f s\n', trajectory_params.duration);
fprintf('  ✓ Samples: %d\n\n', trajectory_params.n_samples);

%% Perform Dynamic Analysis
fprintf('Step 4: Computing joint torques...\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

robots = {robot};

results = Dynamic_SOA_Functions.analyze_multi_robot_dynamics(...
    robots, trajectory_params, dynamic_params);

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('✓ Dynamic analysis complete!\n\n');

%% Display Results
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║  RESULTS SUMMARY                           ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

fprintf('JOINT TORQUES:\n');
for i = 1:6
    peak = results.statistics.robot{1}.peak_torques(i);
    rms = results.statistics.robot{1}.rms_torques(i);
    fprintf('  Joint %d: Peak = %7.2f N·m, RMS = %7.2f N·m\n', i, peak, rms);
end
fprintf('\n');

fprintf('POWER & ENERGY:\n');
fprintf('  Peak power: %.2f W\n', results.statistics.robot{1}.peak_power);
fprintf('  Avg power: %.2f W\n', results.statistics.robot{1}.avg_power);
fprintf('  Total energy: %.2f J\n\n', results.statistics.robot{1}.total_energy);

%% Visualizations
fprintf('Step 5: Creating visualizations...\n');

% Plot torques
Dynamic_SOA_Functions.visualize_dynamics(results, robots, results.time_vector);

% Additional detailed plot
figure('Name', '6-DOF Robot - Detailed Torque Analysis', 'Position', [200, 100, 1400, 900]);

% Individual joint torques
for i = 1:6
    subplot(3, 3, i);
    plot(results.time_vector, results.robots{1}.tau_trajectory(i, :), ...
         'b-', 'LineWidth', 2.5);
    grid on;
    xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Torque (N·m)', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('Joint %d', i), 'FontSize', 12, 'FontWeight', 'bold');
    
    % Highlight peak
    [peak_val, peak_idx] = max(abs(results.robots{1}.tau_trajectory(i, :)));
    hold on;
    plot(results.time_vector(peak_idx), results.robots{1}.tau_trajectory(i, peak_idx), ...
         'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Add text
    text(0.6, 0.9, sprintf('Peak: %.2f N·m', peak_val), ...
         'Units', 'normalized', 'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');
end

% All torques overlay
subplot(3, 3, [7, 8, 9]);
hold on;
colors = lines(6);
for i = 1:6
    plot(results.time_vector, results.robots{1}.tau_trajectory(i, :), ...
         'Color', colors(i, :), 'LineWidth', 2);
end
grid on;
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Torque (N·m)', 'FontSize', 12, 'FontWeight', 'bold');
title('All Joint Torques', 'FontSize', 13, 'FontWeight', 'bold');
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Location', 'best', 'FontSize', 10);

sgtitle('6-DOF Serial Robot - Comprehensive Torque Analysis', ...
        'FontSize', 16, 'FontWeight', 'bold');

fprintf('✓ Visualizations created\n\n');

%% Export Data
fprintf('Step 6: Exporting data...\n');

Dynamic_SOA_Functions.export_dynamics_data(results, robots, results.time_vector, ...
    'example_6dof');

fprintf('✓ Data exported to output/\n\n');

%% Summary
fprintf('╔════════════════════════════════════════════╗\n');
fprintf('║  EXAMPLE COMPLETE!                         ║\n');
fprintf('╚════════════════════════════════════════════╝\n\n');

fprintf('This example demonstrated:\n');
fprintf('  ✓ 6-DOF serial robot configuration\n');
fprintf('  ✓ Mass and inertia properties\n');
fprintf('  ✓ Harmonic trajectory generation\n');
fprintf('  ✓ Recursive Newton-Euler dynamics\n');
fprintf('  ✓ Joint torque computation\n');
fprintf('  ✓ Power and energy analysis\n');
fprintf('  ✓ Comprehensive visualization\n');
fprintf('  ✓ Data export\n\n');

fprintf('Key findings:\n');
fprintf('  - Base joints (1-2) have highest torques (%.1f-%.1f N·m)\n', ...
        results.statistics.robot{1}.peak_torques(1), ...
        results.statistics.robot{1}.peak_torques(2));
fprintf('  - End joints (5-6) have lowest torques (%.1f-%.1f N·m)\n', ...
        results.statistics.robot{1}.peak_torques(5), ...
        results.statistics.robot{1}.peak_torques(6));
fprintf('  - This is typical for serial manipulators\n\n');

fprintf('Files exported:\n');
fprintf('  - output/example_6dof_robot1_torques.csv\n');
fprintf('  - output/example_6dof_robot1_power.csv\n');
fprintf('  - output/example_6dof_robot1_energy.csv\n');
fprintf('  - output/example_6dof_dynamic_summary.txt\n\n');

fprintf('To customize:\n');
fprintf('  1. Modify robot.q_final for different motion\n');
fprintf('  2. Adjust link_masses for different loading\n');
fprintf('  3. Change trajectory_params.type for different profiles\n');
fprintf('  4. Adjust trajectory_params.duration for faster/slower motion\n\n');
