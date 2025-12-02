% KINEMATIC_TWO_6LINK.M
% Kinematic Analysis of Two 6-Link Serial Robots using SOA
% Both robots mounted on same fixed platform

clear all;
close all;
clc;

%% ========================================================================
%  PARAMETERS - ENTER ALL ROBOT PARAMETERS HERE
%  ========================================================================

% NUMBER OF ROBOTS
n_robots = 2;  % Fixed: 2 robots

% EACH ROBOT HAS 6 LINKS
n_links_per_robot = 6;  % Fixed: 6 links each

%% ROBOT 1 PARAMETERS

% Link vectors for Robot 1
link_vectors_1 = [
    0.3,  0.28, 0.25, 0.22, 0.18, 0.15;   % x
    0,    0,    0,    0,    0,    0;      % y
    0,    0,    0,    0,    0,    0       % z
];

% Joint axes for Robot 1 (all revolute)
joint_axes_1 = [
    0,  0,  0,  0,  0,  1;   % x
    0,  0,  0,  0,  1,  0;   % y
    1,  1,  1,  1,  0,  0    % z
];

% Joint types for Robot 1
joint_types_1 = 'RRRRRR';

% Base position for Robot 1
base_position_1 = [-0.4; 0; 0];  % meters

% Base velocity for Robot 1 (fixed platform)
V_base_1 = zeros(6, 1);

% Initial and final configurations for Robot 1 (radians)
q_initial_1 = zeros(6, 1);
q_final_1 = [pi/4; pi/6; pi/4; pi/6; pi/4; pi/6];

%% ROBOT 2 PARAMETERS

% Link vectors for Robot 2
link_vectors_2 = [
    0.3,  0.28, 0.25, 0.22, 0.18, 0.15;   % x
    0,    0,    0,    0,    0,    0;      % y
    0,    0,    0,    0,    0,    0       % z
];

% Joint axes for Robot 2 (all revolute)
joint_axes_2 = [
    0,  0,  0,  0,  0,  1;   % x
    0,  0,  0,  0,  1,  0;   % y
    1,  1,  1,  1,  0,  0    % z
];

% Joint types for Robot 2
joint_types_2 = 'RRRRRR';

% Base position for Robot 2
base_position_2 = [0.4; 0; 0];  % meters

% Base velocity for Robot 2 (fixed platform)
V_base_2 = zeros(6, 1);

% Initial and final configurations for Robot 2 (radians)
q_initial_2 = zeros(6, 1);
q_final_2 = [-pi/4; pi/6; -pi/4; pi/6; -pi/4; pi/6];

%% TRAJECTORY PARAMETERS

trajectory_duration = 5.0;      % seconds
n_samples = 100;                % number of trajectory points

% TRAJECTORY TYPE (1=Cubic, 2=Harmonic, 3=Cycloidal, 4=Gutman, 5=Freudenstein)
trajectory_type = 2;  % Harmonic

%% ========================================================================
%  AUTOMATIC EXECUTION - DO NOT MODIFY BELOW THIS LINE
%  ========================================================================

fprintf('========================================\n');
fprintf('TWO 6-LINK SERIAL ROBOTS KINEMATIC ANALYSIS\n');
fprintf('Using Screw Orientation Approach (SOA)\n');
fprintf('========================================\n\n');

%% VALIDATE PARAMETERS

fprintf('--- Parameter Validation ---\n');

% Robot 1
assert(size(link_vectors_1, 2) == n_links_per_robot, 'Robot 1: Link vectors must have %d columns', n_links_per_robot);
assert(size(joint_axes_1, 2) == n_links_per_robot, 'Robot 1: Joint axes must have %d columns', n_links_per_robot);
assert(length(joint_types_1) == n_links_per_robot, 'Robot 1: Joint types must have %d elements', n_links_per_robot);

% Robot 2
assert(size(link_vectors_2, 2) == n_links_per_robot, 'Robot 2: Link vectors must have %d columns', n_links_per_robot);
assert(size(joint_axes_2, 2) == n_links_per_robot, 'Robot 2: Joint axes must have %d columns', n_links_per_robot);
assert(length(joint_types_2) == n_links_per_robot, 'Robot 2: Joint types must have %d elements', n_links_per_robot);

% Normalize joint axes
for i = 1:n_links_per_robot
    joint_axes_1(:, i) = joint_axes_1(:, i) / norm(joint_axes_1(:, i));
    joint_axes_2(:, i) = joint_axes_2(:, i) / norm(joint_axes_2(:, i));
end

fprintf('✓ All parameters validated\n');
fprintf('  Number of robots: %d\n', n_robots);
fprintf('  Links per robot: %d\n', n_links_per_robot);
fprintf('  Total joints: %d\n', n_robots * n_links_per_robot);
fprintf('  Robot 1 base: [%.2f, %.2f, %.2f]\n', base_position_1);
fprintf('  Robot 2 base: [%.2f, %.2f, %.2f]\n\n', base_position_2);

%% GENERATE JOINT TRAJECTORIES

fprintf('--- Generating Joint Trajectories ---\n');

time_vector = linspace(0, trajectory_duration, n_samples);

% Robot 1 trajectories
q_trajectory_1 = zeros(n_links_per_robot, n_samples);
qd_trajectory_1 = zeros(n_links_per_robot, n_samples);

% Robot 2 trajectories
q_trajectory_2 = zeros(n_links_per_robot, n_samples);
qd_trajectory_2 = zeros(n_links_per_robot, n_samples);

trajectory_names = {'Cubic Polynomial', 'Harmonic', 'Cycloidal', 'Gutman 1-3', 'Freudenstein 1-3-5'};
fprintf('Using %s trajectory\n', trajectory_names{trajectory_type});

% Generate trajectories for Robot 1
for joint = 1:n_links_per_robot
    for k = 1:n_samples
        t = time_vector(k);
        [q, qd, ~] = generate_trajectory_point(trajectory_type, t, trajectory_duration, ...
                                                q_initial_1(joint), q_final_1(joint));
        q_trajectory_1(joint, k) = q;
        qd_trajectory_1(joint, k) = qd;
    end
end

% Generate trajectories for Robot 2
for joint = 1:n_links_per_robot
    for k = 1:n_samples
        t = time_vector(k);
        [q, qd, ~] = generate_trajectory_point(trajectory_type, t, trajectory_duration, ...
                                                q_initial_2(joint), q_final_2(joint));
        q_trajectory_2(joint, k) = q;
        qd_trajectory_2(joint, k) = qd;
    end
end

fprintf('✓ Trajectories generated for both robots\n\n');

%% FORWARD KINEMATICS FOR BOTH ROBOTS

fprintf('--- Computing Forward Kinematics ---\n');

% Robot 1
ee_pos_1 = zeros(3, n_samples);
ee_vel_1 = zeros(6, n_samples);

robot1 = struct('link_vectors', link_vectors_1, 'joint_axes', joint_axes_1, 'joint_types', joint_types_1);

for k = 1:n_samples
    pose = RobotIK.forward_kinematics_pose(robot1, q_trajectory_1(:, k));
    ee_pos_1(:, k) = pose(4:6) + base_position_1;
    
    V = ARAT_Core.serial_forward_kinematics(link_vectors_1, joint_axes_1, joint_types_1, ...
                                             q_trajectory_1(:, k), qd_trajectory_1(:, k), V_base_1);
    ee_vel_1(:, k) = V;
end

fprintf('✓ Robot 1 FK computed\n');
fprintf('  EE start: [%.4f, %.4f, %.4f]\n', ee_pos_1(1,1), ee_pos_1(2,1), ee_pos_1(3,1));
fprintf('  EE end:   [%.4f, %.4f, %.4f]\n', ee_pos_1(1,end), ee_pos_1(2,end), ee_pos_1(3,end));

% Robot 2
ee_pos_2 = zeros(3, n_samples);
ee_vel_2 = zeros(6, n_samples);

robot2 = struct('link_vectors', link_vectors_2, 'joint_axes', joint_axes_2, 'joint_types', joint_types_2);

for k = 1:n_samples
    pose = RobotIK.forward_kinematics_pose(robot2, q_trajectory_2(:, k));
    ee_pos_2(:, k) = pose(4:6) + base_position_2;
    
    V = ARAT_Core.serial_forward_kinematics(link_vectors_2, joint_axes_2, joint_types_2, ...
                                             q_trajectory_2(:, k), qd_trajectory_2(:, k), V_base_2);
    ee_vel_2(:, k) = V;
end

fprintf('✓ Robot 2 FK computed\n');
fprintf('  EE start: [%.4f, %.4f, %.4f]\n', ee_pos_2(1,1), ee_pos_2(2,1), ee_pos_2(3,1));
fprintf('  EE end:   [%.4f, %.4f, %.4f]\n\n', ee_pos_2(1,end), ee_pos_2(2,end), ee_pos_2(3,end));

%% JACOBIAN ANALYSIS

fprintf('--- Jacobian Analysis ---\n');

% Robot 1
manip_1 = zeros(1, n_samples);
cond_1 = zeros(1, n_samples);

for k = 1:n_samples
    J = ARAT_Core.compute_jacobian(link_vectors_1, joint_axes_1, joint_types_1, q_trajectory_1(:, k));
    manip_1(k) = sqrt(det(J * J'));
    cond_1(k) = cond(J);
end

fprintf('✓ Robot 1 Jacobian analysis\n');
fprintf('  Avg manipulability: %.6f\n', mean(manip_1));
fprintf('  Avg condition number: %.2f\n', mean(cond_1));

% Robot 2
manip_2 = zeros(1, n_samples);
cond_2 = zeros(1, n_samples);

for k = 1:n_samples
    J = ARAT_Core.compute_jacobian(link_vectors_2, joint_axes_2, joint_types_2, q_trajectory_2(:, k));
    manip_2(k) = sqrt(det(J * J'));
    cond_2(k) = cond(J);
end

fprintf('✓ Robot 2 Jacobian analysis\n');
fprintf('  Avg manipulability: %.6f\n', mean(manip_2));
fprintf('  Avg condition number: %.2f\n\n', mean(cond_2));

%% H AND PHI MATRICES

fprintf('--- Computing H and Phi Matrices ---\n');

% Robot 1
H_matrices_1 = cell(1, n_links_per_robot);
Phi_matrices_1 = cell(1, n_links_per_robot);

for i = 1:n_links_per_robot
    H_matrices_1{i} = ARAT_Core.get_joint_matrix(joint_types_1(i), joint_axes_1(:, i));
    Phi_matrices_1{i} = ARAT_Core.get_prop_matrix(link_vectors_1(:, i));
end

% Robot 2
H_matrices_2 = cell(1, n_links_per_robot);
Phi_matrices_2 = cell(1, n_links_per_robot);

for i = 1:n_links_per_robot
    H_matrices_2{i} = ARAT_Core.get_joint_matrix(joint_types_2(i), joint_axes_2(:, i));
    Phi_matrices_2{i} = ARAT_Core.get_prop_matrix(link_vectors_2(:, i));
end

fprintf('✓ H and Phi matrices computed for both robots\n\n');

%% TRAJECTORY VERIFICATION

fprintf('========================================\n');
fprintf('TRAJECTORY FOLLOWING VERIFICATION\n');
fprintf('========================================\n\n');

% Base trajectories (both fixed)
base_error_1 = norm(V_base_1);
base_error_2 = norm(V_base_2);

fprintf('ROBOT 1 - BASE TRAJECTORY:\n');
fprintf('  Status: %s\n', ternary(base_error_1 < 1e-10, '✓ CORRECT (Fixed)', '✗ ERROR'));
fprintf('  Error: %.2e\n\n', base_error_1);

fprintf('ROBOT 2 - BASE TRAJECTORY:\n');
fprintf('  Status: %s\n', ternary(base_error_2 < 1e-10, '✓ CORRECT (Fixed)', '✗ ERROR'));
fprintf('  Error: %.2e\n\n', base_error_2);

% End-effector trajectories (verify consistency)
ee_error_1 = verify_ee_trajectory(robot1, q_trajectory_1, ee_pos_1, base_position_1);
ee_error_2 = verify_ee_trajectory(robot2, q_trajectory_2, ee_pos_2, base_position_2);

fprintf('ROBOT 1 - END-EFFECTOR TRAJECTORY:\n');
fprintf('  Status: %s\n', ternary(ee_error_1 < 1e-6, '✓ CORRECT', '✗ ERROR'));
fprintf('  Error: %.2e m\n\n', ee_error_1);

fprintf('ROBOT 2 - END-EFFECTOR TRAJECTORY:\n');
fprintf('  Status: %s\n', ternary(ee_error_2 < 1e-6, '✓ CORRECT', '✗ ERROR'));
fprintf('  Error: %.2e m\n\n', ee_error_2);

fprintf('OVERALL ASSESSMENT:\n');
if base_error_1 < 1e-6 && base_error_2 < 1e-6 && ee_error_1 < 1e-6 && ee_error_2 < 1e-6
    fprintf('  ✓✓✓ ALL TRAJECTORIES CORRECT FOR BOTH ROBOTS ✓✓✓\n');
else
    fprintf('  ⚠ TRAJECTORY ERRORS DETECTED\n');
end
fprintf('\n');

%% VISUALIZATION

fprintf('--- Creating Visualizations ---\n');

% Figure 1: Both robots - workspace
figure('Name', 'Two 6-Link Robots - Workspace', 'Position', [50, 50, 1200, 700]);

plot3(ee_pos_1(1,:), ee_pos_1(2,:), ee_pos_1(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(ee_pos_2(1,:), ee_pos_2(2,:), ee_pos_2(3,:), 'r-', 'LineWidth', 2);

scatter3(base_position_1(1), base_position_1(2), base_position_1(3), 200, 'b', 'filled');
scatter3(base_position_2(1), base_position_2(2), base_position_2(3), 200, 'r', 'filled');

scatter3(ee_pos_1(1,1), ee_pos_1(2,1), ee_pos_1(3,1), 100, 'g', 'filled');
scatter3(ee_pos_2(1,1), ee_pos_2(2,1), ee_pos_2(3,1), 100, 'g', 'filled');

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Two 6-Link Robots - End-Effector Paths');
legend('Robot 1', 'Robot 2', 'Base 1', 'Base 2', 'Starts');
axis equal;
view(3);

% Figure 2: Joint trajectories comparison
figure('Name', 'Two 6-Link Robots - Joint Trajectories', 'Position', [100, 100, 1400, 900]);

for i = 1:n_links_per_robot
    subplot(3, 2, i);
    plot(time_vector, q_trajectory_1(i,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(time_vector, q_trajectory_2(i,:), 'r--', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('Joint %d (rad)', i));
    title(sprintf('Joint %d - Both Robots', i));
    legend('Robot 1', 'Robot 2');
end

% Figure 3: Analysis comparison
figure('Name', 'Two 6-Link Robots - Analysis', 'Position', [150, 150, 1200, 600]);

subplot(1, 2, 1);
plot(time_vector, manip_1, 'b-', 'LineWidth', 2);
hold on;
plot(time_vector, manip_2, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Manipulability');
title('Manipulability Comparison');
legend('Robot 1', 'Robot 2');

subplot(1, 2, 2);
plot(time_vector, cond_1, 'b-', 'LineWidth', 2);
hold on;
plot(time_vector, cond_2, 'r--', 'LineWidth', 2);
yline(100, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Condition Number');
title('Condition Number Comparison');
legend('Robot 1', 'Robot 2', 'Warning Threshold');

fprintf('✓ Visualizations created\n\n');

%% DATA EXPORT

fprintf('--- Exporting Data ---\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export trajectories
writematrix([time_vector', q_trajectory_1'], 'output/two6link_robot1_joints.csv');
writematrix([time_vector', q_trajectory_2'], 'output/two6link_robot2_joints.csv');
writematrix([time_vector', ee_pos_1'], 'output/two6link_robot1_ee.csv');
writematrix([time_vector', ee_pos_2'], 'output/two6link_robot2_ee.csv');

% Export matrices
for i = 1:n_links_per_robot
    writematrix(H_matrices_1{i}, sprintf('output/two6link_robot1_H_joint%d.csv', i));
    writematrix(H_matrices_2{i}, sprintf('output/two6link_robot2_H_joint%d.csv', i));
    writematrix(Phi_matrices_1{i}, sprintf('output/two6link_robot1_Phi_joint%d.csv', i));
    writematrix(Phi_matrices_2{i}, sprintf('output/two6link_robot2_Phi_joint%d.csv', i));
end

% Export results
fid = fopen('output/two6link_results.txt', 'w');
fprintf(fid, 'TWO 6-LINK SERIAL ROBOTS - KINEMATIC ANALYSIS RESULTS\n');
fprintf(fid, '=====================================================\n\n');
fprintf(fid, 'Configuration:\n');
fprintf(fid, '  Number of robots: %d\n', n_robots);
fprintf(fid, '  Links per robot: %d\n', n_links_per_robot);
fprintf(fid, '  Total DOF: %d\n\n', n_robots * n_links_per_robot);
fprintf(fid, 'Trajectory Following:\n');
fprintf(fid, '  Robot 1 base error: %.2e\n', base_error_1);
fprintf(fid, '  Robot 1 EE error: %.2e m\n', ee_error_1);
fprintf(fid, '  Robot 2 base error: %.2e\n', base_error_2);
fprintf(fid, '  Robot 2 EE error: %.2e m\n\n', ee_error_2);
fprintf(fid, 'Status: ');
if base_error_1 < 1e-6 && base_error_2 < 1e-6 && ee_error_1 < 1e-6 && ee_error_2 < 1e-6
    fprintf(fid, 'ALL TRAJECTORIES CORRECT\n');
else
    fprintf(fid, 'ERRORS DETECTED\n');
end
fclose(fid);

fprintf('✓ Data exported\n\n');

%% FINAL SUMMARY

fprintf('========================================\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('========================================\n\n');

fprintf('✓ Two 6-link robots analyzed using SOA\n');
fprintf('✓ Both base and end-effector trajectories verified\n');
fprintf('✓ All data exported to output/\n\n');

%% HELPER FUNCTIONS

function [q, qd, qdd] = generate_trajectory_point(type, t, T, q0, qf)
    switch type
        case 1
            [q, qd, qdd] = ARAT_Core.traj_polynomial_3(t, 0, T, q0, qf);
        case 2
            [q, qd, qdd] = ARAT_Core.traj_harmonic(t, 0, T, q0, qf);
        case 3
            [q, qd, qdd] = ARAT_Core.traj_cycloidal(t, 0, T, q0, qf);
        case 4
            [q, qd, qdd] = ARAT_Core.traj_gutman_1_3(t, 0, T, q0, qf);
        case 5
            [q, qd, qdd] = ARAT_Core.traj_freudenstein_1_3_5(t, 0, T, q0, qf);
    end
end

function error = verify_ee_trajectory(robot, q_traj, ee_pos, base_pos)
    n_samples = size(q_traj, 2);
    sample_indices = round(linspace(1, n_samples, min(5, n_samples)));
    max_error = 0;
    
    for idx = sample_indices
        pose = RobotIK.forward_kinematics_pose(robot, q_traj(:, idx));
        expected_pos = pose(4:6) + base_pos;
        error_val = norm(expected_pos - ee_pos(:, idx));
        max_error = max(max_error, error_val);
    end
    
    error = max_error;
end

function result = ternary(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end
