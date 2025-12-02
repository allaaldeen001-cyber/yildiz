% EXAMPLE_COMPLETE_WORKFLOW
% Complete workflow demonstrating all features of the SOA Robot Simulator
% This example shows: FK, IK, trajectory generation, Jacobian analysis, 
% visualization, animation, and data export

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Complete Workflow Example\n');
fprintf('========================================\n\n');

%% 1. ROBOT DEFINITION

fprintf('STEP 1: Defining Robot Configuration\n');
fprintf('-------------------------------------\n');

robot.n_joints = 4;
robot.n_manipulators = 1;

% 4-DOF robot: RRRP (3 rotational + 1 prismatic)
robot.link_vectors = [
    0.3,  0.25,  0.2,   0;      % x
    0,    0,     0,     0;      % y
    0.2,  0,     0,     0       % z (elevated base)
];

robot.joint_axes = [
    0,  0,  1,  0;   % x
    0,  0,  0,  0;   % y
    1,  1,  0,  1    % z
];

robot.joint_types = 'RRRP';
robot.V_base = zeros(6, 1);

fprintf('Robot: 4-DOF RRRP manipulator\n');
fprintf('  Link 1: 0.3m, rotate about Z\n');
fprintf('  Link 2: 0.25m, rotate about Z\n');
fprintf('  Link 3: 0.2m, rotate about Y\n');
fprintf('  Link 4: prismatic along Z\n\n');

%% 2. FORWARD KINEMATICS

fprintf('STEP 2: Forward Kinematics\n');
fprintf('-------------------------------------\n');

% Test configuration
q_test = [pi/6; pi/4; pi/6; 0.1];

% Compute FK
pose = RobotIK.forward_kinematics_pose(robot, q_test);
position = pose(4:6);
orientation = pose(1:3);

fprintf('Joint configuration: [%.3f, %.3f, %.3f, %.3f]\n', q_test(1), q_test(2), q_test(3), q_test(4));
fprintf('End-effector position: [%.4f, %.4f, %.4f] m\n', position(1), position(2), position(3));
fprintf('End-effector orientation: [%.4f, %.4f, %.4f] rad\n\n', orientation(1), orientation(2), orientation(3));

%% 3. JACOBIAN ANALYSIS

fprintf('STEP 3: Jacobian Analysis\n');
fprintf('-------------------------------------\n');

J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                robot.joint_types, q_test);

fprintf('Jacobian matrix (6x4):\n');
disp(J);

% Compute metrics
cond_num = cond(J);
rank_J = rank(J);
manip = sqrt(det(J * J'));

fprintf('Condition number: %.4f\n', cond_num);
fprintf('Rank: %d\n', rank_J);
fprintf('Manipulability: %.4f\n\n', manip);

% Singularity check
if cond_num > 100
    fprintf('⚠ Warning: Near singular configuration!\n\n');
elseif cond_num < 10
    fprintf('✓ Good manipulability\n\n');
end

%% 4. H AND PHI MATRICES

fprintf('STEP 4: Computing H and Phi Matrices\n');
fprintf('-------------------------------------\n');

H_matrices = cell(1, robot.n_joints);
Phi_matrices = cell(1, robot.n_joints);

for i = 1:robot.n_joints
    H_matrices{i} = ARAT_Core.get_joint_matrix(robot.joint_types(i), robot.joint_axes(:, i));
    Phi_matrices{i} = ARAT_Core.get_prop_matrix(robot.link_vectors(:, i));
    
    fprintf('Joint %d (%c):\n', i, robot.joint_types(i));
    fprintf('  H matrix (6x1):\n');
    disp(H_matrices{i}');
    fprintf('  Phi matrix (6x6) - showing first 3 rows:\n');
    disp(Phi_matrices{i}(1:3, :));
end

%% 5. INVERSE KINEMATICS

fprintf('STEP 5: Inverse Kinematics\n');
fprintf('-------------------------------------\n');

% Multiple target positions
targets = [
    0.5,   0.3,   0.15;
    0.4,  -0.2,   0.25;
   -0.3,   0.4,   0.20
]';

ik_options.max_iter = 100;
ik_options.tolerance = 1e-4;
ik_options.lambda = 0.02;
ik_options.position_only = true;

ik_solutions = zeros(4, size(targets, 2));
q_init = [0; 0; 0; 0];

for i = 1:size(targets, 2)
    target = targets(:, i);
    fprintf('\nTarget %d: [%.3f, %.3f, %.3f]\n', i, target(1), target(2), target(3));
    
    [q_ik, success, iters] = RobotIK.solve_numerical_ik(robot, target, q_init, ik_options);
    
    if success
        fprintf('  ✓ Converged in %d iterations\n', iters);
        fprintf('  Solution: [%.4f, %.4f, %.4f, %.4f]\n', q_ik(1), q_ik(2), q_ik(3), q_ik(4));
        
        % Verify
        achieved_pose = RobotIK.forward_kinematics_pose(robot, q_ik);
        error = norm(target - achieved_pose(4:6));
        fprintf('  Position error: %.6f m\n', error);
        
        ik_solutions(:, i) = q_ik;
        q_init = q_ik; % Use as next initial guess
    else
        fprintf('  ✗ Failed to converge\n');
    end
end

fprintf('\n');

%% 6. TRAJECTORY GENERATION - COMPARE ALL METHODS

fprintf('STEP 6: Trajectory Generation (All Methods)\n');
fprintf('-------------------------------------\n');

q0 = [0; 0; 0; 0];
qf = [pi/3; pi/4; pi/6; 0.15];

t_start = 0;
t_end = 5;
n_samples = 100;
time_vector = linspace(t_start, t_end, n_samples);

% Initialize trajectory arrays
traj_methods = {'Cubic', 'Harmonic', 'Cycloidal', 'Gutman', 'Freudenstein'};
trajectories = cell(1, 5);

for method = 1:5
    trajectories{method} = zeros(4, n_samples);
end

% Generate trajectories
for joint = 1:4
    for k = 1:n_samples
        t = time_vector(k);
        
        [q1, ~, ~] = ARAT_Core.traj_polynomial_3(t, t_start, t_end, q0(joint), qf(joint));
        trajectories{1}(joint, k) = q1;
        
        [q2, ~, ~] = ARAT_Core.traj_harmonic(t, t_start, t_end, q0(joint), qf(joint));
        trajectories{2}(joint, k) = q2;
        
        [q3, ~, ~] = ARAT_Core.traj_cycloidal(t, t_start, t_end, q0(joint), qf(joint));
        trajectories{3}(joint, k) = q3;
        
        [q4, ~, ~] = ARAT_Core.traj_gutman_1_3(t, t_start, t_end, q0(joint), qf(joint));
        trajectories{4}(joint, k) = q4;
        
        [q5, ~, ~] = ARAT_Core.traj_freudenstein_1_3_5(t, t_start, t_end, q0(joint), qf(joint));
        trajectories{5}(joint, k) = q5;
    end
end

fprintf('Generated 5 trajectory types for 4 joints\n');
fprintf('Duration: %.1f seconds with %d samples\n\n', t_end, n_samples);

%% 7. ALPHA MATRIX (SPATIAL INERTIA) - For reference

fprintf('STEP 7: Alpha Matrix Computation\n');
fprintf('-------------------------------------\n');

% Alpha matrix represents spatial inertia in SOA
% For a simple demonstration with uniform mass distribution
fprintf('Computing spatial inertia matrices...\n');

alpha_matrices = cell(1, robot.n_joints);

for i = 1:robot.n_joints
    % Simplified mass properties
    m = 1.0;  % 1 kg per link
    r_com = robot.link_vectors(:, i) / 2;  % CoM at link center
    I_scalar = 0.1;  % Simplified inertia
    
    % Spatial inertia matrix (6x6)
    I_mat = I_scalar * eye(3);
    m_mat = m * eye(3);
    r_skew = ARAT_Core.skew(r_com);
    
    alpha_matrices{i} = [
        I_mat + m * r_skew * r_skew',  m * r_skew;
        m * r_skew',                   m_mat
    ];
    
    fprintf('  Joint %d: Alpha matrix (6x6) computed\n', i);
end

fprintf('Spatial inertia matrices computed for all joints\n\n');

%% 8. VISUALIZATION - MULTIPLE PLOTS

fprintf('STEP 8: Creating Visualizations\n');
fprintf('-------------------------------------\n');

% Plot 8.1: Robot configurations
figure('Name', 'Robot Configurations', 'Position', [50, 50, 1400, 500]);

subplot(1, 3, 1);
RobotVisualizer.plot_robot(robot, q0);
title('Initial Configuration');
view(45, 30);

subplot(1, 3, 2);
RobotVisualizer.plot_robot(robot, qf);
title('Final Configuration');
view(45, 30);

if size(ik_solutions, 2) > 0
    subplot(1, 3, 3);
    RobotVisualizer.plot_robot(robot, ik_solutions(:, 1));
    title('IK Solution Configuration');
    view(45, 30);
end

% Plot 8.2: Trajectory comparison
figure('Name', 'Trajectory Method Comparison', 'Position', [100, 100, 1400, 900]);

colors = {'b', 'r', 'g', 'm', 'c'};
for joint = 1:4
    subplot(2, 2, joint);
    hold on;
    for method = 1:5
        plot(time_vector, trajectories{method}(joint, :), ...
             colors{method}, 'LineWidth', 1.5, 'DisplayName', traj_methods{method});
    end
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('Joint %d', joint));
    title(sprintf('Joint %d: All Trajectory Methods', joint));
    legend('Location', 'best');
    hold off;
end

% Plot 8.3: Jacobian evolution
figure('Name', 'Jacobian Analysis Along Trajectory', 'Position', [150, 150, 1200, 600]);

manip_values = zeros(1, n_samples);
cond_values = zeros(1, n_samples);

for i = 1:n_samples
    q = trajectories{2}(:, i);  % Using harmonic trajectory
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    manip_values(i) = sqrt(det(J * J'));
    cond_values(i) = cond(J);
end

subplot(1, 2, 1);
plot(time_vector, manip_values, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Manipulability');
title('Manipulability Along Trajectory');

subplot(1, 2, 2);
plot(time_vector, cond_values, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Condition Number');
title('Condition Number Along Trajectory');
yline(100, 'k--', 'Singularity Warning');

% Plot 8.4: End-effector path
figure('Name', 'End-Effector Trajectory', 'Position', [200, 200, 800, 600]);

ee_positions = zeros(3, n_samples);
for i = 1:n_samples
    pose = RobotIK.forward_kinematics_pose(robot, trajectories{2}(:, i));
    ee_positions(:, i) = pose(4:6);
end

plot3(ee_positions(1, :), ee_positions(2, :), ee_positions(3, :), ...
      'b-', 'LineWidth', 2);
hold on;
scatter3(ee_positions(1, 1), ee_positions(2, 1), ee_positions(3, 1), ...
         100, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter3(ee_positions(1, end), ee_positions(2, end), ee_positions(3, end), ...
         100, 'r', 'filled', 'MarkerEdgeColor', 'k');

% Plot IK targets
if size(targets, 2) > 0
    scatter3(targets(1, :), targets(2, :), targets(3, :), ...
             80, 'k', 'filled', 'MarkerEdgeColor', 'y', 'LineWidth', 2);
end

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Trajectory in Workspace');
legend('Path', 'Start', 'End', 'IK Targets', 'Location', 'best');
view(3);
axis equal;

fprintf('Created 4 visualization figures\n\n');

%% 9. ANIMATION

fprintf('STEP 9: Animation\n');
fprintf('-------------------------------------\n');

fprintf('Animating harmonic trajectory...\n');
RobotVisualizer.animate_trajectory(robot, trajectories{2}, 0.05, false);

fprintf('Animation complete\n\n');

%% 10. DATA EXPORT

fprintf('STEP 10: Exporting Data\n');
fprintf('-------------------------------------\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export trajectories
for method = 1:5
    filename = sprintf('output/workflow_trajectory_%s.csv', lower(traj_methods{method}));
    writematrix(trajectories{method}', filename);
    fprintf('Exported: %s\n', filename);
end

% Export Jacobians along trajectory (sample every 10 steps)
for i = 1:10:n_samples
    q = trajectories{2}(:, i);
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    filename = sprintf('output/workflow_jacobian_t%03d.csv', i);
    writematrix(J, filename);
end
fprintf('Exported Jacobian samples\n');

% Export H matrices
for i = 1:robot.n_joints
    filename = sprintf('output/workflow_H_joint%d.csv', i);
    writematrix(H_matrices{i}, filename);
end
fprintf('Exported H matrices\n');

% Export Phi matrices
for i = 1:robot.n_joints
    filename = sprintf('output/workflow_Phi_joint%d.csv', i);
    writematrix(Phi_matrices{i}, filename);
end
fprintf('Exported Phi matrices\n');

% Export Alpha matrices
for i = 1:robot.n_joints
    filename = sprintf('output/workflow_Alpha_joint%d.csv', i);
    writematrix(alpha_matrices{i}, filename);
end
fprintf('Exported Alpha matrices\n');

% Export IK solutions and targets
if size(ik_solutions, 2) > 0
    writematrix(ik_solutions', 'output/workflow_ik_solutions.csv');
    writematrix(targets', 'output/workflow_ik_targets.csv');
    fprintf('Exported IK data\n');
end

% Export end-effector trajectory
writematrix(ee_positions', 'output/workflow_ee_trajectory.csv');
fprintf('Exported end-effector trajectory\n');

% Export manipulability and condition number data
analysis_data = [time_vector', manip_values', cond_values'];
writematrix(analysis_data, 'output/workflow_jacobian_analysis.csv');
fprintf('Exported Jacobian analysis data\n');

% Export complete MATLAB data
save('output/workflow_complete_data.mat', 'robot', 'trajectories', ...
     'H_matrices', 'Phi_matrices', 'alpha_matrices', 'ik_solutions', ...
     'targets', 'time_vector', 'ee_positions', 'manip_values', 'cond_values');
fprintf('Exported complete MATLAB workspace\n');

% Create summary report
fid = fopen('output/workflow_summary.txt', 'w');
fprintf(fid, '===========================================\n');
fprintf(fid, 'Complete Workflow Summary Report\n');
fprintf(fid, '===========================================\n\n');
fprintf(fid, 'Date: %s\n\n', datestr(now));

fprintf(fid, 'Robot Configuration:\n');
fprintf(fid, '  Joints: %d\n', robot.n_joints);
fprintf(fid, '  Types: %s\n', robot.joint_types);
fprintf(fid, '  Manipulators: %d\n\n', robot.n_manipulators);

fprintf(fid, 'Trajectory:\n');
fprintf(fid, '  Duration: %.2f seconds\n', t_end);
fprintf(fid, '  Samples: %d\n', n_samples);
fprintf(fid, '  Methods tested: %d\n\n', length(traj_methods));

fprintf(fid, 'Inverse Kinematics:\n');
fprintf(fid, '  Targets tested: %d\n', size(targets, 2));
fprintf(fid, '  Successful solutions: %d\n\n', sum(any(ik_solutions ~= 0, 1)));

fprintf(fid, 'Jacobian Analysis:\n');
fprintf(fid, '  Average manipulability: %.4f\n', mean(manip_values));
fprintf(fid, '  Average condition number: %.4f\n', mean(cond_values));
fprintf(fid, '  Max condition number: %.4f\n', max(cond_values));
fprintf(fid, '  Min manipulability: %.4f\n\n', min(manip_values));

fprintf(fid, 'Files Exported:\n');
fprintf(fid, '  Trajectory files: %d\n', length(traj_methods));
fprintf(fid, '  Jacobian samples: %d\n', length(1:10:n_samples));
fprintf(fid, '  H matrices: %d\n', robot.n_joints);
fprintf(fid, '  Phi matrices: %d\n', robot.n_joints);
fprintf(fid, '  Alpha matrices: %d\n', robot.n_joints);
fprintf(fid, '  Total data files: %d+\n\n', length(traj_methods) + length(1:10:n_samples) + 3*robot.n_joints + 4);

fprintf(fid, '===========================================\n');
fclose(fid);

fprintf('Created summary report: output/workflow_summary.txt\n\n');

%% 11. SUMMARY

fprintf('========================================\n');
fprintf('Workflow Complete!\n');
fprintf('========================================\n\n');

fprintf('Summary:\n');
fprintf('  ✓ Robot configured (%d-DOF %s)\n', robot.n_joints, robot.joint_types);
fprintf('  ✓ Forward kinematics computed\n');
fprintf('  ✓ Jacobian analyzed\n');
fprintf('  ✓ H, Phi, and Alpha matrices computed\n');
fprintf('  ✓ Inverse kinematics solved (%d targets)\n', size(targets, 2));
fprintf('  ✓ Trajectories generated (%d methods)\n', length(traj_methods));
fprintf('  ✓ Visualizations created (4 figures)\n');
fprintf('  ✓ Animation displayed\n');
fprintf('  ✓ Data exported to output/\n\n');

fprintf('Next steps:\n');
fprintf('  - Review plots and animation\n');
fprintf('  - Check exported data in output/ folder\n');
fprintf('  - Read workflow_summary.txt for details\n');
fprintf('  - Modify parameters and re-run\n\n');

fprintf('========================================\n');
