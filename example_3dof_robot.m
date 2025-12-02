% EXAMPLE_3DOF_ROBOT
% Example script demonstrating a 3-DOF robot manipulator
% This shows how to use the SOA Robot Simulator without manual input

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Example: 3-DOF Robot Manipulator\n');
fprintf('========================================\n\n');

%% ROBOT CONFIGURATION

% Define a 3-DOF RRR (3 Rotational joints) robot
robot = struct();
robot.n_joints = 3;
robot.n_manipulators = 1;

% Link vectors (each link in x-direction)
robot.link_vectors = [
    0.5, 0.4, 0.3;  % x components
    0.0, 0.0, 0.0;  % y components  
    0.0, 0.0, 0.0   % z components
];

% Joint axes (all rotating about Z-axis)
robot.joint_axes = [
    0, 0, 0;     % x components
    0, 0, 0;     % y components
    1, 1, 1      % z components (rotation about Z)
];

% Joint types (all rotational)
robot.joint_types = 'RRR';

% Base velocity (stationary base)
robot.V_base = zeros(6, 1);

fprintf('Robot Configuration:\n');
fprintf('  - 3 Rotational joints (RRR)\n');
fprintf('  - Link lengths: 0.5m, 0.4m, 0.3m\n');
fprintf('  - All joints rotate about Z-axis\n\n');

%% TRAJECTORY GENERATION

% Initial and final configurations
q0 = [0; 0; 0];              % All joints at 0 degrees
qf = [pi/4; pi/3; pi/6];     % 45°, 60°, 30°

t_start = 0;
t_end = 5;  % 5 seconds
n_samples = 100;
time_vector = linspace(t_start, t_end, n_samples);

fprintf('--- Generating Trajectories ---\n');

% Generate trajectories using different methods
q_traj_cubic = zeros(3, n_samples);
q_traj_harmonic = zeros(3, n_samples);
q_traj_cycloidal = zeros(3, n_samples);

for i = 1:3  % For each joint
    for j = 1:n_samples
        t = time_vector(j);
        
        % Cubic polynomial trajectory
        [q_cubic, ~, ~] = ARAT_Core.traj_polynomial_3(t, t_start, t_end, q0(i), qf(i));
        q_traj_cubic(i, j) = q_cubic;
        
        % Harmonic trajectory
        [q_harm, ~, ~] = ARAT_Core.traj_harmonic(t, t_start, t_end, q0(i), qf(i));
        q_traj_harmonic(i, j) = q_harm;
        
        % Cycloidal trajectory
        [q_cycl, ~, ~] = ARAT_Core.traj_cycloidal(t, t_start, t_end, q0(i), qf(i));
        q_traj_cycloidal(i, j) = q_cycl;
    end
end

fprintf('Generated trajectories: Cubic, Harmonic, Cycloidal\n\n');

%% INVERSE KINEMATICS EXAMPLE

fprintf('--- Inverse Kinematics Test ---\n');

% Target end-effector position
target_pos = [0.8; 0.5; 0.0];

fprintf('Target position: [%.2f, %.2f, %.2f]\n', target_pos(1), target_pos(2), target_pos(3));

% IK options
ik_options.max_iter = 100;
ik_options.tolerance = 1e-4;
ik_options.lambda = 0.01;
ik_options.position_only = true;

% Solve IK
[q_ik, success, iterations] = RobotIK.solve_numerical_ik(robot, target_pos, q0, ik_options);

if success
    fprintf('IK Solution (converged in %d iterations):\n', iterations);
    fprintf('  Joint 1: %.4f rad (%.2f°)\n', q_ik(1), rad2deg(q_ik(1)));
    fprintf('  Joint 2: %.4f rad (%.2f°)\n', q_ik(2), rad2deg(q_ik(2)));
    fprintf('  Joint 3: %.4f rad (%.2f°)\n', q_ik(3), rad2deg(q_ik(3)));
    
    % Verify solution
    achieved_pose = RobotIK.forward_kinematics_pose(robot, q_ik);
    achieved_pos = achieved_pose(4:6);
    error = norm(target_pos - achieved_pos);
    fprintf('  Position error: %.6f m\n\n', error);
else
    fprintf('IK did not converge\n\n');
end

%% JACOBIAN COMPUTATION

fprintf('--- Jacobian Analysis ---\n');

% Compute Jacobian at initial configuration
J_init = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q0);

fprintf('Jacobian at initial configuration:\n');
disp(J_init);

% Compute Jacobian at final configuration
J_final = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                     robot.joint_types, qf);

fprintf('Jacobian at final configuration:\n');
disp(J_final);

% Manipulability analysis
manip_init = sqrt(det(J_init * J_init'));
manip_final = sqrt(det(J_final * J_final'));

fprintf('Manipulability measure:\n');
fprintf('  Initial: %.4f\n', manip_init);
fprintf('  Final: %.4f\n\n', manip_final);

%% VISUALIZATION

fprintf('--- Visualization ---\n');

% Plot initial and final configurations
figure('Name', '3-DOF Robot - Initial Configuration', 'Position', [100, 100, 800, 600]);
RobotVisualizer.plot_robot(robot, q0);
title('Initial Configuration (All joints at 0°)');

figure('Name', '3-DOF Robot - Final Configuration', 'Position', [150, 150, 800, 600]);
RobotVisualizer.plot_robot(robot, qf);
title('Final Configuration (45°, 60°, 30°)');

if success
    figure('Name', '3-DOF Robot - IK Solution', 'Position', [200, 200, 800, 600]);
    RobotVisualizer.plot_robot(robot, q_ik);
    title(sprintf('IK Solution for target [%.2f, %.2f, %.2f]', ...
                  target_pos(1), target_pos(2), target_pos(3)));
end

% Compare trajectory types
figure('Name', 'Trajectory Comparison', 'Position', [250, 250, 1200, 800]);
for i = 1:3
    % Position
    subplot(3, 3, (i-1)*3 + 1);
    plot(time_vector, q_traj_cubic(i,:), 'b-', 'LineWidth', 2); hold on;
    plot(time_vector, q_traj_harmonic(i,:), 'r--', 'LineWidth', 2);
    plot(time_vector, q_traj_cycloidal(i,:), 'g-.', 'LineWidth', 2);
    grid on;
    ylabel(sprintf('Joint %d (rad)', i));
    if i == 1
        title('Position');
        legend('Cubic', 'Harmonic', 'Cycloidal', 'Location', 'best');
    end
    if i == 3
        xlabel('Time (s)');
    end
    
    % Velocity (numerical derivative)
    subplot(3, 3, (i-1)*3 + 2);
    dt = time_vector(2) - time_vector(1);
    vel_cubic = diff(q_traj_cubic(i,:)) / dt;
    vel_harm = diff(q_traj_harmonic(i,:)) / dt;
    vel_cycl = diff(q_traj_cycloidal(i,:)) / dt;
    plot(time_vector(1:end-1), vel_cubic, 'b-', 'LineWidth', 2); hold on;
    plot(time_vector(1:end-1), vel_harm, 'r--', 'LineWidth', 2);
    plot(time_vector(1:end-1), vel_cycl, 'g-.', 'LineWidth', 2);
    grid on;
    if i == 1
        title('Velocity');
    end
    if i == 3
        xlabel('Time (s)');
    end
    
    % Acceleration (second derivative)
    subplot(3, 3, (i-1)*3 + 3);
    acc_cubic = diff(vel_cubic) / dt;
    acc_harm = diff(vel_harm) / dt;
    acc_cycl = diff(vel_cycl) / dt;
    plot(time_vector(1:end-2), acc_cubic, 'b-', 'LineWidth', 2); hold on;
    plot(time_vector(1:end-2), acc_harm, 'r--', 'LineWidth', 2);
    plot(time_vector(1:end-2), acc_cycl, 'g-.', 'LineWidth', 2);
    grid on;
    if i == 1
        title('Acceleration');
    end
    if i == 3
        xlabel('Time (s)');
    end
end

fprintf('Static plots created.\n');

% Animate robot with harmonic trajectory
fprintf('Animating robot with harmonic trajectory...\n');
RobotVisualizer.animate_trajectory(robot, q_traj_harmonic, 0.05, false);

%% EXPORT DATA

fprintf('\n--- Exporting Data ---\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export matrices
writematrix(q_traj_harmonic', 'output/example_trajectory.csv');
writematrix(J_init, 'output/example_jacobian_initial.csv');
writematrix(J_final, 'output/example_jacobian_final.csv');

% Export H and Phi matrices
for i = 1:3
    H = ARAT_Core.get_joint_matrix(robot.joint_types(i), robot.joint_axes(:, i));
    Phi = ARAT_Core.get_prop_matrix(robot.link_vectors(:, i));
    
    writematrix(H, sprintf('output/example_H_joint%d.csv', i));
    writematrix(Phi, sprintf('output/example_Phi_joint%d.csv', i));
end

fprintf('Data exported to output/ directory\n');

fprintf('\n========================================\n');
fprintf('Example Complete!\n');
fprintf('========================================\n');
