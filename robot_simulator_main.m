% ROBOT_SIMULATOR_MAIN
% Main script for SOA-based robot manipulator simulation
% Supports arbitrary number of joints, links, and manipulators
% Features: Forward/Inverse Kinematics, Trajectory Generation, Animation

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('SOA Robot Manipulator Simulator\n');
fprintf('========================================\n\n');

%% USER INPUT SECTION

% Number of joints
n_joints = input('Enter number of joints (n): ');

% Initialize robot structure
robot = struct();
robot.n_joints = n_joints;
robot.link_vectors = zeros(3, n_joints);
robot.joint_axes = zeros(3, n_joints);
robot.joint_types = repmat('R', 1, n_joints);
robot.joint_limits = [];

fprintf('\n--- Link and Joint Configuration ---\n');

for i = 1:n_joints
    fprintf('\nJoint %d:\n', i);
    
    % Link vector (direction and length)
    fprintf('  Link vector from joint %d to %d [x, y, z]:\n', i-1, i);
    link_x = input('    x: ');
    link_y = input('    y: ');
    link_z = input('    z: ');
    robot.link_vectors(:, i) = [link_x; link_y; link_z];
    
    % Joint type
    joint_type = input('  Joint type (R for Rotational, P for Prismatic): ', 's');
    robot.joint_types(i) = upper(joint_type(1));
    
    % Joint axis (rotation axis or prismatic direction)
    fprintf('  Joint axis [x, y, z] (normalized automatically):\n');
    axis_x = input('    x: ');
    axis_y = input('    y: ');
    axis_z = input('    z: ');
    axis = [axis_x; axis_y; axis_z];
    axis = axis / norm(axis); % Normalize
    robot.joint_axes(:, i) = axis;
end

% Base velocity (w0, v0)
fprintf('\n--- Base Velocity ---\n');
fprintf('Angular velocity w0 [wx, wy, wz]:\n');
w0_x = input('  wx: ');
w0_y = input('  wy: ');
w0_z = input('  wz: ');
w0 = [w0_x; w0_y; w0_z];

fprintf('Linear velocity v0 [vx, vy, vz]:\n');
v0_x = input('  vx: ');
v0_y = input('  vy: ');
v0_z = input('  vz: ');
v0 = [v0_x; v0_y; v0_z];

robot.V_base = [w0; v0];

% Number of manipulators (for parallel/multi-arm systems)
n_manipulators = input('\nEnter number of manipulators (p): ');
robot.n_manipulators = n_manipulators;

fprintf('\n========================================\n');
fprintf('Configuration complete!\n');
fprintf('========================================\n\n');

%% TRAJECTORY GENERATION

fprintf('--- Trajectory Planning ---\n');
fprintf('Available trajectory types:\n');
fprintf('  1. Cubic Polynomial\n');
fprintf('  2. Harmonic (Sinusoidal)\n');
fprintf('  3. Cycloidal\n');
fprintf('  4. Gutman 1-3 (Fourier)\n');
fprintf('  5. Freudenstein 1-3-5 (Fourier)\n');
traj_type = input('Select trajectory type (1-5): ');

% Trajectory parameters
t_start = 0;
t_end = input('Trajectory duration (seconds): ');
n_samples = input('Number of trajectory samples: ');

% Initial and final joint configurations
fprintf('\nInitial joint configuration (q0):\n');
q0 = zeros(n_joints, 1);
for i = 1:n_joints
    q0(i) = input(sprintf('  q%d: ', i));
end

fprintf('\nFinal joint configuration (qf):\n');
qf = zeros(n_joints, 1);
for i = 1:n_joints
    qf(i) = input(sprintf('  q%d: ', i));
end

% Generate time vector
time_vector = linspace(t_start, t_end, n_samples);

% Generate trajectory for each joint
q_trajectory = zeros(n_joints, n_samples);
qd_trajectory = zeros(n_joints, n_samples);
qdd_trajectory = zeros(n_joints, n_samples);

for i = 1:n_joints
    for j = 1:n_samples
        t = time_vector(j);
        
        switch traj_type
            case 1
                [q, qd, qdd] = ARAT_Core.traj_polynomial_3(t, t_start, t_end, q0(i), qf(i));
            case 2
                [q, qd, qdd] = ARAT_Core.traj_harmonic(t, t_start, t_end, q0(i), qf(i));
            case 3
                [q, qd, qdd] = ARAT_Core.traj_cycloidal(t, t_start, t_end, q0(i), qf(i));
            case 4
                [q, qd, qdd] = ARAT_Core.traj_gutman_1_3(t, t_start, t_end, q0(i), qf(i));
            case 5
                [q, qd, qdd] = ARAT_Core.traj_freudenstein_1_3_5(t, t_start, t_end, q0(i), qf(i));
            otherwise
                error('Invalid trajectory type');
        end
        
        q_trajectory(i, j) = q;
        qd_trajectory(i, j) = qd;
        qdd_trajectory(i, j) = qdd;
    end
end

fprintf('Trajectory generation complete!\n\n');

%% INVERSE KINEMATICS

fprintf('--- Inverse Kinematics ---\n');
use_ik = input('Do you want to use Inverse Kinematics? (y/n): ', 's');

if strcmpi(use_ik, 'y')
    fprintf('\nTarget end-effector position [x, y, z]:\n');
    target_x = input('  x: ');
    target_y = input('  y: ');
    target_z = input('  z: ');
    target_pos = [target_x; target_y; target_z];
    
    % IK options
    ik_options.max_iter = 100;
    ik_options.tolerance = 1e-4;
    ik_options.lambda = 0.01;
    ik_options.position_only = true;
    
    % Solve IK
    fprintf('Solving Inverse Kinematics...\n');
    [q_ik, success, iterations] = RobotIK.solve_numerical_ik(robot, target_pos, q0, ik_options);
    
    if success
        fprintf('IK converged in %d iterations!\n', iterations);
        fprintf('Solution:\n');
        for i = 1:n_joints
            fprintf('  q%d = %.4f\n', i, q_ik(i));
        end
        
        % Add IK solution to trajectory
        add_ik = input('\nAdd IK solution as final waypoint? (y/n): ', 's');
        if strcmpi(add_ik, 'y')
            q_trajectory = [q_trajectory, q_ik];
            time_vector = [time_vector, time_vector(end) + 1];
        end
    else
        fprintf('IK did not converge. Using original trajectory.\n');
    end
end

fprintf('\n========================================\n');

%% FORWARD KINEMATICS & JACOBIAN COMPUTATION

fprintf('\n--- Computing Kinematics ---\n');

% Compute Jacobian at each configuration
jacobian_matrices = cell(1, length(time_vector));
H_matrices = cell(1, n_joints);
Phi_matrices = cell(1, n_joints);

for i = 1:length(time_vector)
    q = q_trajectory(:, i);
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    jacobian_matrices{i} = J;
end

% Compute H matrices (joint maps)
for i = 1:n_joints
    H_matrices{i} = ARAT_Core.get_joint_matrix(robot.joint_types(i), ...
                                                robot.joint_axes(:, i));
end

% Compute Phi matrices (propagation matrices)
for i = 1:n_joints
    Phi_matrices{i} = ARAT_Core.get_prop_matrix(robot.link_vectors(:, i));
end

fprintf('Kinematics computation complete!\n');

%% VISUALIZATION

fprintf('\n--- Visualization ---\n');

% Plot initial configuration
figure('Name', 'Initial Configuration');
RobotVisualizer.plot_robot(robot, q0);
title('Initial Configuration');

% Plot final configuration
figure('Name', 'Final Configuration');
RobotVisualizer.plot_robot(robot, q_trajectory(:, end));
title('Final Configuration');

% Plot joint trajectories
RobotVisualizer.plot_joint_trajectories(q_trajectory, time_vector);

% Animate robot
fprintf('Starting animation...\n');
animate = input('Animate robot motion? (y/n): ', 's');
if strcmpi(animate, 'y')
    save_video = input('Save animation as video? (y/n): ', 's');
    dt = 0.05; % Animation time step
    RobotVisualizer.animate_trajectory(robot, q_trajectory, dt, strcmpi(save_video, 'y'));
end

%% EXPORT DATA

fprintf('\n--- Exporting Data ---\n');

% Create output directory
if ~exist('output', 'dir')
    mkdir('output');
end

% Export joint angles (Theta)
writematrix(q_trajectory', 'output/joint_angles_theta.csv');
fprintf('Exported joint angles to: output/joint_angles_theta.csv\n');

% Export Jacobian matrices
for i = 1:length(jacobian_matrices)
    filename = sprintf('output/jacobian_t%d.csv', i);
    writematrix(jacobian_matrices{i}, filename);
end
fprintf('Exported %d Jacobian matrices to: output/jacobian_t*.csv\n', length(jacobian_matrices));

% Export H matrices
for i = 1:n_joints
    filename = sprintf('output/H_matrix_joint%d.csv', i);
    writematrix(H_matrices{i}, filename);
end
fprintf('Exported H matrices to: output/H_matrix_joint*.csv\n');

% Export Phi (Propagation) matrices
for i = 1:n_joints
    filename = sprintf('output/Phi_matrix_joint%d.csv', i);
    writematrix(Phi_matrices{i}, filename);
end
fprintf('Exported Phi matrices to: output/Phi_matrix_joint*.csv\n');

% Export robot configuration
config_file = fopen('output/robot_configuration.txt', 'w');
fprintf(config_file, 'Robot Configuration\n');
fprintf(config_file, '==================\n\n');
fprintf(config_file, 'Number of joints: %d\n', n_joints);
fprintf(config_file, 'Number of manipulators: %d\n\n', n_manipulators);
fprintf(config_file, 'Link vectors:\n');
for i = 1:n_joints
    fprintf(config_file, '  Link %d: [%.4f, %.4f, %.4f]\n', i, ...
            robot.link_vectors(1,i), robot.link_vectors(2,i), robot.link_vectors(3,i));
end
fprintf(config_file, '\nJoint types and axes:\n');
for i = 1:n_joints
    fprintf(config_file, '  Joint %d: Type=%s, Axis=[%.4f, %.4f, %.4f]\n', i, ...
            robot.joint_types(i), robot.joint_axes(1,i), robot.joint_axes(2,i), robot.joint_axes(3,i));
end
fprintf(config_file, '\nBase velocity:\n');
fprintf(config_file, '  w0 = [%.4f, %.4f, %.4f]\n', w0(1), w0(2), w0(3));
fprintf(config_file, '  v0 = [%.4f, %.4f, %.4f]\n', v0(1), v0(2), v0(3));
fclose(config_file);
fprintf('Exported robot configuration to: output/robot_configuration.txt\n');

% Export summary MATLAB file
save('output/robot_simulation_data.mat', 'robot', 'q_trajectory', 'qd_trajectory', ...
     'qdd_trajectory', 'time_vector', 'jacobian_matrices', 'H_matrices', 'Phi_matrices');
fprintf('Exported all simulation data to: output/robot_simulation_data.mat\n');

fprintf('\n========================================\n');
fprintf('Simulation Complete!\n');
fprintf('========================================\n');
