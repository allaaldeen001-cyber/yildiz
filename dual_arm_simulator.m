% DUAL_ARM_SIMULATOR
% Interactive simulator for 2 manipulators working in shared workspace
% User can configure both robots and define coordinated tasks

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Dual-Arm Robot Simulator\n');
fprintf('2 Manipulators - Common Workspace\n');
fprintf('========================================\n\n');

%% USER CONFIGURATION

fprintf('=== ROBOT 1 CONFIGURATION ===\n\n');

% Robot 1 parameters
n_joints_1 = input('Robot 1 - Number of joints: ');

robot1 = struct();
robot1.n_joints = n_joints_1;
robot1.n_manipulators = 1;
robot1.name = 'Robot 1';
robot1.link_vectors = zeros(3, n_joints_1);
robot1.joint_axes = zeros(3, n_joints_1);
robot1.joint_types = repmat('R', 1, n_joints_1);

fprintf('\n--- Robot 1 Links and Joints ---\n');
for i = 1:n_joints_1
    fprintf('\nRobot 1 - Joint %d:\n', i);
    
    fprintf('  Link vector [x, y, z]:\n');
    robot1.link_vectors(1, i) = input('    x: ');
    robot1.link_vectors(2, i) = input('    y: ');
    robot1.link_vectors(3, i) = input('    z: ');
    
    joint_type = input('  Joint type (R/P): ', 's');
    robot1.joint_types(i) = upper(joint_type(1));
    
    fprintf('  Joint axis [x, y, z]:\n');
    axis_x = input('    x: ');
    axis_y = input('    y: ');
    axis_z = input('    z: ');
    axis = [axis_x; axis_y; axis_z];
    robot1.joint_axes(:, i) = axis / norm(axis);
end

fprintf('\n  Robot 1 base position [x, y, z]:\n');
robot1.base_position = zeros(3, 1);
robot1.base_position(1) = input('    x: ');
robot1.base_position(2) = input('    y: ');
robot1.base_position(3) = input('    z: ');
robot1.V_base = zeros(6, 1);

fprintf('\n=== ROBOT 2 CONFIGURATION ===\n\n');

% Option to mirror Robot 1 or configure independently
mirror_config = input('Mirror Robot 1 configuration? (y/n): ', 's');

if strcmpi(mirror_config, 'y')
    % Mirror configuration
    robot2 = robot1;
    robot2.name = 'Robot 2';
    
    % Mirror joint angles (for symmetric setup)
    robot2.joint_axes(1, :) = -robot1.joint_axes(1, :);  % Mirror X
    
    fprintf('  Robot 2 base position [x, y, z]:\n');
    robot2.base_position = zeros(3, 1);
    robot2.base_position(1) = input('    x: ');
    robot2.base_position(2) = input('    y: ');
    robot2.base_position(3) = input('    z: ');
else
    % Independent configuration
    n_joints_2 = input('Robot 2 - Number of joints: ');
    
    robot2 = struct();
    robot2.n_joints = n_joints_2;
    robot2.n_manipulators = 1;
    robot2.name = 'Robot 2';
    robot2.link_vectors = zeros(3, n_joints_2);
    robot2.joint_axes = zeros(3, n_joints_2);
    robot2.joint_types = repmat('R', 1, n_joints_2);
    
    fprintf('\n--- Robot 2 Links and Joints ---\n');
    for i = 1:n_joints_2
        fprintf('\nRobot 2 - Joint %d:\n', i);
        
        fprintf('  Link vector [x, y, z]:\n');
        robot2.link_vectors(1, i) = input('    x: ');
        robot2.link_vectors(2, i) = input('    y: ');
        robot2.link_vectors(3, i) = input('    z: ');
        
        joint_type = input('  Joint type (R/P): ', 's');
        robot2.joint_types(i) = upper(joint_type(1));
        
        fprintf('  Joint axis [x, y, z]:\n');
        axis_x = input('    x: ');
        axis_y = input('    y: ');
        axis_z = input('    z: ');
        axis = [axis_x; axis_y; axis_z];
        robot2.joint_axes(:, i) = axis / norm(axis);
    end
    
    fprintf('\n  Robot 2 base position [x, y, z]:\n');
    robot2.base_position = zeros(3, 1);
    robot2.base_position(1) = input('    x: ');
    robot2.base_position(2) = input('    y: ');
    robot2.base_position(3) = input('    z: ');
    robot2.V_base = zeros(6, 1);
end

%% COMMON WORKSPACE TASK

fprintf('\n=== COMMON WORKSPACE TASK ===\n\n');

fprintf('Task types:\n');
fprintf('  1. Coordinated motion (synchronized)\n');
fprintf('  2. Pick and place with shared load\n');
fprintf('  3. Assembly task (meet at point)\n');
fprintf('  4. Custom waypoints\n');

task_type = input('Select task type (1-4): ');

% Trajectory parameters
t_duration = input('\nTrajectory duration (seconds): ');
n_samples = input('Number of samples: ');

%% GENERATE TRAJECTORIES BASED ON TASK

fprintf('\n--- Generating Trajectories ---\n');

time_vector = linspace(0, t_duration, n_samples);

switch task_type
    case 1  % Coordinated motion
        fprintf('Coordinated motion task\n');
        
        fprintf('\nRobot 1 initial configuration:\n');
        q0_r1 = zeros(robot1.n_joints, 1);
        for i = 1:robot1.n_joints
            q0_r1(i) = input(sprintf('  q%d: ', i));
        end
        
        fprintf('Robot 1 final configuration:\n');
        qf_r1 = zeros(robot1.n_joints, 1);
        for i = 1:robot1.n_joints
            qf_r1(i) = input(sprintf('  q%d: ', i));
        end
        
        fprintf('\nRobot 2 initial configuration:\n');
        q0_r2 = zeros(robot2.n_joints, 1);
        for i = 1:robot2.n_joints
            q0_r2(i) = input(sprintf('  q%d: ', i));
        end
        
        fprintf('Robot 2 final configuration:\n');
        qf_r2 = zeros(robot2.n_joints, 1);
        for i = 1:robot2.n_joints
            qf_r2(i) = input(sprintf('  q%d: ', i));
        end
        
        % Generate synchronized trajectories
        q_traj_1 = zeros(robot1.n_joints, n_samples);
        q_traj_2 = zeros(robot2.n_joints, n_samples);
        
        for i = 1:robot1.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_harmonic(time_vector(j), 0, t_duration, ...
                                                     q0_r1(i), qf_r1(i));
                q_traj_1(i, j) = q;
            end
        end
        
        for i = 1:robot2.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_harmonic(time_vector(j), 0, t_duration, ...
                                                     q0_r2(i), qf_r2(i));
                q_traj_2(i, j) = q;
            end
        end
        
    case 2  % Pick and place
        fprintf('Pick and place with shared load\n');
        fprintf('(Using predefined coordinated motion)\n');
        
        % Simplified: use predefined waypoints
        % Robot 1: approach, grasp, lift, move, place
        q0_r1 = zeros(robot1.n_joints, 1);
        qf_r1 = pi/4 * ones(robot1.n_joints, 1);
        
        q0_r2 = zeros(robot2.n_joints, 1);
        qf_r2 = -pi/4 * ones(robot2.n_joints, 1);
        
        q_traj_1 = zeros(robot1.n_joints, n_samples);
        q_traj_2 = zeros(robot2.n_joints, n_samples);
        
        for i = 1:robot1.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_cycloidal(time_vector(j), 0, t_duration, ...
                                                      q0_r1(i), qf_r1(i));
                q_traj_1(i, j) = q;
            end
        end
        
        for i = 1:robot2.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_cycloidal(time_vector(j), 0, t_duration, ...
                                                      q0_r2(i), qf_r2(i));
                q_traj_2(i, j) = q;
            end
        end
        
    case 3  % Assembly task
        fprintf('Assembly task - meet at target point\n');
        
        fprintf('\nTarget meeting point [x, y, z]:\n');
        target = zeros(3, 1);
        target(1) = input('  x: ');
        target(2) = input('  y: ');
        target(3) = input('  z: ');
        
        % Use IK to find configurations
        fprintf('\nSolving IK for both robots...\n');
        
        ik_options.max_iter = 100;
        ik_options.tolerance = 1e-4;
        ik_options.lambda = 0.02;
        ik_options.position_only = true;
        
        q0_r1 = zeros(robot1.n_joints, 1);
        q0_r2 = zeros(robot2.n_joints, 1);
        
        % Adjust target for each robot's base
        target_r1 = target - robot1.base_position;
        target_r2 = target - robot2.base_position;
        
        [qf_r1, success1, ~] = RobotIK.solve_numerical_ik(robot1, target_r1, q0_r1, ik_options);
        [qf_r2, success2, ~] = RobotIK.solve_numerical_ik(robot2, target_r2, q0_r2, ik_options);
        
        if success1 && success2
            fprintf('IK solved successfully for both robots\n');
        else
            fprintf('Warning: IK may not have fully converged\n');
        end
        
        % Generate trajectories to target
        q_traj_1 = zeros(robot1.n_joints, n_samples);
        q_traj_2 = zeros(robot2.n_joints, n_samples);
        
        for i = 1:robot1.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_harmonic(time_vector(j), 0, t_duration, ...
                                                     q0_r1(i), qf_r1(i));
                q_traj_1(i, j) = q;
            end
        end
        
        for i = 1:robot2.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_harmonic(time_vector(j), 0, t_duration, ...
                                                     q0_r2(i), qf_r2(i));
                q_traj_2(i, j) = q;
            end
        end
        
    otherwise  % Custom waypoints
        fprintf('Custom waypoints\n');
        fprintf('(Using simple start-to-end trajectory)\n');
        
        q0_r1 = zeros(robot1.n_joints, 1);
        qf_r1 = ones(robot1.n_joints, 1) * pi/6;
        q0_r2 = zeros(robot2.n_joints, 1);
        qf_r2 = ones(robot2.n_joints, 1) * pi/6;
        
        q_traj_1 = zeros(robot1.n_joints, n_samples);
        q_traj_2 = zeros(robot2.n_joints, n_samples);
        
        for i = 1:robot1.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_polynomial_3(time_vector(j), 0, t_duration, ...
                                                         q0_r1(i), qf_r1(i));
                q_traj_1(i, j) = q;
            end
        end
        
        for i = 1:robot2.n_joints
            for j = 1:n_samples
                [q, ~, ~] = ARAT_Core.traj_polynomial_3(time_vector(j), 0, t_duration, ...
                                                         q0_r2(i), qf_r2(i));
                q_traj_2(i, j) = q;
            end
        end
end

fprintf('Trajectories generated\n\n');

%% COMPUTE END-EFFECTOR PATHS

fprintf('--- Computing End-Effector Positions ---\n');

ee_pos_1 = zeros(3, n_samples);
ee_pos_2 = zeros(3, n_samples);

for i = 1:n_samples
    pose1 = RobotIK.forward_kinematics_pose(robot1, q_traj_1(:, i));
    ee_pos_1(:, i) = pose1(4:6) + robot1.base_position;
    
    pose2 = RobotIK.forward_kinematics_pose(robot2, q_traj_2(:, i));
    ee_pos_2(:, i) = pose2(4:6) + robot2.base_position;
end

fprintf('End-effector paths computed\n\n');

%% JACOBIAN ANALYSIS

fprintf('--- Jacobian Analysis ---\n');

J_samples = min(10, n_samples);
sample_indices = round(linspace(1, n_samples, J_samples));

fprintf('Computing Jacobians at %d sample points...\n', J_samples);

for idx = 1:J_samples
    i = sample_indices(idx);
    
    J1 = ARAT_Core.compute_jacobian(robot1.link_vectors, robot1.joint_axes, ...
                                     robot1.joint_types, q_traj_1(:, i));
    J2 = ARAT_Core.compute_jacobian(robot2.link_vectors, robot2.joint_axes, ...
                                     robot2.joint_types, q_traj_2(:, i));
    
    if idx == 1 || idx == J_samples
        fprintf('  t=%.2fs: Robot1 manip=%.4f, Robot2 manip=%.4f\n', ...
                time_vector(i), sqrt(det(J1*J1')), sqrt(det(J2*J2')));
    end
end

fprintf('Jacobian analysis complete\n\n');

%% VISUALIZATION

fprintf('--- Creating Visualizations ---\n');

% Workspace plot
figure('Name', 'Dual-Arm Workspace', 'Position', [50, 50, 1200, 700]);

plot3(ee_pos_1(1, :), ee_pos_1(2, :), ee_pos_1(3, :), 'b-', 'LineWidth', 2);
hold on;
plot3(ee_pos_2(1, :), ee_pos_2(2, :), ee_pos_2(3, :), 'r-', 'LineWidth', 2);

scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
         200, 'b', 'filled', 'MarkerEdgeColor', 'k');
scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
         200, 'r', 'filled', 'MarkerEdgeColor', 'k');

scatter3(ee_pos_1(1, 1), ee_pos_1(2, 1), ee_pos_1(3, 1), ...
         100, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter3(ee_pos_2(1, 1), ee_pos_2(2, 1), ee_pos_2(3, 1), ...
         100, 'g', 'filled', 'MarkerEdgeColor', 'k');

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Dual-Arm Robot System - Common Workspace');
legend('Robot 1 Path', 'Robot 2 Path', 'Robot 1 Base', 'Robot 2 Base', ...
       'Start Positions', 'Location', 'best');
axis equal;
view(3);
hold off;

% Joint trajectories
figure('Name', 'Joint Trajectories', 'Position', [100, 100, 1400, 800]);

max_joints = max(robot1.n_joints, robot2.n_joints);
for i = 1:max_joints
    subplot(ceil(max_joints/2), 2, i);
    hold on;
    
    if i <= robot1.n_joints
        plot(time_vector, q_traj_1(i, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Robot 1');
    end
    if i <= robot2.n_joints
        plot(time_vector, q_traj_2(i, :), 'r-', 'LineWidth', 2, 'DisplayName', 'Robot 2');
    end
    
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('Joint %d', i));
    title(sprintf('Joint %d Trajectories', i));
    legend('Location', 'best');
end

fprintf('Visualizations created\n\n');

%% ANIMATION

fprintf('--- Starting Animation ---\n');
animate = input('Animate dual-arm system? (y/n): ', 's');

if strcmpi(animate, 'y')
    dual_arm_animate(robot1, robot2, q_traj_1, q_traj_2, time_vector);
end

%% EXPORT DATA

fprintf('\n--- Exporting Data ---\n');

if ~exist('output', 'dir')
    mkdir('output');
end

writematrix([time_vector', q_traj_1'], 'output/dual_arm_custom_robot1.csv');
writematrix([time_vector', q_traj_2'], 'output/dual_arm_custom_robot2.csv');
writematrix([time_vector', ee_pos_1'], 'output/dual_arm_custom_ee1.csv');
writematrix([time_vector', ee_pos_2'], 'output/dual_arm_custom_ee2.csv');

% Export Jacobians
for i = 1:robot1.n_joints
    H = ARAT_Core.get_joint_matrix(robot1.joint_types(i), robot1.joint_axes(:, i));
    Phi = ARAT_Core.get_prop_matrix(robot1.link_vectors(:, i));
    writematrix(H, sprintf('output/dual_arm_robot1_H_joint%d.csv', i));
    writematrix(Phi, sprintf('output/dual_arm_robot1_Phi_joint%d.csv', i));
end

for i = 1:robot2.n_joints
    H = ARAT_Core.get_joint_matrix(robot2.joint_types(i), robot2.joint_axes(:, i));
    Phi = ARAT_Core.get_prop_matrix(robot2.link_vectors(:, i));
    writematrix(H, sprintf('output/dual_arm_robot2_H_joint%d.csv', i));
    writematrix(Phi, sprintf('output/dual_arm_robot2_Phi_joint%d.csv', i));
end

save('output/dual_arm_custom_data.mat', 'robot1', 'robot2', ...
     'q_traj_1', 'q_traj_2', 'ee_pos_1', 'ee_pos_2', 'time_vector');

fprintf('Data exported to output/\n\n');

fprintf('========================================\n');
fprintf('Dual-Arm Simulation Complete!\n');
fprintf('========================================\n');

%% HELPER FUNCTION

function dual_arm_animate(robot1, robot2, q_traj_1, q_traj_2, time_vec)
    fig = figure('Name', 'Dual-Arm Animation', 'Position', [150, 150, 1200, 800]);
    
    n_frames = size(q_traj_1, 2);
    dt = 0.05;
    
    for i = 1:max(1, floor(n_frames/100)):n_frames
        clf(fig);
        hold on;
        grid on;
        axis equal;
        
        % Robot 1
        [pos1, frames1] = RobotVisualizer.compute_link_positions(robot1, q_traj_1(:, i));
        pos1 = pos1 + robot1.base_position;
        plot_arm(pos1, 'b');
        
        % Robot 2
        [pos2, frames2] = RobotVisualizer.compute_link_positions(robot2, q_traj_2(:, i));
        pos2 = pos2 + robot2.base_position;
        plot_arm(pos2, 'r');
        
        % Bases
        scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
                 150, 'b', 'filled');
        scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
                 150, 'r', 'filled');
        
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title(sprintf('Dual-Arm Animation - t=%.2fs', time_vec(i)));
        view(45, 30);
        
        drawnow;
        pause(dt);
    end
end

function plot_arm(positions, color)
    n = size(positions, 2) - 1;
    for k = 1:n
        p1 = positions(:, k);
        p2 = positions(:, k+1);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'Color', color, 'LineWidth', 3);
        scatter3(p1(1), p1(2), p1(3), 60, color, 'filled');
    end
    p_end = positions(:, end);
    scatter3(p_end(1), p_end(2), p_end(3), 100, color, 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
end
