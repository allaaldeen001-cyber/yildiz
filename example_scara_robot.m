% EXAMPLE_SCARA_ROBOT
% Example of a SCARA-type robot (Selective Compliance Assembly Robot Arm)
% Configuration: R-R-P-R (2 rotational, 1 prismatic, 1 rotational)

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Example: SCARA Robot (RRPR)\n');
fprintf('========================================\n\n');

%% ROBOT CONFIGURATION

robot = struct();
robot.n_joints = 4;
robot.n_manipulators = 1;

% Link vectors
% Link 1: horizontal arm (0.3m in x)
% Link 2: horizontal arm (0.25m in x)
% Link 3: vertical prismatic joint
% Link 4: end-effector rotation
robot.link_vectors = [
    0.3,  0.25,  0.0,   0.0;   % x
    0.0,  0.0,   0.0,   0.0;   % y
    0.0,  0.0,   0.0,   0.0    % z
];

% Joint axes
% Joint 1: Rotate about Z
% Joint 2: Rotate about Z
% Joint 3: Translate along Z (prismatic)
% Joint 4: Rotate about Z
robot.joint_axes = [
    0,  0,  0,  0;   % x
    0,  0,  0,  0;   % y
    1,  1,  1,  1    % z
];

% Joint types
robot.joint_types = 'RRPR';

% Base velocity (stationary)
robot.V_base = zeros(6, 1);

fprintf('SCARA Robot Configuration:\n');
fprintf('  - Joint 1: Rotational (Z-axis) - Shoulder\n');
fprintf('  - Joint 2: Rotational (Z-axis) - Elbow\n');
fprintf('  - Joint 3: Prismatic (Z-axis) - Vertical slide\n');
fprintf('  - Joint 4: Rotational (Z-axis) - Wrist\n');
fprintf('  - Horizontal reach: 0.55m\n\n');

%% TRAJECTORY - PICK AND PLACE OPERATION

fprintf('--- Pick-and-Place Trajectory ---\n');

% Define waypoints for pick-and-place
% [q1, q2, q3, q4]' where q3 is in meters
waypoints = [
    0,       0,       0,      0;      % Home position
    pi/6,    pi/4,    0,      0;      % Above pick location
    pi/6,    pi/4,    -0.15,  0;      % Pick (down)
    pi/6,    pi/4,    0,      0;      % Lift
    -pi/6,   pi/3,    0,      pi/4;   % Move to place
    -pi/6,   pi/3,    -0.12,  pi/4;   % Place (down)
    -pi/6,   pi/3,    0,      pi/4;   % Lift
    0,       0,       0,      0       % Return home
]';

n_waypoints = size(waypoints, 2);
n_samples_per_segment = 30;
n_total = (n_waypoints - 1) * n_samples_per_segment;

q_trajectory = zeros(4, n_total);
time_vector = zeros(1, n_total);

segment_time = 1.5; % seconds per segment
t_start = 0;

idx = 1;
for seg = 1:(n_waypoints - 1)
    q_start = waypoints(:, seg);
    q_end = waypoints(:, seg + 1);
    t_end = t_start + segment_time;
    
    t_seg = linspace(t_start, t_end, n_samples_per_segment);
    
    for i = 1:n_samples_per_segment
        for joint = 1:4
            [q, ~, ~] = ARAT_Core.traj_cycloidal(t_seg(i), t_start, t_end, ...
                                                  q_start(joint), q_end(joint));
            q_trajectory(joint, idx) = q;
        end
        time_vector(idx) = t_seg(i);
        idx = idx + 1;
    end
    
    t_start = t_end;
end

fprintf('Generated pick-and-place trajectory with %d waypoints\n', n_waypoints);
fprintf('Total trajectory time: %.2f seconds\n\n', time_vector(end));

%% INVERSE KINEMATICS FOR SCARA

fprintf('--- Inverse Kinematics Tests ---\n');

% Test multiple target positions
test_targets = [
    0.4,   0.2,   -0.1;
    0.3,  -0.3,   -0.15;
   -0.2,   0.35,  -0.08;
    0.45,  0.0,   -0.12
]';

ik_options.max_iter = 100;
ik_options.tolerance = 1e-4;
ik_options.lambda = 0.05;
ik_options.position_only = true;

fprintf('Testing IK for %d target positions:\n\n', size(test_targets, 2));

ik_solutions = zeros(4, size(test_targets, 2));

for i = 1:size(test_targets, 2)
    target = test_targets(:, i);
    fprintf('Target %d: [%.3f, %.3f, %.3f]\n', i, target(1), target(2), target(3));
    
    [q_ik, success, iters] = RobotIK.solve_numerical_ik(robot, target, ...
                                                         waypoints(:, 1), ik_options);
    
    if success
        fprintf('  Solution: [%.3f, %.3f, %.3f, %.3f] (%d iterations)\n', ...
                q_ik(1), q_ik(2), q_ik(3), q_ik(4), iters);
        
        % Verify
        achieved = RobotIK.forward_kinematics_pose(robot, q_ik);
        error = norm(target - achieved(4:6));
        fprintf('  Position error: %.6f m\n\n', error);
        
        ik_solutions(:, i) = q_ik;
    else
        fprintf('  IK failed\n\n');
    end
end

%% WORKSPACE ANALYSIS

fprintf('--- Workspace Analysis ---\n');

% Sample workspace by varying joint angles
n_samples_workspace = 20;
q1_range = linspace(-pi, pi, n_samples_workspace);
q2_range = linspace(-pi/2, pi/2, n_samples_workspace);
q3_range = linspace(-0.2, 0, 5); % Prismatic range

workspace_points = [];

for q1 = q1_range
    for q2 = q2_range
        for q3 = q3_range
            q_test = [q1; q2; q3; 0];
            pose = RobotIK.forward_kinematics_pose(robot, q_test);
            workspace_points = [workspace_points, pose(4:6)];
        end
    end
end

fprintf('Workspace sampled with %d configurations\n', size(workspace_points, 2));
fprintf('X range: [%.3f, %.3f] m\n', min(workspace_points(1,:)), max(workspace_points(1,:)));
fprintf('Y range: [%.3f, %.3f] m\n', min(workspace_points(2,:)), max(workspace_points(2,:)));
fprintf('Z range: [%.3f, %.3f] m\n\n', min(workspace_points(3,:)), max(workspace_points(3,:)));

%% JACOBIAN ANALYSIS

fprintf('--- Jacobian Analysis ---\n');

% Compute Jacobians at key configurations
configs = {
    waypoints(:, 1), 'Home';
    waypoints(:, 3), 'Pick';
    waypoints(:, 6), 'Place'
};

for i = 1:size(configs, 1)
    q = configs{i, 1};
    name = configs{i, 2};
    
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    
    fprintf('%s configuration:\n', name);
    fprintf('  Jacobian condition number: %.4f\n', cond(J));
    
    % Manipulability
    manip = sqrt(det(J * J'));
    fprintf('  Manipulability: %.4f\n\n', manip);
end

%% VISUALIZATION

fprintf('--- Creating Visualizations ---\n');

% Plot workspace
figure('Name', 'SCARA Workspace', 'Position', [100, 100, 800, 600]);
scatter3(workspace_points(1,:), workspace_points(2,:), workspace_points(3,:), ...
         1, 'b', 'filled');
hold on;

% Plot target positions
scatter3(test_targets(1,:), test_targets(2,:), test_targets(3,:), ...
         100, 'r', 'filled', 'MarkerEdgeColor', 'k');

% Plot trajectory end-effector path
ee_trajectory = zeros(3, n_total);
for i = 1:n_total
    pose = RobotIK.forward_kinematics_pose(robot, q_trajectory(:, i));
    ee_trajectory(:, i) = pose(4:6);
end
plot3(ee_trajectory(1,:), ee_trajectory(2,:), ee_trajectory(3,:), ...
      'g-', 'LineWidth', 3);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('SCARA Robot Workspace and Trajectories');
legend('Workspace', 'Target Positions', 'Pick-Place Path', 'Location', 'best');
grid on;
axis equal;
view(3);

% Plot key configurations
configs_to_plot = [1, 3, 6, 8]; % Home, Pick, Place, Return
figure('Name', 'SCARA Key Configurations', 'Position', [150, 150, 1200, 800]);

for i = 1:length(configs_to_plot)
    idx = configs_to_plot(i);
    
    subplot(2, 2, i);
    q = waypoints(:, idx);
    RobotVisualizer.plot_robot(robot, q);
    
    switch idx
        case 1
            title('1. Home Position');
        case 3
            title('3. Pick (Down)');
        case 6
            title('6. Place (Down)');
        case 8
            title('8. Return Home');
    end
    view(45, 30);
end

% Joint trajectory plots
figure('Name', 'SCARA Joint Trajectories', 'Position', [200, 200, 1200, 800]);

joint_names = {'Joint 1 (Shoulder)', 'Joint 2 (Elbow)', ...
               'Joint 3 (Vertical)', 'Joint 4 (Wrist)'};
joint_units = {'rad', 'rad', 'm', 'rad'};

for i = 1:4
    subplot(2, 2, i);
    plot(time_vector, q_trajectory(i, :), 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('%s (%s)', joint_names{i}, joint_units{i}));
    title(joint_names{i});
    
    % Mark waypoint transitions
    hold on;
    waypoint_times = 0:segment_time:time_vector(end);
    for t = waypoint_times
        xline(t, 'r--', 'LineWidth', 1);
    end
end

% Animate
fprintf('Starting animation...\n');
RobotVisualizer.animate_trajectory(robot, q_trajectory, 0.05, false);

%% EXPORT DATA

fprintf('\n--- Exporting Data ---\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export trajectory
writematrix(q_trajectory', 'output/scara_trajectory.csv');
writematrix(waypoints', 'output/scara_waypoints.csv');

% Export workspace
writematrix(workspace_points', 'output/scara_workspace.csv');

% Export IK solutions
writematrix(ik_solutions', 'output/scara_ik_solutions.csv');
writematrix(test_targets', 'output/scara_ik_targets.csv');

% Export Jacobians at waypoints
for i = 1:n_waypoints
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, waypoints(:, i));
    filename = sprintf('output/scara_jacobian_waypoint%d.csv', i);
    writematrix(J, filename);
end

% Export matrices
for i = 1:4
    H = ARAT_Core.get_joint_matrix(robot.joint_types(i), robot.joint_axes(:, i));
    Phi = ARAT_Core.get_prop_matrix(robot.link_vectors(:, i));
    
    writematrix(H, sprintf('output/scara_H_joint%d.csv', i));
    writematrix(Phi, sprintf('output/scara_Phi_joint%d.csv', i));
end

fprintf('All data exported to output/ directory\n');

fprintf('\n========================================\n');
fprintf('SCARA Example Complete!\n');
fprintf('========================================\n');
