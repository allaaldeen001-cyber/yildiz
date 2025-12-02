% EXAMPLE_DUAL_ARM_COLLABORATIVE
% Simulation of 2 manipulators working together with a common load
% Demonstrates coordinated motion in shared workspace

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('Dual-Arm Collaborative Robot System\n');
fprintf('2 Manipulators with Common Load\n');
fprintf('========================================\n\n');

%% MANIPULATOR 1 (LEFT ARM) CONFIGURATION

fprintf('Configuring Manipulator 1 (Left Arm)...\n');

robot1 = struct();
robot1.n_joints = 3;
robot1.n_manipulators = 1;
robot1.name = 'Left Arm';

% Left arm: positioned at origin, 3-DOF RRR
robot1.link_vectors = [
    0.4,  0.3,  0.25;    % x components
    0,    0,    0;       % y components
    0,    0,    0        % z components
];

robot1.joint_axes = [
    0,  0,  0;   % x
    0,  0,  0;   % y
    1,  1,  1    % z (all rotate about Z-axis)
];

robot1.joint_types = 'RRR';
robot1.V_base = zeros(6, 1);

% Base position for robot 1 (left side)
robot1.base_position = [-0.5; 0; 0];  % 0.5m to the left

fprintf('  - 3-DOF RRR configuration\n');
fprintf('  - Base at [%.2f, %.2f, %.2f]\n', robot1.base_position(1), robot1.base_position(2), robot1.base_position(3));

%% MANIPULATOR 2 (RIGHT ARM) CONFIGURATION

fprintf('\nConfiguring Manipulator 2 (Right Arm)...\n');

robot2 = struct();
robot2.n_joints = 3;
robot2.n_manipulators = 1;
robot2.name = 'Right Arm';

% Right arm: mirror configuration, positioned on right side
robot2.link_vectors = [
    0.4,  0.3,  0.25;    % x components
    0,    0,    0;       % y components
    0,    0,    0        % z components
];

robot2.joint_axes = [
    0,  0,  0;   % x
    0,  0,  0;   % y
    1,  1,  1    % z
];

robot2.joint_types = 'RRR';
robot2.V_base = zeros(6, 1);

% Base position for robot 2 (right side)
robot2.base_position = [0.5; 0; 0];  % 0.5m to the right

fprintf('  - 3-DOF RRR configuration\n');
fprintf('  - Base at [%.2f, %.2f, %.2f]\n\n', robot2.base_position(1), robot2.base_position(2), robot2.base_position(3));

%% COMMON LOAD DEFINITION

fprintf('Defining Common Load...\n');

% Common load properties
common_load = struct();
common_load.mass = 2.0;  % kg
common_load.size = [0.3; 0.2; 0.1];  % [length, width, height] in meters
common_load.color = [0.8, 0.6, 0.2];  % Golden color

fprintf('  - Mass: %.1f kg\n', common_load.mass);
fprintf('  - Size: %.2f x %.2f x %.2f m\n\n', common_load.size(1), common_load.size(2), common_load.size(3));

%% COORDINATED TRAJECTORY - PICK, MOVE, PLACE

fprintf('Planning Coordinated Trajectory...\n');

% Define waypoints for coordinated motion
% Format: [q1_robot1, q2_robot1, q3_robot1; q1_robot2, q2_robot2, q3_robot2]

% Waypoint 1: Home position (arms apart)
wp1_robot1 = [pi/6; pi/4; pi/6];
wp1_robot2 = [-pi/6; pi/4; -pi/6];

% Waypoint 2: Approach load (arms moving toward center)
wp2_robot1 = [pi/3; pi/3; pi/4];
wp2_robot2 = [-pi/3; pi/3; -pi/4];

% Waypoint 3: Grasp load (close together)
wp3_robot1 = [pi/2.5; pi/3; pi/3];
wp3_robot2 = [-pi/2.5; pi/3; -pi/3];

% Waypoint 4: Lift load (both move up)
wp4_robot1 = [pi/2.5; pi/4; pi/6];
wp4_robot2 = [-pi/2.5; pi/4; -pi/6];

% Waypoint 5: Move load to new position (coordinated motion)
wp5_robot1 = [pi/4; pi/6; pi/8];
wp5_robot2 = [-pi/4; pi/6; -pi/8];

% Waypoint 6: Place load down
wp6_robot1 = [pi/4; pi/3; pi/4];
wp6_robot2 = [-pi/4; pi/3; -pi/4];

% Waypoint 7: Release and retract
wp7_robot1 = [pi/6; pi/4; pi/6];
wp7_robot2 = [-pi/6; pi/4; -pi/6];

% Waypoint 8: Return home
wp8_robot1 = [0; 0; 0];
wp8_robot2 = [0; 0; 0];

% Combine waypoints
waypoints_robot1 = [wp1_robot1, wp2_robot1, wp3_robot1, wp4_robot1, ...
                    wp5_robot1, wp6_robot1, wp7_robot1, wp8_robot1];
waypoints_robot2 = [wp1_robot2, wp2_robot2, wp3_robot2, wp4_robot2, ...
                    wp5_robot2, wp6_robot2, wp7_robot2, wp8_robot2];

n_waypoints = size(waypoints_robot1, 2);

fprintf('  - %d waypoints defined\n', n_waypoints);
fprintf('  - Phases: Home -> Approach -> Grasp -> Lift -> Move -> Place -> Release -> Home\n\n');

%% GENERATE SMOOTH TRAJECTORIES

fprintf('Generating synchronized trajectories...\n');

n_samples_per_segment = 40;
n_total = (n_waypoints - 1) * n_samples_per_segment;
segment_time = 2.0;  % seconds per segment

q_trajectory_robot1 = zeros(3, n_total);
q_trajectory_robot2 = zeros(3, n_total);
time_vector = zeros(1, n_total);

% Grasp state (when load is attached to robots)
grasp_states = false(1, n_total);  % Track when load is grasped

t_start = 0;
idx = 1;

for seg = 1:(n_waypoints - 1)
    q_start_r1 = waypoints_robot1(:, seg);
    q_end_r1 = waypoints_robot1(:, seg + 1);
    q_start_r2 = waypoints_robot2(:, seg);
    q_end_r2 = waypoints_robot2(:, seg + 1);
    
    t_end = t_start + segment_time;
    t_seg = linspace(t_start, t_end, n_samples_per_segment);
    
    % Determine if load is grasped in this segment
    % Load is grasped from waypoint 3 to waypoint 6
    is_grasped = (seg >= 3 && seg <= 6);
    
    for i = 1:n_samples_per_segment
        % Robot 1 trajectory
        for joint = 1:3
            [q, ~, ~] = ARAT_Core.traj_harmonic(t_seg(i), t_start, t_end, ...
                                                 q_start_r1(joint), q_end_r1(joint));
            q_trajectory_robot1(joint, idx) = q;
        end
        
        % Robot 2 trajectory (synchronized)
        for joint = 1:3
            [q, ~, ~] = ARAT_Core.traj_harmonic(t_seg(i), t_start, t_end, ...
                                                 q_start_r2(joint), q_end_r2(joint));
            q_trajectory_robot2(joint, idx) = q;
        end
        
        time_vector(idx) = t_seg(i);
        grasp_states(idx) = is_grasped;
        idx = idx + 1;
    end
    
    t_start = t_end;
end

fprintf('  - Total trajectory time: %.1f seconds\n', time_vector(end));
fprintf('  - Total samples: %d\n\n', n_total);

%% COMPUTE END-EFFECTOR POSITIONS

fprintf('Computing end-effector trajectories...\n');

ee_positions_robot1 = zeros(3, n_total);
ee_positions_robot2 = zeros(3, n_total);
load_positions = zeros(3, n_total);

for i = 1:n_total
    % Robot 1 end-effector
    pose1 = RobotIK.forward_kinematics_pose(robot1, q_trajectory_robot1(:, i));
    ee_positions_robot1(:, i) = pose1(4:6) + robot1.base_position;
    
    % Robot 2 end-effector
    pose2 = RobotIK.forward_kinematics_pose(robot2, q_trajectory_robot2(:, i));
    ee_positions_robot2(:, i) = pose2(4:6) + robot2.base_position;
    
    % Load position (center between end-effectors when grasped)
    if grasp_states(i)
        load_positions(:, i) = (ee_positions_robot1(:, i) + ee_positions_robot2(:, i)) / 2;
    else
        % Load stays at grasp position or initial position
        if i > 1 && any(grasp_states(1:i-1))
            % Keep last known position
            last_grasp_idx = find(grasp_states(1:i-1), 1, 'last');
            load_positions(:, i) = load_positions(:, last_grasp_idx);
        else
            % Initial position (between approach points)
            load_positions(:, i) = [0; 0.6; 0];
        end
    end
end

fprintf('  - End-effector paths computed\n\n');

%% JACOBIAN ANALYSIS FOR BOTH ROBOTS

fprintf('Analyzing manipulability...\n');

manip_robot1 = zeros(1, n_total);
manip_robot2 = zeros(1, n_total);
cond_robot1 = zeros(1, n_total);
cond_robot2 = zeros(1, n_total);

for i = 1:n_total
    % Robot 1
    J1 = ARAT_Core.compute_jacobian(robot1.link_vectors, robot1.joint_axes, ...
                                     robot1.joint_types, q_trajectory_robot1(:, i));
    manip_robot1(i) = sqrt(det(J1 * J1'));
    cond_robot1(i) = cond(J1);
    
    % Robot 2
    J2 = ARAT_Core.compute_jacobian(robot2.link_vectors, robot2.joint_axes, ...
                                     robot2.joint_types, q_trajectory_robot2(:, i));
    manip_robot2(i) = sqrt(det(J2 * J2'));
    cond_robot2(i) = cond(J2);
end

fprintf('  - Robot 1 avg manipulability: %.4f\n', mean(manip_robot1));
fprintf('  - Robot 2 avg manipulability: %.4f\n\n', mean(manip_robot2));

%% VISUALIZATION - WORKSPACE AND TRAJECTORIES

fprintf('Creating visualizations...\n');

% Figure 1: Workspace and trajectories
figure('Name', 'Dual-Arm Workspace', 'Position', [50, 50, 1200, 800]);

% Plot end-effector trajectories
plot3(ee_positions_robot1(1, :), ee_positions_robot1(2, :), ee_positions_robot1(3, :), ...
      'b-', 'LineWidth', 2, 'DisplayName', 'Robot 1 EE Path');
hold on;

plot3(ee_positions_robot2(1, :), ee_positions_robot2(2, :), ee_positions_robot2(3, :), ...
      'r-', 'LineWidth', 2, 'DisplayName', 'Robot 2 EE Path');

% Plot load trajectory
grasped_indices = find(grasp_states);
if ~isempty(grasped_indices)
    plot3(load_positions(1, grasped_indices), load_positions(2, grasped_indices), ...
          load_positions(3, grasped_indices), 'g-', 'LineWidth', 3, ...
          'DisplayName', 'Load Path');
end

% Plot base positions
scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
         200, 'b', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Robot 1 Base');
scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
         200, 'r', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Robot 2 Base');

% Plot start and end positions
scatter3(ee_positions_robot1(1, 1), ee_positions_robot1(2, 1), ee_positions_robot1(3, 1), ...
         100, 'b', 'filled', 'MarkerEdgeColor', 'y', 'LineWidth', 2);
scatter3(ee_positions_robot2(1, 1), ee_positions_robot2(2, 1), ee_positions_robot2(3, 1), ...
         100, 'r', 'filled', 'MarkerEdgeColor', 'y', 'LineWidth', 2);

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Dual-Arm Robot System - Workspace and Trajectories');
legend('Location', 'best');
axis equal;
view(45, 30);

% Draw workspace boundary
theta = linspace(0, 2*pi, 50);
r_max = 0.95;  % Maximum reach
x_circle = r_max * cos(theta);
y_circle = r_max * sin(theta);
z_circle = zeros(size(theta));

% Workspace for robot 1
plot3(x_circle + robot1.base_position(1), y_circle + robot1.base_position(2), ...
      z_circle, 'b--', 'LineWidth', 1, 'HandleVisibility', 'off');
% Workspace for robot 2
plot3(x_circle + robot2.base_position(1), y_circle + robot2.base_position(2), ...
      z_circle, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');

hold off;

% Figure 2: Joint trajectories comparison
figure('Name', 'Joint Trajectories', 'Position', [100, 100, 1400, 800]);

for i = 1:3
    % Robot 1 joints
    subplot(3, 2, (i-1)*2 + 1);
    plot(time_vector, q_trajectory_robot1(i, :), 'b-', 'LineWidth', 2);
    hold on;
    
    % Highlight grasped region
    grasped_times = time_vector(grasp_states);
    grasped_q = q_trajectory_robot1(i, grasp_states);
    plot(grasped_times, grasped_q, 'g-', 'LineWidth', 3);
    
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('Joint %d (rad)', i));
    title(sprintf('Robot 1 - Joint %d', i));
    legend('Trajectory', 'Load Grasped', 'Location', 'best');
    
    % Robot 2 joints
    subplot(3, 2, (i-1)*2 + 2);
    plot(time_vector, q_trajectory_robot2(i, :), 'r-', 'LineWidth', 2);
    hold on;
    
    grasped_q2 = q_trajectory_robot2(i, grasp_states);
    plot(grasped_times, grasped_q2, 'g-', 'LineWidth', 3);
    
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('Joint %d (rad)', i));
    title(sprintf('Robot 2 - Joint %d', i));
    legend('Trajectory', 'Load Grasped', 'Location', 'best');
end

% Figure 3: Manipulability comparison
figure('Name', 'Manipulability Analysis', 'Position', [150, 150, 1200, 600]);

subplot(2, 1, 1);
plot(time_vector, manip_robot1, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot 1');
hold on;
plot(time_vector, manip_robot2, 'r-', 'LineWidth', 2, 'DisplayName', 'Robot 2');

% Highlight grasped region
grasped_region = [grasped_times(1), grasped_times(end)];
ylims = ylim;
fill([grasped_region(1), grasped_region(2), grasped_region(2), grasped_region(1)], ...
     [ylims(1), ylims(1), ylims(2), ylims(2)], 'g', 'FaceAlpha', 0.2, ...
     'EdgeColor', 'none', 'DisplayName', 'Load Grasped');

grid on;
xlabel('Time (s)');
ylabel('Manipulability');
title('Manipulability During Coordinated Motion');
legend('Location', 'best');

subplot(2, 1, 2);
% End-effector distance (measure of coordination)
ee_distance = sqrt(sum((ee_positions_robot1 - ee_positions_robot2).^2, 1));
plot(time_vector, ee_distance, 'k-', 'LineWidth', 2);
hold on;
yline(common_load.size(1), 'g--', 'LineWidth', 2, 'DisplayName', 'Load Width');

fill([grasped_region(1), grasped_region(2), grasped_region(2), grasped_region(1)], ...
     [0, 0, max(ee_distance)*1.2, max(ee_distance)*1.2], 'g', 'FaceAlpha', 0.2, ...
     'EdgeColor', 'none');

grid on;
xlabel('Time (s)');
ylabel('Distance (m)');
title('End-Effector Separation Distance');
legend('EE Distance', 'Load Width', 'Location', 'best');

fprintf('Visualizations created.\n\n');

%% DUAL-ARM ANIMATION

fprintf('Starting dual-arm animation...\n');

fig_anim = figure('Name', 'Dual-Arm Collaborative Animation', 'Position', [200, 200, 1200, 800]);

% Compute animation bounds
all_positions = [ee_positions_robot1, ee_positions_robot2];
x_range = [min(all_positions(1,:)) - 0.3, max(all_positions(1,:)) + 0.3];
y_range = [min(all_positions(2,:)) - 0.3, max(all_positions(2,:)) + 0.3];
z_range = [-0.2, max(all_positions(3,:)) + 0.3];

dt = 0.05;  % Animation time step
frame_skip = max(1, floor(0.1 / (time_vector(2) - time_vector(1))));

for i = 1:frame_skip:n_total
    clf(fig_anim);
    hold on;
    grid on;
    axis equal;
    
    xlim(x_range);
    ylim(y_range);
    zlim(z_range);
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('Dual-Arm System - t=%.2fs | Phase: %s', time_vector(i), ...
                  get_phase_name(i, n_total, grasp_states(i))));
    view(45, 20);
    
    % Plot Robot 1
    [pos1, frames1] = compute_link_positions_with_base(robot1, q_trajectory_robot1(:, i), robot1.base_position);
    plot_robot_arm(pos1, frames1, 'b');
    
    % Plot Robot 2
    [pos2, frames2] = compute_link_positions_with_base(robot2, q_trajectory_robot2(:, i), robot2.base_position);
    plot_robot_arm(pos2, frames2, 'r');
    
    % Plot common load if grasped
    if grasp_states(i)
        load_pos = load_positions(:, i);
        draw_box(load_pos, common_load.size, common_load.color);
        
        % Draw connection lines
        plot3([ee_positions_robot1(1, i), load_pos(1)], ...
              [ee_positions_robot1(2, i), load_pos(2)], ...
              [ee_positions_robot1(3, i), load_pos(3)], 'g-', 'LineWidth', 2);
        plot3([ee_positions_robot2(1, i), load_pos(1)], ...
              [ee_positions_robot2(2, i), load_pos(2)], ...
              [ee_positions_robot2(3, i), load_pos(3)], 'g-', 'LineWidth', 2);
    end
    
    % Plot trajectory traces
    if i > 1
        plot3(ee_positions_robot1(1, 1:i), ee_positions_robot1(2, 1:i), ee_positions_robot1(3, 1:i), ...
              'b--', 'LineWidth', 1);
        plot3(ee_positions_robot2(1, 1:i), ee_positions_robot2(2, 1:i), ee_positions_robot2(3, 1:i), ...
              'r--', 'LineWidth', 1);
    end
    
    % Plot bases
    scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
             150, 'b', 'filled', 'MarkerEdgeColor', 'k');
    scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
             150, 'r', 'filled', 'MarkerEdgeColor', 'k');
    
    hold off;
    drawnow;
    pause(dt);
end

fprintf('Animation complete.\n\n');

%% EXPORT DATA

fprintf('Exporting dual-arm data...\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export trajectories
writematrix([time_vector', q_trajectory_robot1'], 'output/dual_arm_robot1_trajectory.csv');
writematrix([time_vector', q_trajectory_robot2'], 'output/dual_arm_robot2_trajectory.csv');

% Export end-effector positions
writematrix([time_vector', ee_positions_robot1'], 'output/dual_arm_robot1_ee_positions.csv');
writematrix([time_vector', ee_positions_robot2'], 'output/dual_arm_robot2_ee_positions.csv');

% Export load trajectory
writematrix([time_vector', load_positions', double(grasp_states)'], 'output/dual_arm_load_trajectory.csv');

% Export manipulability data
writematrix([time_vector', manip_robot1', manip_robot2', cond_robot1', cond_robot2'], ...
            'output/dual_arm_manipulability.csv');

% Export configuration
fid = fopen('output/dual_arm_configuration.txt', 'w');
fprintf(fid, 'Dual-Arm Collaborative Robot Configuration\n');
fprintf(fid, '==========================================\n\n');
fprintf(fid, 'Robot 1 (Left Arm):\n');
fprintf(fid, '  Base position: [%.2f, %.2f, %.2f]\n', robot1.base_position);
fprintf(fid, '  Joints: %d (%s)\n', robot1.n_joints, robot1.joint_types);
fprintf(fid, '  Avg manipulability: %.4f\n\n', mean(manip_robot1));

fprintf(fid, 'Robot 2 (Right Arm):\n');
fprintf(fid, '  Base position: [%.2f, %.2f, %.2f]\n', robot2.base_position);
fprintf(fid, '  Joints: %d (%s)\n', robot2.n_joints, robot2.joint_types);
fprintf(fid, '  Avg manipulability: %.4f\n\n', mean(manip_robot2));

fprintf(fid, 'Common Load:\n');
fprintf(fid, '  Mass: %.1f kg\n', common_load.mass);
fprintf(fid, '  Size: %.2f x %.2f x %.2f m\n\n', common_load.size);

fprintf(fid, 'Trajectory:\n');
fprintf(fid, '  Duration: %.1f seconds\n', time_vector(end));
fprintf(fid, '  Waypoints: %d\n', n_waypoints);
fprintf(fid, '  Samples: %d\n\n', n_total);

fclose(fid);

% Save complete workspace
save('output/dual_arm_complete_data.mat', 'robot1', 'robot2', 'common_load', ...
     'q_trajectory_robot1', 'q_trajectory_robot2', 'time_vector', ...
     'ee_positions_robot1', 'ee_positions_robot2', 'load_positions', ...
     'grasp_states', 'manip_robot1', 'manip_robot2');

fprintf('Data exported to output/\n\n');

%% SUMMARY

fprintf('========================================\n');
fprintf('Dual-Arm Simulation Complete!\n');
fprintf('========================================\n\n');

fprintf('Summary:\n');
fprintf('  ✓ Two 3-DOF manipulators configured\n');
fprintf('  ✓ Coordinated trajectories generated\n');
fprintf('  ✓ Common load handling simulated\n');
fprintf('  ✓ Workspace analysis completed\n');
fprintf('  ✓ Animation displayed\n');
fprintf('  ✓ Data exported\n\n');

fprintf('Key Metrics:\n');
fprintf('  - Total operation time: %.1f seconds\n', time_vector(end));
fprintf('  - Load grasp duration: %.1f seconds\n', sum(grasp_states) * (time_vector(2) - time_vector(1)));
fprintf('  - Avg EE separation: %.3f m\n', mean(ee_distance));
fprintf('  - Robot 1 avg manipulability: %.4f\n', mean(manip_robot1));
fprintf('  - Robot 2 avg manipulability: %.4f\n\n', mean(manip_robot2));

fprintf('========================================\n');

%% HELPER FUNCTIONS

function [positions, frames] = compute_link_positions_with_base(robot, q, base_pos)
    % Compute link positions relative to base position
    [pos, frames] = RobotVisualizer.compute_link_positions(robot, q);
    positions = pos + base_pos;
end

function plot_robot_arm(positions, frames, color)
    % Plot robot arm with given color
    n = size(positions, 2) - 1;
    
    % Plot links
    for k = 1:n
        p1 = positions(:, k);
        p2 = positions(:, k+1);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'Color', color, 'LineWidth', 4);
        scatter3(p1(1), p1(2), p1(3), 80, color, 'filled', ...
                'MarkerEdgeColor', 'k');
    end
    
    % End-effector
    p_end = positions(:, end);
    scatter3(p_end(1), p_end(2), p_end(3), 120, color, 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    % Coordinate frames (smaller)
    frame_scale = 0.08;
    for k = 1:size(frames, 3)
        p = positions(:, k);
        R = frames(:, :, k);
        quiver3(p(1), p(2), p(3), R(1,1)*frame_scale, R(2,1)*frame_scale, ...
               R(3,1)*frame_scale, 'r', 'LineWidth', 1, 'MaxHeadSize', 0.5);
        quiver3(p(1), p(2), p(3), R(1,2)*frame_scale, R(2,2)*frame_scale, ...
               R(3,2)*frame_scale, 'g', 'LineWidth', 1, 'MaxHeadSize', 0.5);
        quiver3(p(1), p(2), p(3), R(1,3)*frame_scale, R(2,3)*frame_scale, ...
               R(3,3)*frame_scale, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.5);
    end
end

function draw_box(center, size, color)
    % Draw a 3D box representing the load
    l = size(1) / 2;
    w = size(2) / 2;
    h = size(3) / 2;
    
    % Define box vertices
    vertices = [
        -l, -w, -h;
         l, -w, -h;
         l,  w, -h;
        -l,  w, -h;
        -l, -w,  h;
         l, -w,  h;
         l,  w,  h;
        -l,  w,  h
    ];
    
    % Translate to center
    vertices = vertices + center';
    
    % Define faces
    faces = [
        1, 2, 3, 4;  % Bottom
        5, 6, 7, 8;  % Top
        1, 2, 6, 5;  % Front
        2, 3, 7, 6;  % Right
        3, 4, 8, 7;  % Back
        4, 1, 5, 8   % Left
    ];
    
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, ...
          'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5);
end

function phase = get_phase_name(idx, n_total, is_grasped)
    % Get current phase name based on progress
    progress = idx / n_total;
    
    if progress < 0.125
        phase = 'Home Position';
    elseif progress < 0.25
        phase = 'Approaching Load';
    elseif progress < 0.375
        phase = 'Grasping Load';
    elseif progress < 0.5
        phase = 'Lifting Load';
    elseif progress < 0.625
        phase = 'Moving Load';
    elseif progress < 0.75
        phase = 'Placing Load';
    elseif progress < 0.875
        phase = 'Releasing';
    else
        phase = 'Returning Home';
    end
    
    if is_grasped
        phase = [phase, ' [GRASPED]'];
    end
end
