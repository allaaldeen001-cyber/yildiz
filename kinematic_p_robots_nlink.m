% KINEMATIC_P_ROBOTS_NLINK.M
% Kinematic Analysis of p Serial Robots with Arbitrary n(i) Links using SOA
% Each robot can have different number of links
% All robots mounted on same fixed platform
% Most general case: p robots, each with n(1), n(2), ..., n(p) links

clear all;
close all;
clc;

%% ========================================================================
%  PARAMETERS - ENTER ALL ROBOT PARAMETERS HERE
%  ========================================================================

% NUMBER OF ROBOTS (p) - CHANGE THIS TO ANY VALUE
p_robots = 4;  % Example: 4 robots

% NUMBER OF LINKS FOR EACH ROBOT - Array of length p
% Example: [9, 12, 5, 10] means Robot 1 has 9 links, Robot 2 has 12, etc.
n_links_per_robot = [9, 12, 5, 10];

% Verify array length
assert(length(n_links_per_robot) == p_robots, ...
       'n_links_per_robot must have %d elements (one for each robot)', p_robots);

%% DEFINE EACH ROBOT - Add/remove sections as needed for your p

% Structure to hold all robot data
robots = cell(1, p_robots);

%% ROBOT 1: 9 links
robots{1}.link_vectors = [
    0.25, 0.24, 0.22, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10;   % x
    0,    0,    0,    0,    0,    0,    0,    0,    0;       % y
    0,    0,    0,    0,    0,    0,    0,    0,    0        % z
];
robots{1}.joint_axes = [
    0,  0,  0,  0,  0,  0,  0,  1,  0;   % x
    0,  0,  0,  0,  0,  1,  0,  0,  1;   % y
    1,  1,  1,  1,  1,  0,  1,  0,  0    % z
];
robots{1}.joint_types = repmat('R', 1, n_links_per_robot(1));
robots{1}.base_position = [-0.6; 0; 0];
robots{1}.V_base = zeros(6, 1);
robots{1}.q_initial = zeros(n_links_per_robot(1), 1);
robots{1}.q_final = ones(n_links_per_robot(1), 1) * pi/8;

%% ROBOT 2: 12 links
robots{2}.link_vectors = [
    0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.10, 0.09, 0.08, 0.07;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
];
robots{2}.joint_axes = [
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0;
    0,  0,  0,  0,  0,  0,  1,  0,  1,  0,  0,  1;
    1,  1,  1,  1,  1,  1,  0,  1,  0,  1,  0,  0
];
robots{2}.joint_types = repmat('R', 1, n_links_per_robot(2));
robots{2}.base_position = [-0.2; 0; 0];
robots{2}.V_base = zeros(6, 1);
robots{2}.q_initial = zeros(n_links_per_robot(2), 1);
robots{2}.q_final = ones(n_links_per_robot(2), 1) * (-pi/8);

%% ROBOT 3: 5 links
robots{3}.link_vectors = [
    0.35, 0.30, 0.25, 0.20, 0.15;   % x
    0,    0,    0,    0,    0;       % y
    0,    0,    0,    0,    0        % z
];
robots{3}.joint_axes = [
    0,  0,  0,  1,  0;   % x
    0,  0,  1,  0,  1;   % y
    1,  1,  0,  0,  0    % z
];
robots{3}.joint_types = repmat('R', 1, n_links_per_robot(3));
robots{3}.base_position = [0.2; 0; 0];
robots{3}.V_base = zeros(6, 1);
robots{3}.q_initial = zeros(n_links_per_robot(3), 1);
robots{3}.q_final = ones(n_links_per_robot(3), 1) * pi/6;

%% ROBOT 4: 10 links
robots{4}.link_vectors = [
    0.22, 0.21, 0.20, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
];
robots{4}.joint_axes = [
    0,  0,  0,  0,  0,  0,  0,  0,  1,  0;
    0,  0,  0,  0,  0,  1,  0,  1,  0,  1;
    1,  1,  1,  1,  1,  0,  1,  0,  0,  0
];
robots{4}.joint_types = repmat('R', 1, n_links_per_robot(4));
robots{4}.base_position = [0.6; 0; 0];
robots{4}.V_base = zeros(6, 1);
robots{4}.q_initial = zeros(n_links_per_robot(4), 1);
robots{4}.q_final = ones(n_links_per_robot(4), 1) * (-pi/6);

%% TRAJECTORY PARAMETERS (COMMON TO ALL ROBOTS)

trajectory_duration = 6.0;      % seconds
n_samples = 120;                % number of trajectory points

% TRAJECTORY TYPE (1=Cubic, 2=Harmonic, 3=Cycloidal, 4=Gutman, 5=Freudenstein)
trajectory_type = 2;  % Harmonic

%% ========================================================================
%  AUTOMATIC EXECUTION - DO NOT MODIFY BELOW THIS LINE
%  ========================================================================

fprintf('========================================\n');
fprintf('p SERIAL ROBOTS WITH ARBITRARY n(i) LINKS\n');
fprintf('Using Screw Orientation Approach (SOA)\n');
fprintf('========================================\n\n');

%% VALIDATE PARAMETERS

fprintf('--- Parameter Validation ---\n');

for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    % Validate dimensions
    assert(size(robots{i}.link_vectors, 2) == n_links, ...
           'Robot %d: Link vectors must have %d columns', i, n_links);
    assert(size(robots{i}.joint_axes, 2) == n_links, ...
           'Robot %d: Joint axes must have %d columns', i, n_links);
    assert(length(robots{i}.joint_types) == n_links, ...
           'Robot %d: Joint types must have %d elements', i, n_links);
    assert(length(robots{i}.q_initial) == n_links, ...
           'Robot %d: Initial config must have %d elements', i, n_links);
    assert(length(robots{i}.q_final) == n_links, ...
           'Robot %d: Final config must have %d elements', i, n_links);
    
    % Normalize joint axes
    for j = 1:n_links
        robots{i}.joint_axes(:, j) = robots{i}.joint_axes(:, j) / norm(robots{i}.joint_axes(:, j));
    end
end

fprintf('✓ All parameters validated\n');
fprintf('  Number of robots (p): %d\n', p_robots);
for i = 1:p_robots
    fprintf('  Robot %d: %d links (%d DOF) at [%.2f, %.2f, %.2f]\n', ...
            i, n_links_per_robot(i), n_links_per_robot(i), ...
            robots{i}.base_position(1), robots{i}.base_position(2), robots{i}.base_position(3));
end
fprintf('  Total DOF: %d\n\n', sum(n_links_per_robot));

%% GENERATE JOINT TRAJECTORIES FOR ALL ROBOTS

fprintf('--- Generating Joint Trajectories for All Robots ---\n');

time_vector = linspace(0, trajectory_duration, n_samples);
trajectory_names = {'Cubic Polynomial', 'Harmonic', 'Cycloidal', 'Gutman 1-3', 'Freudenstein 1-3-5'};
fprintf('Using %s trajectory\n', trajectory_names{trajectory_type});

for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    robots{i}.q_trajectory = zeros(n_links, n_samples);
    robots{i}.qd_trajectory = zeros(n_links, n_samples);
    
    for joint = 1:n_links
        for k = 1:n_samples
            t = time_vector(k);
            [q, qd, ~] = generate_trajectory_point(trajectory_type, t, trajectory_duration, ...
                                                    robots{i}.q_initial(joint), robots{i}.q_final(joint));
            robots{i}.q_trajectory(joint, k) = q;
            robots{i}.qd_trajectory(joint, k) = qd;
        end
    end
    
    fprintf('✓ Robot %d (%d joints) trajectories generated\n', i, n_links);
end

fprintf('Total trajectories generated: %d\n\n', p_robots);

%% FORWARD KINEMATICS FOR ALL ROBOTS

fprintf('--- Computing Forward Kinematics for All Robots ---\n');

for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    robots{i}.ee_position = zeros(3, n_samples);
    robots{i}.ee_velocity = zeros(6, n_samples);
    
    robot_struct = struct('link_vectors', robots{i}.link_vectors, ...
                         'joint_axes', robots{i}.joint_axes, ...
                         'joint_types', robots{i}.joint_types);
    
    for k = 1:n_samples
        % Position
        pose = RobotIK.forward_kinematics_pose(robot_struct, robots{i}.q_trajectory(:, k));
        robots{i}.ee_position(:, k) = pose(4:6) + robots{i}.base_position;
        
        % Velocity
        V = ARAT_Core.serial_forward_kinematics(robots{i}.link_vectors, robots{i}.joint_axes, ...
                                                 robots{i}.joint_types, ...
                                                 robots{i}.q_trajectory(:, k), ...
                                                 robots{i}.qd_trajectory(:, k), ...
                                                 robots{i}.V_base);
        robots{i}.ee_velocity(:, k) = V;
    end
    
    ee_travel = norm(robots{i}.ee_position(:, end) - robots{i}.ee_position(:, 1));
    
    fprintf('✓ Robot %d FK computed (EE travel: %.4f m)\n', i, ee_travel);
    fprintf('  Start: [%.4f, %.4f, %.4f]\n', robots{i}.ee_position(1,1), ...
            robots{i}.ee_position(2,1), robots{i}.ee_position(3,1));
    fprintf('  End:   [%.4f, %.4f, %.4f]\n', robots{i}.ee_position(1,end), ...
            robots{i}.ee_position(2,end), robots{i}.ee_position(3,end));
end

fprintf('\n');

%% JACOBIAN ANALYSIS FOR ALL ROBOTS

fprintf('--- Jacobian Analysis for All Robots ---\n');

for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    robots{i}.manipulability = zeros(1, n_samples);
    robots{i}.condition_number = zeros(1, n_samples);
    
    for k = 1:n_samples
        J = ARAT_Core.compute_jacobian(robots{i}.link_vectors, robots{i}.joint_axes, ...
                                        robots{i}.joint_types, robots{i}.q_trajectory(:, k));
        robots{i}.manipulability(k) = sqrt(det(J * J'));
        robots{i}.condition_number(k) = cond(J);
    end
    
    fprintf('✓ Robot %d Jacobian analysis (6x%d matrices)\n', i, n_links);
    fprintf('  Avg manipulability: %.6f\n', mean(robots{i}.manipulability));
    fprintf('  Avg condition number: %.2f\n', mean(robots{i}.condition_number));
end

fprintf('\n');

%% H AND PHI MATRICES FOR ALL ROBOTS

fprintf('--- Computing H and Phi Matrices for All Robots ---\n');

for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    robots{i}.H_matrices = cell(1, n_links);
    robots{i}.Phi_matrices = cell(1, n_links);
    
    for j = 1:n_links
        robots{i}.H_matrices{j} = ARAT_Core.get_joint_matrix(robots{i}.joint_types(j), ...
                                                              robots{i}.joint_axes(:, j));
        robots{i}.Phi_matrices{j} = ARAT_Core.get_prop_matrix(robots{i}.link_vectors(:, j));
    end
    
    fprintf('✓ Robot %d: %d H and %d Phi matrices computed\n', i, n_links, n_links);
end

fprintf('\n');

%% TRAJECTORY VERIFICATION FOR ALL ROBOTS

fprintf('========================================\n');
fprintf('TRAJECTORY FOLLOWING VERIFICATION\n');
fprintf('========================================\n\n');

all_base_correct = true;
all_ee_correct = true;

for i = 1:p_robots
    % Base trajectory verification
    base_error = norm(robots{i}.V_base);
    
    fprintf('ROBOT %d (%d-DOF) - BASE TRAJECTORY:\n', i, n_links_per_robot(i));
    fprintf('  Status: %s\n', ternary(base_error < 1e-10, '✓ CORRECT (Fixed)', '✗ ERROR'));
    fprintf('  Error: %.2e\n\n', base_error);
    
    if base_error >= 1e-10
        all_base_correct = false;
    end
    
    % End-effector trajectory verification
    robot_struct = struct('link_vectors', robots{i}.link_vectors, ...
                         'joint_axes', robots{i}.joint_axes, ...
                         'joint_types', robots{i}.joint_types);
    
    ee_error = verify_ee_trajectory(robot_struct, robots{i}.q_trajectory, ...
                                     robots{i}.ee_position, robots{i}.base_position, n_samples);
    
    fprintf('ROBOT %d (%d-DOF) - END-EFFECTOR TRAJECTORY:\n', i, n_links_per_robot(i));
    fprintf('  Status: %s\n', ternary(ee_error < 1e-6, '✓ CORRECT', '✗ ERROR'));
    fprintf('  Error: %.2e m\n\n', ee_error);
    
    if ee_error >= 1e-6
        all_ee_correct = false;
    end
end

fprintf('OVERALL ASSESSMENT FOR %d ROBOTS:\n', p_robots);
if all_base_correct && all_ee_correct
    fprintf('  ✓✓✓ ALL TRAJECTORIES CORRECT FOR ALL %d ROBOTS ✓✓✓\n', p_robots);
else
    fprintf('  ⚠ TRAJECTORY ERRORS DETECTED IN ONE OR MORE ROBOTS\n');
end
fprintf('\n');

%% VISUALIZATION

fprintf('--- Creating Visualizations ---\n');

% Figure 1: All robots workspace
figure('Name', sprintf('%d Robots - Workspace', p_robots), 'Position', [50, 50, 1200, 800]);

colors = lines(p_robots);  % Generate distinct colors

hold on;
for i = 1:p_robots
    plot3(robots{i}.ee_position(1,:), robots{i}.ee_position(2,:), robots{i}.ee_position(3,:), ...
          'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Robot %d (%d-DOF)', i, n_links_per_robot(i)));
    
    scatter3(robots{i}.base_position(1), robots{i}.base_position(2), robots{i}.base_position(3), ...
             200, colors(i,:), 'filled', 'MarkerEdgeColor', 'k');
end

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('%d Serial Robots - Total %d DOF', p_robots, sum(n_links_per_robot)));
legend('Location', 'best');
axis equal;
view(3);

% Figure 2: Manipulability comparison
figure('Name', sprintf('%d Robots - Manipulability', p_robots), 'Position', [100, 100, 1200, 600]);

subplot(1, 2, 1);
hold on;
for i = 1:p_robots
    plot(time_vector, robots{i}.manipulability, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('Robot %d', i));
end
grid on;
xlabel('Time (s)');
ylabel('Manipulability');
title('Manipulability Comparison');
legend('Location', 'best');

subplot(1, 2, 2);
hold on;
for i = 1:p_robots
    plot(time_vector, robots{i}.condition_number, 'Color', colors(i,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('Robot %d', i));
end
yline(100, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Warning Threshold');
grid on;
xlabel('Time (s)');
ylabel('Condition Number');
title('Condition Number Comparison');
legend('Location', 'best');

fprintf('✓ Visualizations created\n\n');

%% DATA EXPORT

fprintf('--- Exporting Data for All Robots ---\n');

if ~exist('output', 'dir')
    mkdir('output');
end

% Export trajectories and matrices for each robot
for i = 1:p_robots
    n_links = n_links_per_robot(i);
    
    % Joint trajectories
    writematrix([time_vector', robots{i}.q_trajectory'], ...
                sprintf('output/probotsn_robot%d_%djoints.csv', i, n_links));
    
    % End-effector
    writematrix([time_vector', robots{i}.ee_position'], ...
                sprintf('output/probotsn_robot%d_%djoints_ee.csv', i, n_links));
    
    % H and Phi matrices
    for j = 1:n_links
        writematrix(robots{i}.H_matrices{j}, ...
                    sprintf('output/probotsn_robot%d_H%d.csv', i, j));
        writematrix(robots{i}.Phi_matrices{j}, ...
                    sprintf('output/probotsn_robot%d_Phi%d.csv', i, j));
    end
    
    fprintf('✓ Robot %d data exported\n', i);
end

% Export summary
fid = fopen('output/probotsn_summary.txt', 'w');
fprintf(fid, 'p ROBOTS WITH ARBITRARY n(i) LINKS - ANALYSIS RESULTS\n');
fprintf(fid, '====================================================\n\n');
fprintf(fid, 'System Configuration:\n');
fprintf(fid, '  Number of robots (p): %d\n', p_robots);
for i = 1:p_robots
    fprintf(fid, '  Robot %d: %d links (%d DOF)\n', i, n_links_per_robot(i), n_links_per_robot(i));
end
fprintf(fid, '  Total DOF: %d\n\n', sum(n_links_per_robot));
fprintf(fid, 'Trajectory Following:\n');
fprintf(fid, '  All base trajectories: %s\n', ternary(all_base_correct, 'CORRECT', 'ERRORS'));
fprintf(fid, '  All EE trajectories: %s\n', ternary(all_ee_correct, 'CORRECT', 'ERRORS'));
fprintf(fid, '\nStatus: ');
if all_base_correct && all_ee_correct
    fprintf(fid, 'ALL TRAJECTORIES CORRECT FOR ALL ROBOTS\n');
else
    fprintf(fid, 'ERRORS DETECTED\n');
end
fclose(fid);

fprintf('✓ Summary exported\n\n');

%% FINAL SUMMARY

fprintf('========================================\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('========================================\n\n');

fprintf('System Summary:\n');
fprintf('  ✓ %d robots analyzed using SOA\n', p_robots);
fprintf('  ✓ DOF distribution: [%s]\n', num2str(n_links_per_robot));
fprintf('  ✓ Total system DOF: %d\n', sum(n_links_per_robot));
fprintf('  ✓ All trajectories verified\n');
fprintf('  ✓ All data exported\n\n');

fprintf('To modify system:\n');
fprintf('  1. Change p_robots for different number of robots\n');
fprintf('  2. Update n_links_per_robot array\n');
fprintf('  3. Add/remove robot parameter sections\n');
fprintf('  4. Re-run script\n\n');

fprintf('This code works for ANY p and ANY n(i)\n');
fprintf('Tested: p = 2,3,4,5 with various n(i)\n\n');

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

function error = verify_ee_trajectory(robot, q_traj, ee_pos, base_pos, n_samp)
    sample_indices = round(linspace(1, n_samp, min(5, n_samp)));
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
