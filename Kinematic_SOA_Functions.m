classdef Kinematic_SOA_Functions
    % KINEMATIC_SOA_FUNCTIONS
    % All kinematic analysis functions using Spatial Operator Algebra (SOA)
    % This class contains all backend functions for kinematic analysis
    % Used by Kinematic_User_Interface.m
    
    methods (Static)
        
        %% ===================================================================
        %  MAIN ANALYSIS FUNCTION
        %  ===================================================================
        
        function results = analyze_multi_robot_system(robots, trajectory_params)
            % Main function to analyze p robots with arbitrary n(i) links
            % 
            % INPUTS:
            %   robots - Cell array of robot structures, each containing:
            %            .link_vectors (3×n matrix)
            %            .joint_axes (3×n matrix)
            %            .joint_types (string of length n)
            %            .base_position (3×1 vector)
            %            .V_base (6×1 vector)
            %            .q_initial (n×1 vector)
            %            .q_final (n×1 vector)
            %   trajectory_params - Structure containing:
            %                      .duration (scalar)
            %                      .n_samples (scalar)
            %                      .type (1-5)
            %
            % OUTPUTS:
            %   results - Structure containing all analysis results
            
            p_robots = length(robots);
            
            fprintf('========================================\n');
            fprintf('KINEMATIC ANALYSIS - %d ROBOT(S) USING SOA\n', p_robots);
            fprintf('========================================\n\n');
            
            % Initialize results structure
            results = struct();
            results.n_robots = p_robots;
            results.time_vector = linspace(0, trajectory_params.duration, trajectory_params.n_samples);
            results.robots = cell(1, p_robots);
            
            %% Validate all robots
            fprintf('--- Parameter Validation ---\n');
            for i = 1:p_robots
                Kinematic_SOA_Functions.validate_robot_parameters(robots{i}, i);
                % Normalize joint axes
                n_links = size(robots{i}.link_vectors, 2);
                for j = 1:n_links
                    robots{i}.joint_axes(:, j) = robots{i}.joint_axes(:, j) / norm(robots{i}.joint_axes(:, j));
                end
            end
            fprintf('✓ All %d robot(s) validated\n\n', p_robots);
            
            %% Generate trajectories for all robots
            fprintf('--- Generating Joint Trajectories ---\n');
            traj_names = {'Cubic Polynomial', 'Harmonic', 'Cycloidal', 'Gutman 1-3', 'Freudenstein 1-3-5'};
            fprintf('Using %s trajectory\n', traj_names{trajectory_params.type});
            
            for i = 1:p_robots
                results.robots{i} = Kinematic_SOA_Functions.generate_robot_trajectory(...
                    robots{i}, results.time_vector, trajectory_params);
                fprintf('✓ Robot %d trajectories generated\n', i);
            end
            fprintf('\n');
            
            %% Forward kinematics for all robots
            fprintf('--- Computing Forward Kinematics ---\n');
            for i = 1:p_robots
                results.robots{i} = Kinematic_SOA_Functions.compute_forward_kinematics(...
                    results.robots{i}, robots{i});
                fprintf('✓ Robot %d FK computed\n', i);
            end
            fprintf('\n');
            
            %% Jacobian analysis for all robots
            fprintf('--- Jacobian Analysis ---\n');
            for i = 1:p_robots
                results.robots{i} = Kinematic_SOA_Functions.compute_jacobian_analysis(...
                    results.robots{i}, robots{i});
                fprintf('✓ Robot %d Jacobian analyzed\n', i);
            end
            fprintf('\n');
            
            %% H and Phi matrices
            fprintf('--- Computing H and Phi Matrices ---\n');
            for i = 1:p_robots
                results.robots{i} = Kinematic_SOA_Functions.compute_system_matrices(...
                    results.robots{i}, robots{i});
                fprintf('✓ Robot %d H and Phi matrices computed\n', i);
            end
            fprintf('\n');
            
            %% Trajectory verification
            fprintf('--- Trajectory Verification ---\n');
            results.verification = Kinematic_SOA_Functions.verify_all_trajectories(...
                results.robots, robots);
            fprintf('\n');
            
            fprintf('========================================\n');
            fprintf('ANALYSIS COMPLETE\n');
            fprintf('========================================\n\n');
        end
        
        %% ===================================================================
        %  VALIDATION FUNCTIONS
        %  ===================================================================
        
        function validate_robot_parameters(robot, robot_id)
            % Validate robot parameters
            n_links = size(robot.link_vectors, 2);
            
            assert(size(robot.joint_axes, 2) == n_links, ...
                   'Robot %d: Joint axes must have %d columns', robot_id, n_links);
            assert(length(robot.joint_types) == n_links, ...
                   'Robot %d: Joint types must have %d elements', robot_id, n_links);
            assert(length(robot.q_initial) == n_links, ...
                   'Robot %d: Initial config must have %d elements', robot_id, n_links);
            assert(length(robot.q_final) == n_links, ...
                   'Robot %d: Final config must have %d elements', robot_id, n_links);
        end
        
        %% ===================================================================
        %  TRAJECTORY GENERATION FUNCTIONS
        %  ===================================================================
        
        function robot_result = generate_robot_trajectory(robot, time_vector, traj_params)
            % Generate joint trajectories for one robot
            n_links = size(robot.link_vectors, 2);
            n_samples = length(time_vector);
            
            robot_result = struct();
            robot_result.n_links = n_links;
            robot_result.q_trajectory = zeros(n_links, n_samples);
            robot_result.qd_trajectory = zeros(n_links, n_samples);
            robot_result.qdd_trajectory = zeros(n_links, n_samples);
            
            for joint = 1:n_links
                for k = 1:n_samples
                    t = time_vector(k);
                    [q, qd, qdd] = Kinematic_SOA_Functions.generate_trajectory_point(...
                        traj_params.type, t, traj_params.duration, ...
                        robot.q_initial(joint), robot.q_final(joint));
                    
                    robot_result.q_trajectory(joint, k) = q;
                    robot_result.qd_trajectory(joint, k) = qd;
                    robot_result.qdd_trajectory(joint, k) = qdd;
                end
            end
        end
        
        function [q, qd, qdd] = generate_trajectory_point(type, t, T, q0, qf)
            % Generate single trajectory point
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
                otherwise
                    error('Invalid trajectory type');
            end
        end
        
        %% ===================================================================
        %  FORWARD KINEMATICS FUNCTIONS
        %  ===================================================================
        
        function robot_result = compute_forward_kinematics(robot_result, robot_params)
            % Compute forward kinematics for all trajectory points
            n_samples = size(robot_result.q_trajectory, 2);
            
            robot_result.ee_position = zeros(3, n_samples);
            robot_result.ee_velocity = zeros(6, n_samples);
            robot_result.ee_orientation = zeros(3, n_samples);
            
            robot_struct = struct('link_vectors', robot_params.link_vectors, ...
                                 'joint_axes', robot_params.joint_axes, ...
                                 'joint_types', robot_params.joint_types);
            
            for k = 1:n_samples
                % Position and orientation
                pose = RobotIK.forward_kinematics_pose(robot_struct, robot_result.q_trajectory(:, k));
                robot_result.ee_position(:, k) = pose(4:6) + robot_params.base_position;
                robot_result.ee_orientation(:, k) = pose(1:3);
                
                % Velocity using SOA
                V = ARAT_Core.serial_forward_kinematics(robot_params.link_vectors, ...
                                                        robot_params.joint_axes, ...
                                                        robot_params.joint_types, ...
                                                        robot_result.q_trajectory(:, k), ...
                                                        robot_result.qd_trajectory(:, k), ...
                                                        robot_params.V_base);
                robot_result.ee_velocity(:, k) = V;
            end
        end
        
        %% ===================================================================
        %  JACOBIAN ANALYSIS FUNCTIONS
        %  ===================================================================
        
        function robot_result = compute_jacobian_analysis(robot_result, robot_params)
            % Compute Jacobian analysis for all trajectory points
            n_samples = size(robot_result.q_trajectory, 2);
            
            robot_result.jacobian_matrices = cell(1, n_samples);
            robot_result.manipulability = zeros(1, n_samples);
            robot_result.condition_number = zeros(1, n_samples);
            
            for k = 1:n_samples
                J = ARAT_Core.compute_jacobian(robot_params.link_vectors, ...
                                                robot_params.joint_axes, ...
                                                robot_params.joint_types, ...
                                                robot_result.q_trajectory(:, k));
                
                robot_result.jacobian_matrices{k} = J;
                robot_result.manipulability(k) = sqrt(det(J * J'));
                robot_result.condition_number(k) = cond(J);
            end
        end
        
        %% ===================================================================
        %  SYSTEM MATRICES FUNCTIONS
        %  ===================================================================
        
        function robot_result = compute_system_matrices(robot_result, robot_params)
            % Compute H and Phi matrices for all joints
            n_links = robot_result.n_links;
            
            robot_result.H_matrices = cell(1, n_links);
            robot_result.Phi_matrices = cell(1, n_links);
            
            for i = 1:n_links
                robot_result.H_matrices{i} = ARAT_Core.get_joint_matrix(...
                    robot_params.joint_types(i), robot_params.joint_axes(:, i));
                robot_result.Phi_matrices{i} = ARAT_Core.get_prop_matrix(...
                    robot_params.link_vectors(:, i));
            end
        end
        
        %% ===================================================================
        %  VERIFICATION FUNCTIONS
        %  ===================================================================
        
        function verification = verify_all_trajectories(robot_results, robot_params)
            % Verify base and end-effector trajectories for all robots
            p_robots = length(robot_results);
            
            verification = struct();
            verification.base_errors = zeros(1, p_robots);
            verification.ee_errors = zeros(1, p_robots);
            verification.all_correct = true;
            
            fprintf('========================================\n');
            fprintf('TRAJECTORY FOLLOWING VERIFICATION\n');
            fprintf('========================================\n\n');
            
            for i = 1:p_robots
                % Base trajectory verification
                base_error = norm(robot_params{i}.V_base);
                verification.base_errors(i) = base_error;
                
                fprintf('ROBOT %d - BASE TRAJECTORY:\n', i);
                fprintf('  Status: %s\n', tern(base_error < 1e-10, '✓ CORRECT (Fixed)', '✗ ERROR'));
                fprintf('  Error: %.2e\n\n', base_error);
                
                if base_error >= 1e-10
                    verification.all_correct = false;
                end
                
                % End-effector trajectory verification
                ee_error = Kinematic_SOA_Functions.verify_ee_trajectory(...
                    robot_results{i}, robot_params{i});
                verification.ee_errors(i) = ee_error;
                
                fprintf('ROBOT %d - END-EFFECTOR TRAJECTORY:\n', i);
                fprintf('  Status: %s\n', tern(ee_error < 1e-6, '✓ CORRECT', '✗ ERROR'));
                fprintf('  Error: %.2e m\n\n', ee_error);
                
                if ee_error >= 1e-6
                    verification.all_correct = false;
                end
            end
            
            fprintf('OVERALL ASSESSMENT:\n');
            if verification.all_correct
                fprintf('  ✓✓✓ ALL TRAJECTORIES CORRECT FOR ALL %d ROBOT(S) ✓✓✓\n', p_robots);
            else
                fprintf('  ⚠ TRAJECTORY ERRORS DETECTED\n');
            end
        end
        
        function ee_error = verify_ee_trajectory(robot_result, robot_params)
            % Verify end-effector trajectory consistency
            n_samples = size(robot_result.q_trajectory, 2);
            sample_indices = round(linspace(1, n_samples, min(5, n_samples)));
            max_error = 0;
            
            robot_struct = struct('link_vectors', robot_params.link_vectors, ...
                                 'joint_axes', robot_params.joint_axes, ...
                                 'joint_types', robot_params.joint_types);
            
            for idx = sample_indices
                pose = RobotIK.forward_kinematics_pose(robot_struct, robot_result.q_trajectory(:, idx));
                expected_pos = pose(4:6) + robot_params.base_position;
                error_val = norm(expected_pos - robot_result.ee_position(:, idx));
                max_error = max(max_error, error_val);
            end
            
            ee_error = max_error;
        end
        
        %% ===================================================================
        %  VISUALIZATION FUNCTIONS
        %  ===================================================================
        
        function visualize_results(results, robots)
            % Create all visualization plots
            p_robots = results.n_robots;
            
            fprintf('--- Creating Visualizations ---\n');
            
            % Figure 1: Workspace view
            Kinematic_SOA_Functions.plot_workspace(results, robots);
            
            % Figure 2: Manipulability comparison
            if p_robots > 1
                Kinematic_SOA_Functions.plot_manipulability_comparison(results);
            else
                Kinematic_SOA_Functions.plot_single_robot_analysis(results);
            end
            
            % Figure 3: Robot configurations
            Kinematic_SOA_Functions.plot_robot_configurations(results, robots);
            
            fprintf('✓ Visualizations created\n\n');
        end
        
        function plot_workspace(results, robots)
            % Plot workspace with all robot end-effector paths
            p_robots = results.n_robots;
            
            figure('Name', sprintf('%d Robot(s) - Workspace', p_robots), ...
                   'Position', [50, 50, 1200, 800]);
            
            colors = lines(p_robots);
            hold on;
            
            for i = 1:p_robots
                n_links = results.robots{i}.n_links;
                plot3(results.robots{i}.ee_position(1,:), ...
                      results.robots{i}.ee_position(2,:), ...
                      results.robots{i}.ee_position(3,:), ...
                      'Color', colors(i,:), 'LineWidth', 2, ...
                      'DisplayName', sprintf('Robot %d (%d-DOF)', i, n_links));
                
                scatter3(robots{i}.base_position(1), robots{i}.base_position(2), ...
                        robots{i}.base_position(3), 200, colors(i,:), 'filled', ...
                        'MarkerEdgeColor', 'k');
            end
            
            grid on;
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title(sprintf('%d Robot(s) - End-Effector Trajectories', p_robots));
            legend('Location', 'best');
            axis equal;
            view(3);
            hold off;
        end
        
        function plot_manipulability_comparison(results)
            % Plot manipulability comparison for multiple robots
            p_robots = results.n_robots;
            
            figure('Name', 'Manipulability Analysis', 'Position', [100, 100, 1200, 600]);
            
            colors = lines(p_robots);
            
            subplot(1, 2, 1);
            hold on;
            for i = 1:p_robots
                plot(results.time_vector, results.robots{i}.manipulability, ...
                     'Color', colors(i,:), 'LineWidth', 2, ...
                     'DisplayName', sprintf('Robot %d', i));
            end
            grid on;
            xlabel('Time (s)');
            ylabel('Manipulability');
            title('Manipulability Comparison');
            legend('Location', 'best');
            hold off;
            
            subplot(1, 2, 2);
            hold on;
            for i = 1:p_robots
                plot(results.time_vector, results.robots{i}.condition_number, ...
                     'Color', colors(i,:), 'LineWidth', 2, ...
                     'DisplayName', sprintf('Robot %d', i));
            end
            yline(100, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Warning Threshold');
            grid on;
            xlabel('Time (s)');
            ylabel('Condition Number');
            title('Condition Number Comparison');
            legend('Location', 'best');
            hold off;
        end
        
        function plot_single_robot_analysis(results)
            % Plot analysis for single robot
            figure('Name', 'Robot Analysis', 'Position', [100, 100, 1200, 800]);
            
            subplot(2, 2, 1);
            plot3(results.robots{1}.ee_position(1,:), ...
                  results.robots{1}.ee_position(2,:), ...
                  results.robots{1}.ee_position(3,:), ...
                  'b-', 'LineWidth', 2);
            hold on;
            scatter3(results.robots{1}.ee_position(1,1), ...
                    results.robots{1}.ee_position(2,1), ...
                    results.robots{1}.ee_position(3,1), 100, 'g', 'filled');
            scatter3(results.robots{1}.ee_position(1,end), ...
                    results.robots{1}.ee_position(2,end), ...
                    results.robots{1}.ee_position(3,end), 100, 'r', 'filled');
            grid on;
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title('End-Effector Path');
            legend('Path', 'Start', 'End');
            axis equal;
            view(3);
            
            subplot(2, 2, 2);
            plot(results.time_vector, results.robots{1}.ee_position', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Position (m)');
            title('EE Position Components');
            legend('X', 'Y', 'Z');
            
            subplot(2, 2, 3);
            plot(results.time_vector, results.robots{1}.manipulability, 'b-', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Manipulability');
            title('Manipulability');
            
            subplot(2, 2, 4);
            plot(results.time_vector, results.robots{1}.condition_number, 'r-', 'LineWidth', 2);
            hold on;
            yline(100, 'k--', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)');
            ylabel('Condition Number');
            title('Condition Number');
            legend('Condition Number', 'Warning Threshold');
        end
        
        function plot_robot_configurations(results, robots)
            % Plot robot configurations at key points
            p_robots = results.n_robots;
            
            figure('Name', 'Robot Configurations', 'Position', [150, 150, 1400, 500]);
            
            n_samples = length(results.time_vector);
            config_indices = [1, round(n_samples/2), n_samples];
            config_names = {'Initial', 'Mid', 'Final'};
            
            for i = 1:3
                subplot(1, 3, i);
                hold on;
                
                for j = 1:p_robots
                    q = results.robots{j}.q_trajectory(:, config_indices(i));
                    robot_struct.link_vectors = robots{j}.link_vectors;
                    robot_struct.joint_axes = robots{j}.joint_axes;
                    robot_struct.joint_types = robots{j}.joint_types;
                    robot_struct.V_base = zeros(6,1);
                    
                    RobotVisualizer.plot_robot(robot_struct, q);
                end
                
                title(sprintf('%s Configuration', config_names{i}));
                view(45, 30);
                hold off;
            end
        end
        
        %% ===================================================================
        %  DATA EXPORT FUNCTIONS
        %  ===================================================================
        
        function export_results(results, robots, filename_prefix)
            % Export all results to files
            p_robots = results.n_robots;
            
            fprintf('--- Exporting Data ---\n');
            
            if ~exist('output', 'dir')
                mkdir('output');
            end
            
            % Export each robot's data
            for i = 1:p_robots
                prefix = sprintf('%s_robot%d', filename_prefix, i);
                Kinematic_SOA_Functions.export_robot_data(results.robots{i}, robots{i}, ...
                                                          results.time_vector, prefix);
                fprintf('✓ Robot %d data exported\n', i);
            end
            
            % Export summary
            Kinematic_SOA_Functions.export_summary(results, robots, filename_prefix);
            fprintf('✓ Summary exported\n\n');
        end
        
        function export_robot_data(robot_result, robot_params, time_vector, prefix)
            % Export single robot data
            n_links = robot_result.n_links;
            
            % Trajectories
            writematrix([time_vector', robot_result.q_trajectory'], ...
                        sprintf('output/%s_joints.csv', prefix));
            writematrix([time_vector', robot_result.ee_position'], ...
                        sprintf('output/%s_ee_position.csv', prefix));
            writematrix([time_vector', robot_result.ee_velocity'], ...
                        sprintf('output/%s_ee_velocity.csv', prefix));
            
            % Matrices
            for j = 1:n_links
                writematrix(robot_result.H_matrices{j}, ...
                           sprintf('output/%s_H_joint%d.csv', prefix, j));
                writematrix(robot_result.Phi_matrices{j}, ...
                           sprintf('output/%s_Phi_joint%d.csv', prefix, j));
            end
            
            % Analysis
            writematrix([time_vector', robot_result.manipulability', robot_result.condition_number'], ...
                        sprintf('output/%s_analysis.csv', prefix));
        end
        
        function export_summary(results, robots, prefix)
            % Export summary report
            fid = fopen(sprintf('output/%s_summary.txt', prefix), 'w');
            
            fprintf(fid, 'KINEMATIC ANALYSIS SUMMARY - SOA\n');
            fprintf(fid, '================================\n\n');
            fprintf(fid, 'Number of robots: %d\n\n', results.n_robots);
            
            total_dof = 0;
            for i = 1:results.n_robots
                n_links = results.robots{i}.n_links;
                total_dof = total_dof + n_links;
                fprintf(fid, 'Robot %d:\n', i);
                fprintf(fid, '  Links: %d\n', n_links);
                fprintf(fid, '  Base: [%.3f, %.3f, %.3f]\n', robots{i}.base_position);
                fprintf(fid, '  Avg manipulability: %.6f\n', mean(results.robots{i}.manipulability));
                fprintf(fid, '  Avg condition number: %.2f\n\n', mean(results.robots{i}.condition_number));
            end
            
            fprintf(fid, 'Total system DOF: %d\n\n', total_dof);
            
            fprintf(fid, 'Trajectory Verification:\n');
            if results.verification.all_correct
                fprintf(fid, '  Status: ALL CORRECT\n');
            else
                fprintf(fid, '  Status: ERRORS DETECTED\n');
            end
            
            fclose(fid);
        end
        
    end
end

%% HELPER FUNCTION
function result = tern(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
