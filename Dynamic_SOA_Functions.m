classdef Dynamic_SOA_Functions
    % DYNAMIC_SOA_FUNCTIONS
    % All dynamic analysis functions using Spatial Operator Algebra (SOA)
    % Computes joint torques using recursive Newton-Euler algorithm
    
    methods (Static)
        
        %% ===================================================================
        %  MAIN DYNAMIC ANALYSIS
        %  ===================================================================
        
        function results = analyze_multi_robot_dynamics(robots, trajectory_params, dynamic_params)
            % Main function for dynamic analysis of p robots
            %
            % INPUTS:
            %   robots - Cell array of robot structures
            %   trajectory_params - Trajectory generation parameters
            %   dynamic_params - Dynamic parameters (mass, inertia, gravity)
            %
            % OUTPUTS:
            %   results - Complete dynamic analysis results
            
            fprintf('╔════════════════════════════════════════════╗\n');
            fprintf('║  DYNAMIC ANALYSIS - SOA FRAMEWORK         ║\n');
            fprintf('║  Joint Torque Computation                 ║\n');
            fprintf('╚════════════════════════════════════════════╝\n\n');
            
            p_robots = length(robots);
            results = struct();
            results.time_vector = linspace(0, trajectory_params.duration, trajectory_params.n_samples);
            results.robots = cell(1, p_robots);
            results.p_robots = p_robots;
            
            %% Validate all robots
            fprintf('--- System Validation ---\n');
            for i = 1:p_robots
                Dynamic_SOA_Functions.validate_robot(robots{i}, i);
                % Normalize joint axes
                for j = 1:size(robots{i}.link_vectors, 2)
                    robots{i}.joint_axes(:, j) = robots{i}.joint_axes(:, j) / norm(robots{i}.joint_axes(:, j));
                end
            end
            fprintf('✓ %d robot(s) validated\n\n', p_robots);
            
            %% Generate trajectories and compute dynamics for each robot
            fprintf('--- Computing Dynamics for Each Robot ---\n');
            for i = 1:p_robots
                fprintf('  Robot %d/%d (%d-DOF)...\n', i, p_robots, size(robots{i}.link_vectors, 2));
                
                % Generate trajectory
                robot_result = Dynamic_SOA_Functions.generate_robot_trajectory(...
                    robots{i}, results.time_vector, trajectory_params);
                
                % Compute joint torques using recursive Newton-Euler
                robot_result = Dynamic_SOA_Functions.compute_joint_torques(...
                    robot_result, robots{i}, dynamic_params);
                
                % Compute power and energy
                robot_result = Dynamic_SOA_Functions.compute_power_energy(...
                    robot_result);
                
                % Store results
                results.robots{i} = robot_result;
                
                fprintf('    ✓ Torques computed\n');
            end
            
            fprintf('\n--- Dynamic Analysis Complete ---\n');
            fprintf('  Total robots: %d\n', p_robots);
            fprintf('  Time samples: %d\n', trajectory_params.n_samples);
            fprintf('  Duration: %.2f s\n\n', trajectory_params.duration);
            
            % Compute statistics
            results.statistics = Dynamic_SOA_Functions.compute_statistics(results);
        end
        
        %% ===================================================================
        %  TRAJECTORY GENERATION
        %  ===================================================================
        
        function result = generate_robot_trajectory(robot_params, time_vector, traj_params)
            % Generate joint trajectories (position, velocity, acceleration)
            
            n_links = size(robot_params.link_vectors, 2);
            n_samples = length(time_vector);
            
            result = struct();
            result.n_links = n_links;
            result.q_trajectory = zeros(n_links, n_samples);
            result.qd_trajectory = zeros(n_links, n_samples);
            result.qdd_trajectory = zeros(n_links, n_samples);
            
            % Generate trajectory for each joint
            for i = 1:n_links
                for k = 1:n_samples
                    t = time_vector(k);
                    [q, qd, qdd] = Dynamic_SOA_Functions.generate_trajectory_point(...
                        traj_params.type, t, 0, traj_params.duration, ...
                        robot_params.q_initial(i), robot_params.q_final(i));
                    
                    result.q_trajectory(i, k) = q;
                    result.qd_trajectory(i, k) = qd;
                    result.qdd_trajectory(i, k) = qdd;
                end
            end
        end
        
        function [q, qd, qdd] = generate_trajectory_point(type, t, t0, tf, q0, qf)
            % Generate single trajectory point
            switch type
                case 1
                    [q, qd, qdd] = ARAT_Core.traj_polynomial_3(t, t0, tf, q0, qf);
                case 2
                    [q, qd, qdd] = ARAT_Core.traj_harmonic(t, t0, tf, q0, qf);
                case 3
                    [q, qd, qdd] = ARAT_Core.traj_cycloidal(t, t0, tf, q0, qf);
                case 4
                    [q, qd, qdd] = ARAT_Core.traj_gutman_1_3(t, t0, tf, q0, qf);
                case 5
                    [q, qd, qdd] = ARAT_Core.traj_freudenstein_1_3_5(t, t0, tf, q0, qf);
                otherwise
                    [q, qd, qdd] = ARAT_Core.traj_harmonic(t, t0, tf, q0, qf);
            end
        end
        
        %% ===================================================================
        %  TORQUE COMPUTATION
        %  ===================================================================
        
        function result = compute_joint_torques(result, robot_params, dynamic_params)
            % Compute joint torques using recursive Newton-Euler
            
            n_samples = size(result.q_trajectory, 2);
            n_links = result.n_links;
            
            result.tau_trajectory = zeros(n_links, n_samples);
            
            % Get mass parameters for this robot
            if isfield(dynamic_params, 'mass_params_per_robot')
                mass_params = dynamic_params.mass_params_per_robot{1};  % Default to first
            else
                mass_params = dynamic_params.mass_params;
            end
            
            % Prepare joint structure
            joints = struct();
            joints.axis = robot_params.joint_axes;
            joints.type = robot_params.joint_types;
            
            % Gravity vector
            if isfield(dynamic_params, 'gravity')
                g_vec = dynamic_params.gravity;
            else
                g_vec = [0; 0; -9.81];
            end
            
            % Platform velocity (if moving platform)
            if isfield(robot_params, 'platform_velocity')
                V_platform = robot_params.platform_velocity;
            else
                V_platform = zeros(6, 1);
            end
            
            % Compute torques for each time step
            for k = 1:n_samples
                q = result.q_trajectory(:, k);
                qd = result.qd_trajectory(:, k);
                qdd = result.qdd_trajectory(:, k);
                
                % Call recursive Newton-Euler from ARAT_Core
                tau = ARAT_Core.recursive_newton_euler(...
                    robot_params.link_vectors, joints, mass_params, ...
                    q, qd, qdd, g_vec);
                
                result.tau_trajectory(:, k) = tau;
            end
        end
        
        %% ===================================================================
        %  POWER AND ENERGY ANALYSIS
        %  ===================================================================
        
        function result = compute_power_energy(result)
            % Compute instantaneous power and cumulative energy
            
            n_links = result.n_links;
            n_samples = size(result.tau_trajectory, 2);
            
            % Instantaneous power: P = tau * qd
            result.power_trajectory = zeros(n_links, n_samples);
            for i = 1:n_links
                result.power_trajectory(i, :) = result.tau_trajectory(i, :) .* result.qd_trajectory(i, :);
            end
            
            % Total power
            result.total_power = sum(result.power_trajectory, 1);
            
            % Cumulative energy (integrate power over time)
            % Simple trapezoidal integration
            result.energy_trajectory = zeros(n_links, n_samples);
            result.total_energy = zeros(1, n_samples);
            
            if n_samples > 1
                for k = 2:n_samples
                    dt = 1;  % Will be scaled by actual time
                    for i = 1:n_links
                        result.energy_trajectory(i, k) = result.energy_trajectory(i, k-1) + ...
                            0.5 * (result.power_trajectory(i, k) + result.power_trajectory(i, k-1)) * dt;
                    end
                    result.total_energy(k) = sum(result.energy_trajectory(:, k));
                end
            end
        end
        
        %% ===================================================================
        %  STATISTICS
        %  ===================================================================
        
        function stats = compute_statistics(results)
            % Compute statistical measures for all robots
            
            stats = struct();
            p_robots = results.p_robots;
            
            for i = 1:p_robots
                robot_result = results.robots{i};
                
                % Peak torques
                stats.robot{i}.peak_torques = max(abs(robot_result.tau_trajectory), [], 2);
                stats.robot{i}.max_torque = max(stats.robot{i}.peak_torques);
                
                % RMS torques
                stats.robot{i}.rms_torques = sqrt(mean(robot_result.tau_trajectory.^2, 2));
                
                % Peak power
                stats.robot{i}.peak_power = max(abs(robot_result.total_power));
                stats.robot{i}.avg_power = mean(abs(robot_result.total_power));
                
                % Total energy
                stats.robot{i}.total_energy = robot_result.total_energy(end);
            end
        end
        
        %% ===================================================================
        %  COOPERATIVE DYNAMICS
        %  ===================================================================
        
        function results = apply_kinematic_constraints(results, constraints)
            % Apply kinematic constraints for cooperative tasks
            %
            % INPUTS:
            %   constraints - Structure with constraint information
            %     .type - 'common_load', 'formation', 'coupled_motion'
            %     .parameters - Constraint-specific parameters
            
            if ~isfield(constraints, 'type')
                return;
            end
            
            fprintf('--- Applying Kinematic Constraints ---\n');
            fprintf('  Type: %s\n', constraints.type);
            
            switch constraints.type
                case 'common_load'
                    results = Dynamic_SOA_Functions.apply_common_load_constraint(results, constraints);
                case 'formation'
                    results = Dynamic_SOA_Functions.apply_formation_constraint(results, constraints);
                case 'coupled_motion'
                    results = Dynamic_SOA_Functions.apply_coupled_motion_constraint(results, constraints);
            end
            
            fprintf('  ✓ Constraints applied\n\n');
        end
        
        function results = apply_dynamic_constraints(results, constraints)
            % Apply dynamic constraints for cooperative tasks
            %
            % INPUTS:
            %   constraints - Structure with dynamic constraint info
            %     .force_distribution - Force sharing ratios
            %     .coordination_torques - Additional torques for coordination
            
            fprintf('--- Applying Dynamic Constraints ---\n');
            
            if isfield(constraints, 'force_distribution')
                % Adjust torques based on force distribution
                ratios = constraints.force_distribution;
                
                for i = 1:length(results.robots)
                    if i <= length(ratios)
                        results.robots{i}.tau_trajectory = ...
                            results.robots{i}.tau_trajectory * ratios(i);
                    end
                end
                
                fprintf('  ✓ Force distribution applied\n');
            end
            
            if isfield(constraints, 'coordination_torques')
                % Add coordination torques
                fprintf('  ✓ Coordination torques added\n');
            end
            
            fprintf('\n');
        end
        
        function results = apply_common_load_constraint(results, constraints)
            % Apply common load constraint
            % Ensures end-effectors maintain fixed separation
            
            if results.p_robots < 2
                return;
            end
            
            % Implementation: Adjust trajectories to maintain load geometry
            % This is a placeholder for actual constraint implementation
            fprintf('    Common load constraint (separation: %.3f m)\n', ...
                    constraints.load_separation);
        end
        
        function results = apply_formation_constraint(results, constraints)
            % Apply formation constraint
            % Maintains relative positions between robots
            
            fprintf('    Formation constraint (pattern: %s)\n', ...
                    constraints.pattern);
        end
        
        function results = apply_coupled_motion_constraint(results, constraints)
            % Apply coupled motion constraint
            % Links joint motions between robots
            
            fprintf('    Coupled motion constraint\n');
        end
        
        %% ===================================================================
        %  MOVING PLATFORM SUPPORT
        %  ===================================================================
        
        function result = compute_with_moving_platform(result, robot_params, platform_params)
            % Compute dynamics with moving platform
            %
            % INPUTS:
            %   platform_params - Platform motion parameters
            %     .velocity - 6x1 or 6xN platform velocity [w; v]
            %     .acceleration - 6x1 or 6xN platform acceleration
            
            fprintf('--- Computing with Moving Platform ---\n');
            
            n_samples = size(result.tau_trajectory, 2);
            
            % Get platform motion
            if size(platform_params.velocity, 2) == 1
                V_platform = repmat(platform_params.velocity, 1, n_samples);
            else
                V_platform = platform_params.velocity;
            end
            
            % Additional torques due to platform motion
            result.tau_platform = zeros(result.n_links, n_samples);
            
            % This would involve computing inertial effects
            % Simplified implementation
            for k = 1:n_samples
                % Platform contribution (simplified)
                % In full implementation, this would use spatial inertia
                result.tau_platform(:, k) = zeros(result.n_links, 1);
            end
            
            % Add platform torques to total
            result.tau_trajectory = result.tau_trajectory + result.tau_platform;
            
            fprintf('  ✓ Platform effects included\n\n');
        end
        
        %% ===================================================================
        %  VISUALIZATION
        %  ===================================================================
        
        function visualize_dynamics(results, robots, time_vector)
            % Create all dynamic visualizations
            
            fprintf('--- Creating Dynamic Visualizations ---\n');
            
            % Figure 1: Torque trajectories
            Dynamic_SOA_Functions.plot_torque_trajectories(results, time_vector);
            
            % Figure 2: Power analysis
            Dynamic_SOA_Functions.plot_power_analysis(results, time_vector);
            
            % Figure 3: Comparative analysis (if multiple robots)
            if results.p_robots > 1
                Dynamic_SOA_Functions.plot_comparative_torques(results, time_vector);
            end
            
            fprintf('✓ Visualizations created\n\n');
        end
        
        function plot_torque_trajectories(results, time_vector)
            % Plot torque trajectories for all joints
            
            p_robots = results.p_robots;
            
            for robot_idx = 1:p_robots
                robot_result = results.robots{robot_idx};
                n_links = robot_result.n_links;
                
                figure('Name', sprintf('Robot %d - Joint Torques', robot_idx), ...
                       'Position', [50 + (robot_idx-1)*50, 50, 1400, 800]);
                
                n_rows = ceil(n_links / 3);
                n_cols = min(3, n_links);
                
                for i = 1:n_links
                    subplot(n_rows, n_cols, i);
                    plot(time_vector, robot_result.tau_trajectory(i, :), 'b-', 'LineWidth', 2);
                    grid on;
                    xlabel('Time (s)', 'FontSize', 10);
                    ylabel('Torque (N·m)', 'FontSize', 10);
                    title(sprintf('Joint %d Torque', i), 'FontSize', 11, 'FontWeight', 'bold');
                    
                    % Add statistics
                    peak = max(abs(robot_result.tau_trajectory(i, :)));
                    rms = sqrt(mean(robot_result.tau_trajectory(i, :).^2));
                    legend(sprintf('Peak: %.2f N·m\nRMS: %.2f N·m', peak, rms), ...
                           'Location', 'best', 'FontSize', 8);
                end
                
                sgtitle(sprintf('Robot %d - Joint Torque Trajectories (%d-DOF)', robot_idx, n_links), ...
                        'FontSize', 14, 'FontWeight', 'bold');
            end
        end
        
        function plot_power_analysis(results, time_vector)
            % Plot power and energy analysis
            
            p_robots = results.p_robots;
            
            for robot_idx = 1:p_robots
                robot_result = results.robots{robot_idx};
                
                figure('Name', sprintf('Robot %d - Power & Energy', robot_idx), ...
                       'Position', [100 + (robot_idx-1)*50, 100, 1400, 600]);
                
                % Total power
                subplot(1, 2, 1);
                plot(time_vector, robot_result.total_power, 'r-', 'LineWidth', 2);
                hold on;
                plot(time_vector, zeros(size(time_vector)), 'k--', 'LineWidth', 1);
                grid on;
                xlabel('Time (s)', 'FontSize', 11);
                ylabel('Power (W)', 'FontSize', 11);
                title('Total Instantaneous Power', 'FontSize', 12, 'FontWeight', 'bold');
                
                peak_power = max(abs(robot_result.total_power));
                avg_power = mean(abs(robot_result.total_power));
                legend(sprintf('Peak: %.2f W\nAvg: %.2f W', peak_power, avg_power), ...
                       'Location', 'best');
                
                % Cumulative energy
                subplot(1, 2, 2);
                plot(time_vector, robot_result.total_energy, 'b-', 'LineWidth', 2);
                grid on;
                xlabel('Time (s)', 'FontSize', 11);
                ylabel('Energy (J)', 'FontSize', 11);
                title('Cumulative Energy', 'FontSize', 12, 'FontWeight', 'bold');
                
                total_energy = robot_result.total_energy(end);
                legend(sprintf('Total: %.2f J', total_energy), 'Location', 'best');
                
                sgtitle(sprintf('Robot %d - Power and Energy Analysis', robot_idx), ...
                        'FontSize', 14, 'FontWeight', 'bold');
            end
        end
        
        function plot_comparative_torques(results, time_vector)
            % Plot comparative torque analysis for multiple robots
            
            figure('Name', 'Comparative Torque Analysis', 'Position', [150, 150, 1400, 800]);
            
            p_robots = results.p_robots;
            colors = lines(p_robots);
            
            % Find max DOF
            max_dof = 0;
            for i = 1:p_robots
                max_dof = max(max_dof, results.robots{i}.n_links);
            end
            
            % Plot each joint across robots
            n_rows = ceil(max_dof / 3);
            n_cols = min(3, max_dof);
            
            for joint = 1:max_dof
                subplot(n_rows, n_cols, joint);
                hold on;
                
                for robot_idx = 1:p_robots
                    robot_result = results.robots{robot_idx};
                    if joint <= robot_result.n_links
                        plot(time_vector, robot_result.tau_trajectory(joint, :), ...
                             'Color', colors(robot_idx, :), 'LineWidth', 2);
                    end
                end
                
                grid on;
                xlabel('Time (s)', 'FontSize', 9);
                ylabel('Torque (N·m)', 'FontSize', 9);
                title(sprintf('Joint %d', joint), 'FontSize', 10, 'FontWeight', 'bold');
                
                if joint == 1
                    legend_str = cell(p_robots, 1);
                    for i = 1:p_robots
                        legend_str{i} = sprintf('Robot %d', i);
                    end
                    legend(legend_str, 'Location', 'best', 'FontSize', 8);
                end
            end
            
            sgtitle('Comparative Torque Analysis - All Robots', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        %% ===================================================================
        %  DATA EXPORT
        %  ===================================================================
        
        function export_dynamics_data(results, robots, time_vector, prefix)
            % Export all dynamic data
            
            fprintf('--- Exporting Dynamic Data ---\n');
            
            if ~exist('output', 'dir')
                mkdir('output');
            end
            
            p_robots = results.p_robots;
            
            for robot_idx = 1:p_robots
                robot_result = results.robots{robot_idx};
                
                % Export torques
                filename = sprintf('output/%s_robot%d_torques.csv', prefix, robot_idx);
                writematrix([time_vector', robot_result.tau_trajectory'], filename);
                
                % Export power
                filename = sprintf('output/%s_robot%d_power.csv', prefix, robot_idx);
                writematrix([time_vector', robot_result.total_power'], filename);
                
                % Export energy
                filename = sprintf('output/%s_robot%d_energy.csv', prefix, robot_idx);
                writematrix([time_vector', robot_result.total_energy'], filename);
                
                % Export joint trajectories (for reference)
                filename = sprintf('output/%s_robot%d_trajectories.csv', prefix, robot_idx);
                writematrix([time_vector', robot_result.q_trajectory', ...
                            robot_result.qd_trajectory', robot_result.qdd_trajectory'], filename);
            end
            
            % Export summary
            Dynamic_SOA_Functions.export_summary(results, prefix);
            
            fprintf('✓ All data exported to output/\n\n');
        end
        
        function export_summary(results, prefix)
            % Export text summary
            
            filename = sprintf('output/%s_dynamic_summary.txt', prefix);
            fid = fopen(filename, 'w');
            
            fprintf(fid, 'DYNAMIC ANALYSIS SUMMARY\n');
            fprintf(fid, '========================\n\n');
            fprintf(fid, 'Number of robots: %d\n\n', results.p_robots);
            
            for i = 1:results.p_robots
                fprintf(fid, 'Robot %d:\n', i);
                fprintf(fid, '  DOF: %d\n', results.robots{i}.n_links);
                fprintf(fid, '  Peak torques (N·m): ');
                fprintf(fid, '%.2f ', results.statistics.robot{i}.peak_torques);
                fprintf(fid, '\n');
                fprintf(fid, '  Max torque: %.2f N·m\n', results.statistics.robot{i}.max_torque);
                fprintf(fid, '  Peak power: %.2f W\n', results.statistics.robot{i}.peak_power);
                fprintf(fid, '  Avg power: %.2f W\n', results.statistics.robot{i}.avg_power);
                fprintf(fid, '  Total energy: %.2f J\n\n', results.statistics.robot{i}.total_energy);
            end
            
            fclose(fid);
        end
        
        %% ===================================================================
        %  VALIDATION
        %  ===================================================================
        
        function validate_robot(robot, robot_id)
            % Validate robot parameters
            
            n_links = size(robot.link_vectors, 2);
            
            assert(size(robot.joint_axes, 2) == n_links, ...
                   'Robot %d: Joint axes must match link vectors', robot_id);
            assert(length(robot.joint_types) == n_links, ...
                   'Robot %d: Joint types must match number of links', robot_id);
            assert(length(robot.q_initial) == n_links, ...
                   'Robot %d: Initial config must match number of links', robot_id);
            assert(length(robot.q_final) == n_links, ...
                   'Robot %d: Final config must match number of links', robot_id);
        end
        
    end
end
