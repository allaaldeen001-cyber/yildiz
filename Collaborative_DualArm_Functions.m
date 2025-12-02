classdef Collaborative_DualArm_Functions
    % COLLABORATIVE_DUALARM_FUNCTIONS
    % All functions for two robots working together with a common load
    % Implements coordinated manipulation using SOA framework
    
    methods (Static)
        
        %% ===================================================================
        %  MAIN COLLABORATIVE ANALYSIS
        %  ===================================================================
        
        function results = analyze_multi_robot_system(robot1, robot2, load_params, task_params)
            % Main function for collaborative dual-arm system
            %
            % INPUTS:
            %   robot1 - First robot structure
            %   robot2 - Second robot structure
            %   load_params - Common load parameters
            %   task_params - Task and trajectory parameters
            %
            % OUTPUTS:
            %   results - Complete analysis results
            
            fprintf('╔════════════════════════════════════════════╗\n');
            fprintf('║  COLLABORATIVE DUAL-ARM SYSTEM ANALYSIS    ║\n');
            fprintf('║  Two Robots Working with Common Load       ║\n');
            fprintf('╔════════════════════════════════════════════╗\n\n');
            
            results = struct();
            results.time_vector = linspace(0, task_params.duration, task_params.n_samples);
            
            %% Validate both robots
            fprintf('--- System Validation ---\n');
            Collaborative_DualArm_Functions.validate_robot(robot1, 1);
            Collaborative_DualArm_Functions.validate_robot(robot2, 2);
            
            % Normalize joint axes
            for i = 1:size(robot1.link_vectors, 2)
                robot1.joint_axes(:, i) = robot1.joint_axes(:, i) / norm(robot1.joint_axes(:, i));
            end
            for i = 1:size(robot2.link_vectors, 2)
                robot2.joint_axes(:, i) = robot2.joint_axes(:, i) / norm(robot2.joint_axes(:, i));
            end
            
            fprintf('✓ Both robots validated\n');
            fprintf('  Robot 1: %d-DOF at [%.2f, %.2f, %.2f]\n', ...
                    size(robot1.link_vectors, 2), robot1.base_position);
            fprintf('  Robot 2: %d-DOF at [%.2f, %.2f, %.2f]\n', ...
                    size(robot2.link_vectors, 2), robot2.base_position);
            fprintf('  Common load: %.1f kg, [%.2f × %.2f × %.2f] m\n\n', ...
                    load_params.mass, load_params.size);
            
            %% Generate coordinated waypoints
            fprintf('--- Planning Coordinated Motion ---\n');
            [waypoints1, waypoints2, grasp_phases] = ...
                Collaborative_DualArm_Functions.plan_coordinated_task(...
                    robot1, robot2, task_params);
            
            n_waypoints = size(waypoints1, 2);
            fprintf('✓ %d waypoints planned\n', n_waypoints);
            fprintf('  Task: %s\n\n', task_params.task_type);
            
            %% Generate synchronized trajectories
            fprintf('--- Generating Synchronized Trajectories ---\n');
            [results.robot1, results.robot2, results.grasp_states] = ...
                Collaborative_DualArm_Functions.generate_synchronized_trajectories(...
                    robot1, robot2, waypoints1, waypoints2, grasp_phases, ...
                    results.time_vector, task_params);
            
            fprintf('✓ Synchronized trajectories generated\n\n');
            
            %% Forward kinematics for both robots
            fprintf('--- Computing Forward Kinematics ---\n');
            results.robot1 = Collaborative_DualArm_Functions.compute_fk(results.robot1, robot1);
            results.robot2 = Collaborative_DualArm_Functions.compute_fk(results.robot2, robot2);
            fprintf('✓ FK computed for both robots\n\n');
            
            %% Compute load trajectory
            fprintf('--- Computing Load Trajectory ---\n');
            results.load = Collaborative_DualArm_Functions.compute_load_trajectory(...
                results.robot1, results.robot2, results.grasp_states, load_params);
            fprintf('✓ Load trajectory computed\n\n');
            
            %% Jacobian analysis
            fprintf('--- Jacobian Analysis ---\n');
            results.robot1 = Collaborative_DualArm_Functions.compute_jacobian_analysis(...
                results.robot1, robot1);
            results.robot2 = Collaborative_DualArm_Functions.compute_jacobian_analysis(...
                results.robot2, robot2);
            fprintf('✓ Jacobian analysis complete\n\n');
            
            %% Compute system matrices
            fprintf('--- Computing System Matrices ---\n');
            results.robot1 = Collaborative_DualArm_Functions.compute_matrices(results.robot1, robot1);
            results.robot2 = Collaborative_DualArm_Functions.compute_matrices(results.robot2, robot2);
            fprintf('✓ H and Phi matrices computed\n\n');
            
            %% Coordination analysis
            fprintf('--- Coordination Analysis ---\n');
            results.coordination = Collaborative_DualArm_Functions.analyze_coordination(...
                results.robot1, results.robot2, results.grasp_states, load_params);
            fprintf('✓ Coordination metrics computed\n\n');
            
            %% Verify cooperative motion
            fprintf('--- Verifying Cooperative Motion ---\n');
            results.verification = Collaborative_DualArm_Functions.verify_cooperative_motion(...
                results, load_params);
            fprintf('\n');
            
            fprintf('╔════════════════════════════════════════════╗\n');
            fprintf('║  COLLABORATIVE ANALYSIS COMPLETE           ║\n');
            fprintf('╚════════════════════════════════════════════╝\n\n');
        end
        
        %% ===================================================================
        %  TASK PLANNING
        %  ===================================================================
        
        function [waypoints1, waypoints2, grasp_phases] = plan_coordinated_task(robot1, robot2, task_params)
            % Plan coordinated waypoints based on task type
            
            n1 = size(robot1.link_vectors, 2);
            n2 = size(robot2.link_vectors, 2);
            
            switch task_params.task_type
                case 'pick_and_place'
                    [waypoints1, waypoints2, grasp_phases] = ...
                        Collaborative_DualArm_Functions.plan_pick_and_place(robot1, robot2, task_params);
                    
                case 'transport'
                    [waypoints1, waypoints2, grasp_phases] = ...
                        Collaborative_DualArm_Functions.plan_transport(robot1, robot2, task_params);
                    
                case 'assembly'
                    [waypoints1, waypoints2, grasp_phases] = ...
                        Collaborative_DualArm_Functions.plan_assembly(robot1, robot2, task_params);
                    
                otherwise
                    % Default: simple coordinated motion
                    waypoints1 = [robot1.q_initial, robot1.q_final];
                    waypoints2 = [robot2.q_initial, robot2.q_final];
                    grasp_phases = [false, true];
            end
        end
        
        function [wp1, wp2, grasp] = plan_pick_and_place(robot1, robot2, task_params)
            % Plan pick-and-place operation
            n1 = size(robot1.link_vectors, 2);
            n2 = size(robot2.link_vectors, 2);
            
            % 8-phase pick and place
            wp1 = zeros(n1, 8);
            wp2 = zeros(n2, 8);
            grasp = false(1, 8);
            
            % Phase 1: Home position
            wp1(:, 1) = robot1.q_initial;
            wp2(:, 1) = robot2.q_initial;
            grasp(1) = false;
            
            % Phase 2: Approach load
            wp1(:, 2) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * 0.3;
            wp2(:, 2) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * 0.3;
            grasp(2) = false;
            
            % Phase 3: Grasp load
            wp1(:, 3) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * 0.5;
            wp2(:, 3) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * 0.5;
            grasp(3) = true;
            
            % Phase 4: Lift
            wp1(:, 4) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * 0.6;
            wp2(:, 4) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * 0.6;
            grasp(4) = true;
            
            % Phase 5: Transport
            wp1(:, 5) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * 0.8;
            wp2(:, 5) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * 0.8;
            grasp(5) = true;
            
            % Phase 6: Place
            wp1(:, 6) = robot1.q_final;
            wp2(:, 6) = robot2.q_final;
            grasp(6) = true;
            
            % Phase 7: Release
            wp1(:, 7) = robot1.q_final * 0.8;
            wp2(:, 7) = robot2.q_final * 0.8;
            grasp(7) = false;
            
            % Phase 8: Return home
            wp1(:, 8) = robot1.q_initial;
            wp2(:, 8) = robot2.q_initial;
            grasp(8) = false;
        end
        
        function [wp1, wp2, grasp] = plan_transport(robot1, robot2, task_params)
            % Plan transport operation
            n1 = size(robot1.link_vectors, 2);
            n2 = size(robot2.link_vectors, 2);
            
            % 6-phase transport
            wp1 = zeros(n1, 6);
            wp2 = zeros(n2, 6);
            grasp = [false, true, true, true, true, false];
            
            for i = 1:6
                t = (i-1) / 5;
                wp1(:, i) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * t;
                wp2(:, i) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * t;
            end
            grasp(2:5) = true;
        end
        
        function [wp1, wp2, grasp] = plan_assembly(robot1, robot2, task_params)
            % Plan assembly operation
            n1 = size(robot1.link_vectors, 2);
            n2 = size(robot2.link_vectors, 2);
            
            % 5-phase assembly
            wp1 = zeros(n1, 5);
            wp2 = zeros(n2, 5);
            grasp = [false, true, true, true, false];
            
            for i = 1:5
                t = (i-1) / 4;
                wp1(:, i) = robot1.q_initial + (robot1.q_final - robot1.q_initial) * t;
                wp2(:, i) = robot2.q_initial + (robot2.q_final - robot2.q_initial) * t;
            end
        end
        
        %% ===================================================================
        %  TRAJECTORY GENERATION
        %  ===================================================================
        
        function [result1, result2, grasp_states] = generate_synchronized_trajectories(...
                robot1, robot2, waypoints1, waypoints2, grasp_phases, time_vector, task_params)
            % Generate synchronized trajectories for both robots
            
            n1 = size(robot1.link_vectors, 2);
            n2 = size(robot2.link_vectors, 2);
            n_samples = length(time_vector);
            n_waypoints = size(waypoints1, 2);
            
            result1 = struct();
            result1.n_links = n1;
            result1.q_trajectory = zeros(n1, n_samples);
            result1.qd_trajectory = zeros(n1, n_samples);
            
            result2 = struct();
            result2.n_links = n2;
            result2.q_trajectory = zeros(n2, n_samples);
            result2.qd_trajectory = zeros(n2, n_samples);
            
            grasp_states = false(1, n_samples);
            
            % Generate trajectories segment by segment
            segment_duration = task_params.duration / (n_waypoints - 1);
            n_samples_per_segment = floor(n_samples / (n_waypoints - 1));
            
            idx = 1;
            for seg = 1:(n_waypoints - 1)
                q_start_1 = waypoints1(:, seg);
                q_end_1 = waypoints1(:, seg + 1);
                q_start_2 = waypoints2(:, seg);
                q_end_2 = waypoints2(:, seg + 1);
                
                t_start = (seg - 1) * segment_duration;
                t_end = seg * segment_duration;
                
                for i = 1:n_samples_per_segment
                    if idx > n_samples, break; end
                    
                    t = time_vector(idx);
                    
                    % Robot 1
                    for j = 1:n1
                        [q, qd, ~] = Collaborative_DualArm_Functions.trajectory_point(...
                            task_params.trajectory_type, t, t_start, t_end, ...
                            q_start_1(j), q_end_1(j));
                        result1.q_trajectory(j, idx) = q;
                        result1.qd_trajectory(j, idx) = qd;
                    end
                    
                    % Robot 2 (synchronized)
                    for j = 1:n2
                        [q, qd, ~] = Collaborative_DualArm_Functions.trajectory_point(...
                            task_params.trajectory_type, t, t_start, t_end, ...
                            q_start_2(j), q_end_2(j));
                        result2.q_trajectory(j, idx) = q;
                        result2.qd_trajectory(j, idx) = qd;
                    end
                    
                    % Grasp state (interpolate between waypoints)
                    grasp_states(idx) = grasp_phases(seg) || grasp_phases(seg + 1);
                    
                    idx = idx + 1;
                end
            end
        end
        
        function [q, qd, qdd] = trajectory_point(type, t, t0, tf, q0, qf)
            % Generate single trajectory point
            switch type
                case 1
                    [q, qd, qdd] = ARAT_Core.traj_polynomial_3(t, t0, tf, q0, qf);
                case 2
                    [q, qd, qdd] = ARAT_Core.traj_harmonic(t, t0, tf, q0, qf);
                case 3
                    [q, qd, qdd] = ARAT_Core.traj_cycloidal(t, t0, tf, q0, qf);
                otherwise
                    [q, qd, qdd] = ARAT_Core.traj_harmonic(t, t0, tf, q0, qf);
            end
        end
        
        %% ===================================================================
        %  KINEMATICS
        %  ===================================================================
        
        function result = compute_fk(result, robot_params)
            % Compute forward kinematics
            n_samples = size(result.q_trajectory, 2);
            
            result.ee_position = zeros(3, n_samples);
            result.ee_velocity = zeros(6, n_samples);
            
            robot_struct = struct('link_vectors', robot_params.link_vectors, ...
                                 'joint_axes', robot_params.joint_axes, ...
                                 'joint_types', robot_params.joint_types);
            
            for k = 1:n_samples
                pose = RobotIK.forward_kinematics_pose(robot_struct, result.q_trajectory(:, k));
                result.ee_position(:, k) = pose(4:6) + robot_params.base_position;
                
                V = ARAT_Core.serial_forward_kinematics(robot_params.link_vectors, ...
                                                        robot_params.joint_axes, ...
                                                        robot_params.joint_types, ...
                                                        result.q_trajectory(:, k), ...
                                                        result.qd_trajectory(:, k), ...
                                                        robot_params.V_base);
                result.ee_velocity(:, k) = V;
            end
        end
        
        function result = compute_jacobian_analysis(result, robot_params)
            % Compute Jacobian analysis
            n_samples = size(result.q_trajectory, 2);
            
            result.manipulability = zeros(1, n_samples);
            result.condition_number = zeros(1, n_samples);
            
            for k = 1:n_samples
                J = ARAT_Core.compute_jacobian(robot_params.link_vectors, ...
                                                robot_params.joint_axes, ...
                                                robot_params.joint_types, ...
                                                result.q_trajectory(:, k));
                result.manipulability(k) = sqrt(det(J * J'));
                result.condition_number(k) = cond(J);
            end
        end
        
        function result = compute_matrices(result, robot_params)
            % Compute H and Phi matrices
            n_links = result.n_links;
            
            result.H_matrices = cell(1, n_links);
            result.Phi_matrices = cell(1, n_links);
            
            for i = 1:n_links
                result.H_matrices{i} = ARAT_Core.get_joint_matrix(...
                    robot_params.joint_types(i), robot_params.joint_axes(:, i));
                result.Phi_matrices{i} = ARAT_Core.get_prop_matrix(...
                    robot_params.link_vectors(:, i));
            end
        end
        
        %% ===================================================================
        %  LOAD TRAJECTORY
        %  ===================================================================
        
        function load_result = compute_load_trajectory(robot1_result, robot2_result, grasp_states, load_params)
            % Compute load position and forces
            n_samples = length(grasp_states);
            
            load_result = struct();
            load_result.position = zeros(3, n_samples);
            load_result.velocity = zeros(3, n_samples);
            load_result.force_robot1 = zeros(3, n_samples);
            load_result.force_robot2 = zeros(3, n_samples);
            
            initial_load_pos = (robot1_result.ee_position(:, 1) + robot2_result.ee_position(:, 1)) / 2;
            
            for k = 1:n_samples
                if grasp_states(k)
                    % Load is grasped - position is center between EEs
                    load_result.position(:, k) = (robot1_result.ee_position(:, k) + ...
                                                   robot2_result.ee_position(:, k)) / 2;
                    
                    % Load velocity
                    load_result.velocity(:, k) = (robot1_result.ee_velocity(4:6, k) + ...
                                                   robot2_result.ee_velocity(4:6, k)) / 2;
                    
                    % Force distribution (equal sharing + gravity)
                    g_force = [0; 0; -load_params.mass * 9.81];
                    load_result.force_robot1(:, k) = g_force / 2;
                    load_result.force_robot2(:, k) = g_force / 2;
                else
                    % Load not grasped - stays at last known position
                    if k > 1 && any(grasp_states(1:k-1))
                        last_grasp = find(grasp_states(1:k-1), 1, 'last');
                        load_result.position(:, k) = load_result.position(:, last_grasp);
                    else
                        load_result.position(:, k) = initial_load_pos;
                    end
                    load_result.velocity(:, k) = [0; 0; 0];
                    load_result.force_robot1(:, k) = [0; 0; 0];
                    load_result.force_robot2(:, k) = [0; 0; 0];
                end
            end
        end
        
        %% ===================================================================
        %  COORDINATION ANALYSIS
        %  ===================================================================
        
        function coord = analyze_coordination(robot1_result, robot2_result, grasp_states, load_params)
            % Analyze coordination quality
            n_samples = size(robot1_result.ee_position, 2);
            
            coord = struct();
            coord.ee_separation = zeros(1, n_samples);
            coord.force_balance = zeros(1, n_samples);
            coord.velocity_sync = zeros(1, n_samples);
            
            for k = 1:n_samples
                % End-effector separation distance
                coord.ee_separation(k) = norm(robot1_result.ee_position(:, k) - ...
                                               robot2_result.ee_position(:, k));
                
                if grasp_states(k)
                    % Force balance (should be equal)
                    f1 = norm(robot1_result.ee_velocity(4:6, k));
                    f2 = norm(robot2_result.ee_velocity(4:6, k));
                    coord.force_balance(k) = abs(f1 - f2) / (f1 + f2 + 1e-10);
                    
                    % Velocity synchronization
                    v1 = robot1_result.ee_velocity(4:6, k);
                    v2 = robot2_result.ee_velocity(4:6, k);
                    coord.velocity_sync(k) = norm(v1 - v2);
                else
                    coord.force_balance(k) = 0;
                    coord.velocity_sync(k) = 0;
                end
            end
            
            % Statistics
            grasped_indices = find(grasp_states);
            if ~isempty(grasped_indices)
                coord.avg_separation_grasped = mean(coord.ee_separation(grasped_indices));
                coord.avg_force_balance = mean(coord.force_balance(grasped_indices));
                coord.avg_velocity_sync = mean(coord.velocity_sync(grasped_indices));
            else
                coord.avg_separation_grasped = 0;
                coord.avg_force_balance = 0;
                coord.avg_velocity_sync = 0;
            end
        end
        
        %% ===================================================================
        %  VERIFICATION
        %  ===================================================================
        
        function verification = verify_cooperative_motion(results, load_params)
            % Verify cooperative motion quality
            verification = struct();
            
            fprintf('========================================\n');
            fprintf('COOPERATIVE MOTION VERIFICATION\n');
            fprintf('========================================\n\n');
            
            % Check grasp stability
            grasped_indices = find(results.grasp_states);
            if ~isempty(grasped_indices)
                separations = results.coordination.ee_separation(grasped_indices);
                expected_separation = load_params.size(1);  % Load width
                
                max_deviation = max(abs(separations - expected_separation));
                verification.grasp_stable = max_deviation < 0.05;  % 5cm tolerance
                
                fprintf('GRASP STABILITY:\n');
                fprintf('  Expected EE separation: %.3f m\n', expected_separation);
                fprintf('  Actual avg separation: %.3f m\n', mean(separations));
                fprintf('  Max deviation: %.3f m\n', max_deviation);
                fprintf('  Status: %s\n\n', tern(verification.grasp_stable, '✓ STABLE', '✗ UNSTABLE'));
            else
                verification.grasp_stable = true;
            end
            
            % Check force balance
            verification.forces_balanced = results.coordination.avg_force_balance < 0.1;
            fprintf('FORCE BALANCE:\n');
            fprintf('  Avg imbalance: %.4f\n', results.coordination.avg_force_balance);
            fprintf('  Status: %s\n\n', tern(verification.forces_balanced, '✓ BALANCED', '⚠ IMBALANCED'));
            
            % Check velocity synchronization
            verification.velocities_synced = results.coordination.avg_velocity_sync < 0.05;
            fprintf('VELOCITY SYNCHRONIZATION:\n');
            fprintf('  Avg velocity difference: %.4f m/s\n', results.coordination.avg_velocity_sync);
            fprintf('  Status: %s\n\n', tern(verification.velocities_synced, '✓ SYNCHRONIZED', '⚠ DRIFT'));
            
            % Overall assessment
            verification.cooperative_success = verification.grasp_stable && ...
                                                verification.forces_balanced && ...
                                                verification.velocities_synced;
            
            fprintf('OVERALL COOPERATIVE PERFORMANCE:\n');
            if verification.cooperative_success
                fprintf('  ✓✓✓ EXCELLENT COOPERATION ✓✓✓\n');
            else
                fprintf('  ⚠ COOPERATION NEEDS IMPROVEMENT\n');
            end
        end
        
        %% ===================================================================
        %  VISUALIZATION
        %  ===================================================================
        
        function visualize_collaborative_system(results, robot1, robot2, load_params)
            % Create all visualizations
            fprintf('\n--- Creating Visualizations ---\n');
            
            % Figure 1: Workspace with load
            Collaborative_DualArm_Functions.plot_workspace(results, robot1, robot2, load_params);
            
            % Figure 2: Coordination metrics
            Collaborative_DualArm_Functions.plot_coordination(results);
            
            % Figure 3: Joint trajectories
            Collaborative_DualArm_Functions.plot_joint_trajectories(results);
            
            fprintf('✓ Visualizations created\n\n');
        end
        
        function plot_workspace(results, robot1, robot2, load_params)
            % Plot workspace with both robots and load
            figure('Name', 'Collaborative Workspace with Robot Configurations', 'Position', [50, 50, 1400, 900]);
            
            % Robot 1 path
            plot3(results.robot1.ee_position(1,:), results.robot1.ee_position(2,:), ...
                  results.robot1.ee_position(3,:), 'b--', 'LineWidth', 1.5);
            hold on;
            
            % Robot 2 path
            plot3(results.robot2.ee_position(1,:), results.robot2.ee_position(2,:), ...
                  results.robot2.ee_position(3,:), 'r--', 'LineWidth', 1.5);
            
            % Load trajectory (when grasped)
            grasped_idx = find(results.grasp_states);
            if ~isempty(grasped_idx)
                plot3(results.load.position(1, grasped_idx), ...
                      results.load.position(2, grasped_idx), ...
                      results.load.position(3, grasped_idx), ...
                      'g-', 'LineWidth', 3);
            end
            
            % Show robot configurations at key points (initial, mid, final)
            n_samples = size(results.robot1.ee_position, 2);
            key_indices = [1, round(n_samples/2), n_samples];
            alphas = [0.4, 0.6, 0.8];
            
            for idx = 1:length(key_indices)
                k = key_indices(idx);
                alpha = alphas(idx);
                
                % Robot 1
                Collaborative_DualArm_Functions.draw_robot_links(...
                    results.robot1.q_trajectory(:, k), robot1, 'b', alpha, 3);
                
                % Robot 2
                Collaborative_DualArm_Functions.draw_robot_links(...
                    results.robot2.q_trajectory(:, k), robot2, 'r', alpha, 3);
                
                % Load if grasped
                if results.grasp_states(k)
                    draw_load_box(results.load.position(:, k), load_params, alpha);
                end
            end
            
            % Bases
            scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
                     300, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
            scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
                     300, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
            
            grid on;
            xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
            zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
            title('Collaborative Workspace - Trajectories and Configurations', 'FontSize', 14, 'FontWeight', 'bold');
            legend('Robot 1 Path', 'Robot 2 Path', 'Load Path', 'Location', 'best');
            axis equal;
            view(45, 25);
            hold off;
        end
        
        function plot_coordination(results)
            % Plot coordination metrics
            figure('Name', 'Coordination Analysis', 'Position', [100, 100, 1400, 800]);
            
            % EE separation
            subplot(2, 2, 1);
            plot(results.time_vector, results.coordination.ee_separation, 'k-', 'LineWidth', 2);
            hold on;
            
            % Highlight grasped region
            grasped_idx = find(results.grasp_states);
            if ~isempty(grasped_idx)
                t_grasped = results.time_vector(grasped_idx);
                sep_grasped = results.coordination.ee_separation(grasped_idx);
                plot(t_grasped, sep_grasped, 'g-', 'LineWidth', 3);
            end
            
            grid on;
            xlabel('Time (s)');
            ylabel('Distance (m)');
            title('End-Effector Separation');
            legend('Separation', 'Grasped');
            
            % Force balance
            subplot(2, 2, 2);
            plot(results.time_vector, results.coordination.force_balance, 'b-', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Imbalance Ratio');
            title('Force Balance (lower is better)');
            
            % Velocity sync
            subplot(2, 2, 3);
            plot(results.time_vector, results.coordination.velocity_sync, 'r-', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Velocity Difference (m/s)');
            title('Velocity Synchronization');
            
            % Manipulability comparison
            subplot(2, 2, 4);
            plot(results.time_vector, results.robot1.manipulability, 'b-', 'LineWidth', 2);
            hold on;
            plot(results.time_vector, results.robot2.manipulability, 'r--', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Manipulability');
            title('Manipulability - Both Robots');
            legend('Robot 1', 'Robot 2');
        end
        
        function plot_joint_trajectories(results)
            % Plot joint trajectories for both robots
            figure('Name', 'Joint Trajectories', 'Position', [150, 150, 1400, 600]);
            
            n1 = results.robot1.n_links;
            n2 = results.robot2.n_links;
            n_plots = max(n1, n2);
            
            for i = 1:n_plots
                subplot(2, ceil(n_plots/2), i);
                hold on;
                
                if i <= n1
                    plot(results.time_vector, results.robot1.q_trajectory(i, :), ...
                         'b-', 'LineWidth', 2);
                end
                if i <= n2
                    plot(results.time_vector, results.robot2.q_trajectory(i, :), ...
                         'r--', 'LineWidth', 2);
                end
                
                % Highlight grasped region
                grasped_idx = find(results.grasp_states);
                if ~isempty(grasped_idx)
                    ylims = ylim;
                    fill([results.time_vector(grasped_idx(1)), ...
                          results.time_vector(grasped_idx(end)), ...
                          results.time_vector(grasped_idx(end)), ...
                          results.time_vector(grasped_idx(1))], ...
                         [ylims(1), ylims(1), ylims(2), ylims(2)], ...
                         'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
                end
                
                grid on;
                xlabel('Time (s)');
                ylabel(sprintf('Joint %d (rad)', i));
                title(sprintf('Joint %d', i));
                if i <= n1 && i <= n2
                    legend('Robot 1', 'Robot 2', 'Location', 'best');
                end
            end
        end
        
        %% ===================================================================
        %  ANIMATION
        %  ===================================================================
        
        function animate_collaborative_motion(results, robot1, robot2, load_params)
            % Animate collaborative motion with full robot visualization
            fprintf('--- Starting Animation ---\n');
            fprintf('    This may take a moment...\n');
            
            fig = figure('Name', 'Collaborative Motion Animation', 'Position', [200, 100, 1400, 900]);
            
            n_samples = length(results.time_vector);
            frame_skip = max(1, floor(n_samples / 100));  % Aim for ~100 frames
            dt = 0.05;  % 50ms per frame
            
            for i = 1:frame_skip:n_samples
                clf(fig);
                hold on;
                grid on;
                
                % Draw both robots with links
                Collaborative_DualArm_Functions.draw_robot_links(...
                    results.robot1.q_trajectory(:, i), robot1, 'b', 1.0, 4);
                Collaborative_DualArm_Functions.draw_robot_links(...
                    results.robot2.q_trajectory(:, i), robot2, 'r', 1.0, 4);
                
                % Draw bases
                scatter3(robot1.base_position(1), robot1.base_position(2), robot1.base_position(3), ...
                         300, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
                scatter3(robot2.base_position(1), robot2.base_position(2), robot2.base_position(3), ...
                         300, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
                
                % Draw load if grasped
                if results.grasp_states(i)
                    draw_load_box(results.load.position(:, i), load_params, 0.8);
                    
                    % Connection lines from EE to load
                    plot3([results.robot1.ee_position(1, i), results.load.position(1, i)], ...
                          [results.robot1.ee_position(2, i), results.load.position(2, i)], ...
                          [results.robot1.ee_position(3, i), results.load.position(3, i)], ...
                          'g-', 'LineWidth', 3);
                    plot3([results.robot2.ee_position(1, i), results.load.position(1, i)], ...
                          [results.robot2.ee_position(2, i), results.load.position(2, i)], ...
                          [results.robot2.ee_position(3, i), results.load.position(3, i)], ...
                          'g-', 'LineWidth', 3);
                end
                
                % Trajectory traces (faded)
                if i > 1
                    plot3(results.robot1.ee_position(1, 1:i), results.robot1.ee_position(2, 1:i), ...
                          results.robot1.ee_position(3, 1:i), 'b:', 'LineWidth', 1.5);
                    plot3(results.robot2.ee_position(1, 1:i), results.robot2.ee_position(2, 1:i), ...
                          results.robot2.ee_position(3, 1:i), 'r:', 'LineWidth', 1.5);
                    
                    % Load trace if grasped
                    grasped_so_far = find(results.grasp_states(1:i));
                    if ~isempty(grasped_so_far)
                        plot3(results.load.position(1, grasped_so_far), ...
                              results.load.position(2, grasped_so_far), ...
                              results.load.position(3, grasped_so_far), ...
                              'g:', 'LineWidth', 2);
                    end
                end
                
                % Styling
                xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
                ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
                zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
                
                % Status title
                status_str = sprintf('t = %.2f s / %.2f s | ', ...
                                    results.time_vector(i), results.time_vector(end));
                if results.grasp_states(i)
                    status_str = [status_str, '🤝 LOAD GRASPED'];
                    title_color = [0, 0.5, 0];
                else
                    status_str = [status_str, '✋ LOAD FREE'];
                    title_color = [0.5, 0.5, 0.5];
                end
                
                title(status_str, 'FontSize', 14, 'FontWeight', 'bold', 'Color', title_color);
                
                axis equal;
                view(45, 25);
                
                % Set consistent axis limits
                all_x = [results.robot1.ee_position(1,:), results.robot2.ee_position(2,:)];
                all_y = [results.robot1.ee_position(2,:), results.robot2.ee_position(2,:)];
                all_z = [results.robot1.ee_position(3,:), results.robot2.ee_position(3,:)];
                
                x_range = [min(all_x)-0.3, max(all_x)+0.3];
                y_range = [min(all_y)-0.3, max(all_y)+0.3];
                z_range = [0, max(all_z)+0.3];
                
                xlim(x_range);
                ylim(y_range);
                zlim(z_range);
                
                drawnow;
                pause(dt);
            end
            
            fprintf('✓ Animation complete\n\n');
        end
        
        %% ===================================================================
        %  DATA EXPORT
        %  ===================================================================
        
        function export_collaborative_data(results, robot1, robot2, load_params, prefix)
            % Export all collaborative data
            fprintf('--- Exporting Data ---\n');
            
            if ~exist('output', 'dir')
                mkdir('output');
            end
            
            % Export robot trajectories
            writematrix([results.time_vector', results.robot1.q_trajectory'], ...
                        sprintf('output/%s_robot1_joints.csv', prefix));
            writematrix([results.time_vector', results.robot2.q_trajectory'], ...
                        sprintf('output/%s_robot2_joints.csv', prefix));
            
            % Export EE positions
            writematrix([results.time_vector', results.robot1.ee_position'], ...
                        sprintf('output/%s_robot1_ee.csv', prefix));
            writematrix([results.time_vector', results.robot2.ee_position'], ...
                        sprintf('output/%s_robot2_ee.csv', prefix));
            
            % Export load data
            writematrix([results.time_vector', results.load.position', double(results.grasp_states)'], ...
                        sprintf('output/%s_load_trajectory.csv', prefix));
            
            % Export coordination metrics
            writematrix([results.time_vector', results.coordination.ee_separation', ...
                        results.coordination.force_balance', results.coordination.velocity_sync'], ...
                        sprintf('output/%s_coordination.csv', prefix));
            
            % Export summary
            fid = fopen(sprintf('output/%s_summary.txt', prefix), 'w');
            fprintf(fid, 'COLLABORATIVE DUAL-ARM SYSTEM SUMMARY\n');
            fprintf(fid, '=====================================\n\n');
            fprintf(fid, 'Robot 1: %d-DOF\n', results.robot1.n_links);
            fprintf(fid, 'Robot 2: %d-DOF\n', results.robot2.n_links);
            fprintf(fid, 'Load: %.1f kg\n\n', load_params.mass);
            fprintf(fid, 'Coordination Metrics:\n');
            fprintf(fid, '  Avg EE separation (grasped): %.4f m\n', results.coordination.avg_separation_grasped);
            fprintf(fid, '  Avg force balance: %.4f\n', results.coordination.avg_force_balance);
            fprintf(fid, '  Avg velocity sync: %.4f m/s\n\n', results.coordination.avg_velocity_sync);
            fprintf(fid, 'Verification:\n');
            fprintf(fid, '  Grasp stable: %s\n', tern(results.verification.grasp_stable, 'YES', 'NO'));
            fprintf(fid, '  Forces balanced: %s\n', tern(results.verification.forces_balanced, 'YES', 'NO'));
            fprintf(fid, '  Velocities synced: %s\n', tern(results.verification.velocities_synced, 'YES', 'NO'));
            fclose(fid);
            
            fprintf('✓ All data exported to output/\n\n');
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
        
        function draw_robot_links(q, robot_params, color, alpha, linewidth)
            % Draw robot with links at given configuration
            %
            % INPUTS:
            %   q - Joint configuration
            %   robot_params - Robot structure with link_vectors, joint_axes, etc.
            %   color - Color for links (e.g., 'b', 'r', [0.5 0.5 0.5])
            %   alpha - Transparency (0-1)
            %   linewidth - Line width for links
            
            n_links = size(robot_params.link_vectors, 2);
            
            % Compute link positions using forward kinematics
            positions = zeros(3, n_links + 1);
            positions(:, 1) = robot_params.base_position;
            
            % Current transformation
            current_pos = robot_params.base_position;
            current_rot = eye(3);
            
            for k = 1:n_links
                % Get link vector and joint axis
                link_vec = robot_params.link_vectors(:, k);
                joint_axis = robot_params.joint_axes(:, k);
                joint_type = robot_params.joint_types(k);
                
                % Transform link vector to world frame
                link_vec_world = current_rot * link_vec;
                current_pos = current_pos + link_vec_world;
                
                % Apply joint rotation
                if strcmpi(joint_type, 'R')
                    R_joint = ARAT_Core.rodrigues_rotation(joint_axis, q(k));
                    current_rot = current_rot * R_joint;
                end
                
                positions(:, k + 1) = current_pos;
            end
            
            % Draw links
            for k = 1:n_links
                p1 = positions(:, k);
                p2 = positions(:, k + 1);
                
                % Draw link as thick line
                plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                      'Color', color, 'LineWidth', linewidth);
                
                % Draw joint as sphere
                [X, Y, Z] = sphere(10);
                radius = 0.03;
                surf(X*radius + p1(1), Y*radius + p1(2), Z*radius + p1(3), ...
                     'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
            end
            
            % Draw end-effector as larger sphere
            ee_pos = positions(:, end);
            [X, Y, Z] = sphere(12);
            radius = 0.05;
            surf(X*radius + ee_pos(1), Y*radius + ee_pos(2), Z*radius + ee_pos(3), ...
                 'FaceColor', color, 'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', alpha);
        end
        
    end
end

%% HELPER FUNCTIONS

function result = tern(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end

function draw_load_box(center, load_params, alpha)
    % Draw 3D box representing load
    if nargin < 3
        alpha = 0.8;
    end
    
    l = load_params.size(1) / 2;
    w = load_params.size(2) / 2;
    h = load_params.size(3) / 2;
    
    vertices = [
        -l, -w, -h; l, -w, -h; l, w, -h; -l, w, -h;
        -l, -w, h;  l, -w, h;  l, w, h;  -l, w, h
    ];
    
    vertices = vertices + center';
    
    faces = [
        1, 2, 3, 4; 5, 6, 7, 8; 1, 2, 6, 5;
        2, 3, 7, 6; 3, 4, 8, 7; 4, 1, 5, 8
    ];
    
    patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', [0.8, 0.6, 0.2], 'FaceAlpha', alpha, ...
          'EdgeColor', 'k', 'LineWidth', 1.5);
end
