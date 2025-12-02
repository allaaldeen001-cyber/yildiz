classdef RobotIK
    % ROBOTIK Inverse Kinematics solver using Jacobian-based methods
    % Supports both numerical iterative IK and analytical approaches
    
    methods (Static)
        
        function [q_sol, success, iterations] = solve_numerical_ik(robot, target_pose, q_init, options)
            % Numerical IK using Damped Least Squares (Levenberg-Marquardt)
            % robot: struct containing robot parameters
            % target_pose: 6x1 [position; orientation] or 3x1 [position only]
            % q_init: initial joint configuration
            % options: struct with max_iter, tolerance, lambda (damping)
            
            % Default options
            if nargin < 4 || isempty(options)
                options.max_iter = 100;
                options.tolerance = 1e-4;
                options.lambda = 0.01; % Damping factor
                options.position_only = false;
            end
            
            q = q_init;
            n = length(q);
            success = false;
            
            for iter = 1:options.max_iter
                % Compute current end-effector pose using forward kinematics
                current_pose = RobotIK.forward_kinematics_pose(robot, q);
                
                % Compute error
                if options.position_only
                    error = target_pose(1:3) - current_pose(1:3);
                    error_norm = norm(error);
                else
                    error = target_pose - current_pose;
                    error_norm = norm(error);
                end
                
                % Check convergence
                if error_norm < options.tolerance
                    success = true;
                    iterations = iter;
                    break;
                end
                
                % Compute Jacobian
                J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                               robot.joint_types, q);
                
                if options.position_only
                    J = J(4:6, :); % Use only position part
                end
                
                % Damped Least Squares (DLS) solution
                % delta_q = J^T * (J*J^T + lambda^2*I)^-1 * error
                lambda_sq = options.lambda^2;
                m = size(J, 1);
                delta_q = J' / (J*J' + lambda_sq * eye(m)) * error;
                
                % Update joint angles
                q = q + delta_q;
                
                % Apply joint limits if specified
                if isfield(robot, 'joint_limits')
                    for i = 1:n
                        q(i) = max(robot.joint_limits(i,1), min(q(i), robot.joint_limits(i,2)));
                    end
                end
            end
            
            if ~success
                iterations = options.max_iter;
                warning('IK did not converge within maximum iterations');
            end
            
            q_sol = q;
        end
        
        function pose = forward_kinematics_pose(robot, q)
            % Compute forward kinematics to get end-effector pose
            % Returns 6x1 vector [orientation; position]
            
            n = length(q);
            
            % Build transformation from base to end-effector
            T = eye(4);
            
            for k = 1:n
                % Get link parameters
                l_vec = robot.link_vectors(:, k);
                h_vec = robot.joint_axes(:, k);
                type = robot.joint_types(k);
                
                % Translation to next joint
                T_link = eye(4);
                T_link(1:3, 4) = l_vec;
                
                % Joint transformation
                T_joint = eye(4);
                if strcmpi(type, 'R')
                    % Rotation about axis
                    R = ARAT_Core.rodrigues_rotation(h_vec, q(k));
                    T_joint(1:3, 1:3) = R;
                else
                    % Prismatic: translation along axis
                    T_joint(1:3, 4) = h_vec * q(k);
                end
                
                T = T * T_link * T_joint;
            end
            
            % Extract position
            position = T(1:3, 4);
            
            % Extract orientation (as rotation vector or Euler angles)
            R = T(1:3, 1:3);
            orientation = RobotIK.rotation_matrix_to_axis_angle(R);
            
            pose = [orientation; position];
        end
        
        function axis_angle = rotation_matrix_to_axis_angle(R)
            % Convert rotation matrix to axis-angle representation
            
            theta = acos((trace(R) - 1) / 2);
            
            if abs(theta) < 1e-6
                % No rotation
                axis_angle = [0; 0; 0];
            else
                % Extract axis
                axis = (1 / (2*sin(theta))) * [R(3,2) - R(2,3);
                                                 R(1,3) - R(3,1);
                                                 R(2,1) - R(1,2)];
                axis_angle = axis * theta;
            end
        end
        
        function q_trajectory = solve_ik_trajectory(robot, target_trajectory, q_init, options)
            % Solve IK for a trajectory of target poses
            % target_trajectory: 6xM or 3xM matrix of target poses
            % Returns: NxM matrix of joint configurations
            
            M = size(target_trajectory, 2);
            n = length(q_init);
            q_trajectory = zeros(n, M);
            
            q_prev = q_init;
            
            for i = 1:M
                target = target_trajectory(:, i);
                [q_sol, success, ~] = RobotIK.solve_numerical_ik(robot, target, q_prev, options);
                
                if ~success
                    warning('IK failed at waypoint %d', i);
                end
                
                q_trajectory(:, i) = q_sol;
                q_prev = q_sol; % Use previous solution as initial guess
            end
        end
        
    end
end
