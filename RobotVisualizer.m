classdef RobotVisualizer
    % ROBOTVISUALIZER Visualization and animation tools for robot manipulators
    
    methods (Static)
        
        function handles = plot_robot(robot, q, fig_handle)
            % Plot robot in current configuration
            % robot: struct with robot parameters
            % q: joint configuration (Nx1)
            % fig_handle: figure handle (optional)
            
            if nargin < 3
                fig_handle = figure('Name', 'Robot Manipulator', 'NumberTitle', 'off');
            else
                figure(fig_handle);
            end
            
            hold on;
            grid on;
            axis equal;
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            title('Robot Configuration');
            view(3);
            
            % Compute forward kinematics to get all link positions
            [positions, frames] = RobotVisualizer.compute_link_positions(robot, q);
            
            n = size(positions, 2) - 1; % Number of joints
            
            % Plot links
            for i = 1:n
                p1 = positions(:, i);
                p2 = positions(:, i+1);
                
                % Draw link
                h_link = plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                              'b-', 'LineWidth', 3);
                
                % Draw joint
                h_joint = plot3(p1(1), p1(2), p1(3), 'ro', ...
                               'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
            
            % Draw end-effector
            p_end = positions(:, end);
            h_end = plot3(p_end(1), p_end(2), p_end(3), 'gs', ...
                         'MarkerSize', 12, 'MarkerFaceColor', 'g');
            
            % Draw coordinate frames at each joint
            frame_scale = 0.1; % Scale for coordinate frame arrows
            for i = 1:size(frames, 3)
                p = positions(:, i);
                R = frames(:, :, i);
                
                % X-axis (red)
                quiver3(p(1), p(2), p(3), R(1,1)*frame_scale, R(2,1)*frame_scale, ...
                       R(3,1)*frame_scale, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
                % Y-axis (green)
                quiver3(p(1), p(2), p(3), R(1,2)*frame_scale, R(2,2)*frame_scale, ...
                       R(3,2)*frame_scale, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
                % Z-axis (blue)
                quiver3(p(1), p(2), p(3), R(1,3)*frame_scale, R(2,3)*frame_scale, ...
                       R(3,3)*frame_scale, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            end
            
            hold off;
            
            % Return handles for animation
            handles.fig = fig_handle;
            handles.positions = positions;
        end
        
        function [positions, frames] = compute_link_positions(robot, q)
            % Compute position of each link origin and orientation frames
            % Returns: 3x(N+1) matrix of positions (base + N joints + end-effector)
            %          3x3x(N+1) array of rotation matrices
            
            n = length(q);
            positions = zeros(3, n+1);
            frames = zeros(3, 3, n+1);
            
            % Base frame
            T = eye(4);
            positions(:, 1) = T(1:3, 4);
            frames(:, :, 1) = T(1:3, 1:3);
            
            % Propagate through each joint
            for k = 1:n
                l_vec = robot.link_vectors(:, k);
                h_vec = robot.joint_axes(:, k);
                type = robot.joint_types(k);
                
                % Translation to next joint
                T_link = eye(4);
                T_link(1:3, 4) = l_vec;
                
                % Joint transformation
                T_joint = eye(4);
                if strcmpi(type, 'R')
                    R = ARAT_Core.rodrigues_rotation(h_vec, q(k));
                    T_joint(1:3, 1:3) = R;
                else
                    T_joint(1:3, 4) = h_vec * q(k);
                end
                
                T = T * T_link * T_joint;
                
                positions(:, k+1) = T(1:3, 4);
                frames(:, :, k+1) = T(1:3, 1:3);
            end
        end
        
        function animate_trajectory(robot, q_trajectory, dt, save_video)
            % Animate robot moving through a trajectory
            % q_trajectory: NxM matrix of joint configurations
            % dt: time step between frames (default 0.05s)
            % save_video: true to save as MP4 (default false)
            
            if nargin < 3 || isempty(dt)
                dt = 0.05;
            end
            if nargin < 4
                save_video = false;
            end
            
            M = size(q_trajectory, 2);
            
            % Create figure
            fig = figure('Name', 'Robot Animation', 'NumberTitle', 'off', ...
                        'Position', [100, 100, 800, 600]);
            
            % Setup video writer if saving
            if save_video
                v = VideoWriter('robot_animation.mp4', 'MPEG-4');
                v.FrameRate = 1/dt;
                open(v);
            end
            
            % Compute workspace bounds
            all_positions = [];
            for i = 1:M
                [pos, ~] = RobotVisualizer.compute_link_positions(robot, q_trajectory(:, i));
                all_positions = [all_positions, pos];
            end
            
            x_range = [min(all_positions(1,:)), max(all_positions(1,:))];
            y_range = [min(all_positions(2,:)), max(all_positions(2,:))];
            z_range = [min(all_positions(3,:)), max(all_positions(3,:))];
            
            % Add margin
            margin = 0.2;
            x_range = x_range + [-margin, margin];
            y_range = y_range + [-margin, margin];
            z_range = z_range + [-margin, margin];
            
            % Animate
            for i = 1:M
                clf(fig);
                hold on;
                grid on;
                axis equal;
                
                xlim(x_range);
                ylim(y_range);
                zlim(z_range);
                
                xlabel('X (m)');
                ylabel('Y (m)');
                zlabel('Z (m)');
                title(sprintf('Robot Animation - Frame %d/%d', i, M));
                view(3);
                
                q = q_trajectory(:, i);
                [positions, frames] = RobotVisualizer.compute_link_positions(robot, q);
                
                n = size(positions, 2) - 1;
                
                % Plot links
                for k = 1:n
                    p1 = positions(:, k);
                    p2 = positions(:, k+1);
                    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
                          'b-', 'LineWidth', 4);
                    plot3(p1(1), p1(2), p1(3), 'ro', ...
                          'MarkerSize', 10, 'MarkerFaceColor', 'r');
                end
                
                % End-effector
                p_end = positions(:, end);
                plot3(p_end(1), p_end(2), p_end(3), 'gs', ...
                      'MarkerSize', 15, 'MarkerFaceColor', 'g');
                
                % Draw end-effector trajectory trace
                if i > 1
                    prev_positions = [];
                    for j = 1:i
                        [pos_j, ~] = RobotVisualizer.compute_link_positions(robot, q_trajectory(:, j));
                        prev_positions = [prev_positions, pos_j(:, end)];
                    end
                    plot3(prev_positions(1,:), prev_positions(2,:), prev_positions(3,:), ...
                          'g--', 'LineWidth', 1);
                end
                
                % Draw coordinate frames
                frame_scale = 0.15;
                for k = 1:size(frames, 3)
                    p = positions(:, k);
                    R = frames(:, :, k);
                    quiver3(p(1), p(2), p(3), R(1,1)*frame_scale, R(2,1)*frame_scale, ...
                           R(3,1)*frame_scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
                    quiver3(p(1), p(2), p(3), R(1,2)*frame_scale, R(2,2)*frame_scale, ...
                           R(3,2)*frame_scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
                    quiver3(p(1), p(2), p(3), R(1,3)*frame_scale, R(2,3)*frame_scale, ...
                           R(3,3)*frame_scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
                end
                
                hold off;
                drawnow;
                
                if save_video
                    frame = getframe(fig);
                    writeVideo(v, frame);
                end
                
                pause(dt);
            end
            
            if save_video
                close(v);
                fprintf('Video saved as robot_animation.mp4\n');
            end
        end
        
        function plot_joint_trajectories(q_trajectory, time_vector)
            % Plot joint angles/positions over time
            % q_trajectory: NxM matrix
            % time_vector: 1xM vector of time stamps
            
            n = size(q_trajectory, 1);
            
            figure('Name', 'Joint Trajectories', 'NumberTitle', 'off');
            
            for i = 1:n
                subplot(ceil(n/2), 2, i);
                plot(time_vector, q_trajectory(i, :), 'LineWidth', 2);
                grid on;
                xlabel('Time (s)');
                ylabel(sprintf('Joint %d', i));
                title(sprintf('Joint %d Trajectory', i));
            end
        end
        
    end
end
