classdef ARAT_Core
    % ARAT_CORE Implementation of the mathematical models from the thesis:
    % "Development of a Toolbox for Kinematic and Dynamic Modeling of 
    % Multibody and High Degree of Freedom Robotic Systems" [cite: 1, 5, 267]
    
    methods (Static)
        
        %% --- CHAPTER 1: GENERAL UTILITIES ---
        
        function S = skew(v)
            % Returns the skew-symmetric matrix of a 3x1 vector.
            % Reference: Equation (2.4) [cite: 676]
            S = [0, -v(3), v(2);
                 v(3), 0, -v(1);
                 -v(2), v(1), 0];
        end

        function R = rodrigues_rotation(w, theta)
            % Rodrigues' Rotation Formula.
            % w: rotation axis (3x1, normalized)
            % theta: rotation angle (scalar, radians)
            % Reference: Equation (3.17) [cite: 1187]
            
            I = eye(3);
            w_hat = ARAT_Core.skew(w);
            w_hat_sq = w_hat^2;
            
            % R = I + sin(theta)*w_hat + (1-cos(theta))*w_hat^2
            R = I + sin(theta) * w_hat + (1 - cos(theta)) * w_hat_sq;
        end

        %% --- CHAPTER 2: KINEMATICS (SOA) ---
        
        function Phi_k_km1 = get_prop_matrix(l_vec)
            % Calculates velocity propagation matrix between links (Phi_{k,k-1}).
            % l_vec: Vector from origin k-1 to k
            % Reference: Equation (2.9) block structure [cite: 719]
            
            I = eye(3);
            l_hat = ARAT_Core.skew(l_vec);
            
            % Phi = [I  0; -l_hat  I]
            Phi_k_km1 = [I, zeros(3); -l_hat, I];
        end
        
        function H_k = get_joint_matrix(type, h_axis)
            % Returns the joint map matrix H_k.
            % type: 'R' for Rotational, 'P' for Prismatic
            % h_axis: axis of motion (3x1)
            % Reference: Eq (2.6) for R, Eq (2.10) for P [cite: 684, 722]
            
            if strcmpi(type, 'R')
                % Rotational: H = [h; 0]
                H_k = [h_axis; zeros(3,1)];
            else
                % Prismatic: H = [0; h]
                H_k = [zeros(3,1); h_axis];
            end
        end

        function V_t = serial_forward_kinematics(link_vectors, joint_axes, joint_types, q, q_dot, V_base)
            % Computes Forward Kinematics using SOA recursion.
            % link_vectors: 3xN matrix of link vectors
            % joint_axes: 3xN matrix of axes
            % q: joint positions (Nx1)
            % q_dot: joint velocities (Nx1)
            % V_base: Base velocity 6x1 [w; v]
            % Reference: Equations (2.11) - (2.17) [cite: 768-785]
            
            n = length(q);
            V_prev = V_base;
            
            % Iterate from base to tip
            for k = 1:n
                % Get parameters for current link
                l_vec = link_vectors(:, k);
                h_vec = joint_axes(:, k);
                type = joint_types(k);
                
                % 1. Calculate Propagation Matrix Phi (Eq 2.9)
                Phi = ARAT_Core.get_prop_matrix(l_vec);
                
                % 2. Calculate Joint Matrix H (Eq 2.6 / 2.10)
                H = ARAT_Core.get_joint_matrix(type, h_vec);
                
                % 3. Propagate Velocity V_k = Phi * V_{k-1} + H * q_dot_k
                % Reference: Equation (2.6) [cite: 684]
                V_curr = Phi * V_prev + H * q_dot(k);
                
                % Update for next iteration
                V_prev = V_curr;
            end
            
            V_t = V_prev; % Tip velocity
        end
        
        function J = compute_jacobian(link_vectors, joint_axes, joint_types, q)
            % Computes the System Jacobian Matrix J.
            % Reference: Equation (2.26) J = Phi_t * Phi * H [cite: 896]
            
            n = length(q);
            J = zeros(6, n);
            
            % Using a simplified column-wise computation equivalent to SOA
            % For each joint k, propagate its effect to the tip.
            
            % Identity for tip propagation
            Phi_tip_k = eye(6); 
            
            % Iterate backwards from tip to base to build columns
            for k = n:-1:1
                l_vec = link_vectors(:, k);
                h_vec = joint_axes(:, k);
                type = joint_types(k);
                
                H = ARAT_Core.get_joint_matrix(type, h_vec);
                
                % The kth column of J is Phi_{tip,k} * H_k
                J(:, k) = Phi_tip_k * H;
                
                % Update Phi for the next joint inwards (k-1)
                % Phi_{tip, k-1} = Phi_{tip, k} * Phi_{k, k-1}
                Phi_k_km1 = ARAT_Core.get_prop_matrix(l_vec);
                Phi_tip_k = Phi_tip_k * Phi_k_km1;
            end
        end

        %% --- CHAPTER 3: DYNAMICS ---
        
        function [tau] = recursive_newton_euler(links, joints, mass_params, q, q_d, q_dd, g_vec)
            % Inverse Dynamics using Recursive Newton-Euler.
            % mass_params: struct array with m (mass), c (CoM), I (Inertia)
            % g_vec: Gravity vector [gx; gy; gz] (usually [0;0;-9.81])
            % Reference: Equation (3.69) structure [cite: 1666]
            
            n = length(q);
            
            % --- Forward Pass (Velocities & Accelerations) ---
            % Initialize base states (stationary base assumption + gravity)
            w = zeros(3,1);
            v = zeros(3,1);
            w_dot = zeros(3,1);
            v_dot = -g_vec; % Base acceleration to simulate gravity (Eq 3.47) [cite: 1411]
            
            % Storage
            F_spatial = zeros(6, n); % Spatial forces
            
            for k = 1:n
                l = links(:, k);
                h = joints.axis(:, k);
                type = joints.type(k);
                
                % Rotation matrix for current joint (simplified for code)
                % In full SOA, this is handled by Phi, here we do component wise
                
                % Velocity Propagation (Eq 2.1, 2.2) [cite: 665, 666]
                if strcmpi(type, 'R')
                    w_next = w + h * q_d(k);
                    v_next = v + cross(w, l);
                    
                    % Acceleration Propagation (Eq 3.35, 3.36) [cite: 1294, 1295]
                    w_dot_next = w_dot + cross(w, h*q_d(k)) + h*q_dd(k);
                    v_dot_next = v_dot - cross(l, w_dot) + cross(w, cross(w, l)); % using Eq 3.36 logic
                else
                    % Prismatic logic (Eq 3.39, 3.42) [cite: 1327, 1339]
                    w_next = w;
                    v_next = v + cross(w, l) + h*q_d(k);
                    w_dot_next = w_dot;
                    v_dot_next = v_dot + cross(w_dot, l) + cross(w, cross(w,l)) + 2*cross(w, h*q_d(k)) + h*q_dd(k);
                end
                
                % Update for next iteration
                w = w_next; v = v_next; 
                w_dot = w_dot_next; v_dot = v_dot_next;
                
                % Inertial Forces (Newton-Euler)
                m = mass_params(k).m;
                c = mass_params(k).c; % CoM vector from origin
                I_tens = mass_params(k).I;
                
                % Acceleration at CoM
                v_dot_c = v_dot + cross(w_dot, c) + cross(w, cross(w, c));
                
                % Force and Torque at CoM
                F = m * v_dot_c;
                N = I_tens * w_dot + cross(w, I_tens * w);
                
                % Transform back to Link Origin (Eq 3.53, 3.56 concept) [cite: 1494, 1533]
                f_k = F;
                n_k = N + cross(c, F);
                
                F_spatial(:, k) = [n_k; f_k];
            end
            
            % --- Backward Pass (Forces & Torques) ---
            % Reference: Equation (3.59) - (3.66) [cite: 1546, 1648]
            
            tau = zeros(n, 1);
            f_next = zeros(3,1);
            n_next = zeros(3,1);
            
            for k = n:-1:1
                l_next = [0;0;0]; % Setup for next link if exists
                if k < n
                    l_next = links(:, k+1);
                end
                
                % Balance forces (Eq 3.53, 3.56)
                f_curr = F_spatial(4:6, k) + f_next;
                n_curr = F_spatial(1:3, k) + n_next + cross(l_next, f_next);
                
                % Project to joint axis (Eq 3.65) [cite: 1646]
                if strcmpi(joints.type(k), 'R')
                    tau(k) = dot(n_curr, joints.axis(:, k));
                else
                    tau(k) = dot(f_curr, joints.axis(:, k));
                end
                
                f_next = f_curr;
                n_next = n_curr;
            end
        end

        %% --- CHAPTER 4: TRAJECTORY GENERATION ---
        
        function [q, qd, qdd] = traj_polynomial_3(t, t0, tf, q0, qf)
            % Cubic Polynomial Trajectory.
            % Reference: Equation (4.1) generalized for cubic
            % Note: Thesis details Linear (4.1) and Parabolic (4.5).
            % This implements the standard cubic often used alongside.
            
            T = tf - t0;
            h = qf - q0;
            
            a0 = q0;
            a1 = 0; % Assuming zero initial velocity
            a2 = 3*h / T^2;
            a3 = -2*h / T^3;
            
            dt = t - t0;
            q = a0 + a1*dt + a2*dt.^2 + a3*dt.^3;
            qd = a1 + 2*a2*dt + 3*a3*dt.^2;
            qdd = 2*a2 + 6*a3*dt;
        end

        function [q, qd, qdd] = traj_harmonic(t, t0, tf, q0, qf)
            % Harmonic Trajectory.
            % Reference: Equations (4.9), (4.10) [cite: 2597-2601]
            
            T = tf - t0;
            h = qf - q0;
            dt = t - t0;
            
            % Position (Eq 4.9)
            q = (h/2) * (1 - cos(pi * dt / T)) + q0;
            
            % Velocity (Eq 4.10)
            qd = (pi * h / (2 * T)) * sin(pi * dt / T);
            
            % Acceleration
            qdd = (pi^2 * h / (2 * T^2)) * cos(pi * dt / T);
        end

        function [q, qd, qdd] = traj_cycloidal(t, t0, tf, q0, qf)
            % Cycloidal Trajectory.
            % Reference: Equation (4.11) [cite: 2605-2606]
            
            T = tf - t0;
            h = qf - q0;
            dt = t - t0;
            
            % Position
            q = h * (dt/T - (1/(2*pi)) * sin(2*pi*dt/T)) + q0;
            
            % Velocity
            qd = (h/T) * (1 - cos(2*pi*dt/T));
            
            % Acceleration
            qdd = (2*pi*h / T^2) * sin(2*pi*dt/T);
        end
        
        function [q, qd, qdd] = traj_gutman_1_3(t, t0, tf, q0, qf)
            % Fourier Based Trajectory: Gutman 1-3.
            % Reference: Equations (4.14), (4.15) [cite: 2623-2626]
            
            T = tf - t0;
            h = qf - q0;
            dt = t - t0;
            
            K = 2*pi*dt/T;
            L = 6*pi*dt/T;
            
            % Position
            term1 = (15/(32*pi)) * sin(K);
            term2 = (1/(96*pi)) * sin(L);
            q = q0 + h * (dt/T - term1 - term2);
            
            % Velocity
            qd = (h/T) * (1 - (15/16)*cos(K) - (1/16)*cos(L));
            
            % Acceleration
            qdd = (h*pi / (8*T^2)) * (15*sin(K) + 3*sin(L));
        end
        
        function [q, qd, qdd] = traj_freudenstein_1_3_5(t, t0, tf, q0, qf)
            % Fourier Based Trajectory: Freudenstein 1-3-5.
            % Reference: Equations (4.18) - (4.20) [cite: 2646-2654]
            
            alpha = 1125/1192; % Eq 4.18
            T = tf - t0;
            h = qf - q0;
            dt = t - t0;
            
            K = 2*pi*dt/T;
            L = 6*pi*dt/T;
            M = 10*pi*dt/T;
            
            % Position (Eq 4.20)
            term = sin(K) + (1/54)*sin(L) + (1/1250)*sin(M);
            q = q0 + h*dt/T - (h/(2*pi))*alpha*term;
            
            % Velocity
            term_d = cos(K) + (1/18)*cos(L) + (1/250)*cos(M);
            qd = (h/T) * (1 - alpha*term_d);
            
            % Acceleration
            term_dd = sin(K) + (1/6)*sin(L) + (1/50)*sin(M);
            qdd = (2*pi*h/T^2) * alpha * term_dd;
        end

    end
end
