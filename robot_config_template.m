% ROBOT_CONFIG_TEMPLATE
% Template for defining robot configurations
% Copy and modify this file for your custom robot

function robot = robot_config_template()
    % This function returns a robot structure with all required fields
    
    %% BASIC CONFIGURATION
    robot.n_joints = 3;              % Number of joints
    robot.n_manipulators = 1;        % Number of manipulators
    
    %% LINK VECTORS
    % Each column represents the vector from joint i-1 to joint i
    % Format: [x1, x2, x3, ...;
    %          y1, y2, y3, ...;
    %          z1, z2, z3, ...]
    
    robot.link_vectors = [
        0.5,  0.4,  0.3;    % x components (meters)
        0.0,  0.0,  0.0;    % y components (meters)
        0.0,  0.0,  0.0     % z components (meters)
    ];
    
    %% JOINT AXES
    % Each column represents the axis of motion for joint i
    % For rotational joints: rotation axis (will be normalized)
    % For prismatic joints: translation direction (will be normalized)
    
    robot.joint_axes = [
        0,  0,  0;      % x components
        0,  0,  0;      % y components
        1,  1,  1       % z components
    ];
    
    % Alternative examples:
    % X-axis rotation: [1; 0; 0]
    % Y-axis rotation: [0; 1; 0]
    % Z-axis rotation: [0; 0; 1]
    % Arbitrary axis:  [1; 1; 0] (will be normalized to [0.707; 0.707; 0])
    
    %% JOINT TYPES
    % String of length n_joints
    % 'R' = Rotational (revolute)
    % 'P' = Prismatic (linear)
    
    robot.joint_types = 'RRR';
    
    % Examples:
    % 'RRR'    - 3 rotational joints (common robot arm)
    % 'RRPR'   - SCARA robot
    % 'RRP'    - Cylindrical robot
    % 'PPP'    - Cartesian/gantry robot
    % 'RRRRRR' - 6-DOF industrial robot
    
    %% BASE VELOCITY
    % 6x1 vector: [angular_velocity; linear_velocity]
    % Usually [0; 0; 0; 0; 0; 0] for fixed base
    
    robot.V_base = zeros(6, 1);
    
    % For moving base (e.g., mobile manipulator):
    % robot.V_base = [0; 0; 0.1;    % w_x, w_y, w_z (rad/s)
    %                 0.5; 0; 0];   % v_x, v_y, v_z (m/s)
    
    %% OPTIONAL: JOINT LIMITS
    % Nx2 matrix: [min_1, max_1;
    %              min_2, max_2;
    %              ...]
    % For rotational: radians
    % For prismatic: meters
    
    robot.joint_limits = [
        -pi,    pi;       % Joint 1
        -pi/2,  pi/2;     % Joint 2
        -pi/2,  pi/2      % Joint 3
    ];
    
    %% OPTIONAL: MASS PARAMETERS (for dynamics)
    % Array of structs with fields: m, c, I
    % m: mass (kg)
    % c: center of mass vector from joint origin (3x1, meters)
    % I: inertia tensor (3x3, kg*m^2)
    
    robot.mass_params = struct();
    
    for i = 1:robot.n_joints
        robot.mass_params(i).m = 1.0;           % 1 kg
        robot.mass_params(i).c = [0; 0; 0];     % At joint origin
        robot.mass_params(i).I = 0.01 * eye(3); % Simple inertia
    end
    
    %% OPTIONAL: DH PARAMETERS (for reference)
    % Standard Denavit-Hartenberg parameters
    % Format: [a, alpha, d, theta] for each joint
    
    robot.DH_params = [
        % a      alpha    d      theta
        0.5,     0,       0,     0;
        0.4,     0,       0,     0;
        0.3,     0,       0,     0
    ];
    
    %% METADATA
    robot.name = 'Custom Robot';
    robot.description = '3-DOF RRR manipulator';
    robot.author = 'Your Name';
    robot.date_created = datestr(now);
    
end

%% EXAMPLE USAGE
%{
    % Load robot configuration
    robot = robot_config_template();
    
    % Test forward kinematics
    q = [pi/4; pi/6; pi/3];
    pose = RobotIK.forward_kinematics_pose(robot, q);
    fprintf('End-effector position: [%.3f, %.3f, %.3f]\n', pose(4), pose(5), pose(6));
    
    % Compute Jacobian
    J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                    robot.joint_types, q);
    fprintf('Jacobian computed: %dx%d\n', size(J, 1), size(J, 2));
    
    % Visualize
    RobotVisualizer.plot_robot(robot, q);
%}

%% PREDEFINED ROBOT CONFIGURATIONS

function robot = puma560_config()
    % PUMA 560 style 6-DOF robot
    robot.n_joints = 6;
    robot.n_manipulators = 1;
    
    robot.link_vectors = [
        0,    0.4318,  0.0203,  0,      0,     0;
        0,    0,       0,       0.4331, 0,     0;
        0.2435, 0,     0,       0,      0,     0
    ];
    
    robot.joint_axes = [
        0, 0, 0, 0, 0, 0;
        0, 1, 1, 0, 1, 0;
        1, 0, 0, 1, 0, 1
    ];
    
    robot.joint_types = 'RRRRRR';
    robot.V_base = zeros(6, 1);
    robot.name = 'PUMA 560';
end

function robot = stanford_arm_config()
    % Stanford Arm (RRPRR)
    robot.n_joints = 5;
    robot.n_manipulators = 1;
    
    robot.link_vectors = [
        0,   0,   0,   0,   0;
        0,   0,   0,   0,   0;
        0.1, 0,   0,   0,   0.1
    ];
    
    robot.joint_axes = [
        0, 0, 0, 0, 1;
        0, 1, 0, 1, 0;
        1, 0, 1, 0, 0
    ];
    
    robot.joint_types = 'RRPRR';
    robot.V_base = zeros(6, 1);
    robot.name = 'Stanford Arm';
end

function robot = cartesian_robot_config()
    % 3-axis Cartesian (PPP) robot
    robot.n_joints = 3;
    robot.n_manipulators = 1;
    
    robot.link_vectors = zeros(3, 3);
    
    robot.joint_axes = [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
    ];
    
    robot.joint_types = 'PPP';
    robot.V_base = zeros(6, 1);
    robot.name = 'Cartesian Robot';
end
