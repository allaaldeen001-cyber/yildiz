# SOA Robot Manipulator Simulator

A comprehensive MATLAB project for simulating robot manipulators using **Spatial Operator Algebra (SOA)**. This toolbox supports arbitrary configurations with any number of joints, links, and manipulators.

## 🎯 Features

- ✅ **Spatial Operator Algebra (SOA)** implementation based on academic research
- ✅ **Forward Kinematics** with velocity propagation
- ✅ **Inverse Kinematics** using Damped Least Squares (Levenberg-Marquardt)
- ✅ **Multiple Trajectory Types**: Cubic Polynomial, Harmonic, Cycloidal, Gutman 1-3, Freudenstein 1-3-5
- ✅ **3D Visualization & Animation** with coordinate frames
- ✅ **Jacobian Computation** using SOA propagation matrices
- ✅ **Dynamic Analysis** using Recursive Newton-Euler
- ✅ **Support for any joint configuration**: Rotational (R) and Prismatic (P)
- ✅ **Data Export**: Joint angles (Θ), Jacobian matrices, H matrices (joint maps), Φ matrices (propagation)

## 📁 Project Structure

```
/workspace/
├── 📘 DOCUMENTATION
│   ├── README.md                      # Complete documentation (this file)
│   ├── QUICKSTART.md                  # Quick start guide
│   ├── PROJECT_SUMMARY.md             # Project overview
│   └── INDEX.md                       # File navigation
│
├── 🧮 CORE LIBRARIES
│   ├── ARAT_Core.m                    # SOA mathematical core
│   ├── RobotIK.m                      # Inverse kinematics solver
│   └── RobotVisualizer.m              # Visualization tools
│
├── 🎮 MAIN SCRIPTS
│   └── robot_simulator_main.m         # Interactive simulator
│
├── 📚 EXAMPLES
│   ├── example_3dof_robot.m           # 3-DOF RRR example
│   ├── example_scara_robot.m          # SCARA RRPR example
│   ├── example_complete_workflow.m    # Complete demo
│   ├── example_dual_arm_collaborative.m  # Dual-arm with common load
│   ├── dual_arm_simulator.m           # Interactive dual-arm
│   └── robot_config_template.m        # Configuration templates
│
├── 🧪 TESTING
│   └── test_installation.m            # Installation test suite
│
└── 📂 OUTPUT (generated at runtime)
    └── output/                        # Exported data
```

## 🚀 Quick Start

### Option 1: Interactive Simulator

Run the main interactive simulator:

```matlab
robot_simulator_main
```

You'll be prompted to enter:
1. Number of joints (n)
2. Link vectors [x, y, z] for each joint
3. Joint types (R or P)
4. Joint axes (rotation/translation direction)
5. Base velocity (ω₀, v₀)
6. Number of manipulators (p)
7. Trajectory parameters
8. Target positions for IK (optional)

### Option 2: Pre-configured Examples

#### 3-DOF Robot (RRR Configuration)
```matlab
example_3dof_robot
```
- 3 rotational joints
- Demonstrates trajectory generation and comparison
- Shows IK solution
- Exports all matrices

#### SCARA Robot (RRPR Configuration)
```matlab
example_scara_robot
```
- 2 rotational + 1 prismatic + 1 rotational
- Pick-and-place operation
- Workspace analysis
- Multiple IK tests

#### Dual-Arm Collaborative System
```matlab
example_dual_arm_collaborative
```
- 2 manipulators working together
- Common load handling
- Coordinated motion
- Shared workspace visualization

#### Interactive Dual-Arm Simulator
```matlab
dual_arm_simulator
```
- Configure 2 robots interactively
- Multiple task types (coordinated, pick-place, assembly)
- Synchronized trajectory generation

## 📚 Mathematical Foundation

### Spatial Operator Algebra (SOA)

The implementation is based on the thesis:
*"Development of a Toolbox for Kinematic and Dynamic Modeling of Multibody and High Degree of Freedom Robotic Systems"*

#### Key Equations

**1. Velocity Propagation Matrix (Φ)**
```
Φ_{k,k-1} = [I    0  ]
            [-l̂   I  ]
```
Where `l̂` is the skew-symmetric matrix of link vector `l`.

**2. Joint Map Matrix (H)**
- Rotational: `H = [h; 0]`
- Prismatic: `H = [0; h]`

**3. Forward Kinematics**
```
V_k = Φ_{k,k-1} · V_{k-1} + H_k · q̇_k
```

**4. Jacobian Matrix**
```
J = [Φ_{tip,1}·H_1 | Φ_{tip,2}·H_2 | ... | Φ_{tip,n}·H_n]
```

**5. Rodrigues' Rotation Formula**
```
R = I + sin(θ)·ŵ + (1-cos(θ))·ŵ²
```

## 🎨 Trajectory Generation Methods

The simulator supports 5 different trajectory generation methods:

| Method | Description | Characteristics |
|--------|-------------|-----------------|
| **Cubic Polynomial** | 3rd-order polynomial | Smooth, zero initial/final velocity |
| **Harmonic** | Sinusoidal motion | Very smooth, continuous acceleration |
| **Cycloidal** | Cycloid curve | Zero jerk at endpoints |
| **Gutman 1-3** | Fourier series (1st & 3rd harmonics) | Reduced vibration |
| **Freudenstein 1-3-5** | Fourier series (1st, 3rd & 5th) | Minimal residual vibration |

## 🔧 Core Components

### ARAT_Core Class

Static methods for SOA operations:

```matlab
% Skew-symmetric matrix
S = ARAT_Core.skew(v);

% Rodrigues rotation
R = ARAT_Core.rodrigues_rotation(axis, angle);

% Propagation matrix
Phi = ARAT_Core.get_prop_matrix(link_vector);

% Joint matrix
H = ARAT_Core.get_joint_matrix('R', axis);

% Forward kinematics
V_tip = ARAT_Core.serial_forward_kinematics(links, axes, types, q, q_dot, V_base);

% Jacobian
J = ARAT_Core.compute_jacobian(links, axes, types, q);

% Trajectory generation
[q, qd, qdd] = ARAT_Core.traj_harmonic(t, t0, tf, q0, qf);
```

### RobotIK Class

Inverse kinematics solver:

```matlab
% Define robot
robot.link_vectors = [...];
robot.joint_axes = [...];
robot.joint_types = 'RRR';

% Set target
target_pos = [x; y; z];

% Solve
options.max_iter = 100;
options.tolerance = 1e-4;
options.lambda = 0.01;
[q_solution, success, iterations] = RobotIK.solve_numerical_ik(robot, target_pos, q_init, options);
```

### RobotVisualizer Class

Visualization tools:

```matlab
% Plot robot
RobotVisualizer.plot_robot(robot, q);

% Animate trajectory
RobotVisualizer.animate_trajectory(robot, q_trajectory, dt, save_video);

% Plot joint trajectories
RobotVisualizer.plot_joint_trajectories(q_trajectory, time_vector);
```

## 📊 Data Export

The simulator automatically exports:

### 1. Joint Angles (Θ)
- **File**: `output/joint_angles_theta.csv`
- **Format**: Each row is a time step, columns are joints

### 2. Jacobian Matrices
- **Files**: `output/jacobian_t*.csv`
- **Format**: One file per time step (6×n matrix)

### 3. Joint Map Matrices (H)
- **Files**: `output/H_matrix_joint*.csv`
- **Format**: One file per joint (6×1 vector)

### 4. Propagation Matrices (Φ)
- **Files**: `output/Phi_matrix_joint*.csv`
- **Format**: One file per joint (6×6 matrix)

### 5. Complete Simulation Data
- **File**: `output/robot_simulation_data.mat`
- **Contents**: All variables in MATLAB format

## 🤖 Creating Custom Robots

### Example: Custom 4-DOF Robot

```matlab
% Robot configuration
robot.n_joints = 4;
robot.n_manipulators = 1;

% Link vectors (from joint i-1 to joint i)
robot.link_vectors = [
    0.5, 0.3, 0.2, 0.1;  % x
    0.0, 0.0, 0.0, 0.0;  % y
    0.2, 0.0, 0.0, 0.0   % z
];

% Joint axes (normalized automatically)
robot.joint_axes = [
    0, 0, 1, 0;  % x
    0, 0, 0, 0;  % y
    1, 1, 0, 1   % z
];

% Joint types
robot.joint_types = 'RRPR';

% Base velocity (stationary base)
robot.V_base = zeros(6, 1);

% Now use with any function
J = ARAT_Core.compute_jacobian(robot.link_vectors, robot.joint_axes, ...
                                robot.joint_types, q);
```

## 🎓 Understanding the Output

### Jacobian Matrix
The 6×n Jacobian relates joint velocities to end-effector velocity:
```
[ω]   [J_ω]
[v] = [J_v] · q̇
```
- Top 3 rows (J_ω): Angular velocity mapping
- Bottom 3 rows (J_v): Linear velocity mapping

### Manipulability
Measure of how well the robot can move in all directions:
```matlab
manipulability = sqrt(det(J * J'));
```
Higher values = better dexterity

### Condition Number
Measure of how close the robot is to a singularity:
```matlab
cond_number = cond(J);
```
Lower values = better (< 10 is good, > 100 is concerning)

## 🎬 Animation

The visualizer creates animations showing:
- 3D robot structure with links and joints
- Coordinate frames at each joint (RGB = XYZ)
- End-effector trajectory trace (green dashed line)
- Joint spheres (red) and end-effector marker (green square)

Save animations as MP4:
```matlab
RobotVisualizer.animate_trajectory(robot, q_trajectory, 0.05, true);
```

## 📈 Advanced Usage

### Multi-Manipulator Systems

For parallel or multi-arm configurations:
```matlab
robot.n_manipulators = 2;  % Two arms
% Define each manipulator's parameters separately
```

### Custom Trajectory

Generate your own trajectory:
```matlab
n_samples = 100;
q_trajectory = zeros(n_joints, n_samples);

for i = 1:n_joints
    for j = 1:n_samples
        t = time_vector(j);
        [q, qd, qdd] = ARAT_Core.traj_harmonic(t, 0, 5, q0(i), qf(i));
        q_trajectory(i, j) = q;
    end
end
```

### Workspace Analysis

Sample the workspace:
```matlab
workspace_points = [];
for q1 = linspace(-pi, pi, 50)
    for q2 = linspace(-pi/2, pi/2, 50)
        q = [q1; q2; ...];
        pose = RobotIK.forward_kinematics_pose(robot, q);
        workspace_points = [workspace_points, pose(4:6)];
    end
end

% Visualize
scatter3(workspace_points(1,:), workspace_points(2,:), workspace_points(3,:));
```

## 🔍 Troubleshooting

### IK Not Converging
- Increase `max_iter` (try 500)
- Adjust damping factor `lambda` (try 0.001 to 0.1)
- Ensure target is within workspace
- Try different initial guess `q_init`

### Visualization Issues
- Check MATLAB graphics drivers: `opengl info`
- Try software rendering: `opengl software`
- Reduce animation frame rate

### Singular Configurations
- Check Jacobian condition number
- Avoid extended/fully retracted configurations
- Add joint limits to prevent singularities

## 📝 Requirements

- MATLAB R2020a or newer
- No additional toolboxes required
- Recommended: MATLAB R2021b+ for better graphics

## 🔗 References

Based on academic research in:
- Spatial Operator Algebra
- Recursive robot dynamics
- Trajectory optimization
- Kinematic analysis

## 📄 License

This is an educational implementation for research and learning purposes.

## 👥 Contributing

To extend this toolbox:
1. Add new trajectory methods in `ARAT_Core`
2. Implement analytical IK for specific robot types in `RobotIK`
3. Add visualization modes in `RobotVisualizer`

## 🆘 Support

For issues or questions:
1. Check the examples first
2. Review the mathematical equations
3. Verify robot configuration parameters
4. Check exported data in `output/` folder

---

**Happy Simulating! 🤖**
