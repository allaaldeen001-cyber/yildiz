# Quick Start Guide

Get started with the SOA Robot Simulator in 5 minutes!

## Step 1: Verify Installation

Run the test script to ensure everything is working:

```matlab
test_installation
```

You should see all tests pass with ✓ marks.

## Step 2: Try the Examples

### Example A: 3-DOF Robot (Recommended First)

```matlab
example_3dof_robot
```

**What you'll see:**
- Initial and final robot configurations
- IK solution visualization
- Trajectory comparisons (Cubic, Harmonic, Cycloidal)
- Animated robot motion
- Exported data in `output/` folder

**Duration:** ~30 seconds

### Example B: SCARA Robot

```matlab
example_scara_robot
```

**What you'll see:**
- Pick-and-place operation
- Workspace visualization
- Multiple IK solutions
- Joint trajectory plots
- Animated motion

**Duration:** ~45 seconds

### Example C: Dual-Arm Collaborative System

```matlab
example_dual_arm_collaborative
```

**What you'll see:**
- 2 robots working together
- Common load handling
- Coordinated trajectories
- Shared workspace
- Synchronized animation

**Duration:** ~60 seconds

## Step 3: Create Your Own Robot

### Minimal Example

```matlab
% Define robot
robot.n_joints = 3;
robot.n_manipulators = 1;

robot.link_vectors = [
    0.5, 0.4, 0.3;  % Link lengths in x
    0,   0,   0;    % y components
    0,   0,   0     % z components
];

robot.joint_axes = [
    0, 0, 0;  % x
    0, 0, 0;  % y
    1, 1, 1   % z (all rotate about Z)
];

robot.joint_types = 'RRR';
robot.V_base = zeros(6, 1);

% Generate trajectory
q0 = [0; 0; 0];
qf = [pi/4; pi/3; pi/6];

n_samples = 50;
time = linspace(0, 5, n_samples);
q_traj = zeros(3, n_samples);

for i = 1:3
    for j = 1:n_samples
        [q, ~, ~] = ARAT_Core.traj_harmonic(time(j), 0, 5, q0(i), qf(i));
        q_traj(i, j) = q;
    end
end

% Visualize
RobotVisualizer.plot_robot(robot, q0);
title('Initial Configuration');

figure;
RobotVisualizer.plot_robot(robot, qf);
title('Final Configuration');

% Animate
RobotVisualizer.animate_trajectory(robot, q_traj, 0.05, false);
```

## Step 4: Use Interactive Mode

For full interactive input:

```matlab
robot_simulator_main
```

Follow the prompts to enter:
1. Number of joints
2. Link vectors
3. Joint types (R/P)
4. Joint axes
5. Base velocity
6. Trajectory parameters

## Common Tasks

### Task 1: Compute Forward Kinematics

```matlab
q = [pi/4; pi/6; 0];  % Joint configuration
pose = RobotIK.forward_kinematics_pose(robot, q);

position = pose(4:6);  % [x; y; z]
orientation = pose(1:3);  % axis-angle
```

### Task 2: Solve Inverse Kinematics

```matlab
target = [0.8; 0.5; 0.0];  % Target position

options.max_iter = 100;
options.tolerance = 1e-4;
options.lambda = 0.01;
options.position_only = true;

[q_solution, success, iters] = RobotIK.solve_numerical_ik(...
    robot, target, q_init, options);
```

### Task 3: Compute Jacobian

```matlab
q = [pi/4; pi/6; 0];
J = ARAT_Core.compute_jacobian(...
    robot.link_vectors, robot.joint_axes, robot.joint_types, q);

% Analyze
cond_num = cond(J);
manipulability = sqrt(det(J * J'));

fprintf('Condition number: %.2f\n', cond_num);
fprintf('Manipulability: %.4f\n', manipulability);
```

### Task 4: Generate Custom Trajectory

```matlab
% Freudenstein trajectory (smooth, minimal vibration)
n_samples = 100;
time = linspace(0, 3, n_samples);
q_start = 0;
q_end = pi/2;

q_traj = zeros(1, n_samples);
for i = 1:n_samples
    [q, qd, qdd] = ARAT_Core.traj_freudenstein_1_3_5(...
        time(i), 0, 3, q_start, q_end);
    q_traj(i) = q;
end

plot(time, q_traj);
title('Freudenstein 1-3-5 Trajectory');
```

### Task 5: Export Data

```matlab
% Export joint trajectory
writematrix(q_traj', 'my_trajectory.csv');

% Export Jacobian
J = ARAT_Core.compute_jacobian(...
    robot.link_vectors, robot.joint_axes, robot.joint_types, q);
writematrix(J, 'jacobian.csv');

% Export all matrices
for i = 1:robot.n_joints
    H = ARAT_Core.get_joint_matrix(robot.joint_types(i), robot.joint_axes(:, i));
    Phi = ARAT_Core.get_prop_matrix(robot.link_vectors(:, i));
    
    writematrix(H, sprintf('H_joint%d.csv', i));
    writematrix(Phi, sprintf('Phi_joint%d.csv', i));
end
```

## Coordinate System

The simulator uses the standard robotics convention:

- **X-axis**: Red (forward/right)
- **Y-axis**: Green (left/up)  
- **Z-axis**: Blue (up/forward in vertical)

## Joint Types

- **'R'**: Rotational joint (revolute)
  - Rotates about the specified axis
  - Units: radians
  
- **'P'**: Prismatic joint (linear)
  - Translates along the specified axis
  - Units: meters

## Trajectory Types Comparison

| Type | Smoothness | Computation | Best For |
|------|------------|-------------|----------|
| Cubic Polynomial | Good | Fast | General motion |
| Harmonic | Very Good | Fast | Smooth periodic |
| Cycloidal | Excellent | Fast | Zero jerk needed |
| Gutman 1-3 | Excellent | Medium | Reduced vibration |
| Freudenstein 1-3-5 | Best | Medium | Minimal vibration |

## Tips

1. **Start simple**: Begin with 2-3 joints
2. **Check workspace**: Use FK to verify target is reachable
3. **Normalize axes**: Joint axes are auto-normalized, but verify input
4. **IK convergence**: If IK fails, try:
   - Different initial guess
   - Larger damping (lambda = 0.1)
   - More iterations (max_iter = 500)
5. **Visualization**: Close figures between runs to save memory

## Next Steps

1. Read the full [README.md](README.md) for detailed documentation
2. Study the example scripts to understand the workflow
3. Modify examples for your specific robot
4. Explore the `ARAT_Core` mathematical methods

## Troubleshooting

**Problem**: "Undefined function or variable"
- **Solution**: Make sure all `.m` files are in MATLAB path

**Problem**: IK not converging
- **Solution**: Target may be out of workspace; try different position

**Problem**: Animation is slow
- **Solution**: Reduce `n_samples` or increase `dt` parameter

**Problem**: Figures don't appear
- **Solution**: Check `figure('Visible', 'on')` and graphics drivers

## Need Help?

Check the test results:
```matlab
test_installation
```

All tests should pass. If not, the error messages will guide you.

---

Happy robot simulation! 🤖
