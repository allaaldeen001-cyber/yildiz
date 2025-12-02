# Dual-Arm Robot Systems - User Guide

This guide explains how to use the dual-arm/multi-manipulator features for robots working in a common workspace.

## 🤖 Overview

The simulator supports **2 manipulators working together** in scenarios such as:
- **Coordinated motion** - synchronized movement
- **Common load handling** - carrying objects together
- **Assembly tasks** - meeting at target points
- **Shared workspace** - avoiding collisions

## 📁 Available Scripts

### 1. example_dual_arm_collaborative.m
**Pre-configured dual-arm demonstration**

```matlab
example_dual_arm_collaborative
```

**Features:**
- 2 x 3-DOF RRR manipulators
- Pre-defined pick-and-place operation
- Common load (2kg box)
- 8-waypoint trajectory
- Load grasping visualization
- Complete data export

**Configuration:**
- Robot 1 (Left): Base at [-0.5, 0, 0]
- Robot 2 (Right): Base at [0.5, 0, 0]
- Both have 0.95m reach
- Symmetric configuration

**Phases:**
1. Home position
2. Approach load
3. Grasp load
4. Lift load
5. Move to new position
6. Place load down
7. Release
8. Return home

### 2. dual_arm_simulator.m
**Interactive dual-arm configuration**

```matlab
dual_arm_simulator
```

**User Configures:**
- Number of joints for each robot
- Link vectors and lengths
- Joint types (R/P)
- Joint axes
- Base positions
- Task type

**Task Types:**
1. **Coordinated motion**: Synchronized trajectories
2. **Pick and place**: Shared load handling
3. **Assembly task**: Meet at target point (uses IK)
4. **Custom waypoints**: User-defined motion

## 🎯 Use Cases

### Use Case 1: Symmetric Dual-Arm System

Two identical robots placed symmetrically:

```matlab
% Robot 1 (Left side)
robot1.base_position = [-0.5; 0; 0];
robot1.link_vectors = [0.4, 0.3, 0.25; 0, 0, 0; 0, 0, 0];

% Robot 2 (Right side) - mirrored
robot2.base_position = [0.5; 0; 0];
robot2.link_vectors = [0.4, 0.3, 0.25; 0, 0, 0; 0, 0, 0];
```

**Benefits:**
- Easier trajectory planning
- Natural load sharing
- Intuitive control

### Use Case 2: Asymmetric Configuration

Different robots for specialized tasks:

```matlab
% Robot 1: Heavy-duty (longer reach)
robot1.n_joints = 4;
robot1.link_vectors = [0.6, 0.5, 0.4, 0.3; ...];
robot1.joint_types = 'RRPR';

% Robot 2: Precision (shorter, more DOF)
robot2.n_joints = 6;
robot2.link_vectors = [0.3, 0.25, 0.2, 0.15, 0.1, 0.05; ...];
robot2.joint_types = 'RRRRRR';
```

**Benefits:**
- Task specialization
- Optimized for different roles
- Flexible workspace usage

### Use Case 3: Parallel Processing

Two robots working on separate tasks in shared space:

```matlab
% Different initial/final configurations
q0_robot1 = [0; 0; 0];
qf_robot1 = [pi/4; pi/3; pi/6];

q0_robot2 = [0; 0; 0];
qf_robot2 = [-pi/4; -pi/3; -pi/6];
```

**Benefits:**
- Increased throughput
- Workspace efficiency
- Independent motion planning

## 🔧 Configuration Guide

### Step 1: Define Base Positions

Choose base positions to optimize workspace overlap:

```matlab
% Face-to-face configuration
robot1.base_position = [-0.5; 0; 0];
robot2.base_position = [0.5; 0; 0];

% Side-by-side configuration
robot1.base_position = [0; -0.5; 0];
robot2.base_position = [0; 0.5; 0];

% L-shaped configuration
robot1.base_position = [0; 0; 0];
robot2.base_position = [0.5; 0.5; 0];
```

### Step 2: Plan Trajectories

#### Synchronized Motion
Both robots move in harmony:

```matlab
% Same trajectory type, different targets
for i = 1:n_samples
    [q1, ~, ~] = ARAT_Core.traj_harmonic(t(i), 0, T, q0_r1, qf_r1);
    [q2, ~, ~] = ARAT_Core.traj_harmonic(t(i), 0, T, q0_r2, qf_r2);
end
```

#### Coordinated Load Handling
Maintain constant distance between end-effectors:

```matlab
% Define load dimensions
load_width = 0.3;  % meters

% Plan trajectories maintaining separation
for i = 1:n_samples
    % Compute end-effector positions
    ee1 = FK(robot1, q1(:,i));
    ee2 = FK(robot2, q2(:,i));
    
    % Check separation constraint
    separation = norm(ee1 - ee2);
    assert(abs(separation - load_width) < 0.01);
end
```

### Step 3: Workspace Analysis

Analyze shared workspace:

```matlab
% Sample workspace for both robots
workspace1 = sample_workspace(robot1);
workspace2 = sample_workspace(robot2);

% Find overlap
common_workspace = intersect_workspaces(workspace1, workspace2);

% Visualize
figure;
scatter3(workspace1(:,1), workspace1(:,2), workspace1(:,3), 'b.');
hold on;
scatter3(workspace2(:,1), workspace2(:,2), workspace2(:,3), 'r.');
scatter3(common_workspace(:,1), common_workspace(:,2), common_workspace(:,3), 'g.', 'SizeData', 100);
legend('Robot 1', 'Robot 2', 'Common Workspace');
```

## 📊 Data Export

Dual-arm simulations export:

```
output/
├── dual_arm_robot1_trajectory.csv      # Robot 1 joint angles
├── dual_arm_robot2_trajectory.csv      # Robot 2 joint angles
├── dual_arm_robot1_ee_positions.csv    # Robot 1 end-effector path
├── dual_arm_robot2_ee_positions.csv    # Robot 2 end-effector path
├── dual_arm_load_trajectory.csv        # Load position (if applicable)
├── dual_arm_manipulability.csv         # Jacobian analysis for both
├── dual_arm_robot1_H_joint*.csv        # H matrices robot 1
├── dual_arm_robot2_H_joint*.csv        # H matrices robot 2
├── dual_arm_robot1_Phi_joint*.csv      # Φ matrices robot 1
├── dual_arm_robot2_Phi_joint*.csv      # Φ matrices robot 2
├── dual_arm_configuration.txt          # Human-readable config
└── dual_arm_complete_data.mat          # Full MATLAB workspace
```

## 🎨 Visualization Features

### Color Coding
- **Blue**: Robot 1 (left/primary)
- **Red**: Robot 2 (right/secondary)
- **Green**: Common load / grasped region
- **Yellow**: Target positions / waypoints

### Animation Phases

The animation shows different phases:
- "Home Position"
- "Approaching Load"
- "Grasping Load" [GRASPED]
- "Lifting Load" [GRASPED]
- "Moving Load" [GRASPED]
- "Placing Load" [GRASPED]
- "Releasing"
- "Returning Home"

### Coordinate Frames
- Each joint shows RGB (XYZ) coordinate frame
- Smaller scale to avoid clutter
- Both robots visible simultaneously

## 🔬 Advanced Topics

### 1. Collision Avoidance

Add collision checking between robots:

```matlab
% Minimum safe distance
min_distance = 0.1;  % meters

for i = 1:n_samples
    % Get all link positions
    [links1, ~] = compute_all_link_positions(robot1, q_traj_1(:,i));
    [links2, ~] = compute_all_link_positions(robot2, q_traj_2(:,i));
    
    % Check all link pairs
    for j = 1:size(links1, 2)
        for k = 1:size(links2, 2)
            dist = norm(links1(:,j) - links2(:,k));
            if dist < min_distance
                warning('Collision detected at t=%.2f', time(i));
            end
        end
    end
end
```

### 2. Load Dynamics

Include load mass in dynamics:

```matlab
% Define load properties
load.mass = 2.0;  % kg
load.inertia = 0.1 * eye(3);  % kg*m^2

% Compute combined dynamics
% (Load forces distributed between robots)
force_robot1 = load.mass * g * 0.5;  % Equal sharing
force_robot2 = load.mass * g * 0.5;
```

### 3. Optimal Task Allocation

Determine which robot should do what:

```matlab
% Compute manipulability for both at target
J1 = compute_jacobian(robot1, q_target1);
J2 = compute_jacobian(robot2, q_target2);

manip1 = sqrt(det(J1 * J1'));
manip2 = sqrt(det(J2 * J2'));

% Assign task to robot with better manipulability
if manip1 > manip2
    assigned_robot = robot1;
else
    assigned_robot = robot2;
end
```

### 4. Cooperative IK

Solve IK for both robots simultaneously:

```matlab
% Target position for load center
target_center = [0.5; 0.6; 0.2];

% Grasp separation
grasp_separation = 0.3;

% Targets for each robot
target1 = target_center - [grasp_separation/2; 0; 0];
target2 = target_center + [grasp_separation/2; 0; 0];

% Solve IK for both
[q1, success1] = solve_ik(robot1, target1);
[q2, success2] = solve_ik(robot2, target2);

% Verify both succeeded
if success1 && success2
    fprintf('Cooperative IK solved\n');
end
```

## 📝 Best Practices

### 1. Trajectory Synchronization
- Use same trajectory type for both robots
- Same duration and sampling rate
- Coordinate waypoint timing

### 2. Workspace Design
- Place bases to maximize overlap
- Consider reach limitations
- Plan for load transfer zones

### 3. Load Handling
- Maintain stable grasp geometry
- Distribute forces evenly
- Avoid singular configurations

### 4. Safety
- Add virtual boundaries
- Implement collision checking
- Monitor manipulability metrics

### 5. Performance
- Use sparse Jacobian computations
- Cache repeated calculations
- Optimize animation frame rate

## 🐛 Troubleshooting

### Problem: Robots colliding
**Solution:** 
- Increase base separation
- Adjust trajectory waypoints
- Add collision detection

### Problem: IK not converging for both robots
**Solution:**
- Check targets are in both workspaces
- Try different initial guesses
- Increase IK iterations

### Problem: Load appears to "float"
**Solution:**
- Verify grasp state tracking
- Check end-effector position calculations
- Ensure base positions included

### Problem: Uncoordinated motion
**Solution:**
- Use same trajectory type
- Verify timing synchronization
- Check waypoint definitions

## 📚 Example Workflows

### Workflow 1: Quick Dual-Arm Demo

```matlab
% 1. Run pre-configured example
example_dual_arm_collaborative

% 2. Watch animation
% 3. Check output/ folder for data
% 4. Analyze plots
```

### Workflow 2: Custom Configuration

```matlab
% 1. Start interactive simulator
dual_arm_simulator

% 2. Enter robot 1 parameters
% 3. Mirror or configure robot 2
% 4. Select task type
% 5. Define trajectories
% 6. View results
```

### Workflow 3: Research Application

```matlab
% 1. Define custom robots
robot1 = define_robot_1();
robot2 = define_robot_2();

% 2. Generate optimal trajectories
[q1, q2] = optimize_dual_arm_trajectory(robot1, robot2, task);

% 3. Analyze performance
metrics = analyze_dual_arm_performance(robot1, robot2, q1, q2);

% 4. Export for publication
export_data_for_paper(metrics);
```

## 🎓 Learning Path

1. **Beginner**: Run `example_dual_arm_collaborative.m`
2. **Intermediate**: Use `dual_arm_simulator.m` with different tasks
3. **Advanced**: Modify examples for custom applications
4. **Expert**: Implement cooperative control algorithms

## 📞 Support

For dual-arm specific questions:
1. Check this guide first
2. Review example code comments
3. See main [README.md](README.md)
4. Examine exported data structure

---

**Ready to start?** Run:
```matlab
example_dual_arm_collaborative
```

---

*Last Updated: December 2, 2025*
