# Collaborative Dual-Arm System Guide

## Overview

This guide explains how to use the **Collaborative Dual-Arm System** for simulating two robots working together with a common load. The system is specifically designed for **cooperative manipulation tasks** where both robots must coordinate to handle a shared object.

---

## 📋 Table of Contents

1. [What is Collaborative Manipulation?](#what-is-collaborative-manipulation)
2. [System Architecture](#system-architecture)
3. [Quick Start](#quick-start)
4. [Configuration Guide](#configuration-guide)
5. [Task Types](#task-types)
6. [Understanding the Results](#understanding-the-results)
7. [Advanced Topics](#advanced-topics)
8. [Troubleshooting](#troubleshooting)

---

## What is Collaborative Manipulation?

**Collaborative manipulation** involves two or more robots working together to:
- **Grasp and transport** a common load
- **Share forces** during manipulation
- **Synchronize motion** to maintain stability
- **Coordinate trajectories** to avoid collisions

### Key Differences from Independent Robots

| **Independent Robots** | **Collaborative Robots** |
|------------------------|--------------------------|
| Work in separate workspaces | Share a common workspace |
| Independent trajectories | Synchronized trajectories |
| No force coordination | Shared load and forces |
| Simple coordination | Complex cooperation |

---

## System Architecture

The collaborative dual-arm system consists of **TWO FILES**:

### 1. `Collaborative_DualArm_Functions.m` (Backend)

Contains all the analysis logic:
- ✅ Coordinated task planning
- ✅ Synchronized trajectory generation
- ✅ Forward kinematics for both robots
- ✅ Load trajectory computation
- ✅ Jacobian and manipulability analysis
- ✅ Coordination quality metrics
- ✅ Cooperative motion verification
- ✅ Visualization and animation
- ✅ Data export

### 2. `Collaborative_DualArm_Interface.m` (Frontend)

User interface where you:
- 📝 Define both robot parameters
- 📝 Specify load properties
- 📝 Select task type and trajectory
- 📝 Configure output options
- ▶️ Run the script
- 📊 Get complete analysis automatically

---

## Quick Start

### Step 1: Open the Interface File

```matlab
open Collaborative_DualArm_Interface.m
```

### Step 2: Configure Your System

Edit the **CONFIGURATION** section:

```matlab
%% ---------- ROBOT 1 PARAMETERS ----------
robot1.link_vectors = [0, 0, 0; 0, 0, 0; 0.3, 0.3, 0.25];
robot1.joint_types = 'RRR';
robot1.base_position = [-0.5; 0; 0];
robot1.q_initial = [0; 0; 0];
robot1.q_final = [pi/4; pi/4; pi/4];

%% ---------- ROBOT 2 PARAMETERS ----------
robot2.link_vectors = [0, 0, 0; 0, 0, 0; 0.3, 0.3, 0.25];
robot2.joint_types = 'RRR';
robot2.base_position = [0.5; 0; 0];
robot2.q_initial = [0; 0; 0];
robot2.q_final = [-pi/4; pi/4; -pi/4];

%% ---------- LOAD PARAMETERS ----------
load_params.mass = 5.0;  % kg
load_params.size = [0.3, 0.2, 0.15];  % [width, depth, height] m

%% ---------- TASK ----------
task_params.task_type = 'pick_and_place';
task_params.duration = 10.0;
```

### Step 3: Run the Script

```matlab
run Collaborative_DualArm_Interface.m
```

### Step 4: View Results

The system will automatically:
1. ✅ Validate both robots
2. ✅ Generate coordinated trajectories
3. ✅ Compute kinematics and dynamics
4. ✅ Verify cooperative motion
5. ✅ Create visualizations
6. ✅ Export data to `output/` folder

---

## Configuration Guide

### Robot Parameters

#### Link Vectors (3×N matrix)

Each column is a vector from joint `i-1` to joint `i`:

```matlab
robot1.link_vectors = [
    x1, x2, x3, ..., xn;    % X components
    y1, y2, y3, ..., yn;    % Y components
    z1, z2, z3, ..., zn     % Z components
];
```

**Example:** 3-DOF robot with vertical links of 0.3m each:
```matlab
robot1.link_vectors = [
    0,   0,   0;     % No X displacement
    0,   0,   0;     % No Y displacement
    0.3, 0.3, 0.3    % Vertical links
];
```

#### Joint Axes (3×N matrix)

Rotation axes for each joint (normalized automatically):

```matlab
robot1.joint_axes = [
    0, 0, 0;    % X components
    0, 0, 0;    % Y components
    1, 1, 1     % Z components (all joints rotate about Z)
];
```

#### Joint Types

String with one character per joint:
- `'R'` = Revolute (rotational)
- `'P'` = Prismatic (linear) - *currently only revolute supported for collaborative*

```matlab
robot1.joint_types = 'RRRRRR';  % 6 revolute joints
```

#### Base Position

Location of robot base in world coordinates:

```matlab
robot1.base_position = [-0.6; 0; 0];  % Left side
robot2.base_position = [0.6; 0; 0];   % Right side
```

**Important:** Robots should be positioned close enough to reach the shared workspace!

#### Joint Configurations

Initial and final joint angles (radians):

```matlab
robot1.q_initial = [0; 0; 0; 0; 0; 0];  % Starting pose
robot1.q_final = [pi/4; pi/6; pi/4; pi/6; pi/4; pi/6];  % Ending pose
```

### Load Parameters

#### Mass (kg)

```matlab
load_params.mass = 5.0;  % 5 kg object
```

#### Size (meters)

Dimensions `[width, depth, height]`:

```matlab
load_params.size = [0.3, 0.2, 0.15];  % 30cm × 20cm × 15cm box
```

The width should match the distance between the robot end-effectors when grasping!

### Task Parameters

#### Task Type

```matlab
task_params.task_type = 'pick_and_place';
```

**Available options:**
- `'pick_and_place'` - Complete pick-and-place cycle (8 phases)
- `'transport'` - Transport object from A to B (6 phases)
- `'assembly'` - Assembly operation (5 phases)

See [Task Types](#task-types) for details.

#### Trajectory Type

```matlab
task_params.trajectory_type = 2;
```

**Options:**
- `1` = Cubic Polynomial (smooth)
- `2` = Harmonic (very smooth, sinusoidal)
- `3` = Cycloidal (smooth acceleration)

#### Duration and Sampling

```matlab
task_params.duration = 10.0;      % Total time (seconds)
task_params.n_samples = 200;      % Number of trajectory points
```

### Output Options

```matlab
export_prefix = 'my_collaborative_task';  % Prefix for saved files
create_plots = true;                       % Generate figures
export_data = true;                        % Save CSV files
create_animation = true;                   % Create animation (slow!)
```

---

## Task Types

### 1. Pick-and-Place (8 phases)

Complete cycle for picking up and placing an object:

| **Phase** | **Description** | **Grasp State** |
|-----------|----------------|-----------------|
| 1 | Home position | ❌ Free |
| 2 | Approach load | ❌ Free |
| 3 | Grasp load | ✅ Grasped |
| 4 | Lift load | ✅ Grasped |
| 5 | Transport load | ✅ Grasped |
| 6 | Place load | ✅ Grasped |
| 7 | Release load | ❌ Free |
| 8 | Return home | ❌ Free |

**Best for:** Full manipulation cycles, testing coordination throughout operation

### 2. Transport (6 phases)

Simplified transport from A to B:

| **Phase** | **Description** | **Grasp State** |
|-----------|----------------|-----------------|
| 1 | Approach | ❌ Free |
| 2 | Grasp | ✅ Grasped |
| 3 | Move 1 | ✅ Grasped |
| 4 | Move 2 | ✅ Grasped |
| 5 | Move 3 | ✅ Grasped |
| 6 | Release | ❌ Free |

**Best for:** Simple transport tasks, testing sustained cooperation

### 3. Assembly (5 phases)

Assembly or insertion tasks:

| **Phase** | **Description** | **Grasp State** |
|-----------|----------------|-----------------|
| 1 | Approach parts | ❌ Free |
| 2 | Grasp part | ✅ Grasped |
| 3 | Align | ✅ Grasped |
| 4 | Insert/Join | ✅ Grasped |
| 5 | Release | ❌ Free |

**Best for:** Precision tasks, alignment operations

---

## Understanding the Results

### Console Output

The system prints detailed progress:

```
╔════════════════════════════════════════════╗
║  COLLABORATIVE DUAL-ARM SYSTEM ANALYSIS    ║
║  Two Robots Working with Common Load       ║
╚════════════════════════════════════════════╝

--- System Validation ---
✓ Both robots validated
  Robot 1: 6-DOF at [-0.60, 0.00, 0.00]
  Robot 2: 6-DOF at [0.60, 0.00, 0.00]
  Common load: 5.0 kg, [0.30 × 0.20 × 0.15] m

--- Planning Coordinated Motion ---
✓ 8 waypoints planned
  Task: pick_and_place

--- Generating Synchronized Trajectories ---
✓ Synchronized trajectories generated

--- Computing Forward Kinematics ---
✓ FK computed for both robots

--- Computing Load Trajectory ---
✓ Load trajectory computed

--- Jacobian Analysis ---
✓ Jacobian analysis complete

--- Computing System Matrices ---
✓ H and Phi matrices computed

--- Coordination Analysis ---
✓ Coordination metrics computed

--- Verifying Cooperative Motion ---

========================================
COOPERATIVE MOTION VERIFICATION
========================================

GRASP STABILITY:
  Expected EE separation: 0.300 m
  Actual avg separation: 0.298 m
  Max deviation: 0.015 m
  Status: ✓ STABLE

FORCE BALANCE:
  Avg imbalance: 0.0234
  Status: ✓ BALANCED

VELOCITY SYNCHRONIZATION:
  Avg velocity difference: 0.0187 m/s
  Status: ✓ SYNCHRONIZED

OVERALL COOPERATIVE PERFORMANCE:
  ✓✓✓ EXCELLENT COOPERATION ✓✓✓
```

### Visualization Outputs

#### Figure 1: Collaborative Workspace

Shows:
- **Blue dashed line**: Robot 1 end-effector path
- **Red dashed line**: Robot 2 end-effector path
- **Green solid line**: Load trajectory (when grasped)
- **Robot structures**: Shown at 3 key positions (initial, mid, final) with:
  - Thick colored links
  - Spherical joints
  - Large end-effector spheres
- **Load boxes**: Orange/gold semi-transparent boxes
- **Blue/Red base markers**: Robot base positions

**What to look for:**
- ✅ Smooth, coordinated paths
- ✅ Load trajectory follows EE paths during grasp
- ✅ Paths don't collide
- ✅ Robot configurations show realistic motion
- ✅ Load stays between end-effectors when grasped

#### Figure 2: Coordination Analysis

4 subplots:

1. **End-Effector Separation**
   - Distance between EEs over time
   - Green region = grasped
   - Should be constant during grasp = load width

2. **Force Balance**
   - Lower is better (0 = perfect balance)
   - Should be < 0.1 for good cooperation

3. **Velocity Synchronization**
   - Difference in EE velocities
   - Should be near 0 during grasp

4. **Manipulability Comparison**
   - Blue = Robot 1
   - Red = Robot 2
   - Higher is better (easier motion)

#### Figure 3: Joint Trajectories

Shows joint angles over time for both robots:
- **Blue solid**: Robot 1
- **Red dashed**: Robot 2
- **Green shaded**: Grasp phases

**What to look for:**
- ✅ Smooth trajectories (no jumps)
- ✅ Synchronized motion during grasp
- ✅ Joint limits respected

#### Animation: Real-Time Motion Visualization ⭐ NEW!

The system now includes a **fully animated 3D visualization** showing:

**Robot Visualization:**
- **Complete robot structures** with all links rendered
- **Spherical joints** at each joint location
- **Large end-effector markers** for easy tracking
- **Thick colored links** (blue for Robot 1, red for Robot 2)

**Load Visualization:**
- **3D box** representing the common load
- **Orange/gold color** with semi-transparency
- **Only visible when grasped**
- **Green connection lines** from EEs to load center

**Motion Features:**
- **Trajectory traces** (dotted lines showing past motion)
- **Real-time status** showing time and grasp state
- **~100 frames** for smooth animation
- **Consistent axis scaling** throughout

**Status Display:**
- `🤝 LOAD GRASPED` (green text) - robots holding load
- `✋ LOAD FREE` (gray text) - load released

**To Enable/Disable:**
```matlab
% In Collaborative_DualArm_Interface.m
create_animation = true;   % Enable animation
create_animation = false;  % Disable (faster)
```

**Performance:**
- Animation takes 10-30 seconds depending on trajectory length
- Uses ~100 frames (automatically adjusted)
- 50ms pause between frames for smooth playback

### Exported Data

All files saved to `output/` directory:

| **File** | **Contents** |
|----------|-------------|
| `*_robot1_joints.csv` | Time, q1, q2, ..., qn for Robot 1 |
| `*_robot2_joints.csv` | Time, q1, q2, ..., qn for Robot 2 |
| `*_robot1_ee.csv` | Time, x, y, z of Robot 1 EE |
| `*_robot2_ee.csv` | Time, x, y, z of Robot 2 EE |
| `*_load_trajectory.csv` | Time, x, y, z, grasp_state |
| `*_coordination.csv` | Time, separation, force_balance, velocity_sync |
| `*_summary.txt` | Text summary of all metrics |

---

## Advanced Topics

### Custom Task Sequences

To create your own task sequence, modify `Collaborative_DualArm_Functions.m`:

```matlab
function [wp1, wp2, grasp] = plan_custom_task(robot1, robot2, task_params)
    n1 = size(robot1.link_vectors, 2);
    n2 = size(robot2.link_vectors, 2);
    
    % Define 4 waypoints
    wp1 = zeros(n1, 4);
    wp2 = zeros(n2, 4);
    grasp = [false, true, true, false];
    
    % Define waypoint configurations
    wp1(:, 1) = [...];  % Start
    wp1(:, 2) = [...];  % Grasp
    wp1(:, 3) = [...];  % Move
    wp1(:, 4) = [...];  % Release
    
    % Same for robot 2
    % ...
end
```

Then add your task to the `plan_coordinated_task` switch statement.

### Force Distribution Analysis

The system computes load forces:

```matlab
results.load.force_robot1  % Force on Robot 1 (Nx3)
results.load.force_robot2  % Force on Robot 2 (Nx3)
```

Currently assumes **equal force sharing** + gravity. For unequal distribution:

```matlab
% Custom force distribution
weight_robot1 = 0.6;  % Robot 1 carries 60%
weight_robot2 = 0.4;  % Robot 2 carries 40%

g_force = [0; 0; -load_params.mass * 9.81];
force_robot1(:, k) = g_force * weight_robot1;
force_robot2(:, k) = g_force * weight_robot2;
```

### Manipulability Optimization

To ensure both robots maintain good manipulability:

```matlab
% Check minimum manipulability
min_manip_1 = min(results.robot1.manipulability);
min_manip_2 = min(results.robot2.manipulability);

if min_manip_1 < 0.01 || min_manip_2 < 0.01
    warning('Low manipulability detected - consider adjusting waypoints');
end
```

### Collision Avoidance

Currently **not implemented**. For collision checking:

1. Compute minimum distance between robot links at each timestep
2. Set safety threshold (e.g., 0.05 m)
3. Warn if distance < threshold

---

## Troubleshooting

### Issue: "Robot validation failed"

**Cause:** Mismatched dimensions

**Solution:** Check that for each robot:
- `link_vectors` is 3×N
- `joint_axes` is 3×N
- `joint_types` has length N
- `q_initial` has length N
- `q_final` has length N

### Issue: "Grasp unstable"

**Symptom:** Verification shows `✗ UNSTABLE`

**Causes:**
1. Load width ≠ EE separation
2. Waypoints don't maintain consistent grasp
3. Trajectory too fast

**Solutions:**
- Set `load_params.size(1)` = expected EE separation during grasp
- Smooth waypoint transitions
- Increase `task_params.duration`

### Issue: "Forces imbalanced"

**Symptom:** `force_balance` metric is high (> 0.1)

**Causes:**
1. Asymmetric motion
2. Different robot configurations
3. Unequal manipulability

**Solutions:**
- Design symmetric waypoints for both robots
- Match robot DOF and link lengths
- Check manipulability plots for imbalance

### Issue: "Velocities not synchronized"

**Symptom:** `velocity_sync` metric is high

**Causes:**
1. Different trajectory scaling
2. Waypoint timing mismatch
3. One robot reaching limit

**Solutions:**
- Use same trajectory type for both robots
- Ensure waypoints are reachable
- Check condition number (high = near singularity)

### Issue: Animation is slow

**Symptom:** `create_animation = true` takes forever

**Causes:**
- High `n_samples`
- Complex visualization
- MATLAB rendering speed

**Solutions:**
- Reduce `n_samples` to ~50 for animation
- Disable grid/labels during animation
- Use `create_animation = false` and just view plots

### Issue: Robots collide

**Symptom:** Links intersect in visualization

**Causes:**
1. Bases too close
2. Waypoints overlap
3. No collision checking

**Solutions:**
- Increase base separation
- Plan waypoints more carefully
- Implement custom collision detection (see Advanced Topics)

---

## Example Configurations

### Example 1: Symmetric 6-DOF Arms

```matlab
% Two identical 6-DOF robots
robot1.link_vectors = [0, 0, 0, 0, 0, 0; 
                       0, 0, 0, 0, 0, 0; 
                       0.3, 0.3, 0.25, 0.2, 0.15, 0.1];
robot1.joint_types = 'RRRRRR';
robot1.base_position = [-0.6; 0; 0];

robot2.link_vectors = robot1.link_vectors;  % Same structure
robot2.joint_types = robot1.joint_types;
robot2.base_position = [0.6; 0; 0];  % Opposite side

load_params.mass = 10.0;
load_params.size = [1.2, 0.3, 0.2];  % Large box
```

### Example 2: Asymmetric 3-DOF + 4-DOF

```matlab
% Robot 1: 3-DOF simple
robot1.link_vectors = [0, 0, 0; 0, 0, 0; 0.4, 0.4, 0.3];
robot1.joint_types = 'RRR';
robot1.base_position = [-0.5; 0; 0];

% Robot 2: 4-DOF more dexterous
robot2.link_vectors = [0, 0, 0, 0; 0, 0, 0, 0; 0.3, 0.3, 0.25, 0.2];
robot2.joint_types = 'RRRR';
robot2.base_position = [0.5; 0; 0];

load_params.mass = 3.0;
load_params.size = [0.4, 0.3, 0.15];  % Medium box
```

### Example 3: Heavy Load Transport

```matlab
% Both robots: strong 5-DOF
robot1.link_vectors = [0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0.35, 0.35, 0.3, 0.25, 0.2];
robot1.joint_types = 'RRRRR';
robot1.base_position = [-0.8; 0; 0];

robot2 = robot1;
robot2.base_position = [0.8; 0; 0];

load_params.mass = 25.0;  % Heavy!
load_params.size = [0.8, 0.6, 0.4];

task_params.task_type = 'transport';
task_params.duration = 15.0;  % Slow and steady
```

---

## Summary

The **Collaborative Dual-Arm System** provides:

✅ **Easy Configuration** - Edit one file, run, done  
✅ **Complete Analysis** - Kinematics, dynamics, coordination  
✅ **Automatic Verification** - Checks grasp stability, force balance, synchronization  
✅ **Rich Visualization** - Workspace, metrics, trajectories, animation  
✅ **Data Export** - CSV files for further analysis  

**Use it when you need:**
- Two robots handling a common load
- Coordinated pick-and-place operations
- Transport of large/heavy objects
- Assembly tasks requiring cooperation
- Analysis of collaborative manipulation

**For more information, see:**
- `README.md` - Overall project documentation
- `QUICKSTART.md` - General getting started guide
- `example_dual_arm_collaborative.m` - Pre-configured example

---

**Questions?** Check the troubleshooting section or examine the function implementations in `Collaborative_DualArm_Functions.m`.

**Happy Collaborating!** 🤖🤝🤖
