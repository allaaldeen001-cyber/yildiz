# Dynamic Analysis System Guide

## Overview

This guide explains the **Dynamic Analysis System** for computing joint torques using **Spatial Operator Algebra (SOA)**. The system computes torques using the **Recursive Newton-Euler Algorithm** implemented in `ARAT_Core`.

---

## 📋 Table of Contents

1. [System Structure](#system-structure)
2. [Quick Start](#quick-start)
3. [Configuration Guide](#configuration-guide)
4. [Dynamic Analysis Parts (6-10)](#dynamic-analysis-parts-6-10)
5. [Understanding Results](#understanding-results)
6. [Advanced Features](#advanced-features)
7. [Examples](#examples)
8. [Troubleshooting](#troubleshooting)

---

## System Structure

The dynamic analysis system consists of **TWO FILES**:

### 1. `Dynamic_SOA_Functions.m` (Backend - ~650 lines)

Contains all dynamic analysis logic:
- ✅ Trajectory generation (position, velocity, acceleration)
- ✅ Recursive Newton-Euler algorithm (joint torques)
- ✅ Power and energy computation
- ✅ Moving platform support
- ✅ Cooperative dynamics constraints
- ✅ Visualization (torque plots, power plots, comparative analysis)
- ✅ Data export (CSV and summary files)

### 2. `Dynamic_User_Interface.m` (Frontend - ~400 lines)

User interface where you:
- 📝 Define robot kinematic parameters
- 📝 Specify mass and inertia properties
- 📝 Configure trajectory parameters
- 📝 Enable optional features (moving platform, constraints)
- 📝 Set output options
- ▶️ Run the script
- 📊 Get complete dynamic analysis automatically

---

## Quick Start

### Step 1: Open the Interface

```matlab
open Dynamic_User_Interface.m
```

### Step 2: Configure System

Edit the **CONFIGURATION** section:

```matlab
%% Number of robots
p_robots = 1;  % Start with one robot
n_links_per_robot = [6];  % 6-DOF

%% Robot parameters
robot1.link_vectors = [...];
robot1.joint_types = 'RRRRRR';
robot1.q_initial = [0; 0; 0; 0; 0; 0];
robot1.q_final = [pi/4; pi/4; pi/4; pi/4; pi/4; pi/4];

%% Mass properties
for i = 1:6
    mass_params_1(i).m = 2.0;  % 2 kg per link
    mass_params_1(i).c = [0; 0; 0.05];  % CoM
    mass_params_1(i).I = diag([0.01, 0.01, 0.005]);  % Inertia
end

%% Trajectory
trajectory_params.type = 2;  % Harmonic
trajectory_params.duration = 5.0;
trajectory_params.n_samples = 100;
```

### Step 3: Run

```matlab
run Dynamic_User_Interface.m
```

### Step 4: View Results

- **Console**: Peak torques, RMS torques, power, energy
- **Figures**: Torque plots, power plots, comparative plots
- **Files**: CSV files in `output/` directory

---

## Configuration Guide

### Robot Parameters

Same as kinematic analysis:
- `link_vectors` (3×N): Link vectors
- `joint_axes` (3×N): Joint axes
- `joint_types` (string): 'R' or 'P'
- `q_initial`, `q_final` (N×1): Initial and final configurations

### Mass and Inertia Properties ⭐ NEW!

For each link, define:

#### Mass (kg)
```matlab
mass_params(i).m = 2.0;  % Mass in kilograms
```

#### Center of Mass (m)
Position of CoM relative to link origin:
```matlab
mass_params(i).c = [x; y; z];  % In meters
```

**Example:** For a vertical link of length 0.3m with CoM at midpoint:
```matlab
mass_params(i).c = [0; 0; 0.15];
```

#### Inertia Tensor (kg·m²)
3×3 inertia matrix about CoM:
```matlab
mass_params(i).I = [Ixx, Ixy, Ixz;
                    Ixy, Iyy, Iyz;
                    Ixz, Iyz, Izz];
```

**For diagonal (simplified) inertia:**
```matlab
mass_params(i).I = diag([Ixx, Iyy, Izz]);
```

**Computing inertia for a cylindrical link:**
```matlab
L = 0.3;  % Length (m)
m = 2.0;  % Mass (kg)
r = 0.05;  % Radius (m)

Ixx = (1/12)*m*L^2 + (1/4)*m*r^2;  % About x-axis
Iyy = Ixx;  % Same for y
Izz = (1/2)*m*r^2;  % About z-axis (link axis)

mass_params(i).I = diag([Ixx, Iyy, Izz]);
```

### Gravity Vector

```matlab
dynamic_params.gravity = [gx; gy; gz];  % m/s²
```

**Common values:**
- Earth: `[0; 0; -9.81]`
- Moon: `[0; 0; -1.62]`
- Zero-G: `[0; 0; 0]`
- Horizontal: `[9.81; 0; 0]` (e.g., for testing)

### Trajectory Parameters

```matlab
trajectory_params.type = 2;  % 1-5 (see below)
trajectory_params.duration = 5.0;  % seconds
trajectory_params.n_samples = 100;  % number of points
```

**Trajectory Types:**
1. **Cubic Polynomial** - Smooth with zero endpoints
2. **Harmonic** - Sinusoidal, very smooth
3. **Cycloidal** - Zero jerk at endpoints
4. **Gutman 1-3** - Fourier-based, reduced vibration
5. **Freudenstein 1-3-5** - Fourier-based, minimal residual vibration

---

## Dynamic Analysis Parts (6-10)

### Part 6: Fixed 6-Link Serial Robot

**Purpose:** Compute torques for a specific 6-DOF robot

**Configuration:**
```matlab
p_robots = 1;
n_links_per_robot = [6];
```

**Example:**
```matlab
example_dynamics_6dof
```

**Output:**
- Torque trajectories for all 6 joints
- Power and energy curves
- Exported CSV files

---

### Part 7: n-Link Serial Robot (Generalized)

**Purpose:** Compute torques for any DOF robot

**Configuration:**
```matlab
p_robots = 1;
n_links_per_robot = [9];  % Or any n

% Define robot with 9 links
robot1.link_vectors = [...];  % 3x9
robot1.joint_types = 'RRRRRRRRR';  % 9 joints
robot1.q_initial = zeros(9, 1);
robot1.q_final = ones(9, 1) * pi/6;

% Define 9 mass parameters
for i = 1:9
    mass_params_1(i).m = 2.0;
    mass_params_1(i).c = [0; 0; 0.05];
    mass_params_1(i).I = diag([0.01, 0.01, 0.005]);
end
```

**Key Point:** Code automatically adapts to any `n`!

---

### Part 8: Two 6-Link Serial Robots

**Purpose:** Compute torques for two 6-DOF robots on same platform

**Configuration:**
```matlab
p_robots = 2;
n_links_per_robot = [6, 6];

% Define robot1 (6-DOF)
robot1 = struct();
% ... (6 links)

% Define robot2 (6-DOF)
robot2 = struct();
// ... (6 links)

% Mass parameters for both
dynamic_params.mass_params_per_robot = {mass_params_1, mass_params_2};
```

**Output:**
- Separate torque plots for each robot
- Comparative torque analysis
- Individual power/energy curves

---

### Part 9: Two n-Link Serial Robots (Arbitrary DOF)

**Purpose:** Two robots with different DOF

**Configuration:**
```matlab
p_robots = 2;
n_links_per_robot = [9, 12];  % Robot 1: 9-DOF, Robot 2: 12-DOF

% Define robot1 with 9 links
robot1.link_vectors = [...];  % 3x9

% Define robot2 with 12 links
robot2.link_vectors = [...];  % 3x12

% Mass parameters
% Robot 1: 9 links
for i = 1:9
    mass_params_1(i) = ...;
end

% Robot 2: 12 links
for i = 1:12
    mass_params_2(i) = ...;
end
```

---

### Part 10: p Robots with Arbitrary n (Most General)

**Purpose:** Any number of robots, each with arbitrary DOF

**Configuration:**
```matlab
p_robots = 4;
n_links_per_robot = [6, 9, 5, 12];  % 4 robots

% Define robot1 (6-DOF)
robot1 = struct();
% ...

% Define robot2 (9-DOF)
robot2 = struct();
// ...

% Define robot3 (5-DOF)
robot3 = struct();
% ...

% Define robot4 (12-DOF)
robot4 = struct();
% ...

% Collect into cell array
robots = {robot1, robot2, robot3, robot4};
```

**Note:** System handles any combination automatically!

---

## Understanding Results

### Console Output

```
╔════════════════════════════════════════════╗
║  DYNAMIC ANALYSIS SUMMARY                  ║
╚════════════════════════════════════════════╝

ROBOT 1 (6-DOF):
  Peak torques (N·m): 45.23 38.67 28.91 19.45 12.34 6.78
  Max torque: 45.23 N·m
  RMS torques (N·m): 22.34 18.91 14.56 10.23 6.78 3.45
  Peak power: 234.56 W
  Avg power: 87.34 W
  Total energy: 436.72 J
```

**Interpretation:**
- **Peak torques**: Maximum absolute torque for each joint
- **Max torque**: Highest torque across all joints
- **RMS torques**: Root-mean-square (average magnitude)
- **Peak power**: Maximum instantaneous power
- **Avg power**: Average power consumption
- **Total energy**: Cumulative energy over motion

### Torque Plots

**Figure 1: Joint Torque Trajectories**

Shows torque vs. time for each joint in separate subplots:
- X-axis: Time (s)
- Y-axis: Torque (N·m)
- Each subplot = one joint
- Legend shows peak and RMS values

**What to look for:**
- ✅ **Base joints** typically have highest torques (supporting entire arm)
- ✅ **End joints** typically have lowest torques (only moving end-effector)
- ✅ **Smooth curves** indicate good trajectory
- ⚠️ **Spikes** may indicate discontinuities or singularities

**Figure 2: Power & Energy Analysis**

Two subplots:
1. **Instantaneous Power**: Power vs. time
   - Positive = motors doing work
   - Negative = regenerative (energy recovery possible)
   
2. **Cumulative Energy**: Total energy used over time
   - Always increasing for typical motions
   - Final value = total energy required

**Figure 3: Comparative Torque Analysis** (if p > 1)

Shows same joint across multiple robots:
- Compare torque requirements
- Identify which robot is more loaded
- Verify balanced workload in cooperative tasks

### Exported Data Files

| File | Content |
|------|---------|
| `*_robot1_torques.csv` | Time + torque for all joints of Robot 1 |
| `*_robot1_power.csv` | Time + instantaneous power |
| `*_robot1_energy.csv` | Time + cumulative energy |
| `*_robot1_trajectories.csv` | Time + q, qd, qdd for all joints |
| `*_dynamic_summary.txt` | Text summary of all statistics |

**CSV Format:**
```
time, tau_1, tau_2, ..., tau_n
0.00, 0.00, 0.00, ..., 0.00
0.05, 2.34, 1.89, ..., 0.45
...
```

---

## Advanced Features

### Moving Platform

Compute dynamics when robot base is on a moving platform:

```matlab
% Enable moving platform
enable_moving_platform = true;

% Define platform motion
platform_params = struct();
platform_params.velocity = [0; 0; 0.1; 0.5; 0; 0];  % [wx;wy;wz; vx;vy;vz]
platform_params.acceleration = [0; 0; 0; 0; 0; 0];
```

**Use cases:**
- Mobile manipulator
- Robot on vehicle
- Ship-mounted robot
- Aircraft-mounted arm

**Effect:** Additional inertial forces/torques due to platform motion

### Kinematic Constraints

For cooperative tasks where robots must maintain relative positions:

```matlab
enable_kinematic_constraints = true;

kinematic_constraints = struct();
kinematic_constraints.type = 'common_load';
kinematic_constraints.load_separation = 1.0;  % meters
```

**Constraint Types:**
- `'common_load'` - Fixed separation (load grasping)
- `'formation'` - Maintain formation pattern
- `'coupled_motion'` - Linked joint motions

### Dynamic Constraints

Force/torque distribution in cooperative tasks:

```matlab
enable_dynamic_constraints = true;

dynamic_constraints = struct();
dynamic_constraints.force_distribution = [0.6, 0.4];  % 60-40 split
```

**Use cases:**
- Unequal load sharing
- Stronger/weaker robot coordination
- Asymmetric tasks

### Prismatic Joints

System supports prismatic (linear) joints:

```matlab
robot.joint_types = 'RRPRR';  % 3rd joint is prismatic

% For prismatic joints:
% - q is in meters (not radians)
% - joint_axis is translation direction
% - Torque output is force (N) not torque (N·m)
```

**Example:**
```matlab
robot.link_vectors(:, 3) = [0; 0; 0.5];  % 0.5m nominal length
robot.joint_axes(:, 3) = [0; 0; 1];  % Extends along Z
robot.joint_types = 'RRPRR';

robot.q_initial(3) = 0.0;  % meters
robot.q_final(3) = 0.3;    // meters (0.3m extension)

% Output: tau(3) will be in Newtons (force)
```

---

## Examples

### Example 1: Single 6-DOF Robot

**File:** `example_dynamics_6dof.m`

**What it shows:**
- Standard 6-DOF configuration
- Typical mass distribution (heavier at base)
- Harmonic trajectory
- Complete torque analysis

**Run:**
```matlab
example_dynamics_6dof
```

### Example 2: Custom n-DOF Robot

**Create your own:**
```matlab
open Dynamic_User_Interface.m
% Set n_links_per_robot = [9]  % Or any n
% Configure 9 links
% Run the script
```

### Example 3: Two Robots (Cooperative)

**Configuration:**
```matlab
p_robots = 2;
n_links_per_robot = [6, 6];

% Configure both robots
% Enable constraints if needed
```

### Example 4: Moving Platform

```matlab
% In Dynamic_User_Interface.m
enable_moving_platform = true;
platform_params.velocity = [0; 0; 0; 1.0; 0; 0];  % Moving at 1 m/s in X
```

---

## Troubleshooting

### Issue: "Dimensions don't match"

**Cause:** Mismatch between n_links and mass parameters

**Solution:**
```matlab
% Make sure:
length(mass_params) == n_links_per_robot(robot_idx)
```

### Issue: Torques seem too high/low

**Causes:**
1. Incorrect mass values
2. Wrong inertia tensor
3. Gravity in wrong direction

**Solutions:**
```matlab
% Check masses (typical: 1-5 kg per link)
mass_params(i).m = 2.0;  % kg

% Check gravity direction
dynamic_params.gravity = [0; 0; -9.81];  % Down is negative Z

% Verify inertia scale (should be ~0.001-0.1 kg·m²)
I = 0.01;  % kg·m²
```

### Issue: Torques are NaN or Inf

**Causes:**
1. Singular configuration
2. Zero inertia
3. Trajectory discontinuity

**Solutions:**
```matlab
% Avoid extended configurations
robot.q_final = [pi/4; pi/4; pi/4; ...];  % Not [pi; pi; pi; ...]

% Ensure non-zero inertia
mass_params(i).I = diag([0.01, 0.01, 0.005]);  % Not zeros

% Use smooth trajectories
trajectory_params.type = 2;  % Harmonic (smoothest)
```

### Issue: Power plot shows spikes

**Cause:** Rapid velocity changes

**Solution:**
```matlab
% Increase duration for smoother motion
trajectory_params.duration = 10.0;  % Slower

% Or use smoother trajectory
trajectory_params.type = 5;  // Freudenstein (smoothest acceleration)
```

### Issue: Base joint torque much higher than others

**Response:** **This is normal!**

Base joints support the entire arm's weight and inertia. Typical ratios:
- Joint 1 (base): 100%
- Joint 2: 60-80%
- Joint 3: 40-60%
- Joint 4: 20-30%
- Joint 5: 10-15%
- Joint 6: 5-10%

---

## Key Equations

### Recursive Newton-Euler Algorithm

**Forward Pass (Velocities & Accelerations):**
```
ω_{i} = ω_{i-1} + h_i · q̇_i  (for revolute)
v_{i} = v_{i-1} + ω_{i-1} × l_i  (linear velocity)

ω̇_{i} = ω̇_{i-1} + ω_{i-1} × (h_i · q̇_i) + h_i · q̈_i
v̇_{i} = v̇_{i-1} + ω̇_{i-1} × l_i + ω_{i-1} × (ω_{i-1} × l_i)
```

**Inertial Forces:**
```
F_i = m_i · a_{CoM,i}
N_i = I_i · ω̇_i + ω_i × (I_i · ω_i)
```

**Backward Pass (Forces & Torques):**
```
f_i = f_{i+1} + F_i
n_i = n_{i+1} + N_i + l_{i+1} × f_{i+1} + c_i × F_i

τ_i = n_i^T · h_i  (for revolute)
```

### Power and Energy

**Instantaneous Power:**
```
P_i(t) = τ_i(t) · q̇_i(t)
P_total(t) = Σ P_i(t)
```

**Cumulative Energy:**
```
E(t) = ∫₀ᵗ P(τ) dτ
```

---

## Summary

The Dynamic Analysis System provides:

✅ **Complete torque computation** using recursive Newton-Euler  
✅ **Supports any DOF** (3, 6, 9, 12, ... any n)  
✅ **Multiple robots** (1, 2, 3, ... any p)  
✅ **Power & energy analysis**  
✅ **Moving platform support**  
✅ **Cooperative dynamics** with constraints  
✅ **Prismatic joints** supported  
✅ **Rich visualization** (torque, power, comparative plots)  
✅ **Complete data export** (CSV + summary)  

**Use it to:**
- Size motor/actuators
- Estimate energy consumption
- Compare robot designs
- Verify feasibility of trajectories
- Analyze cooperative tasks
- Study dynamic behavior

---

*Dynamic Analysis Guide - December 2, 2025*
