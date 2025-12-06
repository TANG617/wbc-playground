# PSI WBC (Whole Body Control) Package

This package implements a dual-arm whole body controller using Placo for the A2D robot platform.

## Features

- **Dual-arm coordination**: Simultaneous control of left and right arms
- **Placo-based IK**: Uses Placo's hierarchical task-based inverse kinematics solver
- **VR integration**: Support for PSI Glove and VR tracker input
- **Real-time control**: High-frequency control loop with velocity limiting
- **Safety features**: Workspace limits, joint limits, and velocity constraints
- **Energy efficiency**: Kinetic energy regularization task for smooth, efficient movements

## Tasks and Priorities

The WBC controller uses a hierarchical task-based approach with the following tasks (in priority order):

1. **Frame Tasks** (High Priority): Complete 6DOF end-effector pose tracking for both arms
2. **Posture Task** (Medium Priority): Maintains good robot configuration
3. **Kinetic Energy Task** (Low Priority): Minimizes kinetic energy for efficient movements
4. **Regularization Task** (Lowest Priority): Prevents singular configurations

### Kinetic Energy Task

The kinetic energy regularization task helps reduce unnecessary movements and improve energy efficiency:

- **Purpose**: Minimizes the robot's kinetic energy `(1/2) * q̇ᵀ * M * q̇`
- **Weight**: Configurable via `tasks.kinetic_energy.weight` (default: 0.1)
- **Priority**: Soft task with low weight to avoid interfering with primary objectives
- **Effect**: Results in smoother, more natural movements with reduced oscillations

The task is implemented using Placo's `KineticEnergyRegularizationTask` which computes the mass matrix M and creates a regularization term that penalizes high joint velocities.

## Configuration

### Task Weights

Configure task weights in `config/wbc_config.yaml`:

```yaml
tasks:
  left_ee_position:
    weight: 100.0
  left_ee_orientation:
    weight: 50.0
  right_ee_position:
    weight: 100.0
  right_ee_orientation:
    weight: 50.0
  posture_maintenance:
    weight: 0.02
  current_joint_preference:
    weight: 0.05
  joint_regularization:
    weight: 0.0001
  kinetic_energy:
    weight: 0.1  # Energy efficiency task
```

### Dynamic Weight Updates

You can update the kinetic energy task weight at runtime:

```cpp
// Update kinetic energy task weight
wbc_controller.updateKineticEnergyWeight(0.2);  // Increase energy efficiency
```

## Usage

### Launch the WBC Controller

```bash
ros2 launch psi_wbc ik_node.launch.py
```

### VR Tracker Input

The controller expects VR tracker poses on:
- `/left_tracker/pose` - Left hand tracker
- `/right_tracker/pose` - Right hand tracker

### Joint State Input

Provide current joint states on:
- `/joint_states_input` - Current robot joint positions and velocities

### Output

The controller publishes:
- `/joint_states` - Commanded joint positions and velocities
- `/psi_wbc/state` - Controller state information

## Architecture

The WBC controller uses Placo's task-based approach:

1. **Robot Model**: Loads URDF and creates Placo RobotWrapper
2. **Task Setup**: Configures frame tasks, posture task, and energy task
3. **Solver**: Uses Placo KinematicsSolver with hierarchical task priorities
4. **Solution**: Applies velocity limits and smoothing for safe execution

## Dependencies

- ROS 2 Humble
- Placo (included as third-party)
- Eigen3
- Pinocchio (via Placo)

## Building

```bash
colcon build --packages-select psi_wbc
```

## Testing

The controller includes extensive debug logging and can be configured for different verbosity levels via the YAML configuration file.
