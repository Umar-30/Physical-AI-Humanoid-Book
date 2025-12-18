# Chapter 5: Gazebo Physics, Worlds, and Robot Spawning

**Priority**: P1 (Foundation chapter)
**Estimated Duration**: 2-3 hours
**Target Audience**: Readers who completed Module 1 (ROS 2 fundamentals + basic URDF)

## Learning Objectives

By the end of this chapter, readers will be able to:
1. **Install** Gazebo Harmonic on Ubuntu 22.04 and verify installation
2. **Create** SDF world files with ground planes, lighting, and physics configuration
3. **Spawn** humanoid robot URDF models using both command-line and launch files
4. **Configure** physics engine parameters (timestep, solvers, gravity)
5. **Troubleshoot** common Gazebo installation and runtime issues

## Prerequisites

- Module 1 completed (ROS 2 Humble, basic URDF understanding)
- Ubuntu 22.04 installed (or WSL2 on Windows)
- 16GB RAM, 4GB free disk space
- GPU with OpenGL 3.3+ support

## Chapter Structure

### 5.1 Gazebo Harmonic Installation (15 min)

**Objective**: Install Gazebo Harmonic and verify functionality

**Content**:
- Gazebo Harmonic vs Classic comparison (brief, 2-3 sentences)
- Installation commands for Ubuntu 22.04
- Environment setup and path configuration
- Version verification
- First launch test

**Code Examples**:
```bash
# Installation
sudo apt update
sudo apt install gz-harmonic

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# First launch (empty world)
gz sim
```

**Deliverables**:
- Installation checklist
- Verification procedure
- GPU compatibility check commands

**Troubleshooting**:
- GPU driver issues
- Package dependency conflicts
- Permission errors

---

### 5.2 SDF World File Structure (30 min)

**Objective**: Understand and create SDF world files

**Content**:
- SDF format overview (XML-based robot/world description)
- World file anatomy (physics, gravity, models, lights)
- Ground plane creation
- Lighting configuration (directional, point, spot)
- Physics engine selection (Bullet, DART, TPE)

**Configuration Files**:
1. `minimal_world.sdf` - Simplest possible world
2. `humanoid_world.sdf` - Production-ready world for humanoid robots
3. `complex_world.sdf` - With obstacles and varied terrain

**Code Examples**:
```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_world">
    <physics name="default_physics" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <!-- Ground plane -->
    <model name="ground_plane">...</model>
    <!-- Lighting -->
    <light name="sun" type="directional">...</light>
  </world>
</sdf>
```

**Diagram**:
- `sdf-world-structure.svg`: Hierarchical breakdown of SDF elements

**Deliverables**:
- 3 complete SDF world files (minimal, standard, complex)
- Annotated template with inline comments
- SDF element reference table

**Validation**:
- All SDF files must load in Gazebo without errors
- Physics must be active (objects fall with gravity)

---

### 5.3 Robot Spawning Methods (25 min)

**Objective**: Spawn URDF robots into Gazebo worlds

**Content**:
- Command-line spawning (gz service CLI)
- Launch file spawning (ROS 2 Python launch)
- Initial pose configuration (x, y, z, roll, pitch, yaw)
- URDF vs SDF model formats
- Entity management (list, delete, modify)

**Code Examples**:

**Method 1: Command-Line**
```bash
# Start Gazebo with world
gz sim humanoid_world.sdf

# Spawn robot (in separate terminal)
gz service -s /world/humanoid_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/path/to/robot.urdf", name: "my_robot", pose: {position: {z: 1.0}}'
```

**Method 2: Launch File** (preferred for production)
```python
# spawn_robot.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', 'humanoid_world.sdf', '-r'],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot', '-file', 'robot.urdf', '-z', '1.0'],
            output='screen'
        ),
    ])
```

**Configuration Files**:
1. `spawn_robot_cmd.sh` - Bash script for command-line spawning
2. `spawn_robot.launch.py` - ROS 2 launch file
3. `example_humanoid.urdf` - Simple humanoid for testing

**Deliverables**:
- Both spawning methods documented with examples
- Troubleshooting checklist for spawn failures
- Entity management commands reference

**Validation**:
- Robot appears in Gazebo at specified pose
- Robot responds to gravity (falls if unstable)
- `gz topic -l` shows robot-related topics

---

### 5.4 Physics Engine Configuration (25 min)

**Objective**: Configure physics parameters for humanoid simulation

**Content**:
- Physics engine comparison (Bullet, DART, TPE)
- Timestep selection (trade-off: accuracy vs performance)
- Real-time factor tuning
- Solver parameters (iterations, contact properties)
- Gravity customization (non-Earth environments)

**Key Parameters**:
| Parameter | Recommended | Impact |
|-----------|-------------|--------|
| `max_step_size` | 0.001s (1kHz) | Smaller = more accurate, slower |
| `real_time_factor` | 1.0 | Target real-time multiplier |
| `real_time_update_rate` | 1000Hz | Physics loop frequency |
| `solver_iterations` | 50 (Bullet) | Constraint solver accuracy |

**Code Examples**:
```xml
<!-- High-accuracy configuration (for research) -->
<physics name="precise_physics" type="bullet">
  <max_step_size>0.0005</max_step_size>  <!-- 2kHz -->
  <real_time_factor>0.5</real_time_factor>  <!-- Accept 50% real-time -->
  <bullet>
    <solver>
      <iterations>100</iterations>
    </solver>
  </bullet>
</physics>

<!-- Performance configuration (for large scenes) -->
<physics name="fast_physics" type="bullet">
  <max_step_size>0.002</max_step_size>  <!-- 500Hz -->
  <real_time_factor>1.0</real_time_factor>
  <bullet>
    <solver>
      <iterations>30</iterations>
    </solver>
  </bullet>
</physics>
```

**Diagram**:
- `physics-timestep-tradeoff.svg`: Accuracy vs performance graph

**Deliverables**:
- Physics configuration templates (high-accuracy, balanced, performance)
- Performance monitoring commands
- Benchmark results table (update rate vs robot complexity)

**Validation**:
- `gz stats` command shows target update rate achieved
- Robot motion is stable (no jitter or explosions)

---

### 5.5 Troubleshooting Common Issues (20 min)

**Objective**: Diagnose and resolve typical Gazebo problems

**Content**:
- Installation issues (missing dependencies, GPU drivers)
- World loading failures (SDF syntax errors)
- Robot spawn failures (URDF errors, path issues)
- Performance problems (low update rate, lag)
- GUI issues (black screen, unresponsive interface)

**Troubleshooting Table**:
| Issue | Symptoms | Solution |
|-------|----------|----------|
| GPU driver problem | Black screen, OpenGL errors | Update drivers, check `glxinfo` |
| Low physics rate | `gz stats` shows <100Hz | Reduce timestep, simplify collision geometry |
| Robot falls through ground | Robot sinks into floor | Check collision geometry on ground plane |
| "Entity not found" error | Spawn command fails | Verify URDF path is absolute, check file exists |

**Code Examples**:
```bash
# Check GPU capabilities
glxinfo | grep "OpenGL version"
# Minimum: OpenGL 3.3

# Monitor performance in real-time
gz stats -p /stats

# Validate SDF syntax before loading
gz sdf -k world_file.sdf

# Check Gazebo logs
gz log record  # Record session
gz log playback <log_file>  # Replay session
```

**Deliverables**:
- Troubleshooting flowchart
- Common error messages with solutions
- Performance diagnostic checklist

---

### 5.6 Hands-On Exercises (30-45 min)

**Exercise 5.1: Create Custom World** (Beginner, 15 min)

**Instructions**:
1. Create `my_world.sdf` with ground plane and 2 light sources
2. Set physics timestep to 0.002s
3. Add custom gravity (e.g., Moon: 0 0 -1.62)
4. Launch world and verify it loads

**Success Criteria**:
- [ ] Gazebo launches without errors
- [ ] Ground plane visible
- [ ] Lights illuminate scene
- [ ] `gz stats` shows ~500Hz physics rate

**Solution**: `/files/module-2/solutions/exercise-5-1-my-world.sdf`

---

**Exercise 5.2: Spawn Robot with Launch File** (Intermediate, 20 min)

**Instructions**:
1. Create `my_spawn.launch.py` launch file
2. Configure it to start your custom world from Exercise 5.1
3. Spawn `example_humanoid.urdf` at position (0, 0, 1.5)
4. Verify robot spawns and settles on ground

**Success Criteria**:
- [ ] Launch command starts both world and robot
- [ ] Robot appears at specified height
- [ ] Robot falls and stabilizes on ground
- [ ] No error messages in terminal

**Solution**: `/files/module-2/solutions/exercise-5-2-my-spawn.launch.py`

---

**Exercise 5.3: Performance Tuning** (Advanced, 15 min)

**Instructions**:
1. Open provided `complex_world.sdf` (multiple robots, obstacles)
2. Measure baseline physics rate with `gz stats`
3. Adjust timestep and solver iterations to achieve >200Hz
4. Document your configuration and resulting performance

**Success Criteria**:
- [ ] Initial performance measured and recorded
- [ ] Modified physics configuration achieves target rate
- [ ] Simulation remains stable (no jitter, explosions)
- [ ] Trade-offs documented (accuracy vs speed)

**Solution**: Performance tuning guide with example configurations

---

## Chapter Summary

**Key Takeaways**:
1. Gazebo Harmonic provides physics simulation for humanoid robots via SDF worlds
2. Two robot spawning methods: command-line (testing) and launch files (production)
3. Physics timestep is critical trade-off: smaller = accurate but slower
4. Troubleshooting requires understanding GPU, SDF syntax, and URDF validation

**What's Next**:
- Chapter 6 extends URDF models with sensors (IMU, cameras, lidar)
- Chapter 6 covers collision geometry and inertia configuration for realistic physics

**Further Reading**:
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [SDF Specification](http://sdformat.org/spec)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/ros_gz)

---

## Validation Checklist (for Authors)

- [ ] All code examples tested on Ubuntu 22.04
- [ ] All SDF files validated with `gz sdf -k`
- [ ] All launch files successfully spawn robots
- [ ] Screenshots captured for each major step
- [ ] Troubleshooting section includes real errors encountered
- [ ] Exercises have verified solutions
- [ ] All links resolve correctly
- [ ] Docusaurus build succeeds for this chapter
- [ ] External reviewer completed chapter successfully
