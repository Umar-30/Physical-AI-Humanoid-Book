# Gazebo Harmonic Architecture Research
**Research Date**: 2025-12-17
**Target Module**: Module 2 - The Digital Twin (Gazebo & Unity)
**Purpose**: Educational content for Physical AI & Humanoid Robotics book
**Target Versions**: Gazebo Harmonic + ROS 2 Humble

---

## Executive Summary

Gazebo Harmonic represents a complete architectural reimagining of robot simulation compared to Gazebo Classic. Built on a modular library-based design (gz-sim, gz-physics, gz-rendering, etc.), it offers improved performance, real-time capabilities, and seamless ROS 2 integration. This research provides educational content architects with the technical foundation needed to teach humanoid robot simulation effectively.

**Key Takeaways for Educational Content**:
- Gazebo Harmonic uses a plugin-based Entity-Component-System (ECS) architecture
- SDF (Simulation Description Format) is XML-based and more feature-rich than URDF
- Physics engines are pluggable (DART is default, supports Bullet, TPE)
- ROS 2 integration via `ros_gz_bridge` is bidirectional and type-safe
- Performance targets: 1000 Hz physics updates possible, typical humanoid: 100-500 Hz

---

## 1. Gazebo Harmonic vs. Gazebo Classic

### 1.1 Architectural Differences

| Aspect | Gazebo Classic (v11) | Gazebo Harmonic (2024+) |
|--------|---------------------|------------------------|
| **Architecture** | Monolithic (gazebo-server, gazebo-client) | Modular libraries (gz-sim, gz-physics, gz-rendering) |
| **ECS Pattern** | No | Yes (Entity-Component-System via gz-sim) |
| **Physics Engines** | ODE (default), Bullet, Simbody, DART | DART (default), Bullet3, TPE (Trivial Physics Engine) |
| **Message Passing** | Boost-based Gazebo Transport | Ignition Transport (protobuf-based, cross-platform) |
| **Rendering** | OGRE 1.x | OGRE 2.x, Optix (ray-tracing capable) |
| **ROS Integration** | `gazebo_ros_pkgs` (tightly coupled) | `ros_gz_bridge` (decoupled, topic-based) |
| **SDF Version** | SDF 1.6-1.7 | SDF 1.9+ (richer semantics) |
| **Real-Time** | Best-effort | RT-capable with proper system tuning |
| **Plugin System** | Global namespace, C++ only | Versioned, ABI-stable, supports multiple languages |

### 1.2 Migration Considerations

**For Educators Creating Content**:
1. **Syntax Changes**: World and model SDF files have new XML structure
2. **Launch Files**: ROS 2 launch files replace roslaunch XML
3. **Topic Remapping**: `ros_gz_bridge` requires explicit topic mappings
4. **Plugins**: Gazebo Classic plugins must be rewritten for gz-sim
5. **GUI**: New Qt-based interface (completely redesigned)

**Migration Difficulty**: Moderate to High
- Simple worlds: 2-4 hours to port
- Complex models with plugins: 1-2 weeks

**Recommendation**: Start fresh with Harmonic rather than porting Classic examples.

---

## 2. SDF (Simulation Description Format) Structure

### 2.1 World File Anatomy

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_training_world">

    <!-- Physics Engine Configuration -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>        <!-- 1ms time step = 1000 Hz -->
      <real_time_factor>1.0</real_time_factor>    <!-- 1.0 = real-time, >1.0 = faster -->
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- DART-specific solver settings -->
      <dart>
        <solver>
          <solver_type>dantzig</solver_type>       <!-- Options: dantzig, pgs -->
          <friction_direction>pyramid_model</friction_direction>
        </solver>
        <collision_detector>bullet</collision_detector> <!-- bullet, fcl, dart -->
      </dart>
    </physics>

    <!-- Gravity Vector (Earth standard) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Atmospheric Density (for drag calculations) -->
    <atmosphere type='adiabatic'>
      <temperature>288.15</temperature>
      <pressure>101325</pressure>
      <temperature_gradient>-0.0065</temperature_gradient>
    </atmosphere>

    <!-- Magnetic Field (for IMU simulation) -->
    <magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>

    <!-- Scene Rendering Configuration -->
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>    <!-- Friction coefficient -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Plugin for ROS 2 Clock -->
    <plugin filename="gz-sim-ros-clock-system"
            name="gz::sim::systems::RosGzClock">
      <ros>
        <remapping>/clock:=/sim/clock</remapping>
      </ros>
    </plugin>

  </world>
</sdf>
```

### 2.2 Key SDF Concepts for Education

**1. Hierarchical Structure**:
```
World
├── Physics (engine config)
├── Scene (rendering)
├── Model 1
│   ├── Link 1
│   │   ├── Collision
│   │   ├── Visual
│   │   └── Inertial
│   └── Joint 1
└── Model 2
```

**2. Coordinate Frames**:
- All poses are relative to parent frame
- Format: `<pose>x y z roll pitch yaw</pose>`
- Rotations in radians (Euler XYZ convention)

**3. Physics Time Step**:
- `max_step_size`: Physics engine integration step (default: 0.001s = 1ms)
- Smaller = more accurate but slower (0.0001s = 0.1ms for high-precision)
- Larger = faster but less stable (0.01s = 10ms for simple scenarios)

**Educational Note**: For humanoid robots with many joints, use 0.001s (1ms) as baseline.

---

## 3. Physics Engine Configuration

### 3.1 DART Physics Engine (Default & Recommended)

**Why DART for Humanoids**:
- Excellent constraint solver for closed kinematic chains
- Stable contact dynamics for bipedal locomotion
- Fast forward kinematics/dynamics
- Good balance of speed and accuracy

**Configuration Parameters**:

```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>

  <dart>
    <!-- Solver Selection -->
    <solver>
      <solver_type>dantzig</solver_type>  <!-- Best for humanoids -->
      <!-- Options:
           - dantzig: LCP solver, accurate, slower
           - pgs: Projected Gauss-Seidel, faster, less accurate
      -->
    </solver>

    <!-- Collision Detection -->
    <collision_detector>bullet</collision_detector>
    <!-- Options:
         - bullet: Fast, robust for complex geometries
         - fcl: Flexible Collision Library, good for planning
         - dart: Built-in, simple scenarios
    -->
  </dart>
</physics>
```

### 3.2 Gravity Configuration

**Standard Earth Gravity**:
```xml
<gravity>0 0 -9.81</gravity>  <!-- m/s² in Z-axis (down) -->
```

**Educational Scenarios**:
- **Moon**: `<gravity>0 0 -1.62</gravity>`
- **Mars**: `<gravity>0 0 -3.71</gravity>`
- **Zero-G**: `<gravity>0 0 0</gravity>`
- **Custom Testing**: `<gravity>0 0 -5.0</gravity>` (reduced gravity for easier balancing)

### 3.3 Solver Tuning for Humanoid Stability

**Problem**: Humanoid robots are dynamically unstable and require accurate contact resolution.

**Tuning Parameters** (in model's collision surface):
```xml
<surface>
  <contact>
    <ode>
      <kp>10000000.0</kp>    <!-- Contact stiffness (N/m) -->
      <kd>1.0</kd>            <!-- Contact damping (N*s/m) -->
      <max_vel>0.01</max_vel> <!-- Max penetration correction velocity -->
      <min_depth>0.001</min_depth> <!-- Contact threshold (m) -->
    </ode>
  </contact>
  <friction>
    <ode>
      <mu>1.0</mu>   <!-- Coulomb friction coefficient -->
      <mu2>1.0</mu2> <!-- Secondary friction direction -->
    </ode>
  </friction>
</surface>
```

**Educational Guidance**:
- High `kp` (10⁷) prevents foot penetration into ground
- Low `kd` (1.0) avoids bouncing
- `mu=1.0` provides realistic foot-ground friction
- Increase `mu` to 1.5 for easier balancing exercises

### 3.4 Performance Benchmarks

| Scenario | Physics Update Rate | Notes |
|----------|---------------------|-------|
| Empty world | 5000+ Hz | Baseline |
| Simple humanoid (15 DOF) | 500-1000 Hz | DART with box collisions |
| Complex humanoid (30 DOF) | 200-500 Hz | Mesh collisions |
| Humanoid + environment | 100-200 Hz | Multiple contact points |
| Multi-robot (5 humanoids) | 50-100 Hz | Depends on CPU cores |

**Hardware Reference** (for educational content):
- Minimum: Intel i5-8th gen, 8GB RAM → 100 Hz
- Recommended: Intel i7-10th gen, 16GB RAM → 250 Hz
- High-end: Intel i9/Ryzen 9, 32GB RAM → 500+ Hz

---

## 4. Robot Spawning Methods

### 4.1 Command-Line Spawning (Quick Testing)

**Method 1: Spawn from SDF file**
```bash
gz service -s /world/humanoid_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/path/to/robot.sdf", name: "my_robot"'
```

**Method 2: Spawn from URDF (via ROS 2)**
```bash
# Terminal 1: Launch Gazebo
gz sim humanoid_world.sdf

# Terminal 2: Spawn robot
ros2 run ros_gz_sim create -file /path/to/robot.urdf \
                           -name my_humanoid \
                           -x 0 -y 0 -z 1.0
```

### 4.2 Launch File Spawning (Production Method)

**Complete ROS 2 Launch Example**:
```python
# launch/gazebo_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_humanoid = FindPackageShare('humanoid_description')

    world_file = PathJoinSubstitution([
        pkg_humanoid, 'worlds', 'training_world.sdf'
    ])

    robot_urdf = PathJoinSubstitution([
        pkg_humanoid, 'urdf', 'humanoid.urdf'
    ])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file]  # -r = run on start
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_urdf,
            '-name', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for robot state
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge
    ])
```

**Key Components**:
1. **Gazebo Launch**: Starts gz-sim with world file
2. **Robot Spawning**: Creates robot entity from URDF/SDF
3. **ROS-GZ Bridge**: Maps topics between ROS 2 and Gazebo

### 4.3 Spawn Timing and Initialization

**Common Pitfall**: Spawning robot before Gazebo world is ready.

**Solution Pattern**:
```python
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

# Spawn robot 2 seconds after Gazebo starts
spawn_robot_delayed = TimerAction(
    period=2.0,
    actions=[spawn_robot]
)

# Or wait for specific Gazebo service
spawn_on_ready = RegisterEventHandler(
    OnProcessStart(
        target_action=gazebo,
        on_start=[spawn_robot]
    )
)
```

---

## 5. Best Practices for Humanoid Robot Simulation

### 5.1 URDF/SDF Model Design

**1. Collision Geometry Optimization**
```xml
<!-- BAD: High-poly mesh for collision -->
<collision name="foot_collision">
  <geometry>
    <mesh><uri>model://humanoid/meshes/foot_detailed.dae</uri></mesh>
  </geometry>
</collision>

<!-- GOOD: Simple box approximation -->
<collision name="foot_collision">
  <geometry>
    <box><size>0.2 0.1 0.05</size></box>
  </geometry>
</collision>
```

**Performance Impact**:
- Mesh collision: 50-100 Hz physics updates
- Box collision: 500-1000 Hz physics updates
- **Rule**: Use primitives (box, sphere, cylinder) for collisions whenever possible.

**2. Inertial Properties**

**Critical for Stability**:
```xml
<inertial>
  <mass>5.0</mass>  <!-- kg -->
  <inertia>
    <ixx>0.0347</ixx>  <!-- kg*m² -->
    <ixy>0.0</ixy>
    <ixz>0.0</ixz>
    <iyy>0.0558</iyy>
    <iyz>0.0</iyz>
    <izz>0.0347</izz>
  </inertia>
  <pose>0 0 0 0 0 0</pose>  <!-- Center of mass offset -->
</inertial>
```

**Estimation Methods**:
- **CAD Export**: Most accurate (from SolidWorks, Fusion360)
- **Mesh Analysis**: Use `gz sdf -p model.sdf` to auto-calculate
- **Approximation**: Treat links as simple shapes

**Educational Tool**:
```bash
# Gazebo provides inertia calculator
gz model -i model.sdf -o model_with_inertia.sdf
```

**3. Joint Configuration**

```xml
<joint name="left_knee" type="revolute">
  <parent>left_thigh</parent>
  <child>left_shin</child>
  <axis>
    <xyz>0 1 0</xyz>  <!-- Rotation axis (Y = pitch) -->
    <limit>
      <lower>0</lower>      <!-- 0° (straight) -->
      <upper>2.356</upper>  <!-- 135° -->
      <effort>100</effort>  <!-- Max torque (N*m) -->
      <velocity>5.0</velocity> <!-- Max speed (rad/s) -->
    </limit>
    <dynamics>
      <damping>0.5</damping>    <!-- Joint damping (N*m*s/rad) -->
      <friction>0.1</friction>  <!-- Coulomb friction (N*m) -->
    </dynamics>
  </axis>
</joint>
```

**Tuning Guidance**:
- **Damping**: Prevents oscillation (start with 0.5, increase if joints vibrate)
- **Friction**: Adds realism but can cause sticking (0.1-1.0 typical)
- **Effort Limits**: Match physical motor specs (100 N·m for humanoid leg joint)

### 5.2 Sensor Integration

**IMU (Inertial Measurement Unit)**:
```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.5 0 0 0</pose>  <!-- Torso center -->
  <always_on>true</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz typical -->
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><stddev>0.009</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.009</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.009</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><stddev>0.017</stddev></noise></x>
      <y><noise type="gaussian"><stddev>0.017</stddev></noise></y>
      <z><noise type="gaussian"><stddev>0.017</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    <ros>
      <remapping>~/out:=/imu/data</remapping>
    </ros>
  </plugin>
</sensor>
```

**Camera (RGB)**:
```xml
<sensor name="head_camera" type="camera">
  <pose>0.1 0 1.6 0 0 0</pose>  <!-- Head position -->
  <update_rate>30</update_rate>  <!-- 30 FPS -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin filename="gz-sim-camera-system" name="gz::sim::systems::Camera">
    <ros>
      <remapping>~/image:=/camera/image_raw</remapping>
      <remapping>~/camera_info:=/camera/camera_info</remapping>
    </ros>
  </plugin>
</sensor>
```

**Lidar (3D)**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 1.5 0 0 0</pose>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30° -->
        <max_angle>0.2618</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>50.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <plugin filename="gz-sim-lidar-system" name="gz::sim::systems::Lidar">
    <ros>
      <remapping>~/out:=/lidar/points</remapping>
    </ros>
  </plugin>
</sensor>
```

### 5.3 Performance Optimization

**1. Use Level-of-Detail (LOD) Meshes**:
- Visual: High-poly mesh (for rendering)
- Collision: Low-poly or primitive shapes

**2. Disable Shadows for Indoor Scenes**:
```xml
<scene>
  <shadows>false</shadows>  <!-- Saves 20-30% rendering time -->
</scene>
```

**3. Reduce Sensor Update Rates**:
- Camera: 30 Hz (not 60 Hz unless necessary)
- Lidar: 10 Hz (not 30 Hz unless SLAM required)
- IMU: 100 Hz (can go to 200 Hz if needed)

**4. Physics Multi-Threading**:
```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
  <!-- Gazebo Harmonic automatically uses multiple threads for collision detection -->
</physics>
```

---

## 6. Common Pitfalls and Troubleshooting

### 6.1 Robot Falls Through Ground

**Symptoms**: Robot spawns and immediately falls into void.

**Causes**:
1. Missing collision geometry on ground plane
2. Incorrect spawn height (z-coordinate too low)
3. Robot links have zero or very small mass

**Solutions**:
```xml
<!-- Ensure ground has collision -->
<model name="ground_plane">
  <static>true</static>  <!-- MUST be static -->
  <link name="link">
    <collision name="collision">  <!-- MUST have collision -->
      <geometry>
        <plane><normal>0 0 1</normal></plane>
      </geometry>
    </collision>
  </link>
</model>

<!-- Spawn robot above ground (z ≥ 1.0 for humanoid) -->
ros2 run ros_gz_sim create -z 1.0

<!-- Ensure all links have realistic mass -->
<inertial>
  <mass>5.0</mass>  <!-- NOT 0.0 or 0.001 -->
</inertial>
```

### 6.2 Joints Are Floppy / Robot Collapses

**Symptoms**: Robot spawns correctly but joints don't hold position.

**Causes**:
1. No joint controllers active
2. Joint effort limits too low
3. Missing or insufficient joint damping

**Solutions**:
```xml
<!-- Increase joint effort limits -->
<limit>
  <effort>100</effort>  <!-- N*m, NOT 1.0 -->
</limit>

<!-- Add joint damping to prevent oscillation -->
<dynamics>
  <damping>1.0</damping>  <!-- Start with 1.0, tune as needed -->
</dynamics>
```

**ROS 2 Controller Required**:
```bash
# Spawn joint state broadcaster and position controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_trajectory_controller
```

### 6.3 Simulation Runs Slower Than Real-Time

**Symptoms**: `real_time_factor < 1.0` in Gazebo stats.

**Diagnosis**:
```bash
# Check physics update rate
gz topic -e -t /stats

# Look for:
# real_time_factor: 0.5  (50% of real-time = TOO SLOW)
# sim_time: X
# real_time: Y
```

**Solutions** (in priority order):
1. **Simplify collision geometry** (biggest impact)
2. **Increase max_step_size** to 0.002 or 0.005 (less accurate but faster)
3. **Reduce sensor update rates** (camera 30→15 Hz, lidar 10→5 Hz)
4. **Disable shadows** in scene
5. **Use TPE physics** (Trivial Physics Engine) for simple scenarios

```xml
<!-- Fast physics for simple scenarios -->
<physics name="tpe_physics" type="tpe">
  <max_step_size>0.005</max_step_size>  <!-- 5ms = 200 Hz -->
</physics>
```

### 6.4 ROS 2 Topics Not Appearing

**Symptoms**: `ros2 topic list` doesn't show Gazebo sensor topics.

**Causes**:
1. `ros_gz_bridge` not running
2. Incorrect topic remapping in bridge
3. Gazebo plugin not loaded in sensor

**Solutions**:
```bash
# Check if bridge is running
ros2 node list | grep bridge

# Manually test bridge
ros2 run ros_gz_bridge parameter_bridge /model/humanoid/imu@sensor_msgs/msg/Imu[gz.msgs.IMU

# Verify Gazebo topic exists
gz topic -l | grep imu
```

**Launch File Fix**:
```python
# Ensure bridge runs AFTER robot spawns
spawn_then_bridge = [
    spawn_robot,
    TimerAction(period=1.0, actions=[bridge])
]
```

### 6.5 URDF Validation Errors

**Symptoms**: `Error: Could not load URDF file` or `parse error in <joint>`.

**Debugging Tools**:
```bash
# Check URDF syntax
check_urdf robot.urdf

# Visualize URDF without Gazebo
ros2 run urdf_tutorial display.launch.py model:=robot.urdf

# Convert URDF to SDF (shows conversion errors)
gz sdf -p robot.urdf
```

**Common Errors**:
```xml
<!-- BAD: Missing parent/child links -->
<joint name="elbow" type="revolute">
  <!-- ERROR: parent/child not defined -->
</joint>

<!-- GOOD: Explicit parent/child -->
<joint name="elbow" type="revolute">
  <parent>upper_arm</parent>
  <child>forearm</child>
  <axis><xyz>0 1 0</xyz></axis>
</joint>
```

---

## 7. Integration with ROS 2 Humble

### 7.1 ROS-Gazebo Bridge Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│   ROS 2 Nodes   │◄───────►│  ros_gz_bridge   │◄───────►│  Gazebo (gz-sim)│
│  (Controllers,  │  ROS 2  │  (Topic/Service  │ Gazebo  │  (Physics,      │
│   Planners)     │  Topics │   Translation)   │ Topics  │   Sensors)      │
└─────────────────┘         └──────────────────┘         └─────────────────┘
```

**Key Packages**:
- `ros_gz_sim`: Gazebo Harmonic launch integration
- `ros_gz_bridge`: Topic/service translation
- `ros_gz_image`: Image transport bridge
- `ros_gz_control`: ros2_control integration

### 7.2 Topic Mapping Examples

**Bidirectional Bridge Configuration**:
```yaml
# bridge_config.yaml
topics:
  # Gazebo → ROS 2 (sensor data)
  - gz_topic: /world/training_world/model/humanoid/joint_state
    ros_topic: /joint_states
    gz_type: gz.msgs.Model
    ros_type: sensor_msgs/msg/JointState
    direction: GZ_TO_ROS

  # ROS 2 → Gazebo (commands)
  - gz_topic: /model/humanoid/cmd_vel
    ros_topic: /cmd_vel
    gz_type: gz.msgs.Twist
    ros_type: geometry_msgs/msg/Twist
    direction: ROS_TO_GZ

  # Bidirectional (clock sync)
  - gz_topic: /clock
    ros_topic: /clock
    gz_type: gz.msgs.Clock
    ros_type: rosgraph_msgs/msg/Clock
    direction: BIDIRECTIONAL
```

**Launch Bridge with Config**:
```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config_yaml}],
    output='screen'
)
```

### 7.3 Time Synchronization

**Critical for Control Loops**:
```python
# Python node with Gazebo time sync
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller',
                         parameter_overrides=[
                             {'use_sim_time': True}  # CRITICAL: Use Gazebo time
                         ])
        self.clock = Clock(clock_type=ClockType.ROS_TIME)

    def control_loop(self):
        current_time = self.clock.now()
        # Use current_time for control calculations
```

**Launch File**:
```python
SetParameter(name='use_sim_time', value=True)  # Global parameter
```

### 7.4 Version Compatibility Matrix

| ROS 2 Version | Gazebo Version | Status | Notes |
|---------------|----------------|--------|-------|
| Humble (LTS) | Fortress | Supported | Requires `ros_gz` packages |
| Humble (LTS) | **Harmonic** | **Recommended** | Best performance, latest features |
| Iron | Harmonic | Supported | Non-LTS ROS 2 release |
| Jazzy (LTS) | Harmonic | Supported | Future-proof (2024+) |

**Installation Command** (Ubuntu 22.04):
```bash
# ROS 2 Humble + Gazebo Harmonic
sudo apt install ros-humble-ros-gz
sudo apt install gz-harmonic
```

---

## 8. Educational Content Recommendations

### 8.1 Progressive Learning Path

**Chapter 5: Gazebo Fundamentals**
1. Start with empty world + ground plane
2. Add single rigid body (box)
3. Introduce sensors (camera on static object)
4. Spawn simple robot (2-3 DOF arm)
5. Finally: Humanoid robot

**Pedagogical Rationale**: Build complexity gradually, isolate concepts.

**Chapter 6: URDF Physics Configuration**
1. Start with single-link robot (just a torso)
2. Add one joint (neck rotation)
3. Demonstrate effect of missing inertia
4. Add sensors one at a time
5. Build to full humanoid

**Avoid**: Presenting complete 30-DOF humanoid URDF on first page.

### 8.2 Hands-On Exercises

**Exercise 1: "Break the Physics"**
- Task: Modify URDF to make robot fall through ground
- Learning: Understanding collision geometry importance

**Exercise 2: "Tune for Stability"**
- Task: Adjust joint damping to stop oscillation
- Learning: Physics parameter effects

**Exercise 3: "Sensor Verification"**
- Task: Add IMU, verify orientation matches expected values
- Learning: Sensor placement and ROS 2 integration

**Exercise 4: "Performance Profiling"**
- Task: Measure physics update rate with different collision meshes
- Learning: Performance tradeoffs

### 8.3 Common Student Mistakes to Address

1. **Forgetting `use_sim_time=True`**: ROS 2 nodes use wall clock instead of sim time
2. **Spawning at z=0**: Robot intersects ground, physics explodes
3. **High-poly collision meshes**: 10 Hz physics updates, thinks computer is slow
4. **No joint controllers**: Robot collapses, thinks Gazebo is broken
5. **Mixing URDF and SDF**: Confusion about which format to use when

### 8.4 Troubleshooting Decision Tree (For Students)

```
Robot not appearing?
├─ Yes → Check Gazebo console for errors
│        ├─ "Could not load" → URDF syntax error (run check_urdf)
│        └─ No errors → Spawn position outside camera view (adjust camera)
└─ No, but falls through ground
   ├─ Check ground plane has collision geometry
   ├─ Check spawn height (z ≥ 1.0)
   └─ Check link masses (not zero)

Simulation too slow?
├─ Check real_time_factor in Gazebo stats
├─ Simplify collision geometry (boxes not meshes)
├─ Reduce sensor update rates
└─ Increase max_step_size (0.001 → 0.005)

ROS 2 topics missing?
├─ Check ros_gz_bridge is running (ros2 node list)
├─ Verify topic names (gz topic -l vs ros2 topic list)
├─ Check sensor plugins loaded in URDF
└─ Test manual bridge (ros2 run ros_gz_bridge parameter_bridge)
```

---

## 9. Example: Complete Humanoid Simulation Setup

### 9.1 Minimal Working Example

**File Structure**:
```
humanoid_simulation/
├── worlds/
│   └── training_world.sdf
├── urdf/
│   └── simple_humanoid.urdf
├── launch/
│   └── spawn_humanoid.launch.py
└── config/
    └── ros_gz_bridge.yaml
```

**World File** (`training_world.sdf`):
```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="training_world">
    <physics name="dart" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
    </scene>

    <light name="sun" type="directional">
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch File** (`spawn_humanoid.launch.py`):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r training_world.sdf'
        }.items()
    )

    # Spawn robot (after 2 second delay)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', 'simple_humanoid.urdf',
            '-name', 'humanoid',
            '-z', '1.0'
        ]
    )

    # Bridge joint states
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ]
    )

    return LaunchDescription([gazebo, spawn, bridge])
```

**Usage**:
```bash
ros2 launch humanoid_simulation spawn_humanoid.launch.py
```

### 9.2 Verification Checklist

1. **Gazebo launches without errors**
   ```bash
   gz sim training_world.sdf
   ```

2. **Robot appears in simulation**
   - Visual model visible
   - Standing on ground (not falling through)

3. **Physics is active**
   - Robot responds to gravity
   - Collisions detected (click robot, it moves)

4. **ROS 2 topics available**
   ```bash
   ros2 topic list | grep joint_states
   ros2 topic echo /joint_states
   ```

5. **Time synchronization working**
   ```bash
   ros2 topic echo /clock
   # Should increment with simulation time
   ```

---

## 10. Advanced Topics (For Later Modules)

**Out of Scope for Module 2, but important to mention**:

1. **Custom Gazebo Plugins** (C++)
   - Writing sensor plugins
   - Model plugins for custom physics
   - System plugins for world-level behavior

2. **Reinforcement Learning Integration**
   - Reset mechanisms
   - Reward signal publishing
   - Domain randomization

3. **Sim-to-Real Transfer**
   - Sensor noise modeling
   - Latency injection
   - Domain adaptation techniques

4. **Multi-Robot Simulation**
   - Namespace management
   - Collision avoidance
   - Coordinated spawning

---

## References and Further Reading

### Official Documentation
- **Gazebo Harmonic Docs**: https://gazebosim.org/docs/harmonic
- **SDF Specification**: http://sdformat.org/spec
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **ros_gz Packages**: https://github.com/gazebosim/ros_gz

### Key API References
- `gz-sim` API: https://gazebosim.org/api/sim/8/
- `gz-physics` API: https://gazebosim.org/api/physics/7/
- `ros_gz_bridge`: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge

### Community Resources
- Gazebo Community Forum: https://community.gazebosim.org/
- ROS Answers (Gazebo tag): https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:gazebo/

### Academic Papers (For Deep Dives)
- Koenig, N., & Howard, A. (2004). "Design and use paradigms for Gazebo, an open-source multi-robot simulator." *IEEE/RSJ IROS*.
- Collins, S. H., et al. (2005). "Efficient bipedal robots based on passive-dynamic walkers." *Science*, 307(5712), 1082-1085. (Physics principles)

---

## Appendix: Quick Reference Commands

### Gazebo Commands
```bash
# Launch Gazebo with world
gz sim world.sdf

# Launch in headless mode (no GUI)
gz sim -s world.sdf

# List running worlds
gz world list

# Pause/unpause simulation
gz world pause
gz world play

# Get model info
gz model -m robot_name -i

# Teleport model
gz model -m robot_name -x 1 -y 2 -z 3
```

### ROS 2 Commands
```bash
# Spawn robot
ros2 run ros_gz_sim create -file robot.urdf -name my_robot -z 1.0

# List Gazebo topics
gz topic -l

# Echo Gazebo topic
gz topic -e -t /topic_name

# Bridge single topic
ros2 run ros_gz_bridge parameter_bridge /topic@msg_type

# Check time sync
ros2 param get /node_name use_sim_time
```

### Debugging
```bash
# Validate URDF
check_urdf robot.urdf

# Convert URDF to SDF
gz sdf -p robot.urdf

# Check for URDF errors
urdf_to_graphviz robot.urdf

# Monitor physics performance
gz topic -e -t /stats
```

---

**Document Version**: 1.0
**Last Updated**: 2025-12-17
**Author**: Research synthesis for Physical AI & Humanoid Robotics educational content
**Target Audience**: Technical authors, educators, robotics students (intermediate level)
