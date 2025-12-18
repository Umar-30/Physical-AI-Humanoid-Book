# Chapter 6: URDF Sensors, Collisions, Inertia, and Gravity

**Priority**: P2 (Builds on Chapter 5)
**Estimated Duration**: 3-4 hours
**Target Audience**: Readers who completed Chapter 5 (Gazebo basics, robot spawning)

## Learning Objectives

By the end of this chapter, readers will be able to:
1. **Integrate** sensors (IMU, cameras, lidar, force-torque) into URDF models with Gazebo plugins
2. **Configure** collision geometries using primitives and simplified meshes for performance
3. **Calculate** inertia tensors for robot links using geometric formulas and CAD tools
4. **Set** joint parameters (friction, damping, limits) for realistic humanoid behavior
5. **Validate** URDF physics configuration through testing and simulation

## Prerequisites

- Chapter 5 completed (Gazebo Harmonic installation, SDF worlds, robot spawning)
- Basic understanding of 3D coordinate systems (from Module 1)
- Familiarity with URDF link and joint structure (Module 1, Chapter 4)

## Chapter Structure

### 6.1 Sensor Integration (45 min)

**Objective**: Add sensors to URDF and verify ROS 2 topic publication

**Content**:
- Gazebo sensor plugin architecture
- IMU sensor configuration (angular velocity, linear acceleration, noise models)
- Camera sensor configuration (resolution, FOV, frame rate)
- Lidar/laser scanner configuration (scan range, angular resolution)
- Force-torque sensors (for contact sensing)
- ROS 2 topic remapping and namespacing

**Sensor Types**:

1. **IMU (Inertial Measurement Unit)**
   - Measures orientation, angular velocity, linear acceleration
   - Use case: Robot balance control, state estimation
   - Update rate: 100-200 Hz

2. **Camera (RGB)**
   - Provides visual feedback
   - Use case: Vision-based navigation, object detection
   - Update rate: 30 Hz
   - Resolution: 640x480 (VGA) to 1920x1080 (HD)

3. **Lidar/Laser Scanner**
   - Measures distance to obstacles
   - Use case: Obstacle avoidance, SLAM
   - Update rate: 10-30 Hz
   - Range: 0.1m - 30m

4. **Force-Torque Sensor**
   - Measures contact forces
   - Use case: Manipulation, contact detection
   - Update rate: 100 Hz

**Configuration Files**:
1. `robot_with_imu.urdf` - Humanoid with IMU in torso
2. `robot_with_camera.urdf` - Humanoid with head-mounted camera
3. `robot_with_lidar.urdf` - Humanoid with chest-mounted lidar
4. `robot_full_sensors.urdf` - All sensors integrated

**Code Example: IMU Integration**:
```xml
<link name="imu_link">
  <inertial>
    <mass value="0.001"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Verification Commands**:
```bash
# List ROS 2 topics (sensor data should appear)
ros2 topic list | grep -E '(imu|camera|scan)'

# Echo sensor data
ros2 topic echo /robot/imu/data
ros2 topic echo /robot/camera/image_raw
ros2 topic echo /robot/scan

# Check publishing rate
ros2 topic hz /robot/imu/data
# Expected: ~100 Hz for IMU
```

**Diagrams**:
- `sensor-placement-humanoid.svg`: Recommended sensor locations on humanoid
- `ros2-topic-graph.svg`: Gazebo → ROS 2 topic flow

**Deliverables**:
- 4 URDF files with progressive sensor integration
- Sensor configuration reference table
- ROS 2 topic verification checklist

**Validation**:
- All sensors publish to correct ROS 2 topics
- Data rates match configured update_rate
- Sensor noise is realistic (not zero, not excessive)

---

### 6.2 Collision Geometry Optimization (40 min)

**Objective**: Configure collision meshes for performance and accuracy

**Content**:
- Visual vs collision geometry separation
- Primitive shapes (box, cylinder, sphere, capsule)
- Mesh simplification techniques
- Convex decomposition for complex shapes
- Performance impact measurement

**Best Practices**:

| Geometry Type | Use Case | Performance | Accuracy |
|---------------|----------|-------------|----------|
| Primitives (box, cylinder, sphere) | Simple approximations | Excellent (10-100x faster) | Low-Medium |
| Simplified mesh (<100 triangles) | Moderate detail | Good | Medium |
| Convex hulls | Irregular shapes | Fair | Medium-High |
| Concave meshes | **Avoid** (unstable) | Poor | High (but unreliable) |

**Code Example: Visual vs Collision Separation**:
```xml
<link name="torso">
  <!-- High-poly visual mesh (appearance) -->
  <visual>
    <geometry>
      <mesh>
        <uri>file://meshes/torso_visual.dae</uri>
        <scale>1 1 1</scale>
      </mesh>
    </geometry>
    <material>
      <ambient>0.5 0.5 0.5 1</ambient>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </material>
  </visual>

  <!-- Low-poly collision approximation (physics) -->
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>  <!-- Simple box approximation -->
    </geometry>
  </collision>

  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.417" ixy="0" ixz="0" iyy="0.792" iyz="0" izz="0.542"/>
  </inertial>
</link>
```

**Tools**:
- **MeshLab**: Mesh simplification (Filters → Remeshing → Quadric Edge Collapse Decimation)
- **Blender**: Manual mesh editing and convex hull generation
- **V-HACD**: Automatic convex decomposition (https://github.com/kmammou/v-hacd)

**Configuration Files**:
1. `collision_comparison.urdf` - Same robot with 3 collision complexity levels
2. `collision_primitives.urdf` - All primitives (box, cylinder, sphere)
3. `collision_simplified_mesh.urdf` - Mesh-based collision (<100 tri)

**Benchmarking**:
```bash
# Measure physics update rate with different collision configs
gz stats -p /stats

# Compare:
# - Primitives only: ~500-1000 Hz
# - Simplified meshes: ~200-500 Hz
# - Complex meshes: <100 Hz (avoid!)
```

**Diagrams**:
- `collision-complexity-comparison.png`: Side-by-side visual/collision meshes
- `performance-vs-accuracy.svg`: Graph showing trade-off

**Deliverables**:
- Collision geometry design guidelines
- Mesh simplification tutorial (MeshLab/Blender)
- Performance benchmark results table

**Validation**:
- Physics update rate >100 Hz with full humanoid robot
- Collision detection works correctly (robot doesn't fall through objects)
- No interpenetration or jitter

---

### 6.3 Inertia Tensor Calculation (35 min)

**Objective**: Compute accurate inertia properties for robot links

**Content**:
- Inertia tensor definition (resistance to rotational motion)
- Moment of inertia calculation for common shapes
- CAD-derived inertia (SolidWorks, Fusion 360, FreeCAD)
- Validation through simulation testing
- Impact of incorrect inertia on simulation

**Theory**:
- **Inertia Tensor**: 3x3 matrix describing rotational inertia
  ```
  I = | ixx  ixy  ixz |
      | ixy  iyy  iyz |
      | ixz  iyz  izz |
  ```
- Diagonal elements (ixx, iyy, izz): Resistance to rotation around principal axes
- Off-diagonal elements (ixy, ixz, iyz): Coupling between axes (often zero for symmetric bodies)

**Calculation Methods**:

**Method 1: Geometric Formulas** (for simple shapes):

**Box** (width × depth × height):
```
ixx = (mass / 12) * (depth² + height²)
iyy = (mass / 12) * (width² + height²)
izz = (mass / 12) * (width² + depth²)
```

**Cylinder** (radius, height, along z-axis):
```
ixx = iyy = (mass / 12) * (3 * radius² + height²)
izz = (mass / 2) * radius²
```

**Sphere** (radius):
```
ixx = iyy = izz = (2/5) * mass * radius²
```

**Method 2: CAD Software**:
1. Create 3D model with accurate geometry
2. Assign material density
3. Export mass properties (mass, center of mass, inertia tensor)
4. Copy values to URDF

**Method 3: Gazebo Inertia Calculator**:
```bash
# Estimate inertia from mesh (less accurate but quick)
gz sim --inertia-calculator <mesh_file>
```

**Code Example: Python Inertia Calculator**:
```python
# inertia_calc.py
import math

def box_inertia(mass, width, depth, height):
    ixx = (mass / 12.0) * (depth**2 + height**2)
    iyy = (mass / 12.0) * (width**2 + height**2)
    izz = (mass / 12.0) * (width**2 + depth**2)
    return ixx, iyy, izz

def cylinder_inertia(mass, radius, height):
    ixx = iyy = (mass / 12.0) * (3 * radius**2 + height**2)
    izz = (mass / 2.0) * radius**2
    return ixx, iyy, izz

def sphere_inertia(mass, radius):
    i = (2.0 / 5.0) * mass * radius**2
    return i, i, i

# Example: Torso as box
mass = 10.0  # kg
width, depth, height = 0.3, 0.2, 0.5  # meters
ixx, iyy, izz = box_inertia(mass, width, depth, height)
print(f"Inertia: ixx={ixx:.3f}, iyy={iyy:.3f}, izz={izz:.3f}")
```

**URDF Integration**:
```xml
<link name="torso">
  <inertial>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>  <!-- Center of mass -->
    <mass value="10.0"/>
    <inertia ixx="0.417" ixy="0.0" ixz="0.0"
             iyy="0.792" iyz="0.0"
             izz="0.542"/>
  </inertial>
  <!-- visual and collision elements -->
</link>
```

**Validation Tests**:
1. **Drop Test**: Robot falls straight down without unexpected rotation
2. **Balance Test**: Standing humanoid doesn't drift or tip over
3. **Energy Conservation**: Total energy remains constant in frictionless simulation

**Configuration Files**:
1. `robot_with_inertia_calculated.urdf` - All links with proper inertia
2. `robot_bad_inertia.urdf` - Intentionally wrong inertia (for comparison)

**Diagrams**:
- `inertia-tensor-visualization.svg`: Visual explanation of inertia tensor
- `cad-inertia-export.png`: Screenshot of inertia export from FreeCAD

**Deliverables**:
- Python inertia calculator script
- CAD inertia export tutorial (FreeCAD)
- Inertia validation test procedures

**Validation**:
- Robot passes drop and balance tests
- No unexpected drifting or instability
- Physics behavior appears realistic

---

### 6.4 Joint Configuration (35 min)

**Objective**: Set friction, damping, and limits for realistic joint behavior

**Content**:
- Joint friction (static and dynamic)
- Joint damping (velocity-dependent resistance)
- Effort limits (maximum torque)
- Velocity limits (maximum angular velocity)
- Tuning guidelines for humanoid robots

**Joint Parameters**:

| Parameter | Description | Typical Range (Humanoid) |
|-----------|-------------|--------------------------|
| **friction** | Resistance to motion (energy loss) | 0.05 - 0.2 |
| **damping** | Velocity-proportional resistance | 0.5 - 2.0 |
| **effort** (limit) | Maximum torque (Nm) | 20 - 200 (depends on joint) |
| **velocity** (limit) | Maximum angular velocity (rad/s) | 2.0 - 10.0 |

**Code Example: Joint Configuration**:
```xml
<!-- Knee Joint (revolute, 1 DOF) -->
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->

  <!-- Dynamic properties -->
  <dynamics friction="0.1" damping="0.7"/>

  <!-- Physical limits -->
  <limit lower="0.0" upper="2.356" effort="100.0" velocity="5.0"/>
  <!--   lower: 0° (straight leg)
         upper: 135° (fully bent)
         effort: 100 Nm max torque
         velocity: 5 rad/s max speed -->
</joint>

<!-- Shoulder Pitch Joint (requires higher torque) -->
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.0 0.15 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>

  <dynamics friction="0.15" damping="1.0"/>  <!-- Higher damping for stability -->
  <limit lower="-2.094" upper="2.094" effort="150.0" velocity="4.0"/>
  <!--   Range: -120° to +120° -->
</joint>
```

**Tuning Guidelines**:

**Friction**:
- Start with 0.1 for most joints
- Increase (0.15-0.2) for gear-driven actuators or high-load joints
- Too high → sluggish movement; Too low → unrealistic (frictionless)

**Damping**:
- Start with 0.7 for most joints
- Increase (1.0-2.0) to prevent oscillation in long limbs (legs, arms)
- Too high → overdamped (slow response); Too low → oscillation/instability

**Effort Limits** (match real motor specs):
- Small joints (fingers, neck): 5-20 Nm
- Medium joints (elbows, ankles): 30-80 Nm
- Large joints (hips, knees, shoulders): 100-200 Nm

**Velocity Limits**:
- Fast movements (throwing, kicking): 8-10 rad/s
- Normal locomotion: 3-5 rad/s
- Slow manipulation: 1-2 rad/s

**Debugging Poor Tuning**:
- **Oscillation**: Increase damping
- **Sluggish response**: Decrease friction/damping
- **Joint explosions/instability**: Check inertia tensors first, then reduce velocity limits
- **Unrealistic motion**: Compare to videos of real robots, adjust friction

**Configuration Files**:
1. `robot_joint_tuned.urdf` - Well-tuned humanoid joint parameters
2. `robot_joint_comparison.urdf` - Multiple parameter sets for comparison

**Code Example: Testing Joint Behavior**:
```bash
# Launch robot with joint state publisher GUI (manual control)
ros2 launch robot_description display.launch.py

# Apply joint commands programmatically
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory '{...}'

# Monitor joint states
ros2 topic echo /joint_states
```

**Deliverables**:
- Joint parameter tuning guide
- Reference table for typical humanoid joint limits
- Debugging checklist for joint instability

**Validation**:
- Joints move smoothly without oscillation
- Motion appears realistic (not too slow/fast)
- No explosions or NaN values in simulation

---

### 6.5 Physics Validation Techniques (25 min)

**Objective**: Verify URDF physics configuration accuracy

**Content**:
- Drop test (gravity and collision response)
- Balance test (stability with proper inertia)
- Energy conservation test (no spurious energy gain/loss)
- Sensor data validation (IMU, force-torque)
- Comparison with real-world robot data (if available)

**Test Procedures**:

**Test 1: Drop Test**
```
1. Spawn robot at height (z=2.0m)
2. Observe free fall
3. Check:
   - Falls straight down (no unexpected rotation)
   - Lands on ground plane (no interpenetration)
   - Settles to stable pose
   - No bouncing or jitter
```

**Test 2: Balance Test**
```
1. Spawn humanoid in standing pose
2. Run simulation for 10 seconds
3. Check:
   - Robot remains upright (no tipping)
   - No drift or rotation
   - Center of mass remains stable
```

**Test 3: Energy Conservation**
```
1. Launch robot in frictionless environment (friction=0, damping=0)
2. Apply initial velocity/momentum
3. Monitor total energy over time
4. Check:
   - Total energy remains constant (±1%)
   - No spurious energy gain (explosions)
   - No energy loss (without friction)
```

**Test 4: Sensor Data Validation**
```
1. Check IMU reads gravity when stationary (az ≈ 9.81 m/s²)
2. Rotate robot, verify angular velocity matches
3. Move robot, verify linear acceleration matches
4. Camera images display correctly (no distortion)
5. Lidar scans detect obstacles accurately
```

**Code Example: Automated Test Script**:
```python
# test_physics.py
import rclpy
from sensor_msgs.msg import Imu

def test_imu_gravity():
    """Verify IMU reports gravity when robot is stationary"""
    # Subscribe to IMU topic
    # Check linear_acceleration.z ≈ 9.81 m/s²
    pass

def test_robot_stability():
    """Verify robot doesn't drift when standing"""
    # Record initial pose
    # Wait 10 seconds
    # Check final pose is within tolerance
    pass

# Run tests
if __name__ == '__main__':
    rclpy.init()
    test_imu_gravity()
    test_robot_stability()
    print("All tests passed!")
```

**Diagrams**:
- `validation-workflow.svg`: Flowchart of validation process

**Deliverables**:
- Automated test script (Python)
- Manual validation checklist
- Expected vs actual results comparison table

**Validation**:
- All tests pass without manual intervention
- Results match expected physics behavior
- No warnings or errors in Gazebo logs

---

### 6.6 Hands-On Exercises (45-60 min)

**Exercise 6.1: Add Sensors to Custom Robot** (Intermediate, 30 min)

**Instructions**:
1. Start with `example_humanoid.urdf` (provided)
2. Add IMU sensor to torso
3. Add camera sensor to head
4. Add lidar sensor to chest
5. Launch robot in Gazebo and verify sensors publish data

**Success Criteria**:
- [ ] All 3 sensors appear in `ros2 topic list`
- [ ] IMU publishes at ~100 Hz
- [ ] Camera publishes images at ~30 Hz
- [ ] Lidar publishes scans at ~10 Hz
- [ ] No errors in Gazebo logs

**Solution**: `/files/module-2/solutions/exercise-6-1-robot-with-sensors.urdf`

---

**Exercise 6.2: Optimize Collision Geometry** (Intermediate, 20 min)

**Instructions**:
1. Load provided `robot_complex_collision.urdf` (high-poly collision meshes)
2. Measure baseline physics rate with `gz stats`
3. Replace collision meshes with primitive approximations
4. Measure improved physics rate
5. Verify collision detection still works (robot doesn't fall through floor)

**Success Criteria**:
- [ ] Baseline rate measured (<100 Hz expected)
- [ ] Optimized rate measured (>300 Hz expected)
- [ ] At least 3x performance improvement
- [ ] Collision detection remains functional

**Solution**: Performance tuning guide with before/after comparison

---

**Exercise 6.3: Calculate and Validate Inertia** (Advanced, 25 min)

**Instructions**:
1. Use provided CAD model of robot link (FreeCAD format)
2. Export mass properties (mass, inertia tensor)
3. Update URDF with exported values
4. Run drop test and balance test
5. Compare with intentionally incorrect inertia (provided)

**Success Criteria**:
- [ ] Inertia values correctly exported from CAD
- [ ] URDF updated with proper format
- [ ] Drop test: robot falls straight without rotation
- [ ] Balance test: robot remains stable
- [ ] Incorrect inertia causes visible instability (demonstrates importance)

**Solution**: `/files/module-2/solutions/exercise-6-3-inertia-validation.urdf`

---

## Chapter Summary

**Key Takeaways**:
1. Sensors require Gazebo plugins to publish data to ROS 2 topics
2. Collision geometry should be simpler than visual geometry for performance
3. Accurate inertia tensors are critical for realistic physics simulation
4. Joint friction and damping prevent oscillation and add realism
5. Validation tests ensure URDF physics behaves correctly

**What's Next**:
- Chapter 7 introduces Unity for enhanced visualization
- Chapter 8 integrates Gazebo (physics) with Unity (rendering)

**Further Reading**:
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/ros_gz)

---

## Validation Checklist (for Authors)

- [ ] All URDF files validated with `check_urdf`
- [ ] All sensors successfully publish to ROS 2 topics
- [ ] Collision optimization shows measurable performance improvement
- [ ] Inertia calculation methods tested with geometric formulas and CAD
- [ ] Joint tuning guidelines verified with humanoid model
- [ ] All validation tests pass without errors
- [ ] Exercises have verified solutions
- [ ] Docusaurus build succeeds for this chapter
- [ ] External reviewer completed chapter successfully
