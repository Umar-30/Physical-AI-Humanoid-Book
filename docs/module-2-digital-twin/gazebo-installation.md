# Ignition Gazebo Fortress Installation and Configuration

This section details the installation and basic configuration of Ignition Gazebo Fortress, the default simulator for ROS 2 Humble.

## 1. Install Ignition Gazebo Fortress

Ignition Gazebo (now simply called Gazebo) is installed via `apt` for Ubuntu. Follow the official instructions:

```bash
# Add the Ignition GPG key
sudo apt update
sudo apt install -y software-properties-common lsb-release
sudo add-apt-repository -y "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-noble $(lsb_release -cs) main"
# Note: For Humble (Ubuntu 22.04 Jammy), the packages are in the 'noble' repository. This is intentional.

# Install Gazebo Fortress
sudo apt update
sudo apt install -y ignition-fortress

# Install ROS 2 Gazebo bridge packages
sudo apt install -y ros-humble-ros-gz
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

## 2. Environment Setup

To ensure ROS 2 can find Ignition Gazebo, you'll need to source its setup files. Add the following to your `~/.bashrc` (or `~/.zshrc`):

```bash
echo "source /usr/share/ignition/ignition-fortress/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc # Or rmw_cyclonedds_cpp if preferred
source ~/.bashrc
```

Verify the installation by launching a simple Gazebo world:

```bash
ign gazebo shapes.sdf
```

You should see a Gazebo window with some basic shapes. Close it to continue.

## 3. Basic ROS 2 Integration Test

Let's test the basic integration between ROS 2 and Gazebo.

First, ensure `ros-humble-ros-gz` and `ros-humble-ros-gz-sim` are installed.

Create a ROS 2 workspace (if you don't have one) and a package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_gazebo_pkg
cd my_gazebo_pkg
```

Now, create a simple `box.sdf` file inside `~/ros2_ws/src/my_gazebo_pkg/worlds` (you might need to create the `worlds` directory):

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

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
                <mu>1.0</mu>
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
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="simple_box">
      <link name="box_link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
