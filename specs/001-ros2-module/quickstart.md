# ROS 2 Quickstart Reference

**Feature**: 001-ros2-module
**Date**: 2025-12-16
**Purpose**: Quick reference guide for ROS 2 Humble setup and essential commands

---

## Prerequisites

- Ubuntu 22.04 LTS
- Terminal access
- Internet connection

---

## Installation (Quick)

```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y

# Source ROS 2 (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Verify installation
ros2 --version
```

**Reference**: https://docs.ros.org/en/humble/Installation.html

---

## Essential Commands

### Environment Setup

```bash
# Source ROS 2 (required in every terminal)
source /opt/ros/humble/setup.bash

# Source workspace overlay (after building)
source ~/ros2_ws/install/setup.bash

# Check ROS 2 version
ros2 --version

# Check environment variables
printenv | grep ROS
```

---

### Package Management

```bash
# List installed packages
ros2 pkg list

# Get package information
ros2 pkg prefix <package_name>

# Create new Python package
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy std_msgs

# Create new C++ package
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp std_msgs
```

---

### Workspace Management

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with symbolic links (no reinstall needed for Python changes)
colcon build --symlink-install

# Clean build artifacts
rm -rf build/ install/ log/
```

---

### Node Operations

```bash
# Run a node
ros2 run <package_name> <executable_name>

# Run demo talker/listener
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

# List running nodes
ros2 node list

# Get node information
ros2 node info <node_name>
```

---

### Topic Operations

```bash
# List all topics
ros2 topic list

# List topics with types
ros2 topic list -t

# Echo topic messages (live stream)
ros2 topic echo <topic_name>

# Get topic information
ros2 topic info <topic_name>

# Show topic type
ros2 topic type <topic_name>

# Publish to topic (one-time)
ros2 topic pub --once <topic_name> <msg_type> "<data>"

# Publish to topic (continuous at 10Hz)
ros2 topic pub --rate 10 <topic_name> <msg_type> "<data>"

# Example: Publish String message
ros2 topic pub --once /hello std_msgs/msg/String "{data: 'Hello ROS 2'}"

# Show topic publication rate
ros2 topic hz <topic_name>

# Show topic bandwidth
ros2 topic bw <topic_name>
```

---

### Service Operations

```bash
# List all services
ros2 service list

# List services with types
ros2 service list -t

# Get service type
ros2 service type <service_name>

# Call a service
ros2 service call <service_name> <service_type> "<request_data>"

# Example: Call AddTwoInts service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

---

### Action Operations

```bash
# List all actions
ros2 action list

# List actions with types
ros2 action list -t

# Get action information
ros2 action info <action_name>

# Send action goal
ros2 action send_goal <action_name> <action_type> "<goal_data>"

# Example: Send Fibonacci goal
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Send goal with feedback
ros2 action send_goal --feedback /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

---

### Parameter Operations

```bash
# List parameters for a node
ros2 param list

# Get parameter value
ros2 param get <node_name> <parameter_name>

# Set parameter value
ros2 param set <node_name> <parameter_name> <value>

# Dump all parameters to file
ros2 param dump <node_name> > params.yaml

# Load parameters from file
ros2 param load <node_name> params.yaml
```

---

### Interface Information

```bash
# List all message types
ros2 interface list msg

# List all service types
ros2 interface list srv

# List all action types
ros2 interface list action

# Show message definition
ros2 interface show std_msgs/msg/String

# Show service definition
ros2 interface show example_interfaces/srv/AddTwoInts

# Show action definition
ros2 interface show action_tutorials_interfaces/action/Fibonacci

# Search for interface containing a string
ros2 interface package <package_name>
```

---

### Launch Files

```bash
# Run a launch file
ros2 launch <package_name> <launch_file_name>

# Example: Launch turtlesim demo
ros2 launch turtlesim multisim.launch.py

# List available launch files in package
ros2 pkg list --packages-with-launch-files
```

---

### Visualization and Debugging

```bash
# Run RViz (3D visualization tool)
rviz2

# Run rqt (Qt-based GUI)
rqt

# Run rqt_graph (computation graph visualizer)
rqt_graph

# Run rqt_console (log message viewer)
rqt_console

# View TF tree
ros2 run tf2_tools view_frames
```

---

### URDF Tools

```bash
# Validate URDF file
check_urdf <file.urdf>

# Convert URDF to graphviz (PDF visualization)
urdf_to_graphviz <file.urdf>

# Display URDF in RViz (requires robot_state_publisher)
ros2 launch urdf_tutorial display.launch.py model:=<path_to_urdf>
```

---

## Common QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Sensor data QoS (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Service default QoS (reliable, volatile)
service_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Parameter events QoS (reliable, transient local)
param_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1000
)
```

---

## Minimal Python Publisher Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Minimal Python Subscriber Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Troubleshooting

### Node Discovery Issues

**Problem**: Nodes can't see each other

**Solutions**:
- Check `ROS_DOMAIN_ID` matches: `echo $ROS_DOMAIN_ID`
- Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
- Verify nodes are running: `ros2 node list`
- Check firewall rules (allow UDP multicast)

### QoS Mismatch

**Problem**: Publisher/subscriber don't connect

**Solutions**:
- Check QoS profiles with `ros2 topic info /topic -v`
- Ensure reliability matches (both BEST_EFFORT or both RELIABLE)
- RELIABLE publisher can talk to BEST_EFFORT subscriber, but not vice versa

### Build Errors

**Problem**: `colcon build` fails

**Solutions**:
- Ensure all dependencies in `package.xml`
- Check Python syntax (no tabs, proper indentation)
- Clean build: `rm -rf build/ install/ log/`
- Source ROS 2 before building

### Import Errors

**Problem**: `ModuleNotFoundError: No module named 'my_package'`

**Solutions**:
- Ensure package was built: `colcon build --packages-select my_package`
- Source workspace: `source install/setup.bash`
- Check entry points in `setup.py`

---

## Useful Aliases

Add to `~/.bashrc`:

```bash
alias cb='colcon build --symlink-install'
alias cbs='colcon build --symlink-install --packages-select'
alias source_ros='source /opt/ros/humble/setup.bash'
alias source_ws='source ~/ros2_ws/install/setup.bash'
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rnl='ros2 node list'
```

---

## References

- **Official ROS 2 Docs**: https://docs.ros.org/en/humble/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **rclpy API**: https://docs.ros.org/en/humble/p/rclpy/
- **URDF Tutorials**: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- **ROS Answers**: https://answers.ros.org/

---

**Document Status**: Complete ✅
**Last Updated**: 2025-12-16
