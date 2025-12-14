# Environment Setup and Prerequisites

This module assumes you are running **Ubuntu 22.04 LTS (Jammy Jellyfish)** and have a functional installation of **ROS 2 Humble Hawksbill**. If you do not meet these prerequisites, please follow the official ROS 2 documentation to set up your environment before proceeding.

## 1. Verify Your Ubuntu and ROS 2 Installation

First, ensure your Ubuntu system is up-to-date and your ROS 2 Humble installation is correctly sourced.

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Source your ROS 2 environment (if not already in your .bashrc/.zshrc)
# Replace 'your_ros2_workspace' with the path to your ROS 2 workspace if you have one
# Otherwise, just source the default ROS 2 installation
source /opt/ros/humble/setup.bash
```

To verify your ROS 2 installation, run a simple demo:

```bash
# Open a new terminal and source ROS 2
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node

# Open another terminal and source ROS 2
source /opt/ros/humble/setup.bash
ros2 run teleop_turtle teleop_turtle_node
```

You should see a turtle graphics window and be able to control the turtle using your arrow keys in the teleop terminal.

## 2. Essential Development Tools

Ensure you have common development tools installed:

```bash
sudo apt install -y build-essential cmake git python3-pip
```

## 3. Python Virtual Environments (Recommended)

While not strictly required for ROS 2 itself, using Python virtual environments (`venv`) is highly recommended for managing project-specific Python dependencies.

```bash
# Install venv module if you don't have it
sudo apt install -y python3.venv

# Navigate to your ROS 2 workspace (or desired project directory)
# Example: cd ~/ros2_ws/src

# Create a virtual environment
python3 -m venv .venv

# Activate the virtual environment
source .venv/bin/activate

# Your prompt should change to indicate the active virtual environment (e.g., (.venv) user@host:~)

# To deactivate:
deactivate
```

Remember to activate your virtual environment in every new terminal session where you plan to run Python code related to your ROS 2 projects.
