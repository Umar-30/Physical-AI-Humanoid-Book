# Environment Setup and Prerequisites for Module 3

This section guides you through setting up your development environment for Module 3. It assumes you have a powerful Ubuntu 22.04 LTS workstation (with a dedicated NVIDIA GPU) and a Jetson Orin Nano/NX developer kit.

## 1. Workstation Setup (Ubuntu 22.04 LTS)

Your workstation will be used for running NVIDIA Isaac Sim, which is a powerful GPU-accelerated robotics simulation platform.

### 1.1 Operating System

Ensure your workstation is running **Ubuntu 22.04 LTS**. If not, please refer to Module 1's environment setup for installation instructions or perform a fresh install.

### 1.2 NVIDIA GPU Driver Installation

NVIDIA Isaac Sim requires a recent NVIDIA GPU driver.

1.  **Check existing driver (optional)**:
    ```bash
nvidia-smi
```
    If you see output showing your GPU information, drivers are likely installed. Note down the driver version.

2.  **Install/Update NVIDIA Drivers**:
    It's recommended to use the official NVIDIA driver installation.
    ```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update
# List available drivers and choose the recommended one (e.g., nvidia-driver-535, or a newer version)
ubuntu-drivers devices
sudo apt install nvidia-driver-<version> # Replace <version> with the recommended one
sudo reboot
```
    After rebooting, run `nvidia-smi` again to confirm the new driver is active.

### 1.3 Docker and NVIDIA Container Toolkit Installation

NVIDIA Isaac Sim and Isaac ROS are typically deployed within Docker containers using the NVIDIA Container Toolkit (formerly `nvidia-docker`).

1.  **Install Docker Engine**:
    Follow the official Docker documentation for Ubuntu 22.04 to ensure you have the latest stable version.
    ```bash
# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update

sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
    Verify Docker installation: `sudo docker run hello-world`

2.  **Post-installation steps (manage Docker as a non-root user)**:
    ```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker # You may need to log out and log back in for this to take effect
```
    Verify without `sudo`: `docker run hello-world`

3.  **Install NVIDIA Container Toolkit**:
    This toolkit allows Docker containers to access your NVIDIA GPU.
    ```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
    Verify NVIDIA Container Toolkit installation: `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`
    You should see output similar to `nvidia-smi` showing your GPU.

### 1.4 ROS 2 Humble Installation

Module 3 builds heavily on ROS 2. If you haven't already, install ROS 2 Humble.

1.  **Follow Official ROS 2 Humble Installation Guide**:
    Refer to the official documentation for [ROS 2 Humble Hawksbill on Ubuntu (Canonical)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Ensure you install the **Desktop Full** version.
    ```bash
# Example commands (refer to official docs for latest):
sudo apt install software-properties-common lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install argcomplete for ROS 2 commands
sudo apt install python3-argcomplete
```
    Verify ROS 2 installation: `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener` in separate terminals.

## 2. Jetson Orin Nano/NX Developer Kit Setup

Your Jetson Orin device will be used for deploying Isaac ROS perception pipelines and the Nav2 stack for edge inference and navigation.

### 2.1 Flashing JetPack OS

Ensure your Jetson Orin Nano/NX is running the latest **JetPack SDK (5.x or newer, compatible with Ubuntu 22.04 base)**. This includes the OS, CUDA, cuDNN, TensorRT, and other NVIDIA libraries essential for Isaac ROS.

1.  **Download NVIDIA SDK Manager**:
    Go to the [NVIDIA Developer website](https://developer.nvidia.com/nvidia-sdk-manager) and download the SDK Manager for your host machine (usually Ubuntu).

2.  **Flash JetPack**:
    Follow the official NVIDIA documentation to use SDK Manager to flash your Jetson Orin device. This process will install the complete JetPack OS and development environment.
    *   [NVIDIA Jetson Getting Started Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson)
    *   [JetPack SDK Documentation](https://developer.nvidia.com/jetpack-sdk)

### 2.2 ROS 2 Humble Installation on Jetson

Install ROS 2 Humble on your Jetson Orin device, similar to your workstation setup.

1.  **Follow Official ROS 2 Humble Installation Guide for ARM64**:
    The steps are largely identical to the workstation, but ensure you are building for the ARM64 architecture of the Jetson.
    Refer to the official documentation for [ROS 2 Humble Hawksbill on Ubuntu (Canonical)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Jetson device.

### 2.3 Docker and NVIDIA Container Toolkit on Jetson

Docker and the NVIDIA Container Toolkit are pre-installed or easily installable with JetPack, but ensure they are correctly configured.

1.  **Verify Docker**: `docker run hello-world`
2.  **Verify NVIDIA Container Toolkit**: `docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r35.4.1-runtime nvidia-smi` (replace `r35.4.1-runtime` with your JetPack version's tag from NVIDIA's L4T-Base container registry).

By completing these steps, your workstation and Jetson Orin developer kit will be fully prepared for Module 3's advanced AI-robotics tasks.
