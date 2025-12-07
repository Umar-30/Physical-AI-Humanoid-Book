---
id: chapter-1-2-ros2-core-concepts-nodes-topics-services
title: "ROS 2 Core Concepts: Nodes, Topics, Services"
sidebar_position: 2
---

# Chapter 1.2: ROS 2 Core Concepts: Nodes, Topics, Services

## Focus: Publish-subscribe pattern, service calls, actions
## Learning objectives: Implement basic ROS 2 communication

### Understanding Publish/Subscribe Communication

One of the fundamental communication patterns enabled by robotic middleware is the **publish/subscribe model**. This pattern allows different components of a robot (called "nodes" in ROS) to send and receive messages without direct knowledge of each other. A "publisher" node sends messages on a specific "topic," and any "subscriber" node interested in that data can listen to that topic. This decouples the senders from the receivers, making the system more flexible and modular.

```mermaid
graph TD
    A[Publisher Node] --> B(Topic: Sensor_Data)
    B --> C[Subscriber Node 1]
    B --> D[Subscriber Node 2]
    A -.-> E[ROS 2 Middleware / DDS]
    C -.-> E
    D -.-> E

    subgraph Robot System
        A
        C
        D
    end
```

This diagram illustrates how a Publisher Node sends data (e.g., sensor readings) to a Topic, which is then received by multiple Subscriber Nodes through the ROS 2 Middleware (DDS).

## Installation Guide: ROS 2 Humble on Ubuntu 22.04

This section provides a verified, step-by-step guide to installing ROS 2 Humble Hawksbill on Ubuntu 22.04 (Jammy Jellyfish). Following these instructions carefully will ensure a successful setup.

### 1. Set Up Your Locale

It is recommended to have a UTF-8 locale. If you are in a minimal environment, you might need to uncomment locale generation in `/etc/locale.gen` and then generate the locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 2. Add the ROS 2 APT Repository

First, ensure your system is up-to-date and install necessary utilities:

```bash
sudo apt update && sudo apt install -y software-properties-common apt-transport-https ca-certificates curl
```

Now, add the GPG key for the ROS 2 repository:

```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then, add the repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2 Packages

Update your package list again after adding the new repository:

```bash
sudo apt update
```

Install the ROS 2 Humble Desktop environment (includes ROS, rqt, rviz, robot_state_publisher, and generic libraries):

```bash
sudo apt install -y ros-humble-desktop
```

For development tools, you might also want to install:

```bash
sudo apt install -y ros-humble-ros-base ros-dev-tools
```

### 4. Source the ROS 2 Setup Script

To use ROS 2 commands, you need to source its setup script. It's recommended to add this to your `~/.bashrc` for automatic sourcing on new terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you are using Zsh, you would add it to `~/.zshrc` instead:

```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

### 5. Install argcomplete (Optional but Recommended)

For autocompletion of ROS 2 commands:

```bash
sudo apt install -y python3-pip
pip install -U argcomplete
```

Then, enable it for your shell (for Bash):

```bash
activate-global-python-argcomplete
```

**(End of Installation Guide)**

## Verification & First Commands

After successfully installing ROS 2 Humble, it's crucial to verify that everything is working as expected. This hands-on exercise will guide you through basic ROS 2 commands and demonstrate the publish/subscribe communication model.

### 1. Verify ROS 2 Installation with `ros2 doctor`

The `ros2 doctor` command is a helpful tool to check the health of your ROS 2 environment. It performs a series of checks and reports any potential issues.

**Command:**

```bash
ros2 doctor
```

**Expected Output:**

You should see a report indicating "All OK" or similar success messages. Any warnings or errors will be highlighted, guiding you towards troubleshooting. For a fresh installation, this should be mostly clean.

```text
# Example of a successful output (may vary slightly)
All OK :)
```

### 2. Run a Simple Publish/Subscribe Example

Let's use the `demo_nodes_cpp` package, which contains basic talker (publisher) and listener (subscriber) examples.

**Step A: Start the Listener (Subscriber)**

Open a *new terminal* (remember to source your ROS 2 setup if not in `.bashrc`):

```bash
source /opt/ros/humble/setup.bash # if not in .bashrc
ros2 run demo_nodes_cpp listener
```

**Expected Output:**

The listener terminal should start waiting for messages and eventually print received messages:

```text
[INFO] [minimal_listener]: I heard: "Hello World 0"
[INFO] [minimal_listener]: I heard: "Hello World 1"
[INFO] [minimal_listener]: I heard: "Hello World 2"
...
```

**Step B: Start the Talker (Publisher)**

Open *another new terminal* (again, source ROS 2 setup):

```bash
source /opt/ros/humble/setup.bash # if not in .bashrc
ros2 run demo_nodes_cpp talker
```

**Expected Output:**

The talker terminal should show it's publishing messages, and concurrently, your listener terminal (from Step A) should begin displaying these messages.

```text
[INFO] [minimal_talker]: Publishing: "Hello World 0"
[INFO] [minimal_talker]: Publishing: "Hello World 1"
[INFO] [minimal_talker]: Publishing: "Hello World 2"
...
```

### 3. Inspect ROS 2 Nodes and Topics

While the talker and listener are running, open a *third terminal* (source ROS 2 setup) to inspect the ROS 2 graph.

**List active ROS 2 nodes:**

```bash
ros2 node list
```

**Expected Output:**

You should see the talker and listener nodes:

```bash
/listener
/talker
```

**List active ROS 2 topics:**

```bash
ros2 topic list
```

**Expected Output:**

You should see the `/topic` that the talker is publishing to:

```bash
/parameter_events
/rosout
/topic
```

**Echo messages on the topic:**

```bash
ros2 topic echo /topic
```

**Expected Output:**

This command will display the messages being published on `/topic` in real-time, confirming the communication:

```json
---
data: Hello World 23
---
data: Hello World 24
---
data: Hello World 25
...
```

By successfully completing these steps, you have confirmed your ROS 2 Humble installation is functional and have witnessed basic ROS 2 communication in action!

## Common Pitfalls & Solutions

Even with a detailed guide, installation can sometimes present challenges. Here are some common pitfalls encountered during ROS 2 Humble installation on Ubuntu 22.04 and their corresponding solutions.

### 1. Locale Issues

**Pitfall**: The `locale` command shows warnings, or ROS 2 commands fail with locale-related errors (e.g., "invalid character sequence").
**Solution**: Ensure your locale is correctly set to UTF-8. Revisit "1. Set Up Your Locale" section in the Installation Guide and execute the commands carefully. Specifically, check `/etc/locale.gen` to ensure `en_US.UTF-8 UTF-8` is uncommented, then run `sudo locale-gen`, and finally `sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8` and `export LANG=en_US.UTF-8`.

### 2. Repository Key/Source List Errors

**Pitfall**: `sudo apt update` fails with GPG errors (e.g., "NO_PUBKEY") or the ROS 2 repository is not found.
**Solution**: This usually means the GPG key or the repository source list was not added correctly.
*   **GPG Key**: Double-check the `curl` command for adding the `ros.key`. Ensure there are no typos and that the key is written to `/usr/share/keyrings/ros-archive-keyring.gpg`.
*   **Source List**: Verify the `echo "deb ..."` command. Ensure it correctly pipes the repository line into `/etc/apt/sources.list.d/ros2.list`. You can inspect the file's content with `cat /etc/apt/sources.list.d/ros2.list`.

### 3. Package Not Found Errors

**Pitfall**: `sudo apt install ros-humble-desktop` (or other ROS 2 packages) results in "Unable to locate package" errors.
**Solution**: This typically means your `apt` package list is out of date or the ROS 2 repository was not added correctly.
1.  **Update**: Run `sudo apt update` again to ensure your package list is refreshed.
2.  **Verify Repository**: Re-check step "2. Add the ROS 2 APT Repository" to confirm the repository was added correctly.
3.  **Typos**: Ensure there are no typos in the package names (e.g., `ros-humble-desktop`).

### 4. Sourcing Issues (Commands Not Found)

**Pitfall**: After installation, commands like `ros2 run` or `ros2 node list` are not found.
**Solution**: This means your shell environment has not been correctly set up to recognize ROS 2 commands.
1.  **Source Manually**: In each new terminal, run `source /opt/ros/humble/setup.bash` (or `.zsh` for Zsh users).
2.  **Automate Sourcing**: Add the `source` command to your `~/.bashrc` (or `~/.zshrc`) file as described in "4. Source the ROS 2 Setup Script" in the Installation Guide. Remember to then run `source ~/.bashrc` to apply the changes to your current terminal.

### 5. Network Connectivity Issues

**Pitfall**: Installation commands involving `apt update`, `apt install`, or `curl` fail due to network errors.
**Solution**: Ensure your Ubuntu system has a stable internet connection. Check network settings, DNS resolution, or proxy configurations if applicable.

By being aware of these common issues and their solutions, you can efficiently troubleshoot your ROS 2 installation and get your robotic development environment up and running smoothly.

### Check Your Understanding: Installation & Verification

1.  What is the purpose of `ros2 doctor`, and what kind of output would indicate a successful ROS 2 installation?
2.  Describe the steps to run a simple publish/subscribe example in ROS 2 using `demo_nodes_cpp` and how you would verify communication between the talker and listener.
3.  You encounter an error stating "Unable to locate package ros-humble-desktop" during installation. What are the common reasons for this, and how would you troubleshoot it?
