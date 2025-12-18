# Chapter 1 Contract: ROS 2 Architecture & Philosophy

**Chapter ID**: ch1-architecture
**Directory**: `docs/module-1-ros2/01-architecture/`
**Estimated Hours**: 2 hours
**Status**: Design

---

## Learning Objectives

By the end of this chapter, learners will be able to:

1. Explain the role of middleware in robotics systems with concrete examples
2. Describe ROS 2 architecture including nodes, DDS, and distributed communication model
3. Compare ROS 1 and ROS 2, highlighting at least 5 key differences
4. Explain why DDS (Data Distribution Service) is used as the middleware layer
5. Run basic ROS 2 commands for system introspection (ros2 run, ros2 node list, ros2 topic list)

---

## Prerequisites

- Ubuntu 22.04 LTS installed
- ROS 2 Humble installed following official installation guide
- Basic Linux command-line knowledge (navigating directories, running commands)
- Terminal emulator accessible

---

## Section Breakdown

### 1. index.mdx - Chapter Introduction
**Sidebar Position**: 0
**Estimated Time**: 5 minutes

**Content**:
- Chapter overview and learning objectives
- Why this chapter matters (foundation for all ROS 2 work)
- What learners will build by module end
- Outline of sections

**Elements**:
- Learning objectives list
- Chapter roadmap diagram (simple flowchart)
- Motivational "by the end of this module" statement

**Admonitions**:
- :::tip - "Think of this chapter as learning the nervous system before moving individual limbs"

---

### 2. middleware-role.mdx - The Role of Middleware in Robotics
**Sidebar Position**: 1
**Estimated Time**: 15 minutes

**Content**:
- Definition of middleware in software systems
- Challenges of robot complexity (sensors, actuators, planning, control as separate subsystems)
- Communication problems middleware solves:
  - Discovery: How do subsystems find each other?
  - Abstraction: How do we decouple sender from receiver?
  - Reliability: How do we handle failures?
  - Timing: How do we coordinate real-time events?
- Analogy: Middleware as the nervous system of a robot
- Why robotics can't use generic web/mobile middleware (real-time, reliability, discovery)

**Diagrams**:
- `robot-subsystems.svg`: Robot broken into subsystems (perception, planning, control, actuation)
- `middleware-layer.svg`: Architecture layers (hardware → OS → middleware → application)

**Admonitions**:
- :::note - "Middleware acts like the nervous system, routing messages between brain (planning) and muscles (actuators)"
- :::warning - "Without middleware, each component would need custom code to talk to every other component (N² complexity)"

**Exercises**:
- Name three robot subsystems that need to communicate
- Explain in your own words what problem middleware solves

**Success Criteria**:
- Can explain middleware role in under 3 minutes to a peer
- Can name 3 communication challenges middleware addresses

---

### 3. ros2-overview.mdx - ROS 2 Architecture
**Sidebar Position**: 2
**Estimated Time**: 20 minutes

**Content**:
- ROS 2 as open-source robotics middleware
- Core architectural concepts:
  - **Node**: A process that performs computation (conceptual definition)
  - **Topic**: Named bus for asynchronous communication
  - **Service**: Synchronous request/response
  - **Action**: Long-running tasks with feedback
  - **Graph**: Network of nodes communicating via topics/services/actions
- Distributed nature: Nodes can run on different machines
- ROS 2 layers:
  - Application layer (user nodes)
  - rclpy/rclcpp (client libraries)
  - rcl (common client library logic)
  - rmw (middleware interface)
  - DDS implementation (Fast DDS, Cyclone DDS, etc.)
  - OS and network

**Diagrams**:
- `ros2-architecture.svg`: Layered architecture diagram
- `simple-node-graph.svg`: 3-node example (sensor → filter → controller)

**Admonitions**:
- :::info - "ROS 2 is not an operating system, it's a middleware framework running on Linux/Windows/macOS"
- :::tip - "Think of nodes as independent programs, topics as shared channels"

**Commands Demonstrated**:
```bash
# These are conceptual - actual execution in getting-started.mdx
ros2 node list
ros2 topic list
ros2 topic info /my_topic
```

**Success Criteria**:
- Can draw a simple 2-3 node computation graph
- Can explain the difference between a node and a topic

---

### 4. ros1-vs-ros2.mdx - ROS 1 vs ROS 2 Comparison
**Sidebar Position**: 3
**Estimated Time**: 15 minutes

**Content**:
- Brief history: ROS 1 (2007), ROS 2 (2017)
- Why ROS 2 was created (limitations of ROS 1)
- **Key Differences Table**:

| Aspect | ROS 1 Noetic | ROS 2 Humble | Impact |
|--------|--------------|--------------|--------|
| Middleware | Custom (TCPROS) | DDS (standard) | Interoperability with non-ROS systems |
| Real-time | No | Yes (with RTOS) | Critical for motor control |
| Security | None | DDS Security (SROS2) | Safe for deployed robots |
| Python | 2.7 (EOL) | 3.10+ | Modern language features |
| Platforms | Linux only | Linux/Windows/macOS | Broader development |
| Communication | Master-based (SPOF) | Distributed (no master) | Robustness |
| Lifecycle | None | Managed node states | Controlled startup/shutdown |
| QoS | None | Configurable | Reliability tuning |

- ROS 1 EOL timeline (Noetic May 2025)
- Migration considerations (when to use ROS 2)
- When ROS 1 might still be used (legacy systems)

**Diagrams**:
- `ros1-vs-ros2-architecture.svg`: Side-by-side comparison (master vs distributed)

**Admonitions**:
- :::warning - "ROS 1 Noetic reaches end-of-life in May 2025. Start new projects with ROS 2."
- :::info - "If you're learning ROS for the first time, focus on ROS 2 - it's the current standard"

**Success Criteria**:
- Can list 5+ key differences between ROS 1 and ROS 2
- Can explain why ROS 2 is better for production robots

---

### 5. dds-explained.mdx - Understanding DDS (Data Distribution Service)
**Sidebar Position**: 4
**Estimated Time**: 15 minutes

**Content**:
- What is DDS? (OMG standard for real-time publish/subscribe)
- Why ROS 2 uses DDS:
  - Industry standard (proven in aerospace, defense)
  - Real-time performance
  - Built-in discovery (no master node)
  - Quality of Service policies
  - Security features
- DDS implementations ROS 2 supports:
  - Fast DDS (default in Humble)
  - Cyclone DDS
  - RTI Connext DDS
- RTPS protocol (Real-Time Publish/Subscribe)
- Discovery process (how nodes find each other automatically)

**Diagrams**:
- `dds-layers.svg`: RTPS protocol, DDS layer, ROS 2 layer
- `dds-discovery.svg`: Node discovery sequence diagram

**Admonitions**:
- :::note - "You don't need to know DDS details to use ROS 2, but understanding helps with debugging"
- :::tip - "DDS handles the 'magic' of automatic node discovery - no roscore needed!"

**Success Criteria**:
- Can explain why ROS 2 uses DDS in 2-3 sentences
- Understands that DDS provides automatic node discovery

---

### 6. getting-started.mdx - Installation and Basic Commands
**Sidebar Position**: 5
**Estimated Time**: 30 minutes (includes hands-on)

**Content**:
- **Installation Verification** (assumes ROS 2 Humble already installed):
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 --version  # Expected: ros2 cli version: X.X.X
  ```

- **Essential Commands**:

  1. **Check available packages**:
     ```bash
     ros2 pkg list
     # Expected: Long list including std_msgs, geometry_msgs, etc.
     ```

  2. **Run demo nodes**:
     ```bash
     # Terminal 1
     ros2 run demo_nodes_cpp talker
     # Expected: [INFO] [talker]: Publishing: 'Hello World: 0'

     # Terminal 2
     ros2 run demo_nodes_cpp listener
     # Expected: [INFO] [listener]: I heard: 'Hello World: 0'
     ```

  3. **Introspect running system**:
     ```bash
     ros2 node list
     # Expected: /talker, /listener

     ros2 topic list
     # Expected: /chatter, /parameter_events, /rosout

     ros2 topic info /chatter
     # Expected: Type: std_msgs/msg/String, Publishers: 1, Subscribers: 1

     ros2 topic echo /chatter
     # Expected: Live stream of messages
     ```

  4. **Stop nodes**:
     ```bash
     # Ctrl+C in each terminal
     ```

- **Workspace Concept** (brief introduction):
  - Underlay: ROS 2 installation (`/opt/ros/humble`)
  - Overlay: User workspace (`~/ros2_ws`)
  - Sourcing: Why you need `source setup.bash`

**Admonitions**:
- :::warning - "Always source ROS 2 setup in every new terminal: `source /opt/ros/humble/setup.bash`"
- :::tip - "Add source command to ~/.bashrc to auto-source on terminal startup"
- :::info - "Use `ros2 --help` to see all available commands"

**Common Errors Section**:
| Error | Cause | Solution |
|-------|-------|----------|
| `ros2: command not found` | ROS 2 not sourced | Run `source /opt/ros/humble/setup.bash` |
| `No executable found` | Package not installed | Install package or check spelling |
| Nodes can't see each other | Different ROS_DOMAIN_ID | Check with `echo $ROS_DOMAIN_ID` |

**Success Criteria**:
- Successfully runs talker/listener demo
- Can list running nodes and topics
- Understands why sourcing is necessary

---

## Code Examples

**None** - This chapter is conceptual with command-line demonstrations only.

All commands are embedded directly in section content with expected output.

---

## Diagrams

### 1. robot-subsystems.svg
- **Purpose**: Show robot as collection of communicating subsystems
- **Elements**: Perception, Planning, Control, Actuation boxes with arrows
- **Used in**: middleware-role.mdx

### 2. middleware-layer.svg
- **Purpose**: Illustrate software stack layers
- **Elements**: Hardware → OS → Middleware → Application (vertical)
- **Used in**: middleware-role.mdx

### 3. ros2-architecture.svg
- **Purpose**: Show ROS 2 layered architecture
- **Elements**: Application, rclpy/rclcpp, rcl, rmw, DDS, OS/Network
- **Used in**: ros2-overview.mdx

### 4. simple-node-graph.svg
- **Purpose**: Example computation graph
- **Elements**: 3 nodes (sensor, filter, controller) connected by 2 topics
- **Used in**: ros2-overview.mdx

### 5. ros1-vs-ros2-architecture.svg
- **Purpose**: Side-by-side comparison
- **Elements**: ROS 1 (master-based) vs ROS 2 (distributed)
- **Used in**: ros1-vs-ros2.mdx

### 6. dds-layers.svg
- **Purpose**: DDS protocol stack
- **Elements**: RTPS, DDS API, ROS 2 RMW layers
- **Used in**: dds-explained.mdx

### 7. dds-discovery.svg
- **Purpose**: How nodes discover each other
- **Elements**: Sequence diagram of discovery process
- **Used in**: dds-explained.mdx

---

## Success Criteria

Learners have completed Chapter 1 when they can:

1. ✅ Explain middleware role in robotics in under 5 minutes
2. ✅ Describe ROS 2 architecture layers (application, rclpy/rclcpp, rmw, DDS)
3. ✅ List 5+ differences between ROS 1 and ROS 2
4. ✅ Explain why DDS is used as ROS 2's middleware
5. ✅ Run demo nodes (talker/listener) successfully
6. ✅ Use `ros2 node list`, `ros2 topic list`, `ros2 topic echo` to introspect system
7. ✅ Troubleshoot "command not found" errors (sourcing)

---

## Cross-references to Specification

This chapter fulfills the following functional requirements from spec.md:

- **FR-001**: Explains middleware role with concrete examples ✓
- **FR-002**: Describes ROS 2 architecture including nodes, DDS, distributed communication ✓
- **FR-003**: Compares ROS 1 and ROS 2 with 5+ key differences ✓
- **FR-004**: Explains why DDS is used as middleware layer ✓
- **FR-005**: Provides visual diagrams of ROS 2 architecture ✓
- **FR-006**: Includes getting started section with installation verification ✓
- **FR-007**: Demonstrates basic ROS 2 commands ✓

---

**Contract Status**: Complete ✅
**Ready for Implementation**: Yes
