# Chapter 2 Contract: Communication Primitives

**Chapter ID**: ch2-communication
**Directory**: `docs/module-1-ros2/02-communication/`
**Estimated Hours**: 2.5 hours
**Status**: Design

---

## Learning Objectives

By the end of this chapter, learners will be able to:

1. Choose the appropriate communication primitive (topic/service/action) for a given use case
2. Explain publish/subscribe pattern and when to use topics
3. Explain request/response pattern and when to use services
4. Explain goal-oriented action pattern with feedback and cancellation
5. Configure Quality of Service (QoS) policies (reliability, durability, liveliness, deadline)
6. Understand ROS 2 executors and callback execution models
7. Design a computation graph for a multi-component robot system

---

## Prerequisites

- Completed Chapter 1 (Architecture & Philosophy)
- Understanding of ROS 2 nodes and topics (conceptual)
- ROS 2 Humble installed and sourced

---

## Section Breakdown

### 1. index.mdx - Chapter Introduction (5 min)
- Overview of communication primitives
- When to use which primitive (decision flowchart)
- Learning roadmap

### 2. topics-pubsub.mdx - Topics and Publish/Subscribe (20 min)
- Publish/subscribe pattern explained
- Anonymous many-to-many communication
- Message types (`std_msgs`, `geometry_msgs`, `sensor_msgs`)
- When to use topics (streaming data, sensors, commands)
- Commands: `ros2 topic list`, `ros2 topic echo`, `ros2 topic hz`, `ros2 topic info`
- Example: `/cmd_vel` for robot velocity commands

### 3. services-reqrep.mdx - Services and Request/Response (20 min)
- Request/response pattern explained
- Synchronous, one-to-one communication
- Service types (request/response message definitions)
- When to use services (queries, triggers, computations)
- Commands: `ros2 service list`, `ros2 service call`, `ros2 service type`
- Example: `/add_two_ints` service

### 4. actions-goals.mdx - Actions for Long-Running Tasks (25 min)
- Goal-oriented pattern with feedback and result
- Asynchronous, stateful, cancellable
- Action components (goal, feedback, result)
- When to use actions (navigation, manipulation, long tasks)
- Commands: `ros2 action list`, `ros2 action send_goal`, `ros2 action info`
- Example: Navigate to pose action

### 5. qos-profiles.mdx - Quality of Service Configuration (25 min)
- What is QoS? (communication reliability and performance tuning)
- QoS policies:
  - **Reliability**: Best effort vs reliable
  - **Durability**: Volatile vs transient local
  - **History**: Keep last N vs keep all
  - **Deadline**: Max time between messages
  - **Liveliness**: Automatic vs manual alive signals
- QoS presets: sensor_data, services, parameters
- QoS compatibility (publisher/subscriber must match)
- When to use which QoS
- Commands: `ros2 topic info /topic -v` (shows QoS)

### 6. executors.mdx - Callback Execution Models (15 min)
- What are executors? (thread management for callbacks)
- Single-threaded executor (default)
- Multi-threaded executor
- Callback groups (mutually exclusive vs reentrant)
- When callbacks run
- Brief mention (detailed implementation in Chapter 3)

### 7. graph-design.mdx - Designing Computation Graphs (25 min)
- How to design ROS 2 systems
- Node responsibility (single responsibility principle)
- Topic naming conventions (`/robot/subsystem/topic`)
- Example design: Simple delivery robot
  - Perception node: camera → `/camera/image`, lidar → `/scan`
  - Planner node: subscribes `/scan`, publishes `/cmd_vel`
  - Controller node: subscribes `/cmd_vel`, controls motors
- Visualization with `rqt_graph`
- Best practices: modularity, loose coupling, clear interfaces

---

## Code Examples

**None** - This chapter uses CLI demonstrations and design exercises, no full code implementations yet (those are in Chapter 3).

---

## Diagrams

1. **pubsub-flow.svg**: Topic publish/subscribe data flow (many publishers, many subscribers)
2. **service-flow.svg**: Service request/response sequence diagram
3. **action-flow.svg**: Action goal/feedback/result state machine
4. **qos-scenarios.svg**: QoS policy impact on communication (reliable vs best-effort)
5. **computation-graph-example.svg**: Example robot computation graph (delivery robot)

---

## Success Criteria

✅ Can choose topic vs service vs action for 5 different scenarios
✅ Can configure QoS policies appropriately for sensor data vs commands
✅ Can design a computation graph for a simple robot (3-5 nodes)
✅ Can use `ros2 topic`, `ros2 service`, `ros2 action` commands for introspection
✅ Understands QoS compatibility requirements

---

## Fulfills Requirements

FR-008, FR-009, FR-010, FR-011, FR-012, FR-013, FR-014

---

**Contract Status**: Complete ✅
