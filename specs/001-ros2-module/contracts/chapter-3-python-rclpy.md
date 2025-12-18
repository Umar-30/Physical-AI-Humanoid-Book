# Chapter 3 Contract: Python Implementation with rclpy

**Chapter ID**: ch3-python-rclpy
**Directory**: `docs/module-1-ros2/03-python-rclpy/`
**Estimated Hours**: 3.5 hours
**Status**: Design

---

## Learning Objectives

By the end of this chapter, learners will be able to:

1. Create a ROS 2 Python package with proper structure
2. Implement a publisher node using rclpy
3. Implement a subscriber node with callback handling
4. Implement service servers and clients
5. Implement action servers and clients with feedback
6. Create custom message, service, and action interfaces
7. Handle parameters and dynamic reconfiguration
8. Structure code using classes and modular design
9. Build and run a complete multi-node robot agent system

---

## Prerequisites

- Completed Chapters 1-2 (Architecture and Communication Primitives)
- Python 3 basics (functions, classes, modules)
- Understanding of pub/sub, services, actions (conceptual from Chapter 2)

---

## Section Breakdown

### 1. index.mdx - Chapter Introduction (5 min)
### 2. rclpy-overview.mdx - rclpy API Structure (15 min)
- `rclpy.init()`, `rclpy.spin()`, `rclpy.shutdown()`
- Node lifecycle
- `Node` class and inheritance
- Key methods: `create_publisher`, `create_subscription`, `create_service`, `create_client`

### 3. package-setup.mdx - Creating ROS 2 Python Packages (25 min)
- Package structure (package.xml, setup.py, setup.cfg, resource/)
- `ros2 pkg create --build-type ament_python my_package`
- Dependencies in package.xml
- Entry points in setup.py
- Building with colcon: `colcon build --packages-select my_package`
- Sourcing workspace: `source install/setup.bash`

### 4. publisher-node.mdx - Implementing Publishers (30 min)
- **Code Example 1: simple_publisher**
- Class-based node implementation
- Timer-based publishing
- Message creation and population
- Logging with `get_logger()`
- Complete working code with build/run instructions

### 5. subscriber-node.mdx - Implementing Subscribers (30 min)
- **Code Example 2: simple_subscriber**
- Subscription callback function
- Message processing
- Callback execution timing
- Running publisher and subscriber together

### 6. service-impl.mdx - Service Servers and Clients (35 min)
- **Code Example 3: service_demo**
- Service server implementation
- Service client (sync and async)
- Request/response handling
- Error handling in services

### 7. action-impl.mdx - Action Servers and Clients (40 min)
- **Code Example 4: action_demo**
- Action server with goal acceptance, execution, feedback
- Action client with goal sending, feedback callback, result handling
- Cancellation support
- State machine (IDLE, ACTIVE, SUCCEEDED, ABORTED, CANCELED)

### 8. custom-msgs.mdx - Custom Message/Service/Action Types (30 min)
- **Code Example 5: custom_interfaces**
- Creating msg/ srv/ action/ directories
- Defining custom messages (e.g., `RobotStatus.msg`)
- CMakeLists.txt configuration (even for Python, uses rosidl)
- Building and using custom interfaces

### 9. parameters.mdx - Parameter Handling (20 min)
- Declaring parameters with defaults
- Getting and setting parameters at runtime
- `ros2 param list`, `ros2 param get`, `ros2 param set`
- Parameter callbacks for dynamic reconfiguration

### 10. best-practices.mdx - Code Organization Patterns (15 min)
- Class-based nodes (composition over inheritance)
- Separating business logic from ROS communication
- Error handling and logging
- Testing strategies (unit tests with mocking)
- Naming conventions

### 11. complete-example.mdx - Full Multi-Node System (45 min)
- **Code Example 6: multi_node_system**
- Integrated example: Robot delivery system
  - `sensor_node.py`: Publishes simulated sensor data
  - `planner_node.py`: Subscribes to sensors, provides planning service
  - `controller_node.py`: Calls planning service, sends navigation actions
- Launch file to start all nodes
- System integration testing
- Computation graph visualization with `rqt_graph`

---

## Code Examples

### 1. simple_publisher (publisher-node.mdx)
- Package: `simple_publisher`
- Node: `MinimalPublisher`
- Publishes: `std_msgs/String` on `/hello_topic` at 2Hz
- Demonstrates: Timer, publisher, logging

### 2. simple_subscriber (subscriber-node.mdx)
- Package: `simple_subscriber`
- Node: `MinimalSubscriber`
- Subscribes: `std_msgs/String` on `/hello_topic`
- Demonstrates: Subscriber, callback

### 3. service_demo (service-impl.mdx)
- Package: `service_demo`
- Server Node: `AddTwoIntsServer`
- Client Node: `AddTwoIntsClient`
- Service: `example_interfaces/AddTwoInts`
- Demonstrates: Service server/client, request/response handling

### 4. action_demo (action-impl.mdx)
- Package: `action_demo`
- Server Node: `FibonacciActionServer`
- Client Node: `FibonacciActionClient`
- Action: `action_tutorials_interfaces/Fibonacci`
- Demonstrates: Action goal/feedback/result, cancellation

### 5. custom_interfaces (custom-msgs.mdx)
- Package: `custom_interfaces`
- Interfaces: `RobotStatus.msg`, `GetPlan.srv`, `Navigate.action`
- Demonstrates: Creating and building custom interfaces

### 6. multi_node_system (complete-example.mdx)
- Package: `multi_node_system`
- Nodes: `sensor_node`, `planner_node`, `controller_node`
- Integrates: Topics, services, actions in one system
- Includes: Launch file, system diagram

---

## Diagrams

1. **package-structure.svg**: Python ROS 2 package directory layout
2. **node-lifecycle.svg**: Node initialization, spin, shutdown
3. **multi-node-architecture.svg**: Computation graph for complete example

---

## Success Criteria

✅ Can create a ROS 2 Python package from scratch
✅ Can write a publisher node that compiles and runs
✅ Can write a subscriber node that receives messages
✅ Can implement service server and client
✅ Can implement action server with feedback
✅ Can create custom message types
✅ Can build a multi-node system with 3+ nodes
✅ Can run all examples successfully on Ubuntu 22.04 + ROS 2 Humble

---

## Fulfills Requirements

FR-015 through FR-024

---

**Contract Status**: Complete ✅
