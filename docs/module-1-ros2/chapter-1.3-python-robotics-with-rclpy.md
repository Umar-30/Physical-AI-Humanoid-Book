---
id: chapter-1-3-python-robotics-with-rclpy
title: Python Robotics with rclpy
sidebar_position: 3
---

# Chapter 1.3: Python Robotics with rclpy

# Chapter 1.3: Python Robotics with rclpy

## Focus: Creating Python agents, message types, parameter management
## Learning objectives: Build functional ROS 2 nodes in Python

Python is a widely used language in robotics due to its readability, extensive libraries, and ease of development. `rclpy` is the Python client library for ROS 2, enabling developers to write ROS 2 nodes and interact with the ROS 2 ecosystem using Python. This chapter will guide you through creating various ROS 2 components using `rclpy`.

### 1. Initializing rclpy and Creating a Basic Node

Every ROS 2 application starts by initializing `rclpy` and creating at least one `Node`.

**Example: `my_first_node.py`**
```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create a node with the name 'my_first_node'
    node = Node('my_first_node')
    node.get_logger().info('My First ROS 2 Node has been started!')

    # Keep the node alive until it's explicitly shut down or Ctrl+C is pressed
    rclpy.spin(node)

    # Clean up and shut down rclpy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
*   `rclpy.init(args=None)`: Initializes the ROS 2 system. It must be called before any other `rclpy` functions.
*   `Node('my_first_node')`: Creates a new ROS 2 node instance named `my_first_node`.
*   `node.get_logger().info(...)`: Used to print informational messages from the node.
*   `rclpy.spin(node)`: Keeps the node running, processing callbacks (like subscriber messages or timer events) until the node is explicitly shut down or the process is interrupted (e.g., Ctrl+C).
*   `node.destroy_node()`: Destroys the node, releasing all its resources.
*   `rclpy.shutdown()`: Shuts down the `rclpy` client library.

**How to run:**
```bash
# In your package, usually 'src/<your_package_name>/<your_script>.py'
python3 my_first_node.py
```

### 2. Implementing a Publisher

A publisher node sends messages to a topic.

**Example: `simple_publisher.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher that publishes String messages to a topic named 'chatter'
        # The queue_size is 10, meaning it will buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.i = 0
        # Create a timer that calls the timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Simple Publisher Node has been started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
*   `from std_msgs.msg import String`: Imports the `String` message type from the `std_msgs` package. ROS 2 message types are auto-generated from `.msg` files.
*   `self.create_publisher(String, 'chatter', 10)`: Creates a publisher.
    *   `String`: The message type to publish.
    *   `'chatter'`: The name of the topic.
    *   `10`: The quality of service (QoS) history depth.
*   `self.create_timer(0.5, self.timer_callback)`: Sets up a timer to call `timer_callback` every 0.5 seconds.
*   `self.publisher_.publish(msg)`: Publishes the message `msg` to the `chatter` topic.

### 3. Implementing a Subscriber

A subscriber node receives messages from a topic.

**Example: `simple_subscriber.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the message type

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscriber that listens to String messages on the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10) # QoS history depth
        self.get_logger().info('Simple Subscriber Node has been started!')
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
*   `self.create_subscription(String, 'chatter', self.listener_callback, 10)`: Creates a subscriber.
    *   `String`: The message type to subscribe to.
    *   `'chatter'`: The name of the topic.
    *   `self.listener_callback`: The callback function that will be executed whenever a new message is received.
    *   `10`: The QoS history depth.
*   `listener_callback(self, msg)`: This method is automatically called when a message arrives on the `chatter` topic. The received message is passed as an argument.

**How to run (in separate terminals):**
```bash
# Terminal 1: Subscriber
python3 simple_subscriber.py

# Terminal 2: Publisher
python3 simple_publisher.py
```

### 4. Implementing a Service Server

A service server provides a service that clients can call.

**Example: `add_two_ints_server.py`**
First, define a custom service type (e.g., in `my_package/srv/AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```
Then, the Python server:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your custom service type if needed

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service server
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service has been started!')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    add_two_ints_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Implementing a Service Client

A service client calls a service server and waits for its response.

**Example: `add_two_ints_client.py`**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Replace with your custom service type if needed
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        # Spin until the future is complete
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()

    if len(sys.argv) != 3:
        add_two_ints_client.get_logger().info('Usage: python3 add_two_ints_client.py <int_a> <int_b>')
        add_two_ints_client.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = add_two_ints_client.send_request(a, b)
    add_two_ints_client.get_logger().info(
        f'Result of add_two_ints: for {a} + {b} = {response.sum}')

    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to run (in separate terminals):**
```bash
# Terminal 1: Service Server
python3 add_two_ints_server.py

# Terminal 2: Service Client
python3 add_two_ints_client.py 5 7
```

### 6. Implementing an Action Server and Client (Brief Overview)

Actions are more complex than topics or services, designed for long-running, preemptable tasks with continuous feedback. They are implemented using three message types: Goal, Result, and Feedback.

**Action Definition (e.g., in `my_package/action/Fibonacci.action`):**
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Implementing action servers and clients involves more boilerplate code than topics or services. The `rclpy.action` module provides the necessary tools.

**Action Server (conceptual):**
*   Receives a goal.
*   Executes the task, providing feedback periodically.
*   Sends a final result.
*   Handles preemption requests.

**Action Client (conceptual):**
*   Sends a goal to the server.
*   Receives feedback.
*   Receives the final result.
*   Can send preemption requests.

While full code examples are outside the scope of this introductory chapter, understanding that `rclpy` supports this advanced communication pattern is crucial for complex robotic behaviors.

By mastering `rclpy`, you gain the ability to leverage the full power of ROS 2 within the flexible and productive Python environment, making it an indispensable tool for robotics development.
