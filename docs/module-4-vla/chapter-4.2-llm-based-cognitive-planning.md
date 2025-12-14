# Chapter 4.2: Language as a Planning Tool (Cognitive Planning)

In this chapter, we bridge the gap between spoken commands and robot actions by empowering Large Language Models (LLMs) to generate executable plans. You'll learn to integrate both cloud-based (GPT-4, Claude) and local (quantized `llama_cpp`) LLMs to transform natural language into precise ROS 2 action sequences.

## 4.2.1 The "Tool-Use" Paradigm for LLMs in Robotics

Imagine a skilled artisan who, when given a complex task, doesn't just ponder it but knows exactly which tools to grab from their workbench and how to use them to achieve the goal. This is analogous to the "tool-use" paradigm for LLMs in robotics.

Traditionally, LLMs are trained on vast amounts of text and are excellent at understanding, generating, and summarizing human language. However, they lack direct access to the physical world or the ability to execute code. In robotics, we need LLMs to go beyond just understanding; we need them to *act*.

The "tool-use" paradigm (also known as "function calling" or "agentic LLMs") enables LLMs to interact with external systems and environments by providing them with a set of "tools" (functions, APIs) they can invoke. When an LLM receives a prompt, it doesn't just respond with text; it might decide that to answer or fulfill the request, it first needs to call a specific tool with certain arguments. The output of that tool is then fed back to the LLM, allowing it to continue its reasoning or generate a final response.

**Analogy**:
Think of a human manager (the LLM) who needs to get a physical task done. They don't do the task themselves. Instead, they tell their team members (the tools) what to do.
-   **Manager (LLM)**: "Find me the blue block and pick it up."
-   **Team Member 1 (Tool: `scan_for_object` function)**: The manager identifies that "find blue block" requires a vision tool. It calls `scan_for_object(object_description="blue block")`.
-   **Team Member 1 (Tool) Response**: "Found blue block at coordinates (X, Y, Z)."
-   **Manager (LLM)**: Now knowing the location, the manager identifies that "pick it up" requires a manipulation tool. It calls `pick_object(coordinates=(X, Y, Z))`.
-   **Team Member 2 (Tool) Response**: "Object picked up successfully."
-   **Manager (LLM) Final Report**: "Blue block has been picked up."

In our robotic system, these "tools" are ROS 2 actions or services that correspond to robot capabilities (e.g., `move_to_pose`, `grasp_object`, `scan_for_object`). The LLM's task is to decide *which* tools to use and *in what sequence*, given a natural language command.

## 4.2.2 Cloud vs. Local LLMs: Trade-offs for Robotics

The choice between cloud-based (e.g., GPT-4, Claude) and local (on-device, e.g., quantized `llama_cpp`) LLMs is critical in robotics, presenting distinct trade-offs in latency, cost, and reliability.

| Feature        | Cloud LLM (e.g., GPT-4, Claude)                       | Local LLM (e.g., `llama_cpp` on Jetson Orin)                |
| :------------- | :---------------------------------------------------- | :---------------------------------------------------------- |
| **Latency**    | **High**: Involves network round-trips to remote servers. Varies with network conditions and API load. Can be hundreds of milliseconds to seconds. | **Low**: Inference happens on-device. Milliseconds to tens of milliseconds, depending on model size and hardware. More predictable. |
| **Cost**       | **Variable**: Pay-per-token/API call. Can be expensive for frequent or long interactions. Scales with usage. | **Fixed (Hardware)**: Upfront cost for local compute (Jetson Orin, GPU). No per-usage cost once hardware is acquired. |
| **Reliability**| **Network Dependent**: Prone to outages or slowdowns if internet connection is lost or unstable. External API service reliability. | **Self-Contained**: Operates independently of internet. Reliant only on local hardware and software stability. More robust in isolated environments. |
| **Model Size/Capability**| **Very Large/Advanced**: Access to the most powerful and general-purpose models, capable of complex reasoning and broad knowledge. | **Smaller/Specialized**: Often quantized or fine-tuned versions of larger models to fit edge hardware. May require more careful prompting or domain-specific fine-tuning for desired capabilities. |
| **Data Privacy**| **Less Private**: Data sent to third-party servers. Requires careful consideration of sensitive robot data. | **More Private**: Data stays on-device. Ideal for applications with strict privacy requirements. |
| **Deployment** | Simple API integration.                             | Complex deployment involving model quantization, hardware optimization, and environment setup. |

**Critical Trade-offs for a Moving Robot**:

-   **Latency**: For real-time robot control or safety-critical actions, low and predictable latency is paramount. A robot cannot afford to wait seconds for an instruction from a cloud LLM if it's about to collide or needs to react quickly. Local LLMs offer a significant advantage here.
-   **Reliability**: In scenarios where a robot operates in areas with intermittent or no internet connectivity, cloud LLMs become unusable. Local LLMs provide continuous operation, essential for autonomous navigation or operation in remote locations.
-   **Cost**: For widespread deployment of many robots or for applications with continuous LLM interaction, the cumulative API costs of cloud LLMs can become prohibitive. Local LLMs, despite higher upfront hardware costs, can offer a lower total cost of ownership in the long run.

The optimal solution often involves a hybrid approach: using powerful cloud LLMs for initial, high-level, or less time-sensitive planning (e.g., mission planning) and leveraging local, smaller LLMs for real-time decision-making, error recovery, or frequent, low-level interactions.

## 4.2.3 Implementing a Local Planner Node with `llama_cpp`

We will create a ROS 2 node that uses the `llama_cpp` Python library to run a quantized LLM locally. This node will subscribe to voice commands (from `/voice_command`) and output ROS 2 action sequences.

### Prerequisites

-   Ensure you have completed Chapter 4.1's setup.
-   A suitable quantized LLM model (e.g., a GGUF model from Hugging Face). For Jetson Orin, consider smaller models like `tinyllama` or `phi-2` quantized to 4-bit. Download a `.gguf` file to your workspace (e.g., `~/ros2_ws/src/my_vla_planner/models/`).

### Step-by-Step Instructions

1.  **Create a ROS 2 Package (if not already done)**:
    If you're creating a new package for the planner, follow similar steps to Chapter 4.1:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_vla_planner
    cd my_vla_planner
    mkdir models # For storing your GGUF model
    ```

2.  **Install `llama-cpp-python`**:
    Activate your ROS 2 Python environment and install `llama-cpp-python`. Compiling this library can be resource-intensive; ensure you have appropriate build tools (e.g., `cmake`, `g++`). For Jetson Orin, you might need to specify `CMAKE_ARGS` to enable CUDA acceleration.
    ```bash
    # Ensure virtual environment is active (if used)
    # source ~/ros2_ws/src/my_vla_voice/venv/bin/activate
    
    # On Workstation (CPU only)
    pip install llama-cpp-python
    
    # On Jetson Orin (with CUDA support, ensure CUDA Toolkit is installed and paths are set)
    # pip install llama-cpp-python --force-reinstall --no-cache-dir --verbose --compile-type=llama --global-option="build_ext" --global-option="-mllama" --global-option="--cuda"
    # The --global-option flags are specific to older pip versions or specific build scenarios.
    # A more general approach for CUDA on Jetson might involve setting environment variables:
    # CMAKE_ARGS="-DLLAMA_CUBLAS=on" FORCE_CMAKE=1 pip install llama-cpp-python --no-cache-dir
    # Verify installation: python -c "from llama_cpp import Llama; print(Llama)"
    ```
    *Refer to `llama-cpp-python`'s official documentation for the most up-to-date installation instructions for your specific hardware, especially for GPU acceleration.*

3.  **Create the `planner_node.py` Script**:
    Inside your `my_vla_planner/my_vla_planner` directory, create `planner_node.py` and add the following code. Remember to replace `path/to/your/quantized_model.gguf` with the actual path to your downloaded model.

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from llama_cpp import Llama
    import json
    import threading

    # Configuration for LLM
    MODEL_PATH = "/home/your_user/ros2_ws/src/my_vla_planner/models/tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf" # !!! Update this path
    N_GPU_LAYERS = 0 # Set to > 0 to offload layers to GPU if you have one (e.g., Jetson Orin)
    TEMPERATURE = 0.7
    MAX_TOKENS = 256
    
    # Example ROS 2 Action Service calls (simplified for demonstration)
    # In a real system, these would be proper ROS 2 Action Client calls
    AVAILABLE_TOOLS = {
        "move_robot": {
            "description": "Moves the robot to a specified target location or relative position. Arguments: {'x': float, 'y': float, 'z': float, 'relative': bool}",
            "schema": {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "relative": {"type": "boolean"}}}
        },
        "wave_hand": {
            "description": "Makes the robot wave its specified hand. Arguments: {'hand': 'left' | 'right'}",
            "schema": {"type": "object", "properties": {"hand": {"type": "string", "enum": ["left", "right"]}}}
        },
        "grasp_object": {
            "description": "Commands the robot to grasp an object at given coordinates. Arguments: {'x': float, 'y': float, 'z': float, 'object_name': str}",
            "schema": {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "object_name": {"type": "string"}}}
        }
    }

    def generate_tool_schema():
        tools_list = []
        for tool_name, tool_info in AVAILABLE_TOOLS.items():
            tools_list.append({
                "type": "function",
                "function": {
                    "name": tool_name,
                    "description": tool_info["description"],
                    "parameters": tool_info["schema"]
                }
            })
        return json.dumps(tools_list, indent=2)

    class PlannerNode(Node):
        def __init__(self):
            super().__init__('planner_node')
            self.subscription = self.create_subscription(
                String,
                'voice_command',
                self.voice_command_callback,
                10)
            self.publisher_ = self.create_publisher(String, 'robot_action_sequence', 10)
            self.get_logger().info('Planner Node started. Waiting for voice commands...')

            self.llm = None
            self.llm_lock = threading.Lock() # To prevent multiple concurrent LLM calls

            # Load LLM in a separate thread or non-blocking way if it takes time
            threading.Thread(target=self._load_llm).start()

        def _load_llm(self):
            self.get_logger().info(f"Loading local LLM from {MODEL_PATH}...")
            try:
                self.llm = Llama(
                    model_path=MODEL_PATH,
                    n_gpu_layers=N_GPU_LAYERS,
                    n_ctx=2048, # Context window size
                    verbose=False
                )
                self.get_logger().info("Local LLM loaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load LLM: {e}")
                self.llm = None # Indicate failure

        def voice_command_callback(self, msg):
            command_text = msg.data
            self.get_logger().info(f'Received voice command: "{command_text}"')

            if self.llm is None:
                self.get_logger().warn("LLM not loaded yet or failed to load. Cannot process command.")
                return

            if self.llm_lock.acquire(blocking=False): # Try to acquire lock non-blocking
                try:
                    # Construct the prompt for tool calling
                    # This is a simplified example. Real-world prompts are more complex.
                    prompt = f"""
                    You are a helpful robot assistant. You can perform the following actions:
                    {generate_tool_schema()}
                    
                    Based on the user's command, call one or more of these functions to fulfill the request. 
                    If no suitable function can be called, respond naturally.
                    
                    User command: {command_text}
                    """

                    # Using the chat completion API for function calling
                    # This interface might vary slightly with different llama_cpp-python versions
                    response = self.llm.create_chat_completion(
                        messages=[
                            {"role": "system", "content": "You are a helpful robot assistant with access to tools."},
                            {"role": "user", "content": prompt}
                        ],
                        functions=AVAILABLE_TOOLS, # Pass the tool definitions
                        function_call="auto", # Let the LLM decide if it needs to call a function
                        temperature=TEMPERATURE,
                        max_tokens=MAX_TOKENS,
                    )
                    
                    action_sequence = ""
                    if response["choices"][0]["message"].get("function_call"):
                        function_call = response["choices"][0]["message"]["function_call"]
                        tool_name = function_call["name"]
                        tool_args = json.loads(function_call["arguments"])
                        
                        self.get_logger().info(f"LLM decided to call tool: {tool_name} with args: {tool_args}")
                        
                        # In a real system, you would execute this tool here
                        # For now, we just format it as a string
                        action_sequence = f"TOOL_CALL: {tool_name}({json.dumps(tool_args)})"
                        
                    elif response["choices"][0]["message"].get("content"):
                        action_sequence = response["choices"][0]["message"]["content"]
                        self.get_logger().info(f"LLM responded with natural language: {action_sequence}")
                    else:
                        action_sequence = "No actionable response from LLM."
                        self.get_logger().warn("LLM response did not contain a function call or content.")

                    if action_sequence:
                        msg = String()
                        msg.data = action_sequence
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published action sequence: "{action_sequence}"')

                except Exception as e:
                    self.get_logger().error(f"Error during LLM processing: {e}")
                    msg = String()
                    msg.data = f"ERROR: Could not process command due to LLM error: {e}"
                    self.publisher_.publish(msg)
                finally:
                    self.llm_lock.release()
            else:
                self.get_logger().warn("LLM is busy, skipping current command.")

    def main(args=None):
        rclpy.init(args=args)
        node = PlannerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
    *Important*: The `functions` and `function_call` parameters for `create_chat_completion` are part of newer `llama_cpp-python` versions and are designed to mimic OpenAI's function calling API. Ensure your `llama_cpp-python` version supports this. You might need to adapt the prompt for older versions or models that don't explicitly support function calling, perhaps by instructing the LLM to output a specific JSON format.

4.  **Update `setup.py`**:
    Open `my_vla_planner/setup.py` and configure the `entry_points`:
    ```python
    from setuptools import find_packages, setup

    package_name = 'my_vla_planner'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools', 'llama-cpp-python', 'json'], # Add llama-cpp-python
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your.email@example.com',
        description='ROS 2 package for LLM-based robot planning',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'planner_node = my_vla_planner.planner_node:main',
            ],
        },
    )
    ```
    *Remember to replace `Your Name` and `your.email@example.com`.*

5.  **Build Your ROS 2 Package**:
    Navigate back to your workspace root (`~/ros2_ws`) and build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_vla_planner
    ```

6.  **Source Your Workspace and Run the Node**:
    First, ensure your `voice_input_node` from Chapter 4.1 is running and publishing to `/voice_command`.
    ```bash
    # In Terminal 1: Run voice_input_node
    source install/setup.bash
    ros2 run my_vla_voice voice_input_node

    # In Terminal 2: Run planner_node
    source install/setup.bash
    ros2 run my_vla_planner planner_node
    ```
    Speak commands like "wave your right hand", "move forward 0.5 meters", "grasp the red ball at x 1.0 y 0.2 z 0.1". You should see the `planner_node` process these and publish corresponding action sequences or natural language responses.

## 4.2.4 Example Prompts for Cloud LLMs (GPT-4, Claude)

For more complex reasoning or broader understanding, cloud LLMs like GPT-4 and Claude can be leveraged. The "tool-use" paradigm is typically natively supported by their APIs.

**GPT-4 Example Prompt**:

When using the OpenAI API with `gpt-4` or `gpt-3.5-turbo` for function calling, you define your tools as a list of dictionaries. The LLM then decides if and how to call them.

```python
import openai
import json

# Assume you have your OpenAI API key set as an environment variable
# openai.api_key = os.getenv("OPENAI_API_KEY")

# Define the available tools (similar to our Python dict, but passed to OpenAI)
openai_tools = [
    {
        "type": "function",
        "function": {
            "name": "move_robot",
            "description": "Moves the robot to a specified target location or relative position. Use positive values for forward/right/up, negative for backward/left/down.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "X displacement in meters"},
                    "y": {"type": "number", "description": "Y displacement in meters"},
                    "z": {"type": "number", "description": "Z displacement in meters"},
                    "relative": {"type": "boolean", "description": "True if displacement is relative to current position, False for absolute target."}
                },
                "required": ["x", "y", "z", "relative"]
            },
        }
    },
    {
        "type": "function",
        "function": {
            "name": "wave_hand",
            "description": "Makes the robot wave its specified hand.",
            "parameters": {
                "type": "object",
                "properties": {
                    "hand": {"type": "string", "enum": ["left", "right"]}
                },
                "required": ["hand"]
            },
        }
    },
    # Add more robot actions here
]

def call_gpt4_planner(user_command):
    messages = [
        {"role": "system", "content": "You are a helpful robot assistant with access to tools. Respond using the available tools to fulfill user requests."},
        {"role": "user", "content": user_command}
    ]

    response = openai.chat.completions.create(
        model="gpt-4-turbo-preview", # Or gpt-3.5-turbo
        messages=messages,
        tools=openai_tools,
        tool_choice="auto", # let GPT decide if it wants to call a tool
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    if tool_calls:
        # Assuming one tool call for simplicity; LLM can generate multiple
        function_name = tool_calls[0].function.name
        function_args = json.loads(tool_calls[0].function.arguments)
        
        print(f"GPT-4 decided to call tool: {function_name} with args: {function_args}")
        # Here you would typically execute the tool and pass the result back to GPT-4
        # For demonstration, we just return the call
        return {"tool_call": function_name, "args": function_args}
    else:
        print(f"GPT-4 responded with natural language: {response_message.content}")
        return {"content": response_message.content}

# Example Usage:
# print(call_gpt4_planner("Please wave your right hand."))
# print(call_gpt4_planner("Move the robot forward by 0.5 meters and then to the left by 0.1 meter."))
# print(call_gpt4_planner("What is the weather like today?")) # Should be natural language

```

**Claude Example Prompt**:

Anthropic's Claude models (via the Bedrock or Anthropic API) also support function calling. The structure is slightly different, usually involving an XML-like syntax for describing tools or dedicated tool-use fields.

```python
import anthropic
import json

# Assume you have your Anthropic API key set as an environment variable
# client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

# Tools need to be defined in a specific format for Claude
claude_tools = [
    {
        "name": "move_robot",
        "description": "Moves the robot to a specified target location or relative position. Use positive values for forward/right/up, negative for backward/left/down.",
        "input_schema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "X displacement in meters"},
                "y": {"type": "number", "description": "Y displacement in meters"},
                "z": {"type": "number", "description": "Z displacement in meters"},
                "relative": {"type": "boolean", "description": "True if displacement is relative to current position, False for absolute target."}
            },
            "required": ["x", "y", "z", "relative"]
        }
    },
    {
        "name": "wave_hand",
        "description": "Makes the robot wave its specified hand.",
        "input_schema": {
            "type": "object",
            "properties": {
                "hand": {"type": "string", "enum": ["left", "right"]}
            },
            "required": ["hand"]
        }
    }
]

def call_claude_planner(user_command):
    response = anthropic.Anthropic().messages.create(
        model="claude-3-opus-20240229", # Or claude-3-sonnet-20240229, claude-3-haiku-20240229
        max_tokens=1024,
        tools=claude_tools,
        messages=[
            {"role": "user", "content": user_command}
        ]
    )

    if response.stop_reason == "tool_use":
        for tool_use in response.content:
            if tool_use.type == "tool_use":
                function_name = tool_use.name
                function_args = tool_use.input
                print(f"Claude decided to call tool: {function_name} with args: {function_args}")
                return {"tool_call": function_name, "args": function_args}
    else:
        print(f"Claude responded with natural language: {response.content[0].text}")
        return {"content": response.content[0].text}

# Example Usage:
# print(call_claude_planner("Make the robot wave its left hand please."))
# print(call_claude_planner("Go forward by 0.1 meter and up by 0.05 meter."))
```

## 4.2.5 System Architecture Overview: Cognitive Planning

```mermaid
graph TD
    A[ROS 2 Topic: `/voice_command` (std_msgs/String)] --> B{Planner Node (my_vla_planner)};
    B --> C{Local LLM (llama_cpp)};
    B -- Optional --> D{Cloud LLM API (e.g., GPT-4, Claude)};
    C -- Decides / Generates --> E[ROS 2 Action Sequence (e.g., `TOOL_CALL: move_robot(...)`)];
    D -- Decides / Generates --> E;
    E --> F[ROS 2 Topic: `/robot_action_sequence` (std_msgs/String)];
    F --> G[Next Node in VLA Pipeline (e.g., Orchestrator Node)];
```

*Figure 4.2.1: Data Flow for Cognitive Planning Node*

**Explanation**:
1.  **ROS 2 Topic `/voice_command`**: Receives transcribed voice commands from the `voice_input_node`.
2.  **Planner Node**: The central component of this chapter. It subscribes to voice commands.
3.  **Local LLM (`llama_cpp`)**: An on-device LLM (e.g., running on Jetson Orin) that the Planner Node interacts with. It interprets the voice command and, using the "tool-use" paradigm, generates a sequence of robot actions.
4.  **Cloud LLM API**: An optional, external LLM that the Planner Node can call for more complex reasoning. It also generates robot action sequences.
5.  **ROS 2 Action Sequence**: The output of the LLM, formatted as a string representing a tool call with arguments, or a natural language response if no tool is applicable.
6.  **ROS 2 Topic `/robot_action_sequence`**: The Planner Node publishes the generated action sequence to this topic, making it available for execution by other robot systems.
7.  **Next Node in VLA Pipeline**: Typically the Orchestrator Node, which will parse and execute these action sequences.

## 4.2.6 Troubleshooting & Optimization Guide

### LLM Generates Invalid or Unsafe Plans

-   **Problem**: The LLM, especially smaller local models or poorly prompted cloud models, might generate action sequences that are syntactically incorrect (e.g., wrong tool name, missing arguments) or logically unsafe (e.g., command to move into an obstacle).
-   **Solutions**:
    -   **Robust Prompt Engineering**: Craft clear, precise system prompts that define the LLM's role, the available tools (including their exact signatures and safety constraints), and what constitutes a valid output. Emphasize safety and error handling in the prompt.
    -   **Output Validation**: Implement a robust parsing and validation layer in the `PlannerNode` to check the LLM's output *before* attempting to execute any robot action. This layer should verify tool names, argument types, and argument ranges against known safe operational parameters. Reject invalid outputs.
    -   **Constrained Output (Grammars)**: Some LLM frameworks (like `llama_cpp`) allow the use of grammars (e.g., GBNF) to force the LLM to output JSON or other structured formats, reducing syntactic errors.
    -   **Fine-tuning**: For specific robotics tasks, fine-tuning a smaller LLM on a dataset of safe and valid command-action pairs can significantly improve its reliability.
    -   **Human-in-the-Loop**: For critical applications, introduce a human confirmation step before executing LLM-generated plans.
    -   **Contextual Safety**: Provide the LLM with real-time sensor data or a simplified environment model to help it understand the current state and avoid unsafe actions.

### API Timeouts or High Costs (for Cloud LLMs)

-   **Problem**: Cloud LLM API calls can suffer from network latency, server load, or unexpected high costs for extensive usage, impacting robot responsiveness or budget.
-   **Solutions**:
    -   **Rate Limiting and Retry Mechanisms**: Implement exponential backoff and retry logic for API calls to handle transient network issues and API rate limits gracefully.
    -   **Local Fallback**: Design the `PlannerNode` to preferentially use the local LLM and only fall back to a cloud LLM if the local one fails or cannot handle a specific complexity (and if real-time constraints allow).
    -   **Caching**: Cache frequent or identical LLM responses to avoid redundant API calls.
    -   **Cost Monitoring**: Integrate API usage monitoring and set up alerts for budget overruns.
    -   **Model Selection**: Choose the most cost-effective cloud LLM model for the task (e.g., `gpt-3.5-turbo` or Claude Haiku for simpler planning, reserving `gpt-4` or Claude Opus for more complex scenarios).
    -   **Prompt Optimization**: Keep prompts concise to reduce token usage and thus cost.
    -   **Asynchronous Calls**: If latency is acceptable for certain commands, make cloud API calls asynchronously to avoid blocking the main robot control loop.

## 4.2.7 Chapter Project/Checkpoint: Action Sequence Generator

**Objective**: Demonstrate a functional `planner_node` that receives voice commands and converts them into valid ROS 2 action sequences using a local LLM.

**Deliverables**:
1.  A running `planner_node` on your system with a loaded quantized LLM.
2.  Terminal logs showing the `planner_node` receiving voice commands (e.g., from Chapter 4.1's `voice_input_node`) and publishing structured robot action sequences (tool calls).

**Demonstration Steps**:
1.  Ensure your `voice_input_node` from Chapter 4.1 is running in one terminal.
2.  Launch the `planner_node` in another terminal.
3.  Open a third terminal and echo the `/robot_action_sequence` topic:
    ```bash
    source install/setup.bash
    ros2 topic echo /robot_action_sequence
    ```
4.  Speak the following commands into your microphone:
    -   "Wave your right hand."
    -   "Move forward 0.5 meters."
    -   "Grasp the blue block at x one point zero, y zero point two, z zero point one."
    -   "Tell me a joke." (This should ideally result in a natural language response from the LLM, not a tool call).
5.  Verify that the `ros2 topic echo` terminal displays the correct `TOOL_CALL` formats for the first three commands and a natural language response for the last.

*(End of Chapter 4.2)*