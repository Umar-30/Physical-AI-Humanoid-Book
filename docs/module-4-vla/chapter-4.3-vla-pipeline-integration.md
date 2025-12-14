# Chapter 4.3: Grounding Language in Vision (Vision-Language Integration)

In this chapter, we explore how robots can use natural language to understand and locate objects in their visual field. We'll integrate cutting-edge open-vocabulary object detection, specifically Grounding DINO, with ROS 2 to enable your robot to "see" what you tell it to.

## 4.3.1 Open-Vocabulary Detection vs. Traditional Fixed-Class Detection

Before diving into implementation, let's understand the paradigm shift that open-vocabulary object detection brings compared to traditional methods.

### Traditional Fixed-Class Object Detection

Traditional object detection models (e.g., YOLO, Faster R-CNN, SSD) are trained on datasets with a predefined, fixed set of categories (e.g., "person", "car", "dog", "cup").
-   **Pros**: Highly accurate and fast for the classes they were trained on.
-   **Cons**:
    -   **Limited Vocabulary**: Cannot detect objects outside their trained categories. If you ask it to find a "stapler" and it wasn't in the training data, it simply won't.
    -   **Scalability Issue**: Adding new objects requires retraining the entire model or significant data annotation effort.
    -   **Brittleness**: Performance degrades significantly if objects appear in contexts or variations not seen during training.

### Open-Vocabulary Object Detection

Open-vocabulary detection models aim to detect *any* object described by a natural language query, even if they haven't seen that specific object class during training. They leverage large-scale vision-language pre-training to establish a strong connection between textual descriptions and visual features. **Grounding DINO** is a prime example of such a model.

-   **Pros**:
    -   **Flexible Vocabulary**: Can detect novel objects using text descriptions (e.g., "the shiny object", "a specific brand of coffee mug").
    -   **Zero-Shot/Few-Shot Capabilities**: Detects objects without explicit retraining, or with very few examples.
    -   **Adaptability**: Easily adapts to new tasks or environments by simply changing the text prompt.
-   **Cons**:
    -   **Computational Cost**: Often more computationally intensive than highly optimized fixed-class detectors, making real-time performance on edge devices challenging.
    -   **Ambiguity**: Performance can be sensitive to the quality and specificity of the natural language prompt. Ambiguous descriptions can lead to incorrect or multiple detections.
    -   **Accuracy**: May not match the precision of fine-tuned fixed-class detectors for very specific, well-defined categories they were not trained on.

For robotics, open-vocabulary detection is a game-changer. It allows robots to understand commands like "pick up the red book on the table next to the lamp" without needing to be pre-programmed for every possible object in the environment.

## 4.3.2 Implementing `scan_for_object` ROS Action Server with Grounding DINO

We will create a ROS 2 Action Server that interfaces with a Grounding DINO model (assumed to be running as a separate inference service or integrated within the node for simplicity). This action server will receive natural language object descriptions and return the detected object's pose.

### Prerequisites

-   Completion of Chapter 4.1 and 4.2 setups.
-   A Grounding DINO inference service running. For a quick start, you can adapt a Grounding DINO standalone script or use a pre-built Docker container. For this tutorial, we will assume an HTTP API endpoint for Grounding DINO.
-   A camera publishing images to a ROS 2 topic (e.g., `/camera/image_raw`).

### Step-by-Step Instructions

1.  **Define a Custom ROS 2 Action**:
    Grounding DINO will detect bounding boxes and return confidence scores. For our robotic system, we need the location (pose) of the object. We'll define a simple action for this.
    
    Inside your `~/ros2_ws/src/my_vla_planner` package (or create a new one, e.g., `my_vla_vision`), create a directory `action` and a file `ScanObject.action`:
    ```
    # Request
    string object_description  # e.g., "red cup", "the monitor on the desk"
    ---
    # Result
    bool success
    string message
    geometry_msgs/Pose object_pose  # Pose of the detected object
    ---
    # Feedback
    string status_message # e.g., "Scanning for 'red cup'..."
    ```
    
    Update `package.xml` and `CMakeLists.txt` for the action definition. Add to `package.xml`:
    ```xml
      <build_depend>rosidl_default_generators</build_depend>
      <member_of_group>rosidl_interface_packages</member_of_group>
      <exec_depend>rosidl_default_runtime</exec_depend>
    ```
    Add to `CMakeLists.txt`:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
      action/ScanObject.action
      DEPENDENCIES geometry_msgs
    )
    ```
    Then, rebuild your workspace (`colcon build`) to generate the necessary action interfaces.

2.  **Create the `scan_for_object_action_server.py` Script**:
    Inside `my_vla_vision/my_vla_vision` (create package/folders if necessary), create a file `scan_for_object_action_server.py`:

    ```python
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose, Point, Quaternion
    import cv2
    import numpy as np
    import base64
    import requests
    import threading
    import time
    
    from my_vla_planner_interfaces.action import ScanObject
    
    GROUNDING_DINO_API_URL = "http://localhost:8000/grounding_dino_infer"
    
    class ScanForObjectActionServer(Node):
        def __init__(self):
            super().__init__('scan_for_object_action_server')
            self._action_server = ActionServer(
                self,
                ScanObject,
                'scan_for_object',
                self.execute_callback)
            self.get_logger().info('ScanForObject Action Server started.')
    
            self.bridge = CvBridge()
            self.latest_image = None
            self.image_lock = threading.Lock()
    
            self.image_subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)
            self.get_logger().info(f"Subscribing to {self.image_subscription.topic}")
    
        def image_callback(self, msg):
            with self.image_lock:
                self.latest_image = msg
                
        async def execute_callback(self, goal_handle):
            self.get_logger().info(f'Received goal: "{goal_handle.request.object_description}"')
            feedback_msg = ScanObject.Feedback()
            feedback_msg.status_message = f"Scanning for '{goal_handle.request.object_description}'..."
            goal_handle.publish_feedback(feedback_msg)
    
            result = ScanObject.Result()
            result.success = False
            result.message = "Failed to detect object."
    
            start_time = time.time()
            while self.latest_image is None and rclpy.ok():
                self.get_logger().info("Waiting for camera image...")
                time.sleep(0.5)
                if time.time() - start_time > 10:
                    result.message = "Timed out waiting for camera image."
                    goal_handle.abort()
                    return result
    
            with self.image_lock:
                current_image = self.latest_image
                self.latest_image = None
    
            if current_image is None:
                result.message = "No image received for detection."
                goal_handle.abort()
                return result
    
            try:
                cv_image = self.bridge.imgmsg_to_cv2(current_image, "bgr8")
                _, img_encoded = cv2.imencode('.jpg', cv_image)
                img_base64 = base64.b64encode(img_encoded).decode('utf-8')
    
                payload = {
                    "image": img_base64,
                    "text_prompt": goal_handle.request.object_description
                }
                headers = {"Content-Type": "application/json"}
                
                self.get_logger().info(f"Sending image to Grounding DINO service at {GROUNDING_DINO_API_URL}...")
                response = requests.post(GROUNDING_DINO_API_URL, json=payload, headers=headers, timeout=10)
                response.raise_for_status()
                
                detection_results = response.json()
    
                if detection_results and detection_results.get('detections'):
                    det = detection_results['detections'][0]
                    bbox_xyxy = det.get('box')
                    confidence = det['confidence']
                    label = det['label']
    
                    self.get_logger().info(f"Detected '{label}' with confidence {confidence*100:.2f}% at {bbox_xyxy}")
    
                    center_x = (bbox_xyxy[0] + bbox_xyxy[2]) / 2
                    center_y = (bbox_xyxy[1] + bbox_xyxy[3]) / 2
    
                    object_pose = Pose()
                    object_pose.position.x = 0.5
                    object_pose.position.y = (center_x - current_image.width / 2) * 0.001
                    object_pose.position.z = (current_image.height / 2 - center_y) * 0.001
                    object_pose.orientation.w = 1.0
    
                    result.success = True
                    result.message = f"Successfully detected '{label}'."
                    result.object_pose = object_pose
                    goal_handle.succeed()
                else:
                    result.message = f"No object matching '{goal_handle.request.object_description}' detected."
                    goal_handle.abort()
    
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Grounding DINO service communication error: {e}")
                result.message = f"Service communication error: {e}"
                goal_handle.abort()
            except Exception as e:
                self.get_logger().error(f"Error during object detection: {e}")
                result.message = f"Internal detection error: {e}"
                goal_handle.abort()
                
            return result
    
    def main(args=None):
        rclpy.init(args=args)
        action_server = ScanForObjectActionServer()
        rclpy.spin(action_server)
        action_server.destroy_node()
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()
    ```
3.  **Update `setup.py`**:
    Add the `scan_for_object_action_server` entry point and `requests` dependency to `my_vla_vision/setup.py`:
    ```python
    from setuptools import find_packages, setup

    package_name = 'my_vla_vision' # Adjust package name if different

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'geometry_msgs', 'numpy', 'opencv-python', 'requests'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your.email@example.com',
        description='ROS 2 package for object scanning with Grounding DINO',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'scan_for_object_server = my_vla_vision.scan_for_object_action_server:main',
            ],
        },
    )
    ```

4.  **Build Your ROS 2 Package**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_vla_vision
    ```

5.  **Run a Sample Grounding DINO Inference Service**:
    *This step is crucial but outside the direct scope of this ROS 2 node. You'd typically use a separate Python script with FastAPI to wrap the Grounding DINO model.*

    **Example `grounding_dino_service.py` (FastAPI + Grounding DINO)**:
    ```python
    from fastapi import FastAPI, HTTPException
    from pydantic import BaseModel
    import base64
    import numpy as np
    import cv2
    from PIL import Image as PILImage
    import io
    import torch

    # Assuming Grounding DINO setup is available
    # from GroundingDINO.groundingdino.models import build_model
    # from GroundingDINO.groundingdino.util.slconfig import SLConfig
    # from GroundingDINO.groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap
    # from GroundingDINO.groundingdino.util.inference import annotate, predict

    app = FastAPI()

    class InferenceRequest(BaseModel):
        image: str # Base64 encoded image
        text_prompt: str

    class Detection(BaseModel):
        box: list[float] # [x1, y1, x2, y2]
        confidence: float
        label: str

    class InferenceResponse(BaseModel):
        detections: list[Detection]

    # --- Load Grounding DINO model (Placeholder) ---
    # In a real setup, load your Grounding DINO model here.
    # This might involve downloading weights, configuring model, etc.
    # For demonstration, we'll simulate a detection.
    # config_path = "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
    # checkpoint_path = "groundingdino_swint_ogc.pth" # Download this
    # model = load_model(config_path, checkpoint_path)
    print("WARNING: Grounding DINO model is NOT actually loaded. Simulating detections.")
    # --- End Grounding DINO model loading ---

    @app.post("/grounding_dino_infer", response_model=InferenceResponse)
    async def grounding_dino_infer(request: InferenceRequest):
        try:
            # Decode base64 image
            img_bytes = base64.b64decode(request.image)
            img_pil = PILImage.open(io.BytesIO(img_bytes)).convert("RGB")
            # Convert PIL Image to OpenCV format (optional, if your DINO expects OpenCV)
            # cv_image = np.array(img_pil)
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # --- Simulate Grounding DINO Prediction ---
            # Replace this with actual Grounding DINO inference logic
            # Example: boxes, logits, phrases = predict(model, img_pil, request.text_prompt)
            # detections = []
            # for box, logit, phrase in zip(boxes, logits, phrases):
            #    detections.append(Detection(box=box.tolist(), confidence=float(logit), label=phrase))
            
            # Simulated detection for "red cup" if prompt matches
            simulated_detections = []
            if "cup" in request.text_prompt.lower() or "mug" in request.text_prompt.lower():
                # Example bbox for a 640x480 image
                simulated_detections.append(Detection(box=[100.0, 100.0, 200.0, 250.0], confidence=0.85, label=request.text_prompt))
            if "bottle" in request.text_prompt.lower():
                 simulated_detections.append(Detection(box=[300.0, 200.0, 400.0, 350.0], confidence=0.78, label=request.text_prompt))
            
            if not simulated_detections:
                raise HTTPException(status_code=404, detail="No object detected matching prompt (simulated).")

            return InferenceResponse(detections=simulated_detections)

        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    # To run this service:
    # uvicorn grounding_dino_service:app --reload --port 8000
    ```
    Save this as `grounding_dino_service.py` in a convenient location (e.g., `~/vla_services/`). Install `uvicorn`, `fastapi`, `Pillow`, `torch`, `opencv-python`.
    Start the service: `uvicorn grounding_dino_service:app --reload --port 8000`

6.  **Source Your Workspace and Run the Action Server**:
    First, ensure you have a camera publishing images to `/camera/image_raw` (e.g., a webcam using `ros2 run image_tools cam2image`).
    ```bash
    # In Terminal 1: Camera publisher
    ros2 run image_tools cam2image

    # In Terminal 2: Grounding DINO service (as described above)
    uvicorn grounding_dino_service:app --reload --port 8000

    # In Terminal 3: Action Server
    source install/setup.bash
    ros2 run my_vla_vision scan_for_object_server
    ```

7.  **Send an Action Goal**:
    In a new terminal, send a goal to the action server:
    ```bash
    source install/setup.bash
    ros2 action send_goal scan_for_object my_vla_planner_interfaces/action/ScanObject "{object_description: 'red cup'}"
    ```
    You should see the action server processing the request and eventually returning a result with a detected pose.

## 4.3.3 System Architecture Overview: Vision-Language Integration

```mermaid
graph TD
    A[Camera (/camera/image_raw)] --> B{ScanForObjectActionServer};
    B -- Requests --> C{Grounding DINO Inference Service (HTTP API)};
    C -- Detections --> B;
    B -- Action Goal Received (object_description) --> B;
    B -- Action Result (success, object_pose) --> D[ROS 2 Action Client (e.g., Orchestrator Node)];
```

*Figure 4.3.1: Data Flow for Vision-Language Integration Node*

**Explanation**:
1.  **Camera**: Publishes raw image data to a ROS 2 topic.
2.  **ScanForObjectActionServer**: A ROS 2 action server that subscribes to camera images. When it receives a `ScanObject` action goal with an `object_description`, it processes the latest image.
3.  **Grounding DINO Inference Service**: An external service (e.g., FastAPI application) running the Grounding DINO model. It receives images and text prompts from the action server and returns object detections (bounding boxes, labels, confidence).
4.  **Action Goal Received**: The `scan_for_object` action server receives requests from an action client (e.g., the Orchestrator Node).
5.  **Action Result**: After performing detection, the action server returns a result indicating success or failure, and if successful, the estimated pose of the detected object.
6.  **ROS 2 Action Client**: The entity that initiates the `ScanObject` action, waiting for the result.

## 4.3.4 Troubleshooting & Optimization Guide

### Open-Vocabulary Detector is Slow on Jetson

-   **Problem**: Grounding DINO models, especially larger ones, can be computationally intensive, leading to high latency on edge devices like Jetson Orin.
-   **Solutions**:
    -   **Model Quantization**: Use smaller, quantized versions of Grounding DINO (or similar models) that are optimized for edge inference. Explore `int8` or `int4` quantization if available.
    -   **Hardware Acceleration**: Ensure Grounding DINO inference is fully leveraging the GPU on the Jetson Orin. This often requires specific builds of PyTorch and related libraries (e.g., TensorRT integration).
    -   **Resolution Reduction**: Process images at a lower resolution before feeding them to Grounding DINO. This significantly reduces computational load, though it may impact detection of small objects.
    -   **Batch Processing**: If multiple detection requests can be queued, process them in batches to improve GPU utilization.
    -   **Dedicated Inference Service**: Run the Grounding DINO model as a dedicated service (e.g., with NVIDIA Triton Inference Server) optimized for Jetson, separate from the ROS 2 node, to manage resources effectively.
    -   **Frame Skipping**: For less critical applications, process every Nth frame from the camera feed rather than every single frame.

### Detector Fails on Ambiguous Descriptions

-   **Problem**: Grounding DINO might return no detections or incorrect detections for vague or overly general object descriptions (e.g., "thing," "part").
-   **Solutions**:
    -   **Prompt Engineering**: Emphasize clarity and specificity in the natural language descriptions provided to the LLM that generates the `object_description`. Guide the LLM to ask clarifying questions if a command is ambiguous.
    -   **Contextual Information**: If possible, augment the text prompt with contextual information (e.g., "red cup on the table").
    -   **Confidence Thresholding**: Filter detections based on a confidence threshold. Only accept detections above a certain confidence level.
    -   **User Feedback Loop**: If a detection fails, the robot can ask for clarification from the user ("I couldn't find 'that thing'. Can you be more specific?").
    -   **Multiple Attempts**: Implement a strategy where the robot tries slightly different prompts or scans different areas if the initial detection fails.
    -   **Domain-Specific Fine-tuning**: For highly recurrent objects in a specific environment, fine-tuning an open-vocabulary model (or even using a traditional fixed-class detector) can improve robustness.

## 4.3.5 Chapter Project/Checkpoint: Open-Vocabulary Object Scanner

**Objective**: Demonstrate a functional `scan_for_object` ROS 2 action server that can detect objects based on natural language descriptions using a simulated Grounding DINO service.

**Deliverables**:
1.  A running `scan_for_object_action_server` node.
2.  A running (simulated) Grounding DINO inference service.
3.  Terminal logs showing successful detection of specified objects and returning their (simulated) poses.

**Demonstration Steps**:
1.  Ensure your camera publisher (`ros2 run image_tools cam2image`) is running in one terminal.
2.  Launch your (simulated) `grounding_dino_service.py` (using `uvicorn`) in another terminal.
3.  Launch the `scan_for_object_action_server` in a third terminal.
4.  In a fourth terminal, send several action goals to detect different objects:
    ```bash
    # Example 1: Detect a cup
    ros2 action send_goal scan_for_object my_vla_planner_interfaces/action/ScanObject "{object_description: 'red cup'}"

    # Example 2: Detect a bottle
    ros2 action send_goal scan_for_object my_vla_planner_interfaces/action/ScanObject "{object_description: 'water bottle'}"

    # Example 3: Detect an object not handled by simulation (should fail gracefully)
    ros2 action send_goal scan_for_object my_vla_planner_interfaces/action/ScanObject "{object_description: 'book'}"
    ```
5.  Verify that the `scan_for_object_action_server` logs show the detection attempts and that successful detections return a pose, while undetected objects result in a "No object detected" message.

*(End of Chapter 4.3)*