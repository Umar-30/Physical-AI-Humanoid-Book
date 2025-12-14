# Chapter 1: Photorealistic Simulation & Synthetic Data (Isaac Sim)

This chapter focuses on setting up NVIDIA Isaac Sim, creating high-fidelity virtual environments, and generating synthetic data for AI model training.

## 1.1 Isaac Sim Docker Setup and UI Navigation

NVIDIA Isaac Sim is a powerful robotics simulation and synthetic data generation platform built on NVIDIA Omniverse. It's often deployed via Docker containers to ensure consistency and leverage GPU acceleration. This section will guide you through setting up Isaac Sim using Docker and navigating its user interface.

### 1.1.1 Prerequisites (Review)

Before proceeding, ensure your workstation meets the following:
*   Ubuntu 22.04 LTS.
*   NVIDIA GPU with up-to-date drivers.
*   Docker Engine and NVIDIA Container Toolkit installed and configured (refer to `environment-setup.md`).
*   Sufficient disk space and GPU memory (minimum 24GB VRAM recommended for Isaac Sim).

### 1.1.2 Pulling the Isaac Sim Docker Image

Isaac Sim Docker images are available on NVIDIA NGC (NVIDIA GPU Cloud). You'll need an NGC API key for authenticated pulls, or you can use public versions if available. For detailed instructions, always refer to the official [NVIDIA Isaac Sim documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_container.html).

1.  **Log in to NGC (if using private images or needing authentication)**:
    ```bash
    docker login nvcr.io
    ```
    You'll be prompted for a username (typically `$oauthtoken`) and your NGC API key as the password.

2.  **Pull the latest Isaac Sim image**:
    Replace `5.0.0` with the desired version tag. Always check the [Isaac Sim release notes](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/release_notes.html) for the latest recommended tag.
    ```bash
    docker pull nvcr.io/nvidia/isaac-sim:5.0.0
    ```
    This may take some time as the image is several gigabytes.

### 1.1.3 Running Isaac Sim in a Docker Container

Launching Isaac Sim requires specific Docker configurations to ensure GPU, display, and audio forwarding.

1.  **Grant X Server Access**:
    Isaac Sim's UI runs on your host's X server.
    ```bash
    xhost +
    ```
    *Note: This command grants access to any user on the system. For a more secure approach, consider `xhost +local:` or specific user/IP addresses if in a multi-user environment.*

2.  **Launch Isaac Sim Container**:
    Use a command similar to the following. Adjust paths and versions as necessary.

    ```bash
    docker run --name isaac-sim --privileged --network=host --env="ACCEPT_EULA=Y" \
        -e "DISPLAY" \
        -e "NVIDIA_VISIBLE_DEVICES=all" \
        -e "NVIDIA_DRIVER_CAPABILITIES=all" \
        -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
        -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
        -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
        -v ~/docker/isaac-sim/cache/ov_data:/root/.local/share/ov/data:rw \
        -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs/Kit/IsaacSim:rw \
        -v ~/docker/isaac-sim/data:/root/.local/share/ov/pkg/isaac_sim-5.0.0/data:rw \
        -v ~/docker/isaac-sim/documents:/root/Documents:rw \
        nvcr.io/nvidia/isaac-sim:5.0.0
    ```
    *   **Explanation of Docker arguments**:
        *   `--name isaac-sim`: Assigns a name to your container for easy management.
        *   `--privileged`: Grants extended privileges to the container, often needed for full hardware access.
        *   `--network=host`: Allows the container to use the host's network stack, simplifying communication.
        *   `-e "ACCEPT_EULA=Y"`: Accepts the NVIDIA EULA.
        *   `-e "DISPLAY"`: Forwards the host's `DISPLAY` environment variable to the container for GUI.
        *   `-e "NVIDIA_VISIBLE_DEVICES=all"`: Allows the container to see all NVIDIA GPUs.
        *   `-e "NVIDIA_DRIVER_CAPABILITIES=all"`: Grants all driver capabilities.
        *   `-v <host_path>:<container_path>:rw`: Mounts host directories into the container for persistent storage of cache, logs, and user data. This is crucial to prevent data loss when the container is removed.
        *   `nvcr.io/nvidia/isaac-sim:5.0.0`: The Docker image to run.

    The first launch might take some time as Isaac Sim initializes. You should see the Isaac Sim application window appear on your desktop.

### 1.1.4 Navigating the Isaac Sim User Interface

Once Isaac Sim launches, you'll be greeted by its graphical user interface. Key areas to familiarize yourself with include:

1.  **Viewport**: The central 3D window where your simulation environment and robot models are displayed.
    *   **Navigation**:
        *   **Orbit**: Alt + Left Mouse Button
        *   **Pan**: Alt + Middle Mouse Button
        *   **Zoom**: Alt + Right Mouse Button (or Mouse Wheel)
        *   **Dolly**: Mouse Wheel (scroll)
        *   **Fly Camera**: Right Mouse Button (hold) + W/A/S/D/Q/E (similar to first-person games)

2.  **Stage Panel (Left)**: This panel displays the USD (Universal Scene Description) hierarchy of your simulation. You can select, move, and modify objects here.
    *   **Adding Prims**: Click the `+` icon to add basic primitives (cubes, spheres, cylinders) or import existing assets.
    *   **Searching**: Use the search bar to find specific objects in the hierarchy.

3.  **Properties Panel (Right)**: Shows the properties of the currently selected object in the Stage panel. Here you can adjust its position, rotation, scale, physics properties, materials, and attached components.
    *   **Transform**: Adjust `Translate` (position), `Rotate` (orientation), `Scale` (size).
    *   **Physics**: Enable/disable physics, set mass, friction, etc.

4.  **Content Browser (Bottom)**: Access various assets (3D models, materials, environments) provided by Omniverse or imported by you.
    *   **Local Assets**: Your local project assets.
    *   **Omniverse Nucleus**: Connect to Omniverse Nucleus servers for shared assets and collaboration.

5.  **Toolbar (Top)**: Contains common actions like Play/Stop simulation, save/load scene, etc.

Practice navigating the 3D viewport and interacting with some basic primitives. This foundational understanding of the UI is crucial for building and manipulating your simulation environments.

## 1.2 Creating Photorealistic Environments and Importing Assets

NVIDIA Isaac Sim, built on Universal Scene Description (USD), allows for the creation of rich, photorealistic 3D environments. This section covers the basics of scene composition, lighting, and importing assets to build compelling simulation worlds.

### 1.2.1 Understanding USD (Universal Scene Description)

USD is the foundation of Omniverse and Isaac Sim. It's a powerful framework for describing, composing, simulating, and collaborating on 3D scenes. Key concepts:
*   **Stage**: The root of your USD hierarchy, representing the entire scene.
*   **Prim (Primitive)**: The fundamental object in USD. Everything in your scene (mesh, light, camera, material) is a prim.
*   **Layering**: USD allows non-destructive editing through layers. You can add overrides to existing assets without modifying the original.

### 1.2.2 Building a Basic Scene

Let's compose a simple scene with a ground plane and some objects.

1.  **Start Isaac Sim**: Ensure Isaac Sim is running from the Docker container as described in Section 1.1.

2.  **Create a New Stage**:
    *   In the Isaac Sim UI, go to `File` -> `New Stage`. This clears any previous scene.

3.  **Add a Ground Plane**:
    *   In the `Stage` panel (left), right-click on `/World`.
    *   Select `Create` -> `Physics` -> `Ground Plane`.
    *   This adds a large, flat plane for your robot to sit on.

4.  **Add a Light Source**:
    *   For photorealism, good lighting is essential. In the `Stage` panel, right-click on `/World`.
    *   Select `Create` -> `Light` -> `Dome Light`.
    *   A Dome Light provides ambient lighting, mimicking an outdoor sky. You can adjust its intensity and color in the `Properties` panel (right) when selected.

5.  **Add Basic Primitive Objects**:
    *   In the `Stage` panel, right-click on `/World`.
    *   Select `Create` -> `Mesh` -> `Cube`, `Sphere`, or `Cylinder`.
    *   Select each newly created primitive in the `Stage` panel. Use the `Transform` properties in the `Properties` panel (right) to adjust their `Translate` (position), `Rotate` (orientation), and `Scale` (size) to arrange them in your scene. For example, position them on the ground plane.

### 1.2.3 Importing Assets from the Content Browser

Isaac Sim provides access to a rich library of 3D assets through its Content Browser.

1.  **Open Content Browser**: The `Content Browser` is usually at the bottom of the Isaac Sim UI. If not visible, go to `Window` -> `Content Browser`.

2.  **Navigate Asset Libraries**:
    *   Explore the `Local` folder for your local Omniverse assets.
    *   Explore `Omniverse` -> `localhost` (or your Nucleus server) -> `Assets` for shared Omniverse assets.
    *   Common assets for robotics can be found in `Omniverse` -> `localhost` -> `Isaac` -> `Robots`, `Environments`, etc.

3.  **Import an Asset**:
    *   Drag and drop an asset (e.g., a robot model from `Isaac/Robots`) directly from the Content Browser into your 3D Viewport.
    *   Alternatively, right-click on an asset in the Content Browser and select `Add to Stage`.

4.  **Position and Scale Imported Assets**:
    *   Select the imported asset in the `Stage` panel.
    *   Use the `Transform` tools in the `Properties` panel to adjust its position and scale within your scene. Ensure robots are positioned correctly on the ground plane.

### 1.2.4 Adding Materials

Materials define the visual appearance of objects (color, texture, reflectivity, etc.).

1.  **Apply a Material from Content Browser**:
    *   In the `Content Browser`, navigate to `Omniverse` -> `localhost` -> `Materials`.
    *   Drag and drop a material (e.g., `PBR_Plastic_Black`) onto an object in your Viewport.
    *   Select the object and observe its material properties in the `Properties` panel.

2.  **Adjust Material Properties**:
    *   Select an object with a material in the `Stage` panel.
    *   In the `Properties` panel, scroll down to the `Material` section. You can often adjust parameters like `Base Color`, `Roughness`, `Metallic` for physically based rendering (PBR) materials.

By combining prims, assets, lighting, and materials, you can create increasingly complex and visually appealing simulation environments for your AI robots.

## 1.3 Generating Synthetic Data with Isaac Sim Replicator

NVIDIA Isaac Sim's most powerful feature for AI training is its ability to generate high-quality synthetic data through the Omniverse Replicator API. Replicator allows you to programmatically control scene elements, randomize properties, and automatically generate diverse datasets with ground truth labels (e.g., bounding boxes, semantic segmentation, depth maps).

### 1.3.1 Understanding Omniverse Replicator

Omniverse Replicator is a Python API that extends Isaac Sim's capabilities for synthetic data generation. It enables:
*   **Domain Randomization**: Randomizing aspects of your simulation (textures, lighting, object positions, camera pose) to improve the generalization of models trained on synthetic data to the real world.
*   **Ground Truth Generation**: Automatically capturing precise labels that are difficult or impossible to obtain manually (e.g., pixel-perfect segmentation masks, accurate 3D bounding boxes, instance IDs).
*   **Programmable Simulations**: Creating complex simulation scenarios and data generation pipelines with Python scripts.

### 1.3.2 Setting Up Your Replicator Script

You'll typically write Replicator scripts as Python files that run within Isaac Sim.

1.  **Open the Script Editor**:
    *   In Isaac Sim, go to `Window` -> `Script Editor`.
    *   This opens a Python console and editor where you can write and execute scripts.

2.  **Create a New Python Script**:
    *   In the `Script Editor`, click `File` -> `New` to start a new script.
    *   Save this script to your mounted `documents` folder (e.g., `~/docker/isaac-sim/documents/my_first_replicator.py`).

### 1.3.3 Example: Generating Random Cubes with Segmentation Masks

Let's create a simple Replicator script to spawn random cubes, randomize their properties, and generate semantic segmentation masks.

```python
import omni.replicator.core as rep
from pxr import Gf, UsdLux, Sdf

# 1. Initialize Replicator
# This command connects to Isaac Sim's rendering engine
rep.initialize()

# 2. Setup Render Product (Camera)
# Define a camera and set its resolution
camera = rep.create.camera(position=(0, 0, 1.5))
render_product = rep.create.render_product(camera, (1280, 720))

# 3. Define the scene content
# Create a light source
light = rep.create.light(
    light_type=UsdLux.Tokens.Distant,
    intensity=3000,
    position=(0, 0, 0),
    rotation=(0, 0, 0),
    temperature=6500,
)

# Create a ground plane
plane = rep.create.plane(
    scale=(100, 100, 1),
    position=(0, 0, 0),
    rotation=(0, 0, 0),
    color=Gf.Vec3f(0.5, 0.5, 0.5), # Grey color
)

# 4. Define the randomization logic
# Randomize the position and scale of 10 cubes
with rep.trigger.on_frame():
    # Randomize lights
    with rep.create.group([light]):
        rep.modify.attribute(rep.distribution.uniform((0, 0, 0), (0, 0, 360)), "xformOp:rotateXYZ")
        rep.modify.attribute(rep.distribution.uniform(1000, 5000), "intensity")

    # Randomize up to 10 cubes
    rep.create.cube(
        count=10,
        position=rep.distribution.uniform((-50, -50, 0), (50, 50, 20)),
        scale=rep.distribution.uniform(0.1, 1.0),
        semantics=[('cube', 'class')], # Assign a semantic class "cube"
        color=rep.distribution.uniform((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)), # Random color
    )

# 5. Attach writers to collect data
# Semantic segmentation writer
rep.WriterRegistry.get("BasicWriter").initialize(
    output_dir="~/output_data/semantic_segmentation",
    rgb=False,
    bounding_box_2d_tight=False,
    semantic_segmentation=True,
    instance_segmentation=False,
    distance_to_image_plane=False,
    occlusion_bounding_box_2d=False,
    occlusion_bounding_box_3d=False,
    bounding_box_3d=False,
    depth=False,
    normals=False,
    pointcloud=False,
    camera_param=False,
)

# RGB writer
rep.WriterRegistry.get("BasicWriter").initialize(
    output_dir="~/output_data/rgb",
    rgb=True,
    bounding_box_2d_tight=False,
    semantic_segmentation=False,
    instance_segmentation=False,
    distance_to_image_plane=False,
    occlusion_bounding_box_2d=False,
    occlusion_bounding_box_3d=False,
    bounding_box_3d=False,
    depth=False,
    normals=False,
    pointcloud=False,
    camera_param=False,
)

# 6. Run the simulation and generate data
# Generate 100 frames
rep.orchestrator.run(num_frames=100)

# 7. Cleanup
rep.shutdown()

print("Synthetic data generation complete!")
```

*   **To run this script**:
    1.  Save it as `my_first_replicator.py` in your mounted `documents` folder.
    2.  In Isaac Sim's `Script Editor`, open the file.
    3.  Click the `Run` button (green play icon) in the `Script Editor` toolbar.
    *   **Note**: The script will likely reset the scene on each run. Ensure you have saved any custom environments if you want to use them with your script.

    The script will generate 100 frames, each with a randomized scene, and save RGB images and semantic segmentation masks to `~/output_data/rgb` and `~/output_data/semantic_segmentation` respectively (relative to the container's root, which maps to your host's mounted `data` folder, e.g., `~/docker/isaac-sim/data/output_data/`).

### 1.3.4 Key Replicator Concepts

*   **`rep.initialize()` / `rep.shutdown()`**: Initializes and safely shuts down the Replicator API.
*   **`rep.create.camera()` / `rep.create.render_product()`**: Defines a camera and the resolution of the generated render product.
*   **`rep.create.cube()` / `rep.create.plane()` / `rep.create.light()`**: Programmatically creates prims in the scene.
*   **`with rep.trigger.on_frame():`**: Defines a block of code that executes on each frame of the simulation, allowing for dynamic randomization.
*   **`rep.distribution.uniform()`**: Generates random values within a specified range, used for domain randomization.
*   **`rep.modify.attribute()`**: Modifies attributes of prims, such as position, scale, color, or light intensity.
*   **`semantics=[('cube', 'class')]`**: Assigns semantic labels to objects, which are essential for generating ground truth segmentation masks.
*   **`rep.WriterRegistry.get("BasicWriter").initialize(...)`**: Configures the data writer to specify which types of ground truth data to capture (RGB, semantic segmentation, bounding boxes, depth, etc.) and where to save them (`output_dir`).
*   **`rep.orchestrator.run(num_frames=100)`**: Executes the simulation and triggers the data generation pipeline for the specified number of frames.

This example provides a basic foundation for generating synthetic data. Isaac Sim's Replicator API offers extensive customization for complex scenarios, diverse asset loading, and integration with external training pipelines.

## 1.4 Chapter 1 Project/Checkpoint: Synthetic Dataset Generation

This project challenges you to apply the concepts learned in Chapter 1 to generate a custom synthetic dataset.

### Project Goal

Create a simple Isaac Sim environment, populate it with at least two different types of assets (e.g., a robot model and some primitive shapes), and use Omniverse Replicator to generate a synthetic dataset of RGB images and corresponding semantic segmentation masks.

### Requirements

1.  **Environment Setup**: Ensure your workstation is correctly set up with Docker and Isaac Sim as described in Section 1.1.
2.  **Scene Composition**:
    *   Create a new Isaac Sim stage.
    *   Add a ground plane and appropriate lighting (e.g., a Dome Light).
    *   Import at least one robot model (e.g., from `Isaac/Robots`) and a few primitive shapes (cubes, spheres).
    *   Arrange the assets in a scene that would be relevant for a simple object detection or segmentation task.
3.  **Replicator Script**:
    *   Write a Python script using Omniverse Replicator (similar to the example in Section 1.3).
    *   The script should:
        *   Define a camera and render product.
        *   Apply domain randomization to at least two properties of the scene (e.g., object positions, rotations, lighting intensity, material colors).
        *   Assign semantic labels to the robot and primitive shapes.
        *   Configure writers to output **RGB images** and **semantic segmentation masks**.
        *   Generate at least 50 frames of data.
4.  **Data Output**: The script should successfully generate the synthetic dataset to a specified output directory on your host machine.

### Verification Steps

To verify the successful completion of this project:

1.  **Run Isaac Sim and your Replicator Script**: Execute your Python script within Isaac Sim.
2.  **Check Output Directory**: Navigate to the specified output directory on your host machine.
3.  **Inspect Generated Data**:
    *   Confirm that `rgb` and `semantic_segmentation` folders exist.
    *   Open several `.png` or `.exr` files from the `rgb` folder to visually confirm that diverse images of your scene (with randomized elements) have been generated.
    *   Open corresponding semantic segmentation images. Use an image viewer that can display single-channel images or check the pixel values. Each object you assigned a semantic label to should appear with a distinct color/ID in the segmentation masks.
    *   Optionally, use a Python script with libraries like OpenCV or Pillow to load a semantic segmentation image and display the unique pixel values to confirm distinct object IDs.

By successfully completing this project, you will have demonstrated proficiency in setting up Isaac Sim, composing 3D environments, and generating labeled synthetic data, a fundamental skill for AI-driven robotics development.

## Visual Aids & Diagrams

To enhance understanding and engagement in Chapter 1, the following visual aids and diagrams are recommended:

*   **Figure 1.1: Isaac Sim UI Overview**:
    *   **Description**: A screenshot of the Isaac Sim UI immediately after launch (or with a basic scene loaded), highlighting key panels: Viewport, Stage Panel, Properties Panel, Content Browser, and Toolbar. Arrows and labels should clearly indicate the function of each section.
    *   **Placement Hint**: After Section 1.1.4, "Navigating the Isaac Sim User Interface".

*   **Figure 1.2: USD Hierarchy Example**:
    *   **Description**: A screenshot of the Stage panel showing a simple USD hierarchy with `/World`, `GroundPlane`, `DistantLight`, and a few prims (e.g., `Cube`, `Sphere`, `Cylinder`) added. Illustrates the tree structure and parent-child relationships.
    *   **Placement Hint**: After Section 1.2.1, "Understanding USD (Universal Scene Description)".

*   **Figure 1.3: Asset Import from Content Browser**:
    *   **Description**: A screenshot showing the Content Browser open, with an asset (e.g., a robot model) being dragged and dropped into the Viewport, or the context menu for "Add to Stage" being selected.
    *   **Placement Hint**: After Section 1.2.3, "Importing Assets from the Content Browser".

*   **Figure 1.4: Replicator Data Flow Diagram**:
    *   **Description**: A conceptual diagram illustrating the flow of data in a Replicator pipeline: Isaac Sim Scene -> Replicator API (Python Script) -> Domain Randomization -> Render Product (Camera) -> Ground Truth Generation (Semantic Segmentation, RGB) -> Output Dataset.
    *   **Placement Hint**: After Section 1.3.1, "Understanding Omniverse Replicator".

*   **Figure 1.5: Example Synthetic Dataset Outputs**:
    *   **Description**: A composite image showing side-by-side examples of:
        1.  An RGB image generated by Replicator (from the random cubes example).
        2.  The corresponding semantic segmentation mask, clearly showing different objects with distinct colors/IDs.
    *   **Placement Hint**: After Section 1.3.3, "Example: Generating Random Cubes with Segmentation Masks".

These visuals will aid students in comprehending complex setups and abstract concepts more intuitively.

## Troubleshooting & Debugging Guide

This section addresses common issues encountered when working with Isaac Sim, particularly concerning Docker setup, launch, and rendering.

*   **1. Isaac Sim Container Fails to Launch or Exits Immediately**:
    *   **Problem**: The `docker run` command fails, or the Isaac Sim window briefly appears and then closes.
    *   **Possible Causes**: Incorrect Docker setup, NVIDIA driver issues, insufficient GPU memory, X server access problems, or an existing container with the same name.
    *   **Solution**:
        *   **Check Docker and NVIDIA Toolkit**: Re-verify Docker and NVIDIA Container Toolkit installations as per Section 1.1.1 and `environment-setup.md`. Run `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi` to confirm GPU visibility in Docker.
        *   **`xhost +`**: Ensure `xhost +` was run before launching the container. If you closed the terminal or rebooted, it might need to be re-executed.
        *   **Container Name Conflict**: If you used `--name isaac-sim`, check if a container with that name already exists and is stopped or running: `docker ps -a`. If it exists, remove it (`docker rm isaac-sim`) or use a different name.
        *   **GPU Memory**: Ensure your GPU has at least 24GB VRAM. If not, try closing other GPU-intensive applications.
        *   **Check Container Logs**: After an immediate exit, inspect the container logs: `docker logs <container_id_or_name>`. This often provides specific error messages.

*   **2. Poor Rendering Performance or Graphical Artifacts**:
    *   **Problem**: Isaac Sim UI is very slow, unresponsive, or displays visual glitches.
    *   **Possible Causes**: Outdated NVIDIA drivers, conflicting display settings, insufficient GPU resources, or running Isaac Sim over a remote desktop protocol without proper configuration.
    *   **Solution**:
        *   **Update NVIDIA Drivers**: Ensure your NVIDIA GPU drivers are the latest recommended version for your hardware and Ubuntu 22.04 (refer to `environment-setup.md`).
        *   **Close Other Apps**: Close any other applications that might be consuming significant GPU resources.
        *   **Check X11/Wayland**: Ensure your display server is correctly configured. For some setups, switching between X11 and Wayland (if applicable) can resolve issues.
        *   **Remote Desktop**: If using remote desktop (e.g., VNC, RDP), ensure your server and client are configured for hardware-accelerated OpenGL forwarding. Consider using NVIDIA's official Omniverse Workstation setup if remote access is critical.
        *   **Isaac Sim Settings**: Within Isaac Sim, you can adjust rendering quality settings (e.g., `Edit` -> `Preferences` -> `Render Settings`) to lower quality for better performance.

*   **3. Omniverse Nucleus Connection Issues**:
    *   **Problem**: Unable to connect to Omniverse Nucleus servers from within Isaac Sim, or Content Browser assets fail to load.
    *   **Possible Causes**: Network issues, incorrect Nucleus URL, firewall blocking ports, or Nucleus server not running.
    *   **Solution**:
        *   **Network Connectivity**: Verify your internet connection and ensure there are no network restrictions.
        *   **Nucleus Server Status**: If you are running a local Nucleus server, ensure it is running and accessible (check its logs).
        *   **Firewall**: Temporarily disable your firewall or ensure necessary ports for Omniverse (e.g., 8080, 8000, 3009) are open.
        *   **URL Verification**: Double-check the Nucleus server URL configured in Isaac Sim (`File` -> `Connect` -> `Manage Connections`).

These troubleshooting tips should help resolve the most common problems encountered during the initial stages of working with NVIDIA Isaac Sim. Always refer to the official [NVIDIA Omniverse Documentation](https://docs.omniverse.nvidia.com/) for the most up-to-date and comprehensive support.