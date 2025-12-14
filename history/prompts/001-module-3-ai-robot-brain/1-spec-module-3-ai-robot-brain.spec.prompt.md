---
id: 1
title: Spec for Module 3 AI Robot Brain Content
stage: spec
date: 2025-12-12
surface: agent
model: gemini-1.5-flash-latest
feature: 001-module-3-ai-robot-brain
branch: 001-module-3-ai-robot-brain
user: user
command: /sp.specify "You are an expert instructor in AI-powered robotics, specializing in NVIDIA's Isaac platform, ROS 2, and edge AI deployment. Your task is to write the complete, detailed instructional content for **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**, broken down into its four chapters as specified. For **each chapter and its subsections (1.1, 1.2, etc.)**, please provide: 1. **Detailed Step-by-Step Instructions:** Assume the student has completed Modules 1 & 2. They have a powerful Ubuntu 22.04 workstation with Docker and a Jetson Orin Nano/NX kit. Provide exact commands, configuration file snippets, and code. * Include specific Isaac Sim Docker run commands and navigation of its UI. * Provide complete `launch.py` or `.py` scripts for Isaac Sim's Replicator. * Give exact `ros2 launch` commands and YAML configuration snippets for Isaac ROS and Nav2. * Include sample Behavior Tree XML and Python nodes for task sequencing. 2. **Clear Explanations of Advanced Concepts:** Demystify complex topics. * Explain how Visual SLAM differs from wheel odometry and its importance for humanoids. * Describe the process of converting a PyTorch/TensorFlow model to a TensorRT engine and the performance benefits. * Clarify the role of each Nav2 server (Controller, Planner, Behavior Tree) in the context of bipedal movement. 3. **Visual Aids & Diagrams:** Describe the key diagrams needed (e.g., "Include a system architecture diagram showing the flow from Isaac Sim synthetic data -> model training -> Isaac ROS inference -> Nav2 planning"). 4. **Troubleshooting & Debugging Guide:** List common pitfalls for each chapter. * Chapter 1: Isaac Sim fails to launch or has rendering issues. * Chapter 2: VSLAM loses tracking or produces a drifting map; DNN inference is too slow on Jetson. * Chapter 3: Nav2 fails to find a valid path or the robot gets stuck in local costmap. * Chapter 4: The system works in sim but fails on the edge due to camera calibration or performance issues. 5. **Chapter Project/Checkpoint:** Define a clear, verifiable outcome for each chapter that students must demonstrate (e.g., for Chapter 2: "Show a screenshot of RViz displaying the live Visual SLAM map and the robot's estimated path as you move the camera"). **Structure your output exactly as follows:** ### Module 3: The AI-Robot Brain (NVIDIA Isaac™) #### Chapter 1: Photorealistic Simulation & Synthetic Data (Isaac Sim) *(Proceed with subsections 1.1, 1.2, 1.3, 1.4 as per the outline above)* #### Chapter 2: Hardware-Accelerated Perception (Isaac ROS) *(Proceed with subsections 2.1, 2.2, 2.3)* #### Chapter 3: Path Planning for Bipedal Navigation (Nav2) *(Proceed with subsections 3.1, 3.2, 3.3, 3.4)* #### Chapter 4: Sim-to-Edge Deployment *(Proceed with subsections 4.1, 4.2, 4.3, culminating in the 4.4 Capstone Project)* **Tone:** Be authoritative yet approachable. Write for students who are now comfortable with ROS 2 and simulation basics but are new to high-performance perception pipelines and edge deployment."
labels: ["specification", "ai-robotics", "nvidia-isaac", "module-3"]
links:
  spec: specs/001-module-3-ai-robot-brain/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-module-3-ai-robot-brain/spec.md
  - specs/001-module-3-ai-robot-brain/checklists/requirements.md
tests:
  - null
---

## Prompt

You are an expert instructor in AI-powered robotics, specializing in NVIDIA's Isaac platform, ROS 2, and edge AI deployment. Your task is to write the complete, detailed instructional content for **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**, broken down into its four chapters as specified. For **each chapter and its subsections (1.1, 1.2, etc.)**, please provide: 1. **Detailed Step-by-Step Instructions:** Assume the student has completed Modules 1 & 2. They have a powerful Ubuntu 22.04 workstation with Docker and a Jetson Orin Nano/NX kit. Provide exact commands, configuration file snippets, and code. * Include specific Isaac Sim Docker run commands and navigation of its UI. * Provide complete `launch.py` or `.py` scripts for Isaac Sim's Replicator. * Give exact `ros2 launch` commands and YAML configuration snippets for Isaac ROS and Nav2. * Include sample Behavior Tree XML and Python nodes for task sequencing. 2. **Clear Explanations of Advanced Concepts:** Demystify complex topics. * Explain how Visual SLAM differs from wheel odometry and its importance for humanoids. * Describe the process of converting a PyTorch/TensorFlow model to a TensorRT engine and the performance benefits. * Clarify the role of each Nav2 server (Controller, Planner, Behavior Tree) in the context of bipedal movement. 3. **Visual Aids & Diagrams:** Describe the key diagrams needed (e.g., "Include a system architecture diagram showing the flow from Isaac Sim synthetic data -> model training -> Isaac ROS inference -> Nav2 planning"). 4. **Troubleshooting & Debugging Guide:** List common pitfalls for each chapter. * Chapter 1: Isaac Sim fails to launch or has rendering issues. * Chapter 2: VSLAM loses tracking or produces a drifting map; DNN inference is too slow on Jetson. * Chapter 3: Nav2 fails to find a valid path or the robot gets stuck in local costmap. * Chapter 4: The system works in sim but fails on the edge due to camera calibration or performance issues. 5. **Chapter Project/Checkpoint:** Define a clear, verifiable outcome for each chapter that students must demonstrate (e.g., for Chapter 2: "Show a screenshot of RViz displaying the live Visual SLAM map and the robot's estimated path as you move the camera"). **Structure your output exactly as follows:** ### Module 3: The AI-Robot Brain (NVIDIA Isaac™) #### Chapter 1: Photorealistic Simulation & Synthetic Data (Isaac Sim) *(Proceed with subsections 1.1, 1.2, 1.3, 1.4 as per the outline above)* #### Chapter 2: Hardware-Accelerated Perception (Isaac ROS) *(Proceed with subsections 2.1, 2.2, 2.3)* #### Chapter 3: Path Planning for Bipedal Navigation (Nav2) *(Proceed with subsections 3.1, 3.2, 3.3, 3.4)* #### Chapter 4: Sim-to-Edge Deployment *(Proceed with subsections 4.1, 4.2, 4.3, culminating in the 4.4 Capstone Project)* **Tone:** Be authoritative yet approachable. Write for students who are now comfortable with ROS 2 and simulation basics but are new to high-performance perception pipelines and edge deployment.

## Response snapshot

Successfully created the feature specification for 'Module 3: The AI-Robot Brain (NVIDIA Isaac™) Content' and generated a quality checklist. The specification is ready for the next phase.

## Outcome

- ✅ Impact: Created a comprehensive feature specification document.
- 🧪 Tests: Manual validation against checklist passed.
- 📁 Files: Created `specs/001-module-3-ai-robot-brain/spec.md` and `specs/001-module-3-ai-robot-brain/checklists/requirements.md`.
- 🔁 Next prompts: User can proceed with `/sp.plan` or `/sp.clarify` if needed.
- 🧠 Reflection: Successfully parsed a complex content creation request into a structured specification.
- 📋 Architectural decision detected: Generated a detailed specification for module 3, including content structure, requirements, and evaluation criteria. Document? Run `/sp.adr Module-3-Content-Spec`

## Evaluation notes (flywheel)

- Failure modes observed: `create-new-feature.ps1` script encountered a `ParameterBindingException` error when trying to parse the full feature description string, despite the output indicating a successful branch creation. This did not prevent the creation of the branch or the initial spec.md file.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
