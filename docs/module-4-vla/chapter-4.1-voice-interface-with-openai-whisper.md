# Chapter 4.1: Voice as the Primary Interface (Voice-to-Action)

Welcome to the first chapter of Module 4, where we delve into establishing a robust voice interface for your humanoid robot. This chapter focuses on integrating `faster-whisper` for efficient speech-to-text conversion and publishing these textual commands onto ROS 2 topics, laying the foundation for natural language interaction.

## 4.1.1 Setting Up Your Voice Input Node

In this section, you will create a ROS 2 Python node, `voice_input_node`, responsible for capturing audio from your microphone, processing it with `faster-whisper`, and publishing the transcribed text.

### Prerequisites

Ensure you have:
-   A Jetson Orin device or a powerful workstation (as set up in previous modules).
-   ROS 2 (Humble Hawksbill or later) installed and sourced.
-   A working microphone connected to your system.
-   Python 3.8+ installed.

### Step-by-Step Instructions

1.  **Create a ROS 2 Package**:
    Navigate to your ROS 2 workspace `src` directory and create a new Python package for your voice node:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_vla_voice
    cd my_vla_voice
    ```

2.  **Install `faster-whisper`**:
    Activate your ROS 2 Python environment and install `faster-whisper`. It's recommended to install in your workspace's virtual environment if you're using one, or globally if appropriate for your setup.
    ```bash
    # If using a virtual environment (recommended)
    source /opt/ros/humble/setup.bash
    python3 -m venv venv
    source venv/bin/activate
    pip install faster-whisper pyaudio # pyaudio for audio capture
    
    # If installing globally (use with caution)
    # pip install faster-whisper pyaudio
    ```
    *Note*: You might need to install `portaudio` development headers if `pyaudio` compilation fails: `sudo apt-get install portaudio19-dev python3-pyaudio`.

3.  **Create the `voice_input_node.py` Script**:
    Inside your `my_vla_voice/my_vla_voice` directory, create a file named `voice_input_node.py` and add the following Python code:

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import pyaudio
    import numpy as np
    from faster_whisper import WhisperModel
    import collections
    import threading
    import time

    # Configuration for audio capture
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000  # Sample rate (Hz)
    CHUNK = 1024  # Buffer size
    RECORD_SECONDS_PER_CHUNK = CHUNK / RATE # seconds represented by one CHUNK

    # Whisper model configuration
    WHISPER_MODEL_SIZE = "small" # You can choose from 'tiny', 'base', 'small', 'medium', 'large-v2'
    # On Jetson Orin, 'tiny' or 'base' are recommended for real-time.
    # On a powerful workstation, 'small' or 'medium' might be feasible.
    
    # Audio buffer to store recent audio for transcription
    # Adjust this to control how much "context" Whisper sees
    AUDIO_BUFFER_SECONDS = 5 # Store 5 seconds of audio
    MAX_AUDIO_FRAMES = int(RATE / CHUNK * AUDIO_BUFFER_SECONDS)

    class VoiceInputNode(Node):
        def __init__(self):
            super().__init__('voice_input_node')
            self.publisher_ = self.create_publisher(String, 'voice_command', 10)
            self.get_logger().info('Voice Input Node started. Listening for commands...')

            # Initialize PyAudio
            self.p = pyaudio.PyAudio()
            self.stream = self.p.open(format=FORMAT,
                                      channels=CHANNELS,
                                      rate=RATE,
                                      input=True,
                                      frames_per_buffer=CHUNK)
            
            # Initialize faster-whisper model
            self.get_logger().info(f"Loading Whisper model: {WHISPER_MODEL_SIZE}...")
            self.model = WhisperModel(WHISPER_MODEL_SIZE, device="cpu", compute_type="int8")
            self.get_logger().info("Whisper model loaded.")

            self.audio_buffer = collections.deque(maxlen=MAX_AUDIO_FRAMES)
            self.transcription_lock = threading.Lock()
            self.last_transcription_time = time.time()
            self.transcription_interval = 2 # Transcribe every 2 seconds if new audio is present

            # Start audio recording and transcription threads
            self.audio_thread = threading.Thread(target=self._record_audio)
            self.audio_thread.daemon = True
            self.audio_thread.start()

            self.transcription_thread = threading.Thread(target=self._transcribe_loop)
            self.transcription_thread.daemon = True
            self.transcription_thread.start()

        def _record_audio(self):
            self.get_logger().info("Audio recording thread started.")
            try:
                while rclpy.ok():
                    data = self.stream.read(CHUNK, exception_on_overflow=False)
                    self.audio_buffer.append(np.frombuffer(data, dtype=np.int16))
            except Exception as e:
                self.get_logger().error(f"Error in audio recording: {e}")
            finally:
                self.stream.stop_stream()
                self.stream.close()
                self.p.terminate()
                self.get_logger().info("Audio recording thread stopped.")

        def _transcribe_loop(self):
            self.get_logger().info("Transcription thread started.")
            while rclpy.ok():
                time.sleep(0.1) # Check for new audio every 100ms
                current_time = time.time()
                if current_time - self.last_transcription_time >= self.transcription_interval and len(self.audio_buffer) > 0:
                    self.last_transcription_time = current_time
                    self._transcribe_audio()

        def _transcribe_audio(self):
            with self.transcription_lock:
                if not self.audio_buffer:
                    return

                # Convert buffer to a single numpy array for Whisper
                audio_np = np.concatenate(self.audio_buffer)
                
                # Normalize to float32 as expected by WhisperModel
                audio_float = audio_np.astype(np.float32) / 32768.0

                self.get_logger().info("Transcribing audio...")
                try:
                    segments, info = self.model.transcribe(audio_float, beam_size=5)
                    full_text = ""
                    for segment in segments:
                        full_text += segment.text + " "
                    
                    full_text = full_text.strip()
                    if full_text:
                        msg = String()
                        msg.data = full_text
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published: "{full_text}"')
                    else:
                        self.get_logger().info("No speech detected or transcribed.")
                    
                    # Clear buffer after transcription (or keep some overlap for continuity)
                    # For simplicity, we clear the buffer here after each transcription attempt.
                    # For continuous speech, a more advanced buffering strategy is needed.
                    self.audio_buffer.clear()

                except Exception as e:
                    self.get_logger().error(f"Error during transcription: {e}")

        def destroy_node(self):
            self.get_logger().info("Shutting down Voice Input Node.")
            super().destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        node = VoiceInputNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

4.  **Update `setup.py`**:
    Open `my_vla_voice/setup.py` and ensure the `entry_points` are configured to run your node:
    ```python
    from setuptools import find_packages, setup

    package_name = 'my_vla_voice'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your.email@example.com',
        description='ROS 2 package for voice input using faster-whisper',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'voice_input_node = my_vla_voice.voice_input_node:main',
            ],
        },
    )
    ```
    *Remember to replace `Your Name` and `your.email@example.com`.*

5.  **Build Your ROS 2 Package**:
    Navigate back to your workspace root (`~/ros2_ws`) and build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_vla_voice
    ```

6.  **Source Your Workspace and Run the Node**:
    ```bash
    source install/setup.bash
    ros2 run my_vla_voice voice_input_node
    ```
    You should see the node starting and listening. Speak into your microphone, and you should see the transcribed text published to the `voice_command` topic.

## 4.1.2 Understanding `faster-whisper` and Audio Streaming

### `faster-whisper` Explained

`faster-whisper` is an optimized re-implementation of OpenAI's Whisper model, providing significantly faster inference speed with comparable accuracy. It leverages CTranslate2 to run the models more efficiently on various hardware, including CPUs (which is useful for devices like Jetson Orin that might not have high-end GPUs or for quick prototyping on a workstation).

**Key Concepts**:
-   **Quantization**: `faster-whisper` supports different computation types (`float32`, `float16`, `int8`). `int8` quantization (used in our example `compute_type="int8"`) reduces model size and speeds up inference by using 8-bit integers instead of 32-bit floating-point numbers, with minimal impact on accuracy. This is crucial for edge devices.
-   **Beam Search**: A search algorithm used during decoding to find the most probable sequence of words. `beam_size=5` means the model keeps track of the 5 most likely sequences at each step, improving accuracy over a greedy approach.
-   **Model Size**: Whisper models come in various sizes (`tiny`, `base`, `small`, `medium`, `large-v2`). Smaller models are faster but less accurate; larger models are more accurate but slower. The choice depends on your hardware and latency requirements.

### Audio Stream Handling with `PyAudio`

`PyAudio` provides Python bindings for PortAudio, a cross-platform audio I/O library. It allows us to easily capture real-time audio from microphones.

**Key Concepts**:
-   **`pyaudio.paInt16`**: Specifies the audio format as 16-bit signed integers. This is a common format for microphone input.
-   **`RATE` (Sample Rate)**: The number of audio samples taken per second (e.g., 16000 Hz for telephony audio, 44100 Hz for CD quality). Whisper models are often trained on 16kHz audio.
-   **`CHUNK` (Buffer Size)**: The number of frames (samples) read at a time from the audio input buffer. A smaller chunk size means lower latency but more frequent processing; a larger chunk size means higher latency but less frequent processing overhead.
-   **Audio Buffer (`collections.deque`)**: To ensure `faster-whisper` has enough context to accurately transcribe spoken words (especially sentences), we don't transcribe every tiny chunk individually. Instead, we store a recent history of audio chunks in a `deque` (double-ended queue) which efficiently manages a fixed-size buffer. When we're ready to transcribe, we concatenate these recent chunks.

### ROS 2 Topic Publishing

The transcribed text is published on a standard ROS 2 topic, `'voice_command'`, with message type `std_msgs.msg.String`. This allows other ROS 2 nodes in the VLA pipeline (e.g., the planning node) to subscribe to these commands and react accordingly. This adheres to the ROS 2 decoupled architecture principles, promoting modularity and reusability.

## 4.1.3 System Architecture Overview: Voice Input

This section illustrates the data flow for the voice input component within the broader VLA system.

```mermaid
graph LR
    A[Microphone] --> B{PyAudio - Audio Capture};
    B --> C[Audio Buffer (deque)];
    C -- Periodically --> D{WhisperModel - Speech-to-Text (faster-whisper)};
    D --> E[Transcribed Text (String)];
    E --> F[ROS 2 `voice_input_node`];
    F -- Publishes to --> G(ROS 2 Topic: `/voice_command` [std_msgs/String]);
    G --> H[Next Node in VLA Pipeline (e.g., Planner Node)];
```

*Figure 4.1.1: Data Flow for Voice Input Node*

**Explanation**:
1.  **Microphone**: Captures raw analog sound waves.
2.  **PyAudio - Audio Capture**: Digitizes the analog audio and provides it in small `CHUNK`s to the Python script.
3.  **Audio Buffer**: A `deque` temporarily stores a configurable duration of recent audio chunks, providing context for transcription.
4.  **WhisperModel - Speech-to-Text**: The `faster-whisper` model processes the buffered audio (converted to a `numpy` array of `float32`) to produce text.
5.  **Transcribed Text**: The output of the speech-to-text process, a string containing the recognized speech.
6.  **ROS 2 `voice_input_node`**: The Python node encapsulates the audio capture and transcription logic.
7.  **ROS 2 Topic: `/voice_command`**: The `voice_input_node` publishes the transcribed text to this topic, making it available to any subscribing node in the ROS 2 graph.
8.  **Next Node in VLA Pipeline**: Represents the subsequent processing stage, such as a language understanding or planning node, which will consume these voice commands.

## 4.1.4 Troubleshooting & Optimization Guide

### Whisper Mishears Commands in Noisy Environments

-   **Problem**: `faster-whisper` struggles to accurately transcribe speech when there's significant background noise, leading to incorrect robot commands.
-   **Solutions**:
    -   **Noise Reduction Hardware**: Use directional microphones or microphones with built-in noise cancellation.
    -   **Software Pre-processing**: Implement audio pre-processing techniques (e.g., spectral subtraction, noise gating) before feeding audio to Whisper. Libraries like `webrtcvad` (Voice Activity Detection) can also help filter out non-speech segments.
    -   **Environmental Control**: Operate the robot in quieter environments where possible.
    -   **Model Fine-tuning**: For highly specific noise profiles or command sets, consider fine-tuning a smaller Whisper model on domain-specific noisy speech data.
    -   **Adjust `WHISPER_MODEL_SIZE`**: A larger model *might* be more robust to noise, but at the cost of latency. Test different sizes.
    -   **Confirmation**: Implement a verbal confirmation mechanism where the robot repeats the understood command and waits for confirmation ("Did you say 'move forward'? Say 'yes' or 'no'").

### Audio Stream Latency

-   **Problem**: Delay between speaking a command and its transcription appearing on the ROS 2 topic. This can make the robot feel unresponsive.
-   **Solutions**:
    -   **Optimize `CHUNK` Size**: Smaller `CHUNK` sizes reduce audio capture latency but increase CPU overhead. Find a balance that suits your hardware.
    -   **Reduce `AUDIO_BUFFER_SECONDS`**: A smaller audio buffer means Whisper processes less historical audio, reducing transcription latency. However, too small a buffer might lead to less accurate transcriptions.
    -   **Choose Smaller Whisper Model**: The `WHISPER_MODEL_SIZE` significantly impacts inference time. `tiny` or `base` models are much faster than `medium` or `large-v2`.
    -   **Leverage GPU/Edge AI Accelerators**: If available (e.g., on Jetson Orin), configure `faster-whisper` to use the GPU (`device="cuda"`) for much faster inference. Ensure necessary drivers and CUDA toolkits are installed. `compute_type` can also be set to `float16` for further GPU optimization.
    -   **Dedicated Processing**: Run the `voice_input_node` on a dedicated core or a separate device if possible to minimize interference from other robot processes.
    -   **Asynchronous Processing**: The current node uses threading for audio capture and transcription, which helps, but ensure the transcription logic itself is optimized.

## 4.1.5 Chapter Project/Checkpoint: Voice Command Receiver

**Objective**: Demonstrate a functional `voice_input_node` that transcribes speech in real-time and publishes it to a ROS 2 topic.

**Deliverables**:
1.  A running `voice_input_node` on your workstation or Jetson Orin.
2.  A terminal displaying the `voice_input_node` logs, showing the phrases you speak being published to the `/voice_command` topic.

**Demonstration Steps**:
1.  Launch the `voice_input_node` as described in Section 4.1.1.
2.  Open a new terminal and listen to the `/voice_command` topic:
    ```bash
    source install/setup.bash
    ros2 topic echo /voice_command
    ```
3.  Speak several commands into your microphone (e.g., "Hello robot", "Move forward", "Stop").
4.  Verify that the `ros2 topic echo` terminal displays your spoken commands accurately.

*(End of Chapter 4.1)*