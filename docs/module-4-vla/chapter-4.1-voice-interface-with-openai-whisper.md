---
id: chapter-4-1-voice-interface-with-openai-whisper
title: Voice Interface with OpenAI Whisper
sidebar_position: 1
---

# Chapter 4.1: Voice Interface with OpenAI Whisper

# Chapter 4.1: Voice Interface with OpenAI Whisper

## Focus: Speech-to-text integration, command parsing, ROS 2 audio topics
## Learning objectives: Implement voice command system

Human-robot interaction (HRI) is greatly enhanced when robots can understand and respond to natural human communication. Voice commands offer an intuitive and hands-free way for humans to interact with robots, making complex systems more accessible. This chapter explores how to integrate voice interfaces into robotic systems, leveraging the advanced capabilities of **OpenAI Whisper** for robust speech-to-text transcription, and how to translate these transcriptions into actionable robotic commands.

### 1. The Need for Voice Interfaces in Robotics

For humanoid robots designed to operate alongside humans, voice interfaces are a game-changer:
*   **Natural Interaction:** Mimics human-to-human communication, reducing cognitive load for users.
*   **Hands-Free Operation:** Allows users to interact with robots while performing other tasks.
*   **Accessibility:** Provides an alternative interaction method for users with mobility impairments.
*   **Intuitive Control:** Simplifies complex control sequences into natural language phrases.

### 2. Introduction to OpenAI Whisper

OpenAI Whisper is a general-purpose, open-source Automatic Speech Recognition (ASR) model. Trained on a massive and diverse dataset of audio and text, Whisper excels at:
*   **High Accuracy:** State-of-the-art performance in transcribing audio into text.
*   **Multilingual Support:** Can transcribe and translate speech in many languages.
*   **Robustness:** Performs well even with background noise, varying accents, and different speaking styles.
*   **Speaker Diarization (limited):** Can sometimes distinguish between different speakers (though not its primary focus).

Its robust performance makes it an ideal candidate for converting spoken robot commands into text that can then be processed by the robot's cognitive systems.

### 3. Integrating OpenAI Whisper: From Audio to Text

Integrating Whisper typically involves two main approaches: using the local model or accessing it via an API.

**A. Local Model Integration:**
For local deployment (e.g., on a Jetson board or a high-performance workstation):
1.  **Installation:** Install the `whisper` Python package (or a more optimized variant like `whisper-timestamped` or `faster-whisper`).
2.  **Audio Capture:** Use a microphone to capture audio input. Libraries like `PyAudio` or `Sounddevice` can record audio streams.
3.  **Transcription:** Pass the captured audio segment to the Whisper model for transcription.
    ```python
    import whisper
    import numpy as np
    import sounddevice as sd

    # Load a Whisper model (e.g., "base", "small", "medium", "large")
    model = whisper.load_model("base")

    # Define audio recording parameters
    samplerate = 16000  # Whisper expects 16kHz audio
    duration = 5       # Record for 5 seconds

    def record_audio(duration, samplerate):
        print("Recording...")
        audio_data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='float32')
        sd.wait() # Wait until recording is finished
        print("Recording finished.")
        return audio_data.flatten()

    def transcribe_audio(audio_segment):
        # Make sure the audio is in the correct format (16kHz, mono, float32)
        # Whisper automatically handles resamplinbg if needed, but it's good practice.
        result = model.transcribe(audio_segment)
        return result["text"]

    if __name__ == "__main__":
        audio_segment = record_audio(duration, samplerate)
        text = transcribe_audio(audio_segment)
        print(f"Transcribed: {text}")
    ```

**B. API Integration:**
For cloud-based Whisper (e.g., OpenAI API):
1.  **Audio Capture:** Capture audio locally as above.
2.  **API Call:** Send the audio file or stream to the OpenAI Whisper API.
    ```python
    import openai
    import sounddevice as sd
    import wavio

    # Set your OpenAI API key
    openai.api_key = "YOUR_OPENAI_API_KEY"

    # Define audio recording parameters
    samplerate = 16000
    duration = 5
    filename = "input_audio.wav"

    def record_and_save_audio(duration, samplerate, filename):
        print("Recording...")
        audio_data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
        sd.wait()
        wavio.write(filename, audio_data, samplerate, sampwidth=2)
        print(f"Recording finished and saved to {filename}")

    def transcribe_audio_api(filename):
        with open(filename, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)
        return transcript["text"]

    if __name__ == "__main__":
        record_and_save_audio(duration, samplerate, filename)
        text = transcribe_audio_api(filename)
        print(f"Transcribed (API): {text}")
    ```

### 4. Speech-to-Text to Action: Command Parsing

Once speech is transcribed into text, the next step is to interpret this text as a robotic command. This involves:

*   **Keyword Spotting:** Simple approach where the system looks for predefined keywords or phrases (e.g., "stop", "go forward", "pick up").
*   **Rule-Based Parsing:** Using grammatical rules and templates to extract commands and their parameters (e.g., "move [distance] meters [direction]").
*   **Natural Language Understanding (NLU):** More advanced methods using machine learning (often LLMs) to understand the intent and entities within a natural language command, even if phrased differently.

**Example (Simple Keyword/Rule-Based):**
```python
def parse_command(text):
    text = text.lower()
    if "stop" in text:
        return {"command": "stop_robot"}
    elif "move forward" in text:
        distance = 1.0 # Default
        if "meters" in text:
            try:
                # Basic extraction, would need more robust parsing
                parts = text.split("move forward")[1].split("meters")[0].strip().split()
                if parts and parts[-1].isdigit():
                    distance = float(parts[-1])
            except (ValueError, IndexError):
                pass
        return {"command": "move_linear", "distance": distance, "direction": "forward"}
    elif "pick up the box" in text:
        return {"command": "pick_object", "object": "box"}
    return {"command": "unknown"}

# Usage
command_text = "robot, please move forward 2 meters"
action = parse_command(command_text)
print(action) # Output: {'command': 'move_linear', 'distance': 2.0, 'direction': 'forward'}
```

### 5. ROS 2 Audio Handling and Integration

For integration with ROS 2, audio processing can be handled by a dedicated ROS 2 node.

*   **Audio Input Node:** A ROS 2 node can be responsible for capturing audio from a microphone and publishing it as a raw audio message (e.g., `audio_common_msgs/msg/AudioData`) on a specific topic.
*   **Whisper Transcription Node:** Another ROS 2 node subscribes to the audio topic, performs the Whisper transcription (either locally or via API), and publishes the resulting text as a `std_msgs/msg/String` on a "transcribed_text" topic.
*   **Command Parser Node:** A third ROS 2 node subscribes to the "transcribed_text" topic, parses the text into a structured command, and publishes it to a command topic (e.g., `geometry_msgs/msg/Twist` for movement, or a custom action goal).

This modular approach ensures that each component can be developed, tested, and maintained independently within the ROS 2 ecosystem.

### 6. Hardware Considerations

*   **Microphone Quality:** A good quality microphone with noise cancellation is essential for accurate ASR, especially in noisy robotic environments. Far-field microphones are often preferred for robots.
*   **Audio Processing:** Techniques like echo cancellation, noise reduction, and beamforming can significantly improve Whisper's performance.
*   **Computational Resources:** Running larger Whisper models locally (e.g., "large") requires substantial computational power (CPU and/or GPU), which can be a limiting factor on edge devices like NVIDIA Jetson. Smaller models or API-based solutions might be necessary.

By effectively implementing a voice interface with OpenAI Whisper, you empower your humanoid robots with a more natural and powerful mode of interaction, bringing them closer to truly intelligent and helpful companions.
