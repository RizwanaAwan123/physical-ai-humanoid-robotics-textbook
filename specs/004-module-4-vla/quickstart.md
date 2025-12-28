# Module 4 Quickstart: Vision-Language-Action (VLA)

## Overview

Module 4 teaches you to build cutting-edge humanoid robots that understand natural language voice commands and autonomously execute complex tasks through LLM-powered cognitive planning.

**What You'll Build**: Voice-controlled humanoid robot that can understand commands like "Bring me the red cup from the kitchen" and autonomously plan and execute multi-step actions (navigate, detect objects, grasp, deliver).

## Prerequisites Checklist

- ✅ Completed Module 1 (ROS 2 fundamentals)
- ✅ Completed Module 3 Chapters 1, 2, 4 (Isaac Sim, Isaac ROS perception, Nav2)
- ✅ NVIDIA RTX GPU (2060+ or cloud instance)
- ✅ Ubuntu 22.04 + ROS 2 Humble
- ✅ OpenAI API key OR local LLM setup (Ollama)

## Quick Installation (15 minutes)

### 1. Install OpenAI Whisper

```bash
# Create Python virtual environment
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate

# Install Whisper and dependencies
pip install openai-whisper torch torchaudio
pip install sounddevice pyaudio webrtcvad librosa

# Test installation
whisper --help
```

### 2. Install ROS 2 Audio Packages

```bash
# Install audio_common for ROS 2
sudo apt update
sudo apt install ros-humble-audio-common ros-humble-audio-capture

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 3. Set Up LLM Access

**Option A: OpenAI API (Recommended for beginners)**

```bash
# Get API key from https://platform.openai.com/api-keys
export OPENAI_API_KEY="sk-your-key-here"

# Test API access
pip install openai
python3 -c "from openai import OpenAI; client = OpenAI(); print('API OK')"
```

**Option B: Local LLM (Free, offline)**

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Download LLaMA 3.1 8B model
ollama pull llama3.1:8b

# Test local LLM
ollama run llama3.1:8b "Hello, robot!"
```

### 4. Verify Isaac Sim Setup (from Module 3)

```bash
# Check Isaac Sim installation
ls ~/.local/share/ov/pkg/isaac-sim-*

# Launch Isaac Sim (should open without errors)
~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh
```

## First VLA Demo (30 minutes)

### Chapter 1: Voice Command Test

```bash
# Clone VLA examples repository
cd ~/ros2_ws/src
git clone https://github.com/your-course/vla_examples.git

# Build ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select vla_whisper_node
source install/setup.bash

# Run Whisper transcription node
ros2 run vla_whisper_node whisper_transcriber

# In another terminal, speak commands and see transcriptions
ros2 topic echo /voice_command
```

**Expected Output**: Real-time transcriptions of your speech appearing on `/voice_command` topic

### Chapter 2: LLM Task Planning Test

```bash
# Build LLM planner node
colcon build --packages-select vla_llm_planner
source install/setup.bash

# Run LLM planner (uses OpenAI API or Ollama)
ros2 run vla_llm_planner task_planner

# Send test command
ros2 topic pub /voice_command std_msgs/String "data: 'Navigate to the kitchen and pick up the red cup'"

# View planned actions
ros2 topic echo /planned_actions
```

**Expected Output**: JSON action sequence like:
```json
[
  {"action": "navigate_to", "params": {"location": "kitchen"}},
  {"action": "detect_objects", "params": {"color": "red", "type": "cup"}},
  {"action": "grasp_object", "params": {"object_id": "cup_0"}}
]
```

### Chapter 3: Full VLA Integration in Isaac Sim

```bash
# Launch Isaac Sim with humanoid robot
~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh &

# In Isaac Sim, load example scene:
# File → Open → vla_examples/isaac_scenes/kitchen_scene.usd

# Launch full VLA stack
ros2 launch vla_capstone full_vla_demo.launch.py

# Issue voice command (or type in terminal)
# "Pick up the red mug and bring it to me"
```

**Expected Output**: Humanoid robot in Isaac Sim autonomously navigates to mug, grasps it, and returns to user location

## Learning Path

### Week 1: Chapter 1 - Voice-to-Action Pipeline (8-12 hours)

- **Theory**: Speech recognition, Whisper architecture, multilingual support
- **Hands-On**: Integrate Whisper with ROS 2, implement wake word detection
- **Exercises**: 5 exercises (beginner → advanced)
- **Deliverable**: Working voice command system with >90% accuracy

### Week 2: Chapter 2 - Cognitive Planning with LLMs (10-14 hours)

- **Theory**: LLM fundamentals, prompt engineering, structured outputs
- **Hands-On**: Integrate GPT-4 or LLaMA, design prompts, execute ROS 2 actions
- **Exercises**: 6 exercises (API calls → action execution → replanning)
- **Deliverable**: LLM planner generating valid action sequences (>80% accuracy)

### Week 3-4: Chapter 3 - Capstone Project (20-30 hours)

- **Project Selection**: Choose from 6 templates (household, warehouse, search-rescue, etc.)
- **Integration**: Combine all modules (voice + LLM + perception + navigation + manipulation)
- **Testing**: 10+ test scenarios, measure success rates
- **Deliverable**: Video demo, technical report, code repository

## Common Issues & Quick Fixes

### Whisper GPU Out of Memory

```bash
# Use smaller model
whisper audio.wav --model tiny  # Instead of large

# Or run on CPU
whisper audio.wav --device cpu
```

### OpenAI API Rate Limit

```python
# Add retry logic with exponential backoff
from openai import OpenAI
import time

client = OpenAI()
for attempt in range(3):
    try:
        response = client.chat.completions.create(...)
        break
    except Exception as e:
        time.sleep(2 ** attempt)
```

### LLM Generates Invalid Actions

```python
# Add validation layer
valid_actions = ["navigate_to", "grasp_object", "detect_objects"]
if action_name not in valid_actions:
    print(f"Invalid action: {action_name}. Replanning...")
```

## Resources

- **Course Slack**: `#module-4-vla` channel for questions
- **Code Repository**: https://github.com/your-course/vla_examples
- **Office Hours**: Tuesdays 3-5pm (zoom link in syllabus)
- **Troubleshooting Guide**: `/docs/troubleshooting/module-4.md`

## Next Steps

1. ✅ Complete quickstart setup (above)
2. 📖 Read Chapter 1: Voice-to-Action Pipeline
3. 💻 Complete Chapter 1 exercises
4. 📖 Read Chapter 2: Cognitive Planning
5. 💻 Complete Chapter 2 exercises
6. 🏆 Start Capstone Project (Chapter 3)

**Estimated Total Time**: 38-56 hours (spread over 3-4 weeks)

---

**Questions?** Post in `#module-4-vla` Slack channel or attend office hours!
