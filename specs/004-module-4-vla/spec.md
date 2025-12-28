# Feature Specification: Module 4 - Vision-Language-Action (VLA)

**Feature Branch**: `004-module-4-vla`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "The Vision-Language-Action (VLA) Module covers the cutting edge of robotics by integrating Large Language Models (LLMs) with the robot's control stack. The module focuses on converting human intent (voice/text) into executable robot actions through cognitive planning. The culmination of this module is the Capstone Project, demonstrating a fully autonomous humanoid robot."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Voice-to-Action Pipeline with OpenAI Whisper (Priority: P1)

As a robotics developer or HRI (Human-Robot Interaction) researcher, I want to learn how to implement voice command interfaces using OpenAI Whisper for speech-to-text so that humanoid robots can understand natural spoken language commands and convert them into actionable intents for robot control.

**Why this priority**: Voice interaction is the fundamental user interface for advanced humanoid robots. Without reliable speech recognition, users cannot naturally communicate with robots. OpenAI Whisper provides state-of-the-art multilingual speech recognition with high accuracy and robustness to noise. This forms the essential input layer for the VLA pipeline and is prerequisite for all cognitive planning and action execution tasks.

**Independent Test**: Can be fully tested by having a student complete Chapter 1, set up OpenAI Whisper (local or API-based), integrate Whisper with ROS 2 audio input nodes, implement real-time speech recognition from microphone or audio topics, demonstrate transcription accuracy across multiple languages and accents, handle audio preprocessing (noise reduction, VAD - Voice Activity Detection), and publish transcribed text to ROS 2 topics for downstream processing by LLM-based cognitive planners.

**Acceptance Scenarios**:

1. **Given** no prior voice interface experience, **When** a student completes Chapter 1 learning objectives, **Then** they can explain speech recognition fundamentals (acoustic models, language models, end-to-end neural approaches), understand Whisper architecture (encoder-decoder Transformer, multilingual training, prompting techniques), describe advantages over traditional ASR systems (CMU Sphinx, Kaldi), and relate Whisper to robotics HRI requirements (low latency, robustness, semantic understanding)

2. **Given** Whisper installation tutorials, **When** a student sets up the speech recognition environment, **Then** they can install OpenAI Whisper via pip (whisper-openai) or use cloud-based OpenAI API, configure Whisper model sizes (tiny, base, small, medium, large) based on accuracy/latency trade-offs, set up Python environments with PyTorch and audio processing libraries (sounddevice, pyaudio, wave, librosa), and verify GPU acceleration for faster-than-real-time transcription on NVIDIA GPUs

3. **Given** audio input integration examples, **When** a student captures audio for Whisper processing, **Then** they can configure ROS 2 audio_common nodes for microphone input (USB microphones, array microphones), implement audio stream processing with VAD to detect speech segments, convert ROS 2 audio messages to Whisper-compatible formats (16kHz mono WAV), and handle audio buffering and chunking for real-time streaming transcription

4. **Given** Whisper transcription tutorials, **When** a student implements speech-to-text, **Then** they can run Whisper inference on audio chunks, configure language hints and prompting for domain-specific vocabulary (robotics commands like "navigate to kitchen", "pick up the red cube"), implement confidence scoring and filtering for low-quality transcriptions, and publish transcribed text to ROS 2 String topics for downstream cognitive planning

5. **Given** multilingual and accent robustness examples, **When** a student tests Whisper across diverse inputs, **Then** they can demonstrate transcription accuracy >90% for clear speech in English, test multilingual support (Spanish, Chinese, etc.) for international deployments, evaluate robustness to background noise (music, crowd chatter, industrial sounds), and handle non-native accents and speech variations

6. **Given** intent extraction from transcriptions, **When** a student prepares Whisper output for LLM processing, **Then** they can implement post-processing to normalize text (capitalization, punctuation), extract keywords and entities (locations, object names, action verbs), map common phrases to robot command templates ("go to X" → navigation intent), and format prompts for LLM cognitive planning (Chapter 2 integration preview)

7. **Given** wake word and activation patterns, **When** a student implements human-robot dialogue flow, **Then** they can integrate wake word detection models (Porcupine, Snowboy, or custom CNN models) for "Hey Robot" activation, implement state machines for multi-turn dialogue (waiting for command, processing, confirming intent, executing), and provide audio feedback (beeps, synthesized speech via TTS engines like gTTS or Coqui TTS) to acknowledge commands

8. **Given** performance optimization and latency reduction, **When** a student tunes the voice pipeline, **Then** they can measure end-to-end latency from speech start to text output (<1 second target for responsive HRI), optimize Whisper model selection (tiny model for <300ms latency on GPU vs large model for maximum accuracy), implement parallel processing with threaded or async audio capture and inference, and profile CPU/GPU usage for resource-constrained robots (Jetson platforms)

9. **Given** integration with Isaac Sim humanoid robots, **When** a student demonstrates voice control in simulation, **Then** they can connect Whisper ROS 2 nodes to Isaac Sim humanoid models, issue voice commands for basic actions (wave hand, walk forward, turn around), visualize transcription outputs in RViz or terminal logs, and validate end-to-end voice-to-motion latency in simulation

10. **Given** edge cases and error handling, **When** a student encounters speech recognition failures, **Then** they can handle silence or unintelligible audio (timeout mechanisms, "I didn't understand" feedback), manage overlapping speech or interruptions (cancel current command, priority handling), and implement fallback mechanisms (gesture input, touchscreen GUI, predefined command buttons)

---

### User Story 2 - Cognitive Planning with LLMs for Action Translation (Priority: P2)

As a robotics AI engineer or autonomy researcher, I want to learn how to use Large Language Models (LLMs like GPT-4, Claude, LLaMA) to translate natural language commands into structured ROS 2 action sequences so that humanoid robots can understand complex task instructions and autonomously plan execution steps.

**Why this priority**: LLMs provide the "cognitive brain" that interprets human intent from natural language (from Whisper Chapter 1) and translates it into executable robot actions. This is the core innovation of Vision-Language-Action systems—bridging the semantic gap between human language and robot control primitives. Without cognitive planning, robots are limited to predefined command templates. LLMs enable open-ended task understanding, multi-step planning, and contextual reasoning.

**Independent Test**: Can be fully tested by having a student complete Chapter 2, set up LLM integration (OpenAI API, Anthropic API, or local LLaMA/Mistral models), design prompt templates for robot task planning, implement LLM-to-ROS 2 action translators (text output → ROS 2 action goals), demonstrate planning for multi-step tasks (navigation + manipulation sequences), validate plans in Isaac Sim, and measure success rates for intent-to-action translation accuracy (>80% correct action sequences for common commands).

**Acceptance Scenarios**:

1. **Given** LLM fundamentals for robotics covered in the chapter, **When** a student studies the theory, **Then** they can explain LLM capabilities (natural language understanding, reasoning, code generation), understand Transformer architecture basics (self-attention, pre-training, fine-tuning), describe LLM prompting strategies (zero-shot, few-shot, chain-of-thought), and relate LLMs to robot task planning (action decomposition, constraint satisfaction, world model reasoning)

2. **Given** LLM API integration tutorials, **When** a student sets up LLM access, **Then** they can configure OpenAI API (GPT-4, GPT-4o) or Anthropic API (Claude 3.5 Sonnet, Claude Opus) with authentication keys, implement local LLM deployment using Ollama or vLLM with LLaMA 3.1, Mistral 7B, or Phi-3 models for offline operation, set up ROS 2 service nodes for LLM query/response interfaces, and implement retry logic and rate limiting for API calls

3. **Given** robot capability documentation, **When** a student defines the robot's action space for LLMs, **Then** they can create structured descriptions of available ROS 2 actions (navigation actions: /navigate_to_pose, manipulation actions: /grasp_object, /place_object, perception queries: /detect_objects, motion primitives: /move_joint, /execute_trajectory), define action parameters (positions, orientations, object IDs, speeds), and document preconditions and postconditions for each action (e.g., grasp requires object detection first)

4. **Given** prompt engineering tutorials, **When** a student designs LLM prompts for task planning, **Then** they can create system prompts defining the robot's role and capabilities ("You are a humanoid robot assistant with navigation and manipulation skills..."), implement few-shot examples demonstrating natural language → action sequence mappings (e.g., "Bring me the red cup" → [detect_objects, navigate_to(cup_location), grasp_object(cup_id), navigate_to(user_location), hand_over]), use chain-of-thought prompting for complex multi-step reasoning, and constrain outputs to structured formats (JSON action lists, Python code snippets calling ROS 2 APIs)

5. **Given** structured output parsing examples, **When** a student extracts action plans from LLM responses, **Then** they can parse JSON-formatted action sequences from LLM text outputs, implement schema validation (ensuring action names and parameters are valid), handle LLM output errors or ambiguities (invalid actions, missing parameters, conflicting goals), and convert parsed actions into ROS 2 action goals (actionlib or rclpy action clients)

6. **Given** contextual reasoning and memory integration, **When** a student enhances LLM planning with state awareness, **Then** they can provide LLMs with robot state context (current location, detected objects, joint configurations) in prompts, implement dialogue history tracking for multi-turn commands ("Pick up the cup" → "Now place it on the table" requires remembering cup is grasped), use vector databases or retrieval-augmented generation (RAG) for long-term memory (past task executions, environment maps, object locations), and enable LLMs to query ROS 2 topics for real-time state information

7. **Given** plan validation and safety checks, **When** a student ensures LLM outputs are safe and feasible, **Then** they can implement plan validators checking action preconditions (can't grasp if object not detected, can't navigate if path blocked), constraint checkers for robot limits (joint limits, reachability, collision avoidance), cost estimation for plan efficiency (prefer shorter paths, fewer actions), and user confirmation prompts for potentially dangerous actions (moving near humans, dropping objects)

8. **Given** execution monitoring and replanning, **When** a student handles dynamic environments, **Then** they can integrate LLM planners with ROS 2 action feedback (monitor action execution status, detect failures like navigation blocked or grasp failed), trigger LLM replanning on failure (query LLM with failure context: "Navigation to kitchen blocked by obstacle, replan"), implement timeout and retry logic for stuck actions, and log execution traces for debugging and learning

9. **Given** multimodal LLM integration (GPT-4 Vision, Claude 3.5 Sonnet vision), **When** a student adds visual grounding to language commands, **Then** they can send robot camera images to vision-language models (VLMs) alongside text commands, enable object referring ("Pick up the red mug on the left") with visual confirmation, implement scene understanding queries ("What objects are on the table?"), and use VLM outputs to refine action parameters (object 3D poses from visual detection)

10. **Given** integration with Isaac Sim humanoid robots, **When** a student demonstrates cognitive planning in simulation, **Then** they can connect LLM ROS 2 nodes to Isaac Sim environments, issue natural language commands via Whisper (from Chapter 1) or text input, have LLM generate navigation and manipulation action sequences, execute actions in Isaac Sim using Nav2 (from Module 3 Chapter 4) and manipulation controllers, and validate task completion success rates (>80% for common household tasks like "Fetch the book from the shelf")

11. **Given** fine-tuning and domain adaptation, **When** a student customizes LLMs for robotics, **Then** they can collect robotics-specific datasets (command-action pairs, task demonstrations), fine-tune open-source LLMs (LLaMA, Mistral) using LoRA or full fine-tuning on robot task data, evaluate fine-tuned models against base models for robotics accuracy, and document training procedures and hyperparameters

12. **Given** edge cases and failure modes, **When** a student handles LLM limitations, **Then** they can detect and handle hallucinations (LLM proposes non-existent actions or invalid parameters), manage ambiguous commands ("Get me something to drink" requires clarification or context-based guessing), implement graceful degradation (fallback to simpler template-based command parsers if LLM fails), and provide user feedback for unachievable requests ("I cannot climb stairs with my current capabilities")

---

### User Story 3 - Capstone Project: Autonomous Humanoid Robot (Priority: P3)

As a robotics student or engineer completing the Physical AI & Humanoid Robotics course, I want to integrate all learned skills (ROS 2 fundamentals, digital twin simulation, NVIDIA Isaac AI technologies, voice-language-action pipelines) into a comprehensive Capstone Project demonstrating a fully autonomous humanoid robot capable of understanding voice commands, planning complex tasks, and executing them in simulation (Isaac Sim) or optionally on real hardware.

**Why this priority**: The Capstone Project is the culmination of the entire 4-module course, synthesizing voice interfaces (Chapter 1), LLM cognitive planning (Chapter 2), and all prior modules (Module 1: ROS 2, Module 2: Digital Twins, Module 3: Isaac Sim/ROS/RL/Nav2). It validates that students have mastered end-to-end humanoid robotics development—from human input to autonomous task execution—and provides a portfolio-ready demonstration of Physical AI skills. This is the final validation of course learning outcomes.

**Independent Test**: Can be fully tested by having a student complete Chapter 3 (Capstone Project) by selecting a project scenario (household assistance, warehouse logistics, search-and-rescue, or custom), implementing the full VLA pipeline (voice input via Whisper, LLM task planning, ROS 2 action execution), integrating with Isaac Sim humanoid robot (from Module 3) in a realistic environment, demonstrating autonomous task completion (e.g., "Bring me the first-aid kit from the cabinet" → navigate, open cabinet, grasp kit, return to user), measuring performance metrics (task success rate >75%, latency <10 seconds from command to action start, robustness to perturbations), and documenting the system architecture, code, and results in a final report or video demo.

**Acceptance Scenarios**:

1. **Given** Capstone Project overview and requirements, **When** a student plans their project, **Then** they can select from provided project templates (household assistance robot, warehouse picking robot, search-and-rescue robot, interactive museum guide robot, or propose a custom project), define project scope with measurable success criteria (specific tasks, environments, performance metrics), identify required technologies from Modules 1-4, and create a project timeline and milestones (system architecture, component integration, testing, documentation)

2. **Given** system architecture design guidance, **When** a student designs their humanoid system, **Then** they can create architecture diagrams showing voice input (Whisper), cognitive planning (LLM), perception (Isaac ROS VSLAM, object detection from Module 3), navigation (Nav2 from Module 3), manipulation (RL-trained policies or inverse kinematics from Module 3), and ROS 2 communication layers, define interfaces between components (ROS 2 topics, services, actions), and document data flow from user command to robot action execution

3. **Given** Isaac Sim environment setup for Capstone, **When** a student creates a project-specific simulation, **Then** they can design custom Isaac Sim USD scenes matching project requirements (kitchen for household task, warehouse with shelves for logistics, disaster site with obstacles for search-and-rescue), populate environments with interactive objects (doors, cabinets, manipulable items), configure humanoid robot models (NVIDIA H1, Unitree G1, or custom) with appropriate sensors (cameras, LiDAR, IMU), and tune PhysX physics for realistic interactions (object grasping, collisions, friction)

4. **Given** voice command interface implementation (from Chapter 1), **When** a student integrates Whisper, **Then** they can deploy Whisper ROS 2 nodes for real-time speech recognition, implement wake word activation for natural HRI dialogue flow, configure domain-specific prompts for project vocabulary (medical terms for first-aid robot, object names for warehouse items), and test transcription accuracy with expected user commands

5. **Given** LLM cognitive planning implementation (from Chapter 2), **When** a student integrates LLM task planners, **Then** they can configure OpenAI GPT-4 or local LLaMA models with project-specific system prompts, define robot action libraries covering required capabilities (navigation actions, manipulation actions, perception queries), implement structured output parsing for action sequences, and add plan validation and safety checks for project constraints (e.g., warehouse robot cannot leave designated areas)

6. **Given** perception integration (from Module 3 Chapter 2), **When** a student adds vision capabilities, **Then** they can use Isaac ROS object detection to identify manipulable objects (cups, tools, boxes), run VSLAM for localization and mapping in Isaac Sim environments, optionally integrate GPT-4 Vision or Claude Vision for visual question answering ("Is the door open?"), and publish detected object poses to LLM planner for grounding ("the red box on the middle shelf")

7. **Given** navigation integration (from Module 3 Chapter 4), **When** a student implements autonomous movement, **Then** they can use Nav2 configured for humanoid robots to navigate from current location to goal poses specified by LLM planner, implement dynamic obstacle avoidance during task execution, handle navigation failures with replanning (inform LLM of blocked path), and measure navigation success rates and path efficiency

8. **Given** manipulation integration (from Module 3 Chapter 3 or IK solvers), **When** a student implements object interaction, **Then** they can use RL-trained grasping policies (if trained in Module 3) or inverse kinematics controllers for reaching and grasping objects, implement pre-grasp positioning and approach trajectories, handle grasp failure detection and retries, and execute place actions for delivering objects to target locations

9. **Given** end-to-end task execution, **When** a student demonstrates the Capstone system, **Then** they can issue natural language voice commands ("Bring me the wrench from the toolbox"), have Whisper transcribe command to text, have LLM planner decompose into action sequence ([detect_objects, navigate_to(toolbox), open(toolbox), grasp_object(wrench), navigate_to(user), hand_over]), execute actions sequentially in Isaac Sim with real-time monitoring, handle execution failures with LLM replanning, and complete tasks with >75% success rate across 10+ test scenarios

10. **Given** performance evaluation and metrics, **When** a student assesses Capstone performance, **Then** they can measure task success rate (percentage of completed tasks without human intervention), latency metrics (time from command to action start, time from action start to task completion), action efficiency (number of actions executed vs optimal plan), robustness to perturbations (success rate with added noise: object displacement, navigation obstacles, speech recognition errors), and user experience (subjective ratings for naturalness, responsiveness, trust)

11. **Given** documentation and presentation requirements, **When** a student completes the Capstone, **Then** they can write a final report documenting system architecture, implementation details, challenges encountered and solutions, performance metrics with quantitative results, video demonstrations of task executions, code repository with README and setup instructions, and reflection on learning outcomes and potential real-world deployment considerations

12. **Given** optional real hardware deployment, **When** a student extends Capstone to physical robots, **Then** they can transfer trained RL policies from Isaac Sim to real humanoid hardware (if available), calibrate real sensors (cameras, microphones, LiDAR) with Isaac ROS pipelines, test voice commands in real-world noisy environments, validate LLM planning with real-world state uncertainty, measure sim-to-real performance gaps, and document hardware-specific challenges and adaptations (latency, actuation limits, safety constraints)

13. **Given** advanced Capstone extensions, **When** a student explores cutting-edge features, **Then** they can implement multi-robot coordination (multiple humanoids collaborating on tasks via LLM coordination), learning from human feedback (LLMs query users for plan approval or corrections, update prompts based on feedback), explainable AI (LLMs generate natural language explanations for action choices), or integration with emerging VLA models (Google RT-2, OpenVLA, or similar robotics foundation models if available)

---

### Edge Cases

- **Speech Recognition Failures in Noisy Environments**: What if Whisper accuracy degrades below 70% in industrial or outdoor settings with high ambient noise? (Approach: Provide noise reduction preprocessing tutorials using spectral subtraction or RNNoise, recommend beamforming array microphones for directional audio capture, document accuracy/noise trade-offs for different Whisper model sizes, and implement multi-attempt clarification dialogues—"Did you say 'fetch the red box' or 'fetch the bread box'?")

- **LLM API Latency and Cost**: What if students cannot afford OpenAI API costs or experience >5 second latency? (Approach: Provide comprehensive local LLM deployment guides using Ollama with LLaMA 3.1 8B or Mistral 7B, benchmark latency and accuracy trade-offs between cloud GPT-4 and local models, offer cost estimation tools for API usage, and recommend caching frequent queries or using smaller models for simple commands)

- **LLM Hallucinations Producing Invalid Action Plans**: What if LLM generates syntactically correct but semantically invalid actions (e.g., "navigate_to(flying_position)")? (Approach: Implement robust action validators checking against robot capability definitions, use schema-based output parsers (JSON schemas, Pydantic models), design prompts with explicit constraints ("Only use actions from this list"), enable plan preview and user confirmation for critical tasks, and log invalid plans for prompt refinement)

- **Multimodal LLM Access**: What if students lack access to GPT-4 Vision or Claude 3.5 Sonnet for visual grounding? (Approach: Provide alternative workflows using open-source vision-language models like LLaVA, MiniGPT-4, or InstructBLIP running locally, demonstrate two-stage pipelines with separate object detectors + text-only LLMs, and document accuracy comparisons between proprietary and open-source VLMs)

- **Isaac Sim Simulation Reality Gap for VLA Testing**: What if voice-to-action systems validated in Isaac Sim fail on real hardware due to sensor noise, latency, or dynamics mismatch? (Approach: Provide simulation-to-reality checklist covering sensor noise injection in Isaac Sim, latency modeling for audio and action execution, domain randomization for robust policies, and recommend iterative validation—test in sim, deploy on hardware, refine sim parameters, retest)

- **Complex Multi-Step Task Failures**: What if LLM plans 10-step sequences but execution fails at step 7 due to unforeseen obstacles? (Approach: Implement hierarchical planning with checkpoints and partial rollback, enable LLM replanning from failure state with updated world knowledge, use execution monitoring to detect failures early, and design tasks with graceful degradation—complete as much as possible even if full task fails)

- **Language Diversity Beyond English**: What if students need multilingual support for non-English deployments? (Approach: Demonstrate Whisper's multilingual capabilities with Spanish, Chinese, Arabic examples, provide LLM prompting in multiple languages using GPT-4's multilingual support or local multilingual models, and document language-specific accuracy benchmarks)

- **Limited GPU Resources for Whisper + LLM + Isaac Sim**: What if students only have entry-level GPUs (RTX 2060) struggling to run Whisper, LLM inference, and Isaac Sim simultaneously? (Approach: Provide resource allocation strategies—run Whisper on CPU, use quantized Whisper models, offload LLM to cloud API, reduce Isaac Sim environment complexity, or use cloud GPU instances for Capstone development with cost estimates)

- **Ethical and Safety Concerns for Autonomous Humanoids**: What if students implement systems that could be unsafe in real deployments? (Approach: Dedicate section to robotics ethics and safety—emergency stop mechanisms, human-in-the-loop overrides, transparency of LLM decision-making, privacy considerations for voice/camera data, and responsible AI guidelines for robot autonomy)

- **Capstone Project Scope Creep**: What if students attempt overly ambitious Capstone projects beyond course timeframe (e.g., full home assistance with 50+ tasks)? (Approach: Provide scoping guidelines with minimum viable Capstone (3-5 tasks, single environment), recommended scope (5-10 tasks, 2 environments), and advanced scope (10+ tasks, multi-environment), along with time estimates and prioritization advice)

## Requirements *(mandatory)*

### Functional Requirements

#### Chapter 1: Voice-to-Action Pipeline with OpenAI Whisper Requirements

- **FR-001**: Chapter 1 MUST include 3-5 measurable learning objectives covering speech recognition fundamentals, OpenAI Whisper architecture, ROS 2 audio integration, real-time transcription pipelines, multilingual support, and voice-to-intent extraction for robot commands

- **FR-002**: Chapter 1 MUST provide theoretical foundations explaining automatic speech recognition (ASR) principles (acoustic modeling, language modeling, end-to-end neural ASR), Transformer-based sequence-to-sequence models (encoder-decoder architecture, multi-head attention), Whisper training methodology (680,000 hours of multilingual web data, weakly supervised learning, prompting for task specification), and advantages of Whisper for robotics (robustness to noise, zero-shot multilingual, semantic understanding)

- **FR-003**: Chapter 1 MUST include OpenAI Whisper installation tutorials covering Python environment setup (Python 3.9+, PyTorch), Whisper installation via pip (openai-whisper package), model downloads (tiny, base, small, medium, large, large-v2/v3), GPU acceleration configuration (CUDA, cuDNN for NVIDIA GPUs), and cloud API alternative using OpenAI API with Whisper endpoint

- **FR-004**: Chapter 1 MUST provide audio capture and preprocessing tutorials demonstrating ROS 2 audio_common package installation for microphone input, configuring USB or array microphones in Linux (ALSA, PulseAudio), implementing Voice Activity Detection (VAD) using webrtcvad or Silero VAD models to segment speech from silence, audio format conversion (ROS 2 audio_msgs to 16kHz mono PCM WAV for Whisper), and audio quality assessment (SNR measurement, clipping detection)

- **FR-005**: Chapter 1 MUST demonstrate real-time speech transcription by creating ROS 2 nodes wrapping Whisper inference, implementing streaming transcription with audio chunking (1-10 second windows), publishing transcription results to ROS 2 String topics (std_msgs/String), adding confidence scores and language detection outputs, and measuring latency from audio capture to text output (<1 second target for responsive HRI)

- **FR-006**: Chapter 1 MUST cover multilingual and accent robustness testing by evaluating Whisper accuracy across 10+ languages (English, Spanish, Mandarin, French, German, Japanese, Arabic, Hindi, Portuguese, Russian), testing with non-native accents and speech variations, measuring Word Error Rate (WER) and Character Error Rate (CER) metrics, and documenting accuracy trade-offs for different model sizes (tiny: fast but less accurate, large: slow but highly accurate)

- **FR-007**: Chapter 1 MUST provide intent extraction and command parsing examples demonstrating post-processing of transcriptions (lowercasing, punctuation removal), keyword extraction for robot commands (action verbs: "go", "pick", "place", "find"; object nouns; location names), implementing simple rule-based parsers for common command patterns ("verb + object + location"), and preparing formatted prompts for LLM cognitive planning in Chapter 2

- **FR-008**: Chapter 1 MUST include wake word detection and dialogue management by integrating wake word models (Picovoice Porcupine, Mycroft Precise, or custom CNN wake word models) for "Hey Robot" or custom triggers, implementing state machines for multi-turn dialogue (listening → processing → responding → idle), providing audio feedback (beeps, LEDs, synthesized speech via gTTS or Coqui TTS) to confirm command reception, and handling dialogue timeouts or user interruptions

- **FR-009**: Chapter 1 MUST demonstrate integration with Isaac Sim humanoid robots by connecting Whisper ROS 2 nodes to Isaac Sim, simulating microphone inputs from virtual audio or text-to-speech synthesis, issuing voice commands for basic humanoid actions (wave hand, walk forward, turn), and visualizing transcriptions in RViz text displays or Isaac Sim UI overlays

- **FR-010**: Chapter 1 MUST provide performance optimization guidance including benchmarking Whisper model sizes on target hardware (latency and accuracy for tiny/base/small/medium/large on CPU, GPU, Jetson), implementing parallel audio processing with multi-threading or asyncio for non-blocking inference, optimizing VAD parameters to balance responsiveness and false positives, and profiling memory usage and GPU utilization

- **FR-011**: Chapter 1 MUST include exercises ranging from beginner (install Whisper, transcribe pre-recorded audio file) to intermediate (integrate Whisper with ROS 2, implement streaming transcription from microphone) to advanced (deploy multilingual wake word + Whisper system on Jetson, optimize for <500ms latency, integrate with Isaac Sim robot voice control)

- **FR-012**: Chapter 1 MUST include a "Further Reading" section with references to OpenAI Whisper paper, ASR research (wav2vec 2.0, Conformer models), ROS 2 audio_common documentation, HRI speech interface best practices, and multilingual speech recognition benchmarks

#### Chapter 2: Cognitive Planning with LLMs for Action Translation Requirements

- **FR-013**: Chapter 2 MUST include 3-5 learning objectives covering LLM fundamentals for robotics, prompt engineering for task planning, structured output parsing, ROS 2 action integration, multimodal vision-language models, plan validation and safety checks, and execution monitoring with replanning

- **FR-014**: Chapter 2 MUST explain theoretical foundations of Large Language Models including Transformer architecture (self-attention, positional encodings, layer normalization), pre-training objectives (next-token prediction, masked language modeling), fine-tuning and instruction-following (RLHF, SFT), emergent capabilities (reasoning, code generation, planning), and LLM limitations for robotics (hallucinations, lack of physics grounding, token limits)

- **FR-015**: Chapter 2 MUST provide LLM API integration tutorials covering OpenAI API setup (GPT-4, GPT-4 Turbo, GPT-4o) with authentication and rate limiting, Anthropic API setup (Claude 3.5 Sonnet, Claude Opus) for long-context tasks, local LLM deployment using Ollama (LLaMA 3.1 8B/70B, Mistral 7B/8x7B), vLLM serving for optimized inference, and HuggingFace Transformers for custom model deployment

- **FR-016**: Chapter 2 MUST demonstrate robot capability documentation by creating structured JSON or YAML files defining available ROS 2 actions (action name, parameters with types, preconditions, postconditions, examples), documenting robot state observables (pose, detected objects, joint configurations, sensor readings), and generating natural language descriptions of capabilities for inclusion in LLM system prompts

- **FR-017**: Chapter 2 MUST provide prompt engineering tutorials demonstrating system prompt design ("You are a humanoid robot assistant. You can navigate, grasp objects, and answer questions. Your available actions are..."), few-shot examples showing natural language commands mapped to action sequences (minimum 5 examples covering navigation, manipulation, multi-step tasks), chain-of-thought prompting for complex reasoning ("Let's break down the task step-by-step..."), and output format constraints (JSON schemas, Python code templates)

- **FR-018**: Chapter 2 MUST include structured output parsing examples using JSON parsing with error handling (try/except for malformed JSON), Pydantic models for schema validation (defining action classes with typed parameters), regular expressions for extracting action calls from natural language LLM outputs, and fallback mechanisms for ambiguous or unparseable LLM responses

- **FR-019**: Chapter 2 MUST demonstrate ROS 2 action execution by creating LLM-to-ROS 2 translator nodes that parse action sequences and invoke ROS 2 action clients (rclpy action clients for navigation, manipulation), implementing action parameter filling from LLM outputs (goal poses, object IDs), handling action feedback and status updates, and implementing sequential or parallel action execution based on dependencies

- **FR-020**: Chapter 2 MUST provide contextual reasoning and memory integration examples by including robot state in LLM prompts (current pose from /tf or /odom, detected objects from perception nodes, joint states), implementing dialogue history tracking for multi-turn commands (storing conversation context with sliding window or summarization), using vector databases (ChromaDB, Pinecone, FAISS) for retrieving past task experiences, and enabling LLM queries to ROS 2 topics/services for real-time state awareness

- **FR-021**: Chapter 2 MUST include plan validation and safety check implementations by checking action preconditions (navigation requires valid map, grasping requires detected object), validating action parameters (goal poses within robot workspace, object IDs from current detection list), estimating plan costs (action count, estimated time, energy usage), and implementing user confirmation prompts for high-risk actions (lifting heavy objects, moving near humans)

- **FR-022**: Chapter 2 MUST demonstrate execution monitoring and replanning by subscribing to ROS 2 action feedback (goal status, progress updates), detecting action failures (navigation blocked, grasp failed), triggering LLM replanning with failure context ("Previous action 'navigate_to(kitchen)' failed because path is blocked. Current state: robot at hallway. Replan to reach kitchen."), and logging execution traces for offline analysis and LLM fine-tuning datasets

- **FR-023**: Chapter 2 MUST provide multimodal vision-language model (VLM) integration examples using GPT-4 Vision or GPT-4o for image understanding, Claude 3.5 Sonnet vision capabilities for detailed scene analysis, or open-source VLMs (LLaVA, MiniGPT-4, InstructBLIP) for local deployment, demonstrating visual question answering ("What objects are on the table?"), visual grounding for referring expressions ("the red cup on the left"), and scene understanding for task planning

- **FR-024**: Chapter 2 MUST include integration with Isaac Sim humanoid robots by deploying LLM ROS 2 nodes in Isaac Sim environments, chaining Whisper voice commands (from Chapter 1) → LLM planning → ROS 2 action execution in simulation, demonstrating multi-step tasks (navigate to location, detect object, grasp object, navigate to user), and measuring task success rates (>80% for common household tasks with 10+ test scenarios)

- **FR-025**: Chapter 2 MUST provide LLM fine-tuning and domain adaptation tutorials by collecting robotics-specific datasets (command-action pairs from human demonstrations, task execution logs), fine-tuning LLaMA or Mistral models using LoRA or QLoRA for parameter-efficient training, evaluating fine-tuned models on held-out test commands (comparing accuracy, action sequence correctness), and documenting training hyperparameters (learning rate, batch size, epochs, rank for LoRA)

- **FR-026**: Chapter 2 MUST include exercises ranging from beginner (call OpenAI API to generate robot plan for "Pick up the book", parse JSON output) to intermediate (integrate LLM with ROS 2, execute planned actions in Isaac Sim, handle action failures) to advanced (fine-tune LLaMA on custom robotics dataset, implement RAG with vector database for task memory, integrate GPT-4 Vision for visual grounding)

- **FR-027**: Chapter 2 MUST include a "Further Reading" section with references to LLM papers (GPT-4 report, LLaMA 3.1, Claude 3.5), prompt engineering guides (OpenAI best practices, Anthropic prompt engineering), robotics LLM research (SayCan, Code as Policies, Instruct2Act, RT-2), and VLM papers (GPT-4V, LLaVA, Flamingo)

#### Chapter 3: Capstone Project - Autonomous Humanoid Robot Requirements

- **FR-028**: Chapter 3 MUST provide Capstone Project overview and objectives defining the goal as integrating all course modules (Module 1: ROS 2, Module 2: Digital Twins, Module 3: Isaac Sim/ROS/RL/Nav2, Module 4: Voice-Language-Action), demonstrating end-to-end autonomous humanoid robot system (voice command input → cognitive task planning → multi-modal perception → navigation and manipulation execution), and producing portfolio-ready documentation (video demo, technical report, code repository)

- **FR-029**: Chapter 3 MUST offer 4-6 Capstone Project templates with varying difficulty levels: (1) Household Assistance Robot (fetch and deliver objects in kitchen/living room, beginner-intermediate), (2) Warehouse Logistics Robot (pick items from shelves, place in bins, intermediate), (3) Search-and-Rescue Robot (navigate disaster environment, locate and report objects, intermediate-advanced), (4) Interactive Museum Guide Robot (answer visitor questions, navigate exhibits, beginner-intermediate), (5) Healthcare Assistant Robot (fetch medical supplies, assist with mobility, advanced), and (6) Custom Project (student-defined with instructor approval, any difficulty)

- **FR-030**: Chapter 3 MUST require students to define measurable success criteria for their Capstone including task success rate (percentage of successfully completed tasks across 10+ test scenarios, target >75%), latency metrics (time from voice command to action start <10 seconds, time from action start to task completion <120 seconds), action efficiency (number of executed actions within 20% of optimal plan length), robustness (success rate maintained under perturbations: object displacement, navigation obstacles, speech recognition errors), and user experience (subjective ratings for naturalness, responsiveness, safety)

- **FR-031**: Chapter 3 MUST guide students through system architecture design by creating component diagrams (Whisper voice input, LLM planner, Isaac ROS perception, Nav2 navigation, manipulation controllers, ROS 2 communication), defining interfaces between components (ROS 2 topics, services, actions with message types and QoS settings), documenting data flow from user command to robot state updates, and identifying failure modes and recovery strategies for each component

- **FR-032**: Chapter 3 MUST provide Isaac Sim environment setup guidance for Capstone by offering pre-built USD scenes for each project template (kitchen scene for household robot, warehouse with shelves for logistics, disaster site with rubble for search-and-rescue), documenting how to customize scenes (add objects, modify lighting, adjust physics parameters), configuring humanoid robot models (NVIDIA H1, Unitree G1, or custom from Module 1), and setting up sensors (stereo cameras, LiDAR, IMU, microphones for simulation)

- **FR-033**: Chapter 3 MUST require integration of voice command interface (from Chapter 1) by deploying Whisper ROS 2 nodes for real-time transcription, implementing wake word activation for natural interaction, configuring domain-specific vocabulary for project tasks, and testing speech recognition accuracy (WER <20% for project-related commands)

- **FR-034**: Chapter 3 MUST require integration of LLM cognitive planning (from Chapter 2) by selecting and configuring LLM (OpenAI GPT-4, Claude, or local LLaMA), designing project-specific system prompts and few-shot examples, implementing structured output parsing and ROS 2 action translation, and validating planning accuracy (>80% correct action sequences for common commands)

- **FR-035**: Chapter 3 MUST require integration of perception capabilities (from Module 3 Chapter 2) by using Isaac ROS object detection to identify task-relevant objects (target success rate >90% detection accuracy for known objects), running VSLAM for localization and mapping in Isaac Sim environments, optionally integrating VLMs (GPT-4 Vision, LLaVA) for visual question answering, and publishing detected object 6D poses to LLM planner

- **FR-036**: Chapter 3 MUST require integration of autonomous navigation (from Module 3 Chapter 4) by configuring Nav2 for humanoid robot with project-appropriate parameters, implementing LLM-directed navigation to goal poses, handling dynamic obstacle avoidance during task execution, and achieving navigation success rate >85% in project environment

- **FR-037**: Chapter 3 MUST require integration of manipulation capabilities (from Module 3 Chapter 3 or IK solvers) by implementing grasping using RL-trained policies (if available from Module 3) or inverse kinematics + gripper control, demonstrating pre-grasp positioning and approach trajectories, handling grasp failure detection and retries (maximum 3 attempts), and achieving grasp success rate >70% for target objects

- **FR-038**: Chapter 3 MUST require end-to-end task demonstrations by defining 10+ test scenarios covering project-specific tasks (e.g., for household robot: "Bring me a water bottle from the fridge", "Put the book back on the shelf", "Find my keys"), executing tasks in Isaac Sim with full VLA pipeline, recording video demonstrations of successful and failed attempts, and computing quantitative success metrics (success rate, latency, action efficiency)

- **FR-039**: Chapter 3 MUST require performance evaluation and analysis by measuring task success rate across all test scenarios, computing latency metrics (voice command to action start, action start to task completion), analyzing failure modes (speech recognition errors, LLM planning errors, perception failures, navigation/manipulation failures), comparing against baseline systems (template-based command parser, hand-coded task plans), and documenting performance bottlenecks and potential improvements

- **FR-040**: Chapter 3 MUST require comprehensive documentation including final technical report (10-20 pages covering system architecture, implementation details, experimental setup, quantitative results, failure analysis, future work), video demonstration (5-10 minutes showing system overview, task demonstrations, highlighting key features), code repository with README (installation instructions, dependencies, launch files, usage examples), and optional presentation slides for public demos or portfolio websites

- **FR-041**: Chapter 3 MUST provide optional real hardware deployment guidance for students with access to physical humanoid robots by covering sim-to-real transfer considerations (sensor calibration, latency compensation, safety constraints), RL policy transfer from Isaac Sim to real actuators (testing in controlled environment first), real-world testing protocols (safety zones, emergency stop procedures, human supervision), and documenting sim-to-real performance gaps (expected degradation: 10-30% lower success rate on first real-world deployment)

- **FR-042**: Chapter 3 MUST suggest advanced Capstone extensions for students seeking extra challenges including multi-robot coordination (2+ humanoids collaborating on tasks with LLM-based coordination protocols), learning from human feedback (LLM queries user for plan approval or corrections, updates prompt database with user preferences), explainable AI (LLM generates natural language explanations for action choices, decision trees, failure causes), long-term autonomy (24-hour autonomous operation with battery management, task scheduling), or integration with emerging VLA foundation models (Google RT-2, RT-X, OpenVLA if available)

- **FR-043**: Chapter 3 MUST include a rubric for Capstone evaluation covering technical implementation (40%: correct integration of all components, adherence to ROS 2 best practices, code quality), performance metrics (30%: task success rate, latency, robustness), documentation quality (20%: clarity of report, quality of video demo, code documentation), and innovation/creativity (10%: novel features, advanced extensions, elegant solutions to challenges)

- **FR-044**: Chapter 3 MUST provide troubleshooting guidance for common Capstone issues including debugging multi-component systems (using ROS 2 tools: topic echo, service call, action send_goal; logging and visualization in RViz), optimizing performance bottlenecks (profiling CPU/GPU usage, reducing latency in LLM calls, tuning perception/navigation parameters), handling integration failures (version mismatches, API rate limits, simulation crashes), and managing project scope (prioritizing MVP features, deferring advanced extensions if time-constrained)

- **FR-045**: Chapter 3 MUST include 5+ example Capstone projects from past students or instructors (with permission) demonstrating household robot fetching objects, warehouse robot shelf picking, search-and-rescue robot navigating obstacles, museum guide answering questions, and healthcare robot delivering medical supplies, each with video demo, brief description, performance metrics, and key implementation insights

- **FR-046**: Chapter 3 MUST include a "Further Reading" section with references to embodied AI research (SayCan, RT-2, PaLM-E, Robotic Foundation Models), VLA papers (OpenVLA, Octo, RT-X), humanoid robotics projects (Tesla Optimus, Figure 01, 1X Technologies), open-source robotics frameworks (ROS 2, Isaac Sim, MoveIt), and career pathways in Physical AI (roles in robotics companies, research labs, PhD programs)

### Key Entities *(include if feature involves data)*

- **Voice Command**: Natural language speech input from user, transcribed by Whisper, containing task intent for humanoid robot
  - Attributes: audio waveform (16kHz PCM), transcribed text, language code, confidence score, timestamp, speaker ID (optional)
  - Relationships: Input to LLM Cognitive Planner

- **Task Plan**: Structured sequence of robot actions generated by LLM to accomplish user-specified task
  - Attributes: action list (ordered steps), action parameters (positions, object IDs), preconditions, estimated cost (time, actions), confidence score, alternative plans (backups)
  - Relationships: Output from LLM Cognitive Planner, input to Action Executor

- **Robot Action**: Atomic executable operation (navigation, manipulation, perception query) with defined inputs, outputs, and success criteria
  - Attributes: action type (navigate, grasp, detect, etc.), parameters (goal pose, object ID, sensor config), status (pending, executing, succeeded, failed), feedback (progress, errors), duration
  - Relationships: Element of Task Plan, invokes ROS 2 action servers

- **Detected Object**: Physical object in environment identified by perception system (Isaac ROS, VLM) with 6D pose and semantic label
  - Attributes: object ID, semantic class (cup, book, box), 6D pose (position, orientation), bounding box, point cloud, confidence score, timestamp
  - Relationships: Input to LLM Cognitive Planner, target of manipulation actions

- **Robot State**: Current configuration and status of humanoid robot including pose, joint states, sensor data, and task execution status
  - Attributes: base pose (position, orientation), joint positions/velocities/torques, battery level, sensor readings (cameras, LiDAR, IMU), current action status, detected objects list
  - Relationships: Provided to LLM Cognitive Planner for contextual planning, queried by Action Executor

- **Dialogue History**: Record of multi-turn conversation between user and robot for contextual understanding and memory
  - Attributes: message list (user and robot utterances), timestamps, task outcomes (success/failure), user preferences, conversation summary
  - Relationships: Provided to LLM Cognitive Planner for context-aware planning

- **Capstone Project**: Student's final integrated VLA humanoid robot system demonstrating course learning outcomes
  - Attributes: project type (household, warehouse, etc.), USD environment scenes, robot model, test scenarios list, performance metrics (success rate, latency, efficiency), code repository URL, video demo URL, technical report PDF
  - Relationships: Synthesizes all course modules (1-4), evaluated by rubric

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students completing Chapter 1 can deploy OpenAI Whisper for real-time speech recognition with <1 second latency and >90% Word Error Rate (WER) accuracy for clear English speech in low-noise environments, integrated with ROS 2 audio pipelines publishing transcriptions to topics

- **SC-002**: Students completing Chapter 1 can demonstrate multilingual Whisper transcription for 5+ languages (English, Spanish, Mandarin, French, German) with documented accuracy metrics (WER) and configure domain-specific prompts for robotics vocabulary achieving >85% accuracy on robot command keywords

- **SC-003**: Students completing Chapter 2 can integrate LLM APIs (OpenAI GPT-4 or local LLaMA models) to generate structured robot action plans from natural language commands with >80% correct action sequence accuracy (validated by human evaluation on 50+ test commands covering navigation, manipulation, and multi-step tasks)

- **SC-004**: Students completing Chapter 2 can implement LLM-to-ROS 2 action translators that parse LLM outputs (JSON or natural language), validate action parameters against robot capabilities, invoke ROS 2 action clients, and handle execution feedback with success rates >85% for action invocations in Isaac Sim

- **SC-005**: Students completing Chapter 2 can demonstrate multimodal vision-language model integration (GPT-4 Vision, LLaVA, or equivalent) for visual question answering and object referring expressions with >80% accuracy on standardized test sets (e.g., "What is the red object on the left?" correctly identifying and localizing objects in Isaac Sim scenes)

- **SC-006**: Students completing Chapter 3 Capstone Project can demonstrate end-to-end autonomous humanoid robot systems achieving >75% task success rate across 10+ predefined test scenarios (covering voice command input via Whisper, LLM task planning, perception-based object detection, Nav2 navigation, and manipulation execution in Isaac Sim)

- **SC-007**: Students completing Chapter 3 Capstone Project can achieve <10 second latency from voice command to first robot action initiation (measured from Whisper transcription start to ROS 2 action goal acceptance) and <120 second total task completion time for standard tasks (fetch object, navigate to location, deliver object)

- **SC-008**: Students completing Chapter 3 Capstone Project can produce comprehensive documentation including technical report (10-20 pages), video demonstration (5-10 minutes), and code repository with README, achieving minimum scores of 70% on evaluation rubric covering implementation quality, performance metrics, documentation clarity, and innovation

- **SC-009**: Students completing Chapter 3 Capstone Project can demonstrate robustness to perturbations maintaining >60% task success rate under at least 3 types of disturbances: speech recognition errors (simulated by adding 10% word error rate), object displacement (moving target objects 10-20cm from expected positions), and navigation obstacles (adding dynamic obstacles in planned path)

- **SC-010**: Students completing Module 4 can explain the full VLA pipeline architecture (Whisper speech recognition → LLM task planning → ROS 2 action execution → perception/navigation/manipulation integration) and identify key design decisions, trade-offs, and failure modes, validated through written report or oral examination scoring >80% on technical understanding

- **SC-011**: 90% of students successfully complete Chapter 1 exercises (install Whisper, integrate with ROS 2, implement real-time transcription) within estimated 8-12 hours of study time, based on course analytics or student self-reports

- **SC-012**: 80% of students successfully complete Chapter 2 exercises (set up LLM API, design prompts, parse structured outputs, execute ROS 2 actions from LLM plans) within estimated 10-14 hours of study time

- **SC-013**: 70% of students successfully complete Chapter 3 Capstone Project (full system integration, 10+ test scenarios, documentation) within estimated 20-30 hours of development time (spread over 2-4 weeks)

- **SC-014**: Student satisfaction with Module 4 content averages >4.0/5.0 on post-module surveys covering clarity of explanations, quality of examples, relevance to industry needs, and overall learning value

- **SC-015**: Capstone Project artifacts (video demos, reports) from at least 30% of students are portfolio-quality (suitable for showcasing to potential employers or graduate school applications) as assessed by instructor or industry reviewers

## Scope and Boundaries

### In Scope

- **Voice Interface Development**: Complete tutorial coverage of OpenAI Whisper integration with ROS 2 for speech-to-text, including audio capture, preprocessing (VAD), real-time transcription, multilingual support, wake word detection, and voice command parsing for robot task intents

- **LLM Cognitive Planning**: Comprehensive guides for integrating Large Language Models (GPT-4, Claude, LLaMA) with ROS 2 for natural language task planning, covering prompt engineering, structured output parsing, ROS 2 action translation, execution monitoring, replanning on failure, and multimodal vision-language models for visual grounding

- **Isaac Sim Integration**: All examples and exercises run in NVIDIA Isaac Sim environments from Module 3, demonstrating VLA pipelines on simulated humanoid robots (NVIDIA H1, Unitree G1, or custom models) with GPU-accelerated physics, perception, and rendering

- **ROS 2 Humble**: All ROS 2 integration targets ROS 2 Humble (LTS) running on Ubuntu 22.04, leveraging ROS 2 action servers, topics, services, and consistent with Module 1-3 ROS 2 usage

- **Capstone Project Framework**: Structured Capstone Project templates (household, warehouse, search-and-rescue, museum guide, healthcare, custom) with pre-built Isaac Sim scenes, evaluation rubrics, documentation requirements, and example projects demonstrating successful end-to-end VLA integration

- **Cloud and Local LLM Options**: Coverage of both cloud-based LLM APIs (OpenAI, Anthropic) and local LLM deployment (Ollama, vLLM, HuggingFace) to accommodate students with varying budgets, API access, and compute resources

- **Multilingual Support**: Documentation and examples for multilingual voice interfaces (English, Spanish, Mandarin, French, German minimum) leveraging Whisper's 99-language capability

- **Performance Optimization**: Guidance on latency reduction, GPU utilization, model selection trade-offs (Whisper tiny vs large, GPT-4 vs LLaMA 8B), and resource allocation for students with entry-level GPUs or cloud instances

- **Ethical and Safety Considerations**: Dedicated section on responsible AI for robotics covering safety mechanisms (emergency stops, human-in-the-loop overrides), transparency of LLM decision-making, privacy for voice/camera data, and ethical guidelines for autonomous robot behavior

- **Troubleshooting and Debugging**: Common failure modes, debugging strategies using ROS 2 tools, LLM output validation, and integration testing best practices

### Out of Scope

- **Real Hardware Deployment (Primary Focus)**: While optional real hardware guidance is provided, Module 4 primarily focuses on simulation-based VLA systems in Isaac Sim. Full real-world deployment with hardware-specific calibration, safety certifications, and field testing is beyond scope (suitable for follow-up courses or thesis projects)

- **Custom LLM Training from Scratch**: Training Large Language Models from scratch (pre-training on large corpora) is computationally prohibitive and beyond course scope. Coverage is limited to fine-tuning pre-trained models (LLaMA, Mistral) on robotics datasets and prompt engineering for existing models

- **Advanced Speech Synthesis (TTS)**: While basic TTS for robot audio feedback is mentioned (gTTS, Coqui TTS), in-depth coverage of expressive speech synthesis, voice cloning, or prosody modeling is out of scope (students use off-the-shelf TTS libraries)

- **Vision-Language Model Training**: Training multimodal VLMs from scratch (like LLaVA, Flamingo) is out of scope. Students use pre-trained VLMs (GPT-4 Vision, LLaVA) and learn integration patterns, not model development

- **Embedded AI Optimization (TensorRT, Quantization)**: While mentioned for context, deep optimization of LLMs for edge deployment (TensorRT-LLM, INT4/INT8 quantization, kernel fusion) is out of scope. Students use standard inference frameworks (Ollama, vLLM, OpenAI API)

- **Multi-Robot Coordination (Primary Focus)**: While mentioned as advanced Capstone extension, detailed multi-robot task allocation, communication protocols, and distributed LLM coordination are out of scope for base curriculum (suitable for advanced projects)

- **Human-Robot Collaboration Safety Standards**: Formal safety certification processes (ISO 10218, ISO 15066 for collaborative robots) and risk assessment methodologies are out of scope. General safety principles are covered but not industry-specific compliance processes

- **Physical Manipulation Hardware Details**: In-depth gripper design, tactile sensing, force control hardware, and electromechanical actuator specifications are out of scope. Students use simulated grippers in Isaac Sim with abstracted grasp success models

- **Natural Language Understanding Theory**: Deep NLP theory (dependency parsing, semantic role labeling, coreference resolution, discourse analysis) is out of scope. Coverage focuses on practical LLM integration, not linguistic fundamentals

- **Specific Company APIs Beyond Examples**: While OpenAI and Anthropic APIs are primary examples, exhaustive coverage of all commercial LLM providers (Google PaLM, Cohere, AI21, etc.) is out of scope. Students learn patterns applicable to any LLM API

- **Dialogue Management Frameworks**: Dedicated dialogue management systems (Rasa, Dialogflow, Alexa Skills Kit) are out of scope. Students implement simple state machine-based dialogues sufficient for robot task commands, not complex multi-domain conversational AI

## Dependencies and Prerequisites

### Technical Prerequisites (from Previous Modules)

- **Module 1: ROS 2 & Humanoid Fundamentals** (REQUIRED):
  - ROS 2 Humble installation and workspace management (`colcon build`, `source install/setup.bash`)
  - ROS 2 communication patterns (topics, services, actions, parameters)
  - Launch files (Python launch scripts, YAML parameter files)
  - URDF robot modeling (joints, links, sensors for humanoid structures)
  - ROS 2 CLI tools (`ros2 topic echo`, `ros2 action send_goal`, `ros2 service call`)

- **Module 2: Digital Twin Simulation** (RECOMMENDED):
  - Understanding of simulation concepts (world models, physics engines, sensor simulation)
  - Experience with robot sensor integration (cameras, LiDAR, IMU)
  - Familiarity with simulation-reality gap challenges

- **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** (REQUIRED):
  - **Chapter 1**: Isaac Sim installation, USD scene creation, humanoid robot import, ROS 2 bridge setup
  - **Chapter 2**: Isaac ROS perception (object detection, VSLAM, depth estimation) for providing perception inputs to VLA system
  - **Chapter 3** (OPTIONAL): RL-trained manipulation policies for Capstone grasping tasks (alternative: use IK-based manipulation)
  - **Chapter 4**: Nav2 navigation stack configured for humanoid robots, essential for VLA autonomous navigation tasks

### Software and Tool Dependencies

- **Operating System**: Ubuntu 22.04 LTS (native installation, not WSL2) for ROS 2 Humble and Isaac Sim compatibility
- **ROS 2**: ROS 2 Humble (LTS) with `ros-humble-desktop`, `ros-humble-navigation2`, `ros-humble-audio-common` packages
- **NVIDIA GPU**: RTX 2060 or higher (6GB+ VRAM) for Whisper GPU acceleration and Isaac Sim (or cloud GPU alternatives: AWS G4/G5 instances)
- **NVIDIA Driver**: 525+ (verify with `nvidia-smi`), CUDA 11.8+, cuDNN 8.6+
- **Python**: Python 3.9+ with pip, virtual environment tools (venv or conda)
- **OpenAI Whisper**: `pip install openai-whisper` (includes PyTorch dependency)
- **LLM Access**: OpenAI API key (paid tier for GPT-4 access, ~$20-50 budget for module) OR Anthropic API key (Claude 3.5 Sonnet) OR local LLM via Ollama (`ollama pull llama3.1:8b`)
- **Audio Libraries**: `pyaudio`, `sounddevice`, `webrtcvad`, `librosa` for audio capture and preprocessing
- **ROS 2 Audio**: `ros-humble-audio-common` for microphone integration, `ros-humble-audio-capture` for audio streaming
- **Isaac Sim**: NVIDIA Isaac Sim 2023.1+ installed via Omniverse Launcher (from Module 3 Chapter 1)
- **Version Control**: Git for cloning repositories and managing Capstone Project code

### External API and Service Dependencies

- **OpenAI API** (OPTIONAL but RECOMMENDED for Chapter 2): Requires paid API key, estimated cost $20-50 for module completion (GPT-4 inference costs $0.01-0.03 per request depending on prompt length)
- **Anthropic API** (ALTERNATIVE to OpenAI): Claude 3.5 Sonnet or Opus, similar pricing to OpenAI, long context windows useful for detailed robot state descriptions
- **Wake Word Detection Services** (OPTIONAL): Picovoice Porcupine (free tier available for non-commercial use), or Mycroft Precise (open-source)
- **Text-to-Speech Services** (OPTIONAL): Google Text-to-Speech (`gTTS` - free), Coqui TTS (open-source local TTS), or Amazon Polly (paid)

### Hardware Requirements

- **Minimum Configuration** (for learning, may have reduced performance):
  - GPU: NVIDIA RTX 2060 (6GB VRAM) or Tesla T4 (cloud)
  - RAM: 16GB (32GB recommended for running Whisper + LLM + Isaac Sim concurrently)
  - Storage: 120GB free (Whisper models 1-3GB, LLM models 4-50GB for local deployment, Isaac Sim ~50GB from Module 3)
  - CPU: 8-core Intel i7 or AMD Ryzen 7
  - Microphone: USB microphone or laptop built-in microphone (headset mic recommended for noise reduction)

- **Recommended Configuration** (for optimal performance and Capstone development):
  - GPU: NVIDIA RTX 3080 (16GB VRAM) or RTX 4090 (24GB VRAM)
  - RAM: 32GB or 64GB
  - Storage: 250GB+ NVMe SSD
  - CPU: 12-core Intel i9 or AMD Ryzen 9
  - Microphone: USB array microphone with beamforming (e.g., ReSpeaker Mic Array v2.0) for improved speech recognition in noisy environments

- **Cloud GPU Alternatives**:
  - AWS EC2 G4dn.xlarge (Tesla T4, 16GB VRAM): $0.526/hour, ~$10-20 for module completion
  - AWS EC2 G5.xlarge (A10G, 24GB VRAM): $1.006/hour, recommended for Capstone Project development (~$30-50 total)
  - Google Cloud Platform N1 instances with NVIDIA T4 or A100
  - NVIDIA LaunchPad (free trial GPU instances for Isaac Sim)

### Mathematical and Conceptual Prerequisites

- **Linear Algebra**: Vectors, matrices, transformations (for understanding robot pose, object 6D poses, perception outputs) - covered in Modules 1-3
- **Probability**: Confidence scores, uncertainty quantification (for speech recognition accuracy, LLM output confidence, perception detection scores)
- **Natural Language Processing (Basic)**: Understanding of tokenization, embeddings, sequence-to-sequence models (introduced in Chapter 1-2 theory sections, no deep NLP background required)
- **State Machines**: For implementing dialogue management and multi-turn command handling (Chapter 1 wake word + command flow)
- **Software Engineering**: Python programming proficiency, object-oriented design, error handling, asynchronous programming (asyncio) for non-blocking audio/LLM processing

### Estimated Prerequisite Completion Time

- Students completing Modules 1-3 (75-100 hours total) will have all necessary prerequisites for Module 4
- Minimum prerequisites (Module 1 + Module 3 Chapters 1, 2, 4) require ~40-50 hours
- Students with external ROS 2 and Isaac Sim experience may enter Module 4 directly after validating prerequisite skills via assessment quiz or demonstration

## Risk Analysis and Mitigation

### Technical Risks

**Risk 1: OpenAI API Cost Overruns for Students**
- **Description**: Students exceed budgeted $20-50 for OpenAI API usage due to debugging loops, large prompts, or unoptimized LLM calls
- **Impact**: High (financial barrier for students, course access equity issues)
- **Mitigation**:
  - Provide comprehensive local LLM alternatives (Ollama, vLLM, LLaMA 3.1 8B) with installation guides and performance comparisons
  - Implement prompt caching strategies (cache system prompts, reuse few-shot examples)
  - Offer API usage monitoring tutorials (tracking costs via OpenAI dashboard)
  - Provide course credits or vouchers for students with financial need (if institutional budget available)
  - Design exercises with prompt length limits and expected API call counts

**Risk 2: Whisper GPU Memory Exhaustion on Entry-Level GPUs**
- **Description**: Students with RTX 2060 (6GB VRAM) encounter out-of-memory errors when running Whisper large model + Isaac Sim simultaneously
- **Impact**: Medium (degrades learning experience, forces model compromises)
- **Mitigation**:
  - Document GPU memory requirements for each Whisper model size (tiny: 1GB, small: 2GB, medium: 5GB, large: 10GB)
  - Recommend Whisper tiny/base models for resource-constrained setups (acceptable accuracy for clear speech)
  - Provide CPU fallback instructions (Whisper runs on CPU with 2-5x slower inference)
  - Offer cloud GPU rental guides for Capstone Project (AWS G4/G5 spot instances)
  - Implement model switching based on available VRAM (check `torch.cuda.mem_get_info()`)

**Risk 3: LLM Hallucinations Causing Dangerous Robot Actions**
- **Description**: LLMs generate invalid or unsafe action plans (e.g., "navigate through wall", "grasp hot object", "move at high speed near humans") that pass validation and execute
- **Impact**: High (safety risk in real deployments, erosion of trust in VLA systems)
- **Mitigation**:
  - Mandatory plan validation layer checking action preconditions and physical constraints
  - Implement action whitelisting (only allow explicitly defined safe actions)
  - Require user confirmation prompts for high-risk actions (movement near humans, object manipulation above certain heights)
  - Log all LLM outputs for post-execution review and debugging
  - Provide red-teaming exercises where students intentionally probe LLM for unsafe outputs and design defensive checks
  - For real hardware (optional), enforce emergency stop mechanisms and geofencing (robot cannot leave designated safe zone)

**Risk 4: Speech Recognition Failure in Noisy Environments**
- **Description**: Whisper accuracy degrades to <60% WER in realistic noisy environments (industrial sites, outdoor settings, crowded spaces), making voice commands unusable
- **Impact**: Medium (limits real-world applicability, frustrates users in Capstone demos)
- **Mitigation**:
  - Provide audio preprocessing tutorials (spectral subtraction noise reduction, RNNoise)
  - Recommend beamforming array microphones for directional audio capture
  - Implement multi-attempt clarification dialogues ("Did you say X or Y?")
  - Document noise robustness benchmarks for different Whisper models and preprocessing configurations
  - Offer fallback input modes (text input GUI, gesture recognition, predefined command buttons) for high-noise scenarios

**Risk 5: Isaac Sim-to-Real Sim2Real Gap for VLA Systems**
- **Description**: VLA systems validated in Isaac Sim fail on real hardware due to sensor noise (microphone, cameras), actuation delays, dynamics mismatch, causing >50% drop in success rate
- **Impact**: High (limits practical deployment, requires significant real-world fine-tuning)
- **Mitigation**:
  - Set realistic expectations: Sim2Real requires iterative refinement, 10-30% degradation is expected on first deployment
  - Provide Sim2Real checklist: sensor calibration, latency modeling, domain randomization in simulation, conservative action execution speeds
  - Recommend real-world testing protocols: controlled environment first (low risk), human supervision, emergency stop procedures
  - Document Sim2Real case studies from research (NVIDIA, Boston Dynamics, Google robotics) showing iterative improvement cycles
  - Offer optional real hardware module or follow-up course for students with access to physical robots

**Risk 6: LLM Latency Exceeding Real-Time Requirements**
- **Description**: GPT-4 API calls take 3-10 seconds, violating <1 second user expectation for responsive HRI, leading to poor user experience
- **Impact**: Medium (frustrates users, makes robots feel unresponsive)
- **Mitigation**:
  - Optimize prompts for brevity (shorter system prompts, fewer few-shot examples) to reduce token count
  - Use faster models (GPT-4 Turbo, Claude 3.5 Sonnet) or local LLaMA 8B for <1 second inference on GPU
  - Implement streaming responses (parse partial LLM outputs to start action execution before full response)
  - Provide progress indicators (audio feedback, LED lights) to signal robot is "thinking"
  - Design tasks tolerant of latency (planning complex multi-step tasks allows 5-10 second planning time, simple commands require <1 second)
  - Cache frequent commands or use template-based parsers for common intents (bypass LLM for "stop", "yes", "no")

### Pedagogical Risks

**Risk 7: Students Overwhelmed by Multi-Disciplinary Integration**
- **Description**: Module 4 requires synthesizing knowledge from speech processing, NLP, LLMs, ROS 2, perception, navigation, manipulation—students struggle to integrate so many domains
- **Impact**: High (high dropout rate, low Capstone completion, negative course evaluations)
- **Mitigation**:
  - Scaffold learning with incremental complexity: Chapter 1 (Whisper only), Chapter 2 (LLM only), Chapter 3 (full integration)
  - Provide modular code templates for each component (Whisper ROS 2 node, LLM planner node, action executor node) that students customize rather than build from scratch
  - Offer office hours, discussion forums, and peer study groups for collaborative problem-solving
  - Create troubleshooting guides for common integration issues (ROS 2 node communication failures, LLM API errors, Isaac Sim crashes)
  - Reduce Capstone scope for struggling students (allow 3-task minimum instead of 10-task requirement)

**Risk 8: Insufficient Capstone Development Time**
- **Description**: Students underestimate Capstone complexity, run out of time to complete integration, testing, and documentation (20-30 hour estimate proves too optimistic)
- **Impact**: Medium (incomplete projects, rushed documentation, low-quality demos)
- **Mitigation**:
  - Provide realistic time estimates with buffer (25-40 hours recommended including debugging)
  - Enforce milestone deadlines (week 1: architecture design, week 2: component integration, week 3: testing, week 4: documentation)
  - Offer Capstone project templates with pre-integrated components (students focus on customization and testing rather than full implementation)
  - Encourage early start (introduce Capstone requirements in Chapter 1, allow parallel development during Chapters 2-3)
  - Provide partial credit rubric (students can pass with MVP even if advanced features incomplete)

**Risk 9: Ethical and Safety Concerns Underemphasized**
- **Description**: Students focus on technical implementation, neglect safety mechanisms, privacy considerations, or ethical implications of autonomous robots
- **Impact**: Medium (unsafe real-world deployments, privacy violations, unethical AI use cases)
- **Mitigation**:
  - Dedicate mandatory section in each chapter on ethics and safety (voice data privacy, LLM transparency, robot safety mechanisms)
  - Require Capstone reports to include section on ethical considerations and safety design
  - Provide case studies of robotics failures (accidents, privacy breaches, bias incidents) and lessons learned
  - Integrate safety checks into rubric (emergency stop implementation, user confirmation for risky actions, data anonymization)
  - Offer optional ethics module or guest lectures from robotics ethicists

### Resource and Accessibility Risks

**Risk 10: Limited Access to NVIDIA GPUs for Global Student Population**
- **Description**: Students in regions with limited cloud GPU access or unable to afford cloud rentals cannot complete GPU-intensive tasks (Whisper large model, local LLM inference, Isaac Sim)
- **Impact**: High (accessibility barrier, course inequity)
- **Mitigation**:
  - Provide comprehensive CPU fallback instructions (Whisper base on CPU, GPT-4 API avoids local GPU need, Isaac Sim can run in headless mode on CPU with reduced performance)
  - Partner with cloud providers for educational credits (AWS Educate, Google Cloud for Education, NVIDIA LaunchPad)
  - Offer institutional GPU clusters or remote lab access for students without personal GPUs
  - Create "lightweight" Capstone track using CPU-only workflows and smaller environments
  - Provide pre-recorded video demos and virtual lab tours for students unable to run full stack

**Risk 11: Language Barrier for Non-English Speaking Students**
- **Description**: Course content primarily in English, limiting accessibility for international students; LLM examples assume English commands
- **Impact**: Medium (reduces global reach, less inclusive)
- **Mitigation**:
  - Leverage Whisper's multilingual support to provide examples in Spanish, Mandarin, French, German, Arabic
  - Design LLM prompts to work in multiple languages (GPT-4 and Claude are multilingual)
  - Translate key course materials (chapter summaries, exercise instructions) into 5+ languages
  - Provide subtitles and transcripts for video content
  - Encourage student community translations of course materials (incentivize with course credit or certificates)

## Evaluation and Validation

### Content Quality Validation

- **Technical Accuracy Review**: All tutorial code, command examples, and theoretical explanations reviewed by 2+ domain experts (speech processing, NLP, robotics) before publication
- **Code Testing**: All code examples (Whisper ROS 2 nodes, LLM integrations, Capstone templates) tested on 3 hardware configurations (RTX 2060, RTX 3080, AWS G4 instance) to validate cross-platform compatibility
- **Performance Benchmarking**: Document quantitative performance metrics for all examples (Whisper WER on LibriSpeech, LLM planning accuracy on custom test set, Capstone success rates) to set realistic student expectations
- **Pedagogical Review**: Content reviewed by instructional designers and educators for clarity, scaffolding, exercise difficulty progression, and alignment with learning objectives

### Student Learning Assessment

- **Chapter Quizzes**: Each chapter includes 10-15 multiple-choice and short-answer questions covering theory, best practices, and troubleshooting (pass threshold: 80%)
- **Hands-On Exercises**: Each chapter includes 5-8 coding exercises with auto-graded components (unit tests for ROS 2 nodes, expected outputs for Whisper transcription) and human-reviewed components (LLM prompt quality, architecture diagrams)
- **Capstone Project Rubric**: Detailed rubric (40% implementation, 30% performance metrics, 20% documentation, 10% innovation) with clear grading criteria and example projects demonstrating each scoring level
- **Peer Review**: Students review 2 peer Capstone projects using rubric, providing feedback and scoring (calibrated against instructor grading to ensure consistency)
- **Portfolio Artifacts**: Students produce portfolio-ready artifacts (video demos, technical reports, GitHub repositories) that can be shared with employers or graduate schools

### Continuous Improvement Mechanisms

- **Student Feedback Surveys**: Post-chapter surveys (5 Likert-scale questions + open-ended feedback) collect data on content clarity, exercise difficulty, time estimates, and satisfaction
- **Usage Analytics**: Track common failure points (high error rates in specific exercises, long completion times, forum question clusters) to identify content gaps or unclear instructions
- **Office Hours and Forum Analysis**: Analyze recurring student questions in office hours and discussion forums to prioritize FAQ updates and supplementary tutorials
- **Industry Advisory Board**: Annual review by industry professionals (robotics engineers from Tesla, Boston Dynamics, NVIDIA, etc.) to ensure content reflects current industry practices and job market needs
- **Research Paper Integration**: Yearly updates to incorporate latest research in VLA (new foundation models like RT-X, OpenVLA, Octo), LLM advances (GPT-5, Claude 4), and Whisper improvements

### Success Metrics for Course

- **Completion Rate**: >70% of enrolled students complete all Module 4 chapters and Capstone Project
- **Capstone Quality**: >50% of Capstone projects achieve "portfolio-quality" rating (suitable for showcasing to employers)
- **Employment Outcomes**: Track alumni employment in robotics roles (Physical AI engineer, ML robotics engineer, autonomy engineer) 6 months post-graduation (target: >30% placement in robotics companies or PhD programs)
- **Student Satisfaction**: Average rating >4.0/5.0 for Module 4 content on post-course evaluations
- **Community Contributions**: >10 student projects open-sourced and shared with community (GitHub stars, forks, citations in external projects)

## Further Reading and Resources

### Whisper and Speech Recognition

- **OpenAI Whisper Paper**: Radford et al., "Robust Speech Recognition via Large-Scale Weak Supervision" (2022) - https://arxiv.org/abs/2212.04356
- **Whisper Documentation**: Official OpenAI Whisper GitHub - https://github.com/openai/whisper
- **ASR Fundamentals**: "Speech and Language Processing" (Jurafsky & Martin, 3rd ed., Chapter 16) - https://web.stanford.edu/~jurafsky/slp3/
- **Multilingual ASR**: "Massively Multilingual ASR: 50 Languages, 1 Model, 1 Billion Parameters" (Pratap et al., 2020) - https://arxiv.org/abs/2007.03001
- **ROS 2 Audio**: audio_common package documentation - https://github.com/ros-drivers/audio_common

### Large Language Models and Robotics

- **GPT-4 Technical Report**: OpenAI (2023) - https://arxiv.org/abs/2303.08774
- **LLaMA 3 Paper**: Meta AI (2024) - https://ai.meta.com/blog/llama-3/
- **Claude 3.5 Model Card**: Anthropic - https://www.anthropic.com/claude
- **SayCan (LLMs for Robotics)**: "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (Ahn et al., 2022) - https://arxiv.org/abs/2204.01691
- **Code as Policies**: "Code as Policies: Language Model Programs for Embodied Control" (Liang et al., 2023) - https://arxiv.org/abs/2209.07753
- **RT-2 (Vision-Language-Action)**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (Brohan et al., 2023) - https://arxiv.org/abs/2307.15818
- **PaLM-E**: "PaLM-E: An Embodied Multimodal Language Model" (Driess et al., 2023) - https://arxiv.org/abs/2303.03378
- **Prompt Engineering Guide**: OpenAI Best Practices - https://platform.openai.com/docs/guides/prompt-engineering
- **Anthropic Prompt Engineering**: Claude prompt engineering documentation - https://docs.anthropic.com/claude/docs/prompt-engineering

### Vision-Language Models

- **GPT-4 Vision**: OpenAI GPT-4V System Card - https://cdn.openai.com/papers/GPTV_System_Card.pdf
- **LLaVA**: "Visual Instruction Tuning" (Liu et al., 2023) - https://arxiv.org/abs/2304.08485
- **Flamingo**: "Flamingo: a Visual Language Model for Few-Shot Learning" (Alayrac et al., 2022) - https://arxiv.org/abs/2204.14198
- **CLIP**: "Learning Transferable Visual Models From Natural Language Supervision" (Radford et al., 2021) - https://arxiv.org/abs/2103.00020

### Robotics and VLA Systems

- **OpenVLA**: "OpenVLA: An Open-Source Vision-Language-Action Model" (Kim et al., 2024) - https://openvla.github.io/
- **RT-X (Robotic Transformer X)**: "Open X-Embodiment: Robotic Learning Datasets and RT-X Models" (Open X-Embodiment Collaboration, 2023) - https://arxiv.org/abs/2310.08864
- **Octo Model**: "Octo: An Open-Source Generalist Robot Policy" (Ghosh et al., 2024) - https://octo-models.github.io/
- **ROS 2 Documentation**: Official ROS 2 Humble documentation - https://docs.ros.org/en/humble/
- **NVIDIA Isaac Sim**: Official documentation - https://docs.omniverse.nvidia.com/isaacsim/latest/index.html
- **Nav2 Documentation**: ROS 2 Navigation Stack - https://navigation.ros.org/

### Ethics and Safety

- **IEEE Ethically Aligned Design**: Guidelines for autonomous systems - https://ethicsinaction.ieee.org/
- **ISO 13482**: Safety standard for personal care robots
- **Responsible AI Principles**: NVIDIA AI Ethics guidelines - https://www.nvidia.com/en-us/about-nvidia/ethics/
- **Privacy in Robotics**: "Privacy in Human-Robot Interaction: A Survey" (Rueben et al., 2018)

### Tools and Frameworks

- **Ollama**: Local LLM deployment - https://ollama.ai/
- **vLLM**: Fast LLM inference server - https://github.com/vllm-project/vllm
- **HuggingFace Transformers**: LLM library - https://huggingface.co/docs/transformers
- **Pydantic**: Data validation for structured outputs - https://docs.pydantic.dev/
- **ReSpeaker**: USB microphone arrays - https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/
- **Weights & Biases**: ML experiment tracking - https://wandb.ai/

---

**End of Specification**
