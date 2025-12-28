# Data Model: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

**Feature**: Module 3 - The AI-Robot Brain
**Created**: 2025-12-09
**Branch**: `001-isaac-ai-brain`

## Purpose

This document defines the data models and schemas used across Module 3 content generation. All specialized subagents (IsaacSimAgent, PerceptionAgent, RLTrainingAgent, Nav2IntegrationAgent) MUST adhere to these models when generating chapter content and code examples.

---

## Chapter Structure Model

All 4 chapters MUST follow this 9-section structure per Constitution Content Standards:

```yaml
ChapterStructure:
  metadata:
    chapter_number: integer  # 1-4
    chapter_title: string    # e.g., "NVIDIA Isaac Sim Setup and Simulation Integration"
    slug: string             # e.g., "chapter-1-isaac-sim-setup"
    estimated_time: string   # e.g., "6-8 hours"
    prerequisites: list[string]  # e.g., ["Module 1 ROS 2 fundamentals", "Module 2 digital twin basics"]

  sections:
    - name: "Overview"
      required: true
      content: |
        - 2-3 paragraph introduction to chapter topic
        - Connection to previous modules/chapters
        - Learning outcomes (3-5 bullet points)
        - Hardware/software requirements
        - Estimated completion time

    - name: "Learning Objectives"
      required: true
      content: |
        - 3-5 specific, measurable learning objectives
        - Format: "Students will be able to..."
        - Must align with functional requirements (FR-XXX)

    - name: "Theoretical Foundations"
      required: true
      content: |
        - Core concepts and theory (e.g., PhysX physics, RL MDP formulation, Nav2 path planning algorithms)
        - Mathematical formulations using KaTeX/LaTeX ($$equation$$)
        - Architecture diagrams using Mermaid or high-quality images
        - References to academic papers or technical documentation
      min_length: 500 words

    - name: "Hands-On Implementation"
      required: true
      subsections:
        - "Installation and Setup"
        - "Configuration and Tuning"
        - "Step-by-Step Tutorial"
        - "Validation and Testing"
      content: |
        - Detailed installation tutorials with verification steps
        - Configuration file explanations (YAML, JSON, XML)
        - Screenshots at each major step
        - Command-line examples with expected outputs
      min_code_blocks: 5
      min_screenshots: 10

    - name: "Practical Examples"
      required: true
      min_examples: 3
      content: |
        - 3+ executable code examples from examples/module-3/chapter-N/
        - Each example includes: title, difficulty (beginner/intermediate/advanced), description, files, README
        - Examples demonstrate progressive complexity
        - All examples MUST be tested and verified on target hardware (RTX 2060+ GPU)

    - name: "Exercises"
      required: true
      min_exercises: 5
      difficulty_distribution:
        beginner: 2
        intermediate: 2
        advanced: 1
      content: |
        - 5+ exercises with clear acceptance criteria
        - 2 beginner: Focus on basic concepts and tool familiarization
        - 2 intermediate: Apply concepts to novel scenarios
        - 1 advanced: Open-ended challenge requiring creativity and integration
        - Each exercise includes: title, difficulty, objective, acceptance criteria, hints

    - name: "Further Reading"
      required: true
      min_references: 5
      content: |
        - 5+ curated references (documentation, papers, tutorials)
        - Format: - [Title](URL) - Brief description of focus area
        - Mix of official documentation, research papers, and community tutorials
        - Prioritize NVIDIA official docs, ROS 2 docs, and peer-reviewed papers

    - name: "Summary"
      required: true
      content: |
        - 200-300 word recap of chapter
        - Key skills learned (bullet list)
        - Importance of topic in broader context (humanoid robotics, Physical AI)
        - Preview of next chapter or module
      min_length: 200 words
      max_length: 350 words

    - name: "Troubleshooting"
      required: true
      min_issues: 5
      content: |
        - 5+ common issues with solutions
        - Format for each: Symptom → Cause → Solution
        - Include parameter tuning guidance where applicable
        - Cover hardware issues (GPU drivers, CUDA), software issues (dependencies, versioning), and usage errors
```

---

## Code Example Models

### Isaac Sim Example Model

```yaml
IsaacSimExample:
  id: string           # e.g., "01-hello-isaac-sim"
  title: string        # e.g., "Hello Isaac Sim - Basic Simulation Script"
  type: "isaac_sim"
  difficulty: enum     # "beginner" | "intermediate" | "advanced"

  files:
    python:
      - path: string   # e.g., "hello_isaac_sim.py"
        description: string
        min_lines: 20
    usd:               # Optional
      - path: string   # e.g., "scene.usd"
        description: string
    launch:            # Optional
      - path: string   # e.g., "launch_sim.sh"
        description: string

  readme:
    prerequisites:
      - software: list[string]  # e.g., ["Isaac Sim 2023.1+", "Python 3.10+"]
      - hardware: list[string]  # e.g., ["NVIDIA RTX 2060+ GPU", "16GB RAM"]

    installation_steps: list[string]

    execution_steps: list[string]

    expected_output:
      description: string
      success_criteria: list[string]

    troubleshooting:
      - symptom: string
        cause: string
        solution: string

  testing_criteria:
    - criterion: string
      validation_method: string

  hardware_requirements:
    min_gpu: string     # e.g., "RTX 2060 (6GB VRAM)"
    min_ram: string     # e.g., "16GB"
    min_cpu_cores: integer

  estimated_time: string  # e.g., "15-30 minutes"

  performance_targets:   # Optional
    rtf: float           # Real-Time Factor ≥1.0
    fps: integer         # Frames per second
```

### Isaac ROS Example Model

```yaml
IsaacROSExample:
  id: string
  title: string
  type: "isaac_ros"
  difficulty: enum     # "beginner" | "intermediate" | "advanced"

  files:
    launch:
      - path: string   # e.g., "launch/stereo_depth.launch.py"
        description: string
    yaml:
      - path: string   # e.g., "config/stereo_params.yaml"
        description: string
    rviz:              # Optional
      - path: string   # e.g., "config/visualization.rviz"
        description: string
    docker:            # Optional
      - dockerfile: string
      - docker_compose: string

  readme:
    prerequisites:
      - software: list[string]  # e.g., ["Isaac Sim 2023.1+", "ROS 2 Humble", "Isaac ROS packages"]
      - hardware: list[string]  # e.g., ["NVIDIA RTX 3080+ GPU", "CUDA 11.8+"]

    installation_steps: list[string]

    execution_steps: list[string]

    expected_output:
      description: string
      ros2_topics:
        - topic: string
          message_type: string
          expected_rate: string  # e.g., ">30 Hz"
      success_criteria: list[string]

    troubleshooting:
      - symptom: string
        cause: string
        solution: string

  testing_criteria:
    - criterion: string
      validation_method: string

  performance_targets:
    stereo_depth_fps: integer    # e.g., >30 FPS
    vslam_latency_ms: integer    # e.g., <50 ms
    dnn_inference_fps: integer   # e.g., >30 FPS
```

### RL Training Example Model

```yaml
RLTrainingExample:
  id: string
  title: string
  type: "rl_training"
  difficulty: enum     # "beginner" | "intermediate" | "advanced"

  files:
    training_script:
      - path: string   # e.g., "train_humanoid_walk.py"
        description: string
    environment:       # Custom Isaac Lab environment
      - path: string   # e.g., "env/humanoid_walk_env.py"
        description: string
    config:
      - path: string   # e.g., "config.yaml"
        description: string
    checkpoints:       # Pre-trained policies
      - path: string   # e.g., "checkpoints/walk_policy_final.pth"
        description: string
        file_size_mb: integer

  readme:
    prerequisites:
      - software: list[string]  # e.g., ["Isaac Lab", "PyTorch 2.0+", "CUDA 11.8+"]
      - hardware: list[string]  # e.g., ["NVIDIA RTX 3080 GPU (10GB+ VRAM)", "32GB RAM"]

    installation_steps: list[string]

    execution_steps:
      training: list[string]
      evaluation: list[string]

    expected_output:
      description: string
      training_metrics:
        - metric: string       # e.g., "episode_reward"
          target_value: string # e.g., ">100 after 10K episodes"
      success_criteria: list[string]

    troubleshooting:
      - symptom: string
        cause: string
        solution: string

  task_description:
    observation_space:
      description: string
      dimensions: integer
      components: list[string]
    action_space:
      description: string
      dimensions: integer
      type: enum  # "continuous" | "discrete"
    reward_function:
      description: string
      components: list[string]
    termination_conditions: list[string]

  training_config:
    algorithm: string           # e.g., "PPO"
    hyperparameters:
      learning_rate: float
      batch_size: integer
      n_parallel_envs: integer
      training_episodes: integer
    domain_randomization:
      - parameter: string
        range: string

  performance_targets:
    success_rate: float         # e.g., >0.70 (70%)
    training_time_hours: float  # e.g., <4 hours on RTX 3080
    convergence_episodes: integer
```

### Nav2 Integration Example Model

```yaml
Nav2Example:
  id: string
  title: string
  type: "nav2_integration"
  difficulty: enum     # "beginner" | "intermediate" | "advanced"

  files:
    launch:
      - path: string   # e.g., "launch/nav2_bringup.launch.py"
        description: string
    yaml:
      - path: string   # e.g., "config/costmap_params.yaml"
        description: string
    behavior_tree:     # Optional
      - path: string   # e.g., "behavior_trees/custom_navigate.xml"
        description: string
    rviz:
      - path: string   # e.g., "config/nav2_visualization.rviz"
        description: string

  readme:
    prerequisites:
      - software: list[string]  # e.g., ["Nav2", "Isaac Sim", "Isaac ROS"]
      - hardware: list[string]

    installation_steps: list[string]

    execution_steps: list[string]

    expected_output:
      description: string
      navigation_metrics:
        - metric: string       # e.g., "goal_reach_success_rate"
          target_value: string # e.g., ">80% over 10 trials"
      success_criteria: list[string]

    troubleshooting:
      - symptom: string
        cause: string
        solution: string

  nav2_configuration:
    costmap_layers: list[string]
    global_planner: string
    local_controller: string
    recovery_behaviors: list[string]

  humanoid_adaptations:
    - adaptation: string
      rationale: string
      parameters: dict

  performance_targets:
    goal_reach_success_rate: float  # e.g., >0.80
    path_planning_time_ms: integer  # e.g., <1000 ms
    control_frequency_hz: integer   # e.g., 10-20 Hz
```

---

## File Organization Model

```yaml
FileOrganization:
  textbook_structure:
    base_path: "docs/textbook/module-3-ai-robot-brain/"
    chapters:
      - chapter_number: 1
        directory: "chapter-1-isaac-sim-setup/"
        files:
          - "index.md"           # Chapter overview
          - "1-1-architecture.md"
          - "1-2-installation.md"
          - "1-3-urdf-to-usd.md"
          - "1-4-scene-creation.md"
          - "1-5-ros2-integration.md"
          - "1-6-python-scripting.md"
          - "1-7-exercises.md"

  examples_structure:
    base_path: "examples/module-3/"
    chapters:
      - chapter_number: 1
        directory: "chapter-1-isaac-sim/"
        subdirectories:
          - "01-hello-isaac-sim/"
          - "02-urdf-import/"
          - "03-urdf-to-usd-batch/"
          - "04-custom-scene/"
          - "05-physics-config/"
          - "06-rtx-lighting/"
          - "07-pbr-materials/"
          - "08-ros2-bridge-basic/"
          - "09-ros2-control/"
          - "10-python-automation/"

  static_assets_structure:
    base_path: "static/img/module-3/"
    subdirectories:
      - "isaac-sim-screenshots/"
      - "isaac-ros-rviz/"
      - "rl-training-plots/"
      - "nav2-visualizations/"
```

---

## Validation Rules

### Chapter Content Validation

```yaml
ChapterValidation:
  mdx_files:
    - All MDX files MUST have front matter with title and description
    - Code blocks MUST specify language for syntax highlighting
    - Images MUST have alt text for accessibility
    - Internal links MUST use relative paths
    - External links MUST include descriptions

  code_examples:
    - All Python scripts MUST include docstrings
    - All configuration files MUST include comments
    - README files MUST follow example model structure
    - All examples MUST be tested on target hardware

  performance_requirements:
    isaac_sim:
      - RTF ≥1.0 in headless mode
      - Examples run without errors on RTX 2060+
    isaac_ros:
      - Stereo depth >30 FPS
      - VSLAM latency <50 ms
      - DNN inference >30 FPS
    rl_training:
      - Policy convergence >70% success within documented time
      - Training completes within estimated hours on RTX 3080
    nav2:
      - Goal-reach success >80% over 10 trials
      - Path planning <1 second

  accessibility:
    - All images MUST have descriptive alt text
    - Headings MUST follow hierarchical structure (h1 → h2 → h3)
    - Code examples MUST have clear descriptions
    - Color-coded visualizations MUST also use patterns/labels
```

---

## Constitution Compliance

This data model enforces:

- **Principle I (Technical Accuracy & Depth)**: Requires theoretical foundations, performance validation, and tested code examples
- **Principle II (Modular Structure Integrity)**: 9-section chapter structure, organized file hierarchy
- **Principle III (AI-Native Development)**: Structured YAML models enable AI agent code generation
- **Principle IV (Professional UI/UX Standards)**: Accessibility requirements, screenshot guidelines, clear documentation
- **Principle V (Free-Tier Infrastructure)**: Examples tested on minimum hardware (RTX 2060), cloud GPU guidance

---

**Status**: Ready for specialized subagent consumption
**Usage**: All IsaacSimAgent, PerceptionAgent, RLTrainingAgent, Nav2IntegrationAgent MUST reference this data model when generating content
