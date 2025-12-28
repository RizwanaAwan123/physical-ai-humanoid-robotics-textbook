# Implementation Plan: Module 2 - Digital Twin Simulation (Gazebo & Unity)

**Branch**: `002-module-2-digital-twin` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-module-2-digital-twin/spec.md`

**Note**: This plan details the implementation approach for Module 2's four chapters using specialized Claude Code Subagents to generate educational content for Digital Twin simulation.

## Summary

Module 2 - Digital Twin Simulation is the second educational module for the Physical AI & Humanoid Robotics Textbook, covering four distinct chapters: (1) Gazebo Physics Simulation, (2) Unity Rendering & Human-Robot Interaction, (3) Sensor Simulation (LiDAR, Depth Cameras, IMUs), and (4) Gazebo-Unity Integration for Physical AI workflows.

**Technical Approach**: Content will be generated using four specialized Claude Code Subagents (GazeboPhysicsAgent, UnityRenderingAgent, SensorSimulationAgent, IntegrationAgent), each responsible for one chapter. Content is delivered in Docusaurus MDX format with executable simulation examples tested in Gazebo Classic 11/Fortress and Unity 2022 LTS. Physics simulations validated against real-world benchmarks, rendering examples achieve photorealistic quality, and sensor models conform to ROS 2 message standards. All content adheres to Constitution Principles I (Technical Accuracy & Depth), II (Modular Structure Integrity), III (AI-Native Development), and IV (Professional UI/UX Standards).

## Technical Context

**Language/Version**:
- Content: Markdown (MDX for Docusaurus)
- Gazebo Code Examples: Python 3.10+ (ROS 2 launch files), XML/SDF (world files, model configs)
- Unity Code Examples: C# 10.0+ (Unity 2022 LTS scripts)
- ROS 2 Integration: Python 3.10+ (rclpy nodes), C++17 (optional rclcpp examples)
- Math: LaTeX (KaTeX rendering)
- Diagrams: Mermaid syntax

**Primary Dependencies**:
- Docusaurus 3.x (@docusaurus/preset-classic)
- Gazebo Classic 11 or Gazebo Fortress/Garden (target environment for physics simulation)
- Unity 2022 LTS (target environment for rendering and interaction)
- ROS 2 Humble Hawksbill (integration layer, sensor data, robot state)
- remark-math, rehype-katex (math rendering)
- @docusaurus/theme-mermaid (diagram rendering)
- Docker (osrf/ros:humble-desktop, custom Gazebo testing containers)

**Storage**:
- Git repository (content versioning)
- File-based (MDX files, Gazebo world/SDF files, Unity C# scripts, launch files)
- N/A (no database for Module 2 content)

**Testing**:
- Docker-based Gazebo example validation (osrf/ros:humble-desktop + Gazebo Classic 11)
- Unity build validation (Unity 2022 LTS on Windows 11 and Ubuntu 22.04)
- Gazebo RTF (Real-Time Factor) performance testing
- Unity FPS performance testing
- Integration latency testing (Gazebo-Unity ROS 2 bridge)
- Manual QA (readability, accessibility, cross-references, screenshot accuracy)

**Target Platform**:
- Web (Vercel or GitHub Pages deployment for Docusaurus site)
- Student Gazebo environment: Ubuntu 22.04/24.04 with Gazebo Classic 11 or Gazebo Fortress/Garden
- Student Unity environment: Windows 10/11 or Ubuntu 22.04/24.04 with Unity 2022 LTS

**Project Type**:
- Documentation site (Docusaurus-based textbook) with simulation examples

**Performance Goals**:
- Gazebo RTF ≥ 0.9 (90% of real-time) on reference hardware (8GB RAM, 4-core CPU)
- Unity FPS ≥30 on reference hardware (integrated graphics, URP pipeline, 1080p)
- Gazebo-Unity integration latency <100ms (physics update to visual rendering)
- Page load time: <2 seconds (SC-016)
- Lighthouse score: 90+ (SC-016)
- Module 2 build time: <5 minutes

**Constraints**:
- Gazebo Classic 11 compatibility (widely adopted, stable) with migration notes for Gazebo Fortress/Garden
- Unity 2022 LTS compatibility with forward compatibility notes for Unity 6
- Free-tier deployment (Vercel/GitHub Pages)
- CPU-only simulation examples (no GPU requirement for core learning, GPU optional for acceleration)
- ROS 2 Humble compatibility (LTS, supported until 2027)
- Cross-platform: Linux (Gazebo primary), Windows/Linux/macOS (Unity)
- Flesch-Kincaid grade level: 12-16 (SC-008)
- WCAG 2.1 AA accessibility compliance

**Scale/Scope**:
- 4 chapters (Module 2 only)
- 30-60 Gazebo simulation examples (worlds, SDF models, launch files)
- 20-40 Unity project examples (C# scripts, scenes, prefabs)
- 15-25 ROS 2 integration examples (sensor plugins, bridge configurations)
- 50,000-80,000 words total (12,000-20,000 per chapter)
- Estimated student learning time: 24-32 hours

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Technical Accuracy & Depth ✅ PASS

- [x] All chapters require executable simulation examples (FR-002 to FR-009 for Gazebo, FR-010 to FR-019 for Unity, FR-020 to FR-028 for Sensors, FR-029 to FR-039 for Integration)
- [x] Theoretical foundations mandated for all chapters (rigid body dynamics, PBR rendering, sensor physics, integration architectures)
- [x] Progressive exercises from beginner to advanced (FR-007, FR-018, FR-027, FR-037)
- [x] Mathematical formulations required (SC-004: physics equations, sensor noise models, rendering equations)
- [x] Simulations tested in isolated environments (Docker for Gazebo, clean Unity projects)
- [x] Physics parameters validated against real-world benchmarks (SC-020, SC-021)

**Status**: PASS - All functional requirements enforce technical depth per Principle I. Physics engines (ODE, Bullet, DART) and rendering pipelines (URP, HDRP) require rigorous understanding.

### Principle II: Modular Structure Integrity ✅ PASS (NON-NEGOTIABLE)

- [x] Exactly 4 chapters specified (Chapter 1: Gazebo Physics, Chapter 2: Unity Rendering, Chapter 3: Sensor Simulation, Chapter 4: Integration)
- [x] Fixed sequence: Physics → Rendering → Sensors → Integration
- [x] No additions or removals to 4-chapter structure
- [x] Each chapter independently testable (user stories P1-P4 with acceptance scenarios)
- [x] Aligns with Module 2 in overall 4-module textbook structure

**Status**: PASS - Specification strictly adheres to 4-chapter Module 2 structure. No deviations.

### Principle III: AI-Native Development & Spec-Driven Approach ✅ PASS

- [x] Specification created via `/sp.specify` (spec.md exists)
- [x] Planning via `/sp.plan` (this document being created)
- [x] Claude Subagents explicitly designed (4 specialized agents: GazeboPhysicsAgent, UnityRenderingAgent, SensorSimulationAgent, IntegrationAgent)
- [x] Implementation to follow `/sp.tasks` → `/sp.implement` workflow
- [x] PHRs documented (constitution requirement)

**Status**: PASS - Plan mandates use of 4 specialized Claude Code Subagents, aligning with bonus criteria. Each agent has domain expertise (Gazebo physics, Unity rendering, sensor modeling, platform integration).

### Principle IV: Professional UI/UX Standards ✅ PASS

- [x] Custom Docusaurus theme inherited from Module 1 (Indigo/Amber color palette)
- [x] Responsive design required (SC-017: 320px-1920px viewports)
- [x] Syntax highlighting configured (Prism.js: Python, C#, XML, YAML, SDF)
- [x] Mathematical notation via KaTeX (LaTeX for physics equations)
- [x] Mermaid diagrams for visual hierarchy (simulation architectures, data flow)
- [x] Screenshots required for all tutorials (NFR-006, SC-018, SC-019)
- [x] Accessibility compliance (SC-009: alt text, WCAG 2.1 AA)

**Status**: PASS - Technical stack and success criteria enforce professional UI/UX. Module 2 builds on Module 1's established design system.

### Content Standards Compliance ✅ PASS

- [x] All chapters require 9 content sections: Overview, Learning Objectives, Theoretical Foundations, Hands-On Implementation, Practical Examples, Exercises, Further Reading, Summary, Troubleshooting (FR-040)
- [x] Code Quality Standards enforced via Docker/Unity testing and validation (FR-043, FR-044, NFR-004, NFR-005)
- [x] Content Review Gates defined (SC-006 to SC-010: 100% simulation success, proper formatting, inline comments)
- [x] Cross-references to Module 1 required (FR-041: URDF from Module 1 Chapter 3, ROS 2 topics from Module 1 Chapter 1)

**Status**: PASS - data-model.md will enforce all 9 required sections plus cross-referencing structure.

### Free-Tier Infrastructure Optimization ⚠️ NOT APPLICABLE (Module 2)

- **Status**: NOT APPLICABLE - Module 2 has no backend services. This principle applies to RAG chatbot features (separate from educational content).

### RAG System Content Fidelity ⚠️ NOT APPLICABLE (Module 2)

- **Status**: NOT APPLICABLE - RAG chatbot is a separate feature. Module 2 provides source content for RAG system embeddings.

**Overall Constitution Check**: ✅ **PASS** - All applicable principles satisfied, no violations to justify.

## Project Structure

### Documentation (this feature)

```text
specs/002-module-2-digital-twin/
├── spec.md                          # Feature specification (User input: requirements)
├── plan.md                          # This file (/sp.plan command output)
├── research.md                      # Phase 0 output: Design decisions and research findings
├── data-model.md                    # Phase 1 output: Content structure schema for Module 2
├── quickstart.md                    # Phase 1 output: Developer guide for adding/modifying chapters
├── contracts/                       # Phase 1 output: Subagent input/output contracts
│   └── chapter-content-contract.md  # Chapter delivery expectations for all 4 chapters
├── checklists/                      # Validation checklists
│   └── requirements.md              # Spec validation (created by /sp.specify, validated)
└── tasks.md                         # Phase 2 output: Actionable tasks (created by /sp.tasks - NOT by /sp.plan)
```

### Source Code (repository root)

**Structure Decision**: This is a Docusaurus documentation site with simulation examples. The structure follows Docusaurus conventions with educational content (docs/) and executable examples (examples/) for both Gazebo and Unity.

```text
# Documentation Content (Docusaurus docs plugin)
docs/
├── module-2/                        # Module 2: Digital Twin Simulation
│   ├── _diagrams/                   # Shared diagram assets
│   │   ├── chapter-1/               # Chapter 1 Mermaid diagrams (physics engine architecture, collision pipeline)
│   │   ├── chapter-2/               # Chapter 2 Mermaid diagrams (Unity rendering pipeline, UI architecture)
│   │   ├── chapter-3/               # Chapter 3 Mermaid diagrams (sensor architectures, data flow)
│   │   └── chapter-4/               # Chapter 4 Mermaid diagrams (integration patterns, ROS 2 bridge architecture)
│   ├── _screenshots/                # Tutorial screenshots and expected outputs
│   │   ├── chapter-1/               # Gazebo GUI screenshots, physics visualizations
│   │   ├── chapter-2/               # Unity Editor screenshots, rendered scenes
│   │   ├── chapter-3/               # Sensor data visualizations (point clouds, depth images)
│   │   └── chapter-4/               # Integration examples (synchronized Gazebo-Unity)
│   ├── chapter-1.mdx                # Chapter 1: Gazebo Physics Simulation
│   ├── chapter-2.mdx                # Chapter 2: Unity Rendering & Human-Robot Interaction
│   ├── chapter-3.mdx                # Chapter 3: Sensor Simulation (LiDAR, Depth Cameras, IMUs)
│   └── chapter-4.mdx                # Chapter 4: Gazebo-Unity Integration Best Practices
├── module-1/                        # Existing: Module 1 ROS 2 Fundamentals
├── intro.md                         # Existing: Course introduction
└── index.mdx                        # Homepage

# Executable Gazebo Simulation Examples
examples/
└── module-2/
    ├── chapter-1/                   # Gazebo Physics Simulation examples
    │   ├── README.md                # Chapter 1 examples overview
    │   ├── example-01-world-creation/
    │   │   ├── worlds/
    │   │   │   └── empty_world.world  # Basic SDF world file
    │   │   ├── launch/
    │   │   │   └── world.launch.py    # ROS 2 launch file
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-02-spawn-humanoid/
    │   │   ├── urdf/
    │   │   │   └── humanoid.urdf      # Humanoid URDF from Module 1
    │   │   ├── launch/
    │   │   │   └── spawn_robot.launch.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-03-collision-config/
    │   │   ├── models/
    │   │   │   └── obstacle.sdf       # SDF model with collision geometry
    │   │   ├── launch/
    │   │   │   └── collision_test.launch.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-04-physics-tuning/
    │   │   ├── worlds/
    │   │   │   └── physics_demo.world # World with custom physics parameters
    │   │   ├── launch/
    │   │   │   └── physics.launch.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-05-contact-forces/
    │   │   ├── scripts/
    │   │   │   └── apply_force.py     # ROS 2 node to apply forces
    │   │   ├── launch/
    │   │   │   └── force_demo.launch.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   └── test-all.sh              # Test all Chapter 1 examples
    ├── chapter-2/                   # Unity Rendering & Interaction examples
    │   ├── README.md                # Chapter 2 examples overview
    │   ├── example-01-unity-basics/
    │   │   ├── UnityBasicsProject/  # Unity 2022 LTS project
    │   │   │   ├── Assets/
    │   │   │   │   ├── Scenes/
    │   │   │   │   │   └── IntroScene.unity
    │   │   │   │   └── Scripts/
    │   │   │   ├── Packages/
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/         # Expected output images
    │   ├── example-02-urdf-import/
    │   │   ├── URDFImportProject/
    │   │   │   ├── Assets/
    │   │   │   │   ├── URDF/
    │   │   │   │   │   └── humanoid.urdf
    │   │   │   │   ├── Scripts/
    │   │   │   │   │   └── ArticulationConfig.cs
    │   │   │   │   └── Scenes/
    │   │   │   ├── Packages/
    │   │   │   │   └── manifest.json  # Unity URDF Importer package
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/
    │   ├── example-03-pbr-materials/
    │   │   ├── PBRRenderingProject/
    │   │   │   ├── Assets/
    │   │   │   │   ├── Materials/
    │   │   │   │   │   ├── Metal.mat
    │   │   │   │   │   └── Rubber.mat
    │   │   │   │   ├── HDRI/
    │   │   │   │   │   └── studio_env.hdr
    │   │   │   │   └── Scenes/
    │   │   │   ├── Packages/
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/
    │   ├── example-04-ui-interaction/
    │   │   ├── HRIProject/
    │   │   │   ├── Assets/
    │   │   │   │   ├── Scripts/
    │   │   │   │   │   ├── RobotController.cs
    │   │   │   │   │   └── UIManager.cs
    │   │   │   │   ├── UI/
    │   │   │   │   │   └── Canvas.prefab
    │   │   │   │   └── Scenes/
    │   │   │   ├── Packages/
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/
    │   └── test-all.md              # Manual test checklist (Unity projects require manual validation)
    ├── chapter-3/                   # Sensor Simulation examples
    │   ├── README.md                # Chapter 3 examples overview
    │   ├── example-01-gazebo-lidar/
    │   │   ├── models/
    │   │   │   └── robot_with_lidar.sdf
    │   │   ├── launch/
    │   │   │   └── lidar_sim.launch.py
    │   │   ├── scripts/
    │   │   │   └── visualize_pointcloud.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-02-gazebo-depth-camera/
    │   │   ├── models/
    │   │   │   └── robot_with_camera.sdf
    │   │   ├── launch/
    │   │   │   └── depth_camera_sim.launch.py
    │   │   ├── scripts/
    │   │   │   └── process_depth_image.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-03-gazebo-imu/
    │   │   ├── models/
    │   │   │   └── robot_with_imu.sdf
    │   │   ├── launch/
    │   │   │   └── imu_sim.launch.py
    │   │   ├── scripts/
    │   │   │   └── plot_imu_data.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-04-unity-lidar/
    │   │   ├── UnityLiDARProject/
    │   │   │   ├── Assets/
    │   │   │   │   ├── Scripts/
    │   │   │   │   │   └── LiDARRaycaster.cs
    │   │   │   │   └── Scenes/
    │   │   │   ├── Packages/
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/
    │   ├── example-05-unity-perception/
    │   │   ├── PerceptionProject/
    │   │   │   ├── Assets/
    │   │   │   │   ├── Scripts/
    │   │   │   │   │   └── PerceptionConfig.cs
    │   │   │   │   └── Scenes/
    │   │   │   ├── Packages/
    │   │   │   │   └── manifest.json  # Unity Perception package
    │   │   │   └── ProjectSettings/
    │   │   ├── README.md
    │   │   └── screenshots/
    │   ├── example-06-sensor-noise/
    │   │   ├── scripts/
    │   │   │   └── add_noise.py       # ROS 2 node adding Gaussian noise to sensor data
    │   │   ├── launch/
    │   │   │   └── noisy_sensors.launch.py
    │   │   ├── README.md
    │   │   └── test.sh
    │   └── test-all.sh              # Test all Chapter 3 Gazebo examples
    └── chapter-4/                   # Gazebo-Unity Integration examples
        ├── README.md                # Chapter 4 examples overview
        ├── example-01-ros2-bridge-setup/
        │   ├── unity/
        │   │   └── ROSBridgeProject/
        │   │       ├── Assets/
        │   │       │   ├── Scripts/
        │   │       │   │   └── ROSTCPConnector.cs
        │   │       │   └── Scenes/
        │   │       ├── Packages/
        │   │       │   └── manifest.json  # ROS-TCP-Connector
        │   │       └── ProjectSettings/
        │   ├── gazebo/
        │   │   └── launch/
        │   │       └── gazebo_headless.launch.py
        │   ├── README.md
        │   └── test.md              # Manual integration test checklist
        ├── example-02-realtime-sync/
        │   ├── unity/
        │   │   └── SyncProject/
        │   ├── gazebo/
        │   │   ├── launch/
        │   │   └── scripts/
        │   │       └── state_publisher.py
        │   ├── README.md
        │   └── test.md
        ├── example-03-rl-training/
        │   ├── gazebo/
        │   │   ├── launch/
        │   │   └── scripts/
        │   │       └── rl_env.py      # Gymnasium environment
        │   ├── training/
        │   │   ├── train_ppo.py       # stable-baselines3 training script
        │   │   └── requirements.txt
        │   ├── unity/
        │   │   └── RLVisualization/
        │   ├── README.md
        │   └── test.md
        └── test-all.md              # Manual integration test checklist for all Chapter 4 examples

# CI/CD and Testing
.github/
└── workflows/
    ├── docusaurus-build.yml         # Build and deploy Docusaurus to Vercel/GitHub Pages
    ├── test-gazebo-examples.yml     # Test Gazebo examples in Docker
    └── test-unity-builds.yml        # Test Unity project builds (if Unity CLI license available)

# Docker for Testing
.docker/
├── gazebo-humble.Dockerfile         # Custom Dockerfile extending osrf/ros:humble-desktop + Gazebo Classic 11
└── unity-test.Dockerfile            # Optional: Unity headless build container (license required)
```

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: NOT APPLICABLE - No constitution violations identified. All gates pass.

## Phase 0: Research & Design Decisions (To be completed)

**Output**: `research.md`

**Research Questions to Answer**:

1. **Gazebo Version Strategy**: Which Gazebo version to prioritize? Gazebo Classic 11 (stable, widely adopted) vs Gazebo Fortress/Garden (modern, actively developed)?
   - **Decision needed**: Choose primary target, provide migration paths for secondary
   - **Research**: Check Ubuntu 22.04/24.04 default packages, community adoption rates, ROS 2 Humble compatibility

2. **Unity Rendering Pipeline**: Which Unity rendering pipeline to target? Built-in, URP (Universal Render Pipeline), or HDRP (High Definition Render Pipeline)?
   - **Decision needed**: Balance visual quality vs performance on reference hardware
   - **Research**: Benchmark FPS on integrated graphics, evaluate PBR material workflows

3. **ROS 2 Bridge for Unity**: Which Unity-ROS 2 communication method to recommend? ROS-TCP-Connector vs ROS# (C# ROS bridge)?
   - **Decision needed**: Choose primary method based on reliability, performance, community support
   - **Research**: Test latency, message throughput, DDS compatibility

4. **Sensor Simulation Approach**: Gazebo plugins vs Unity Perception package - which provides better sensor modeling accuracy?
   - **Decision needed**: Determine when to use each platform for sensor simulation
   - **Research**: Validate LiDAR point cloud accuracy, depth camera noise models

5. **RL Training Framework**: Which RL library to recommend for Gazebo integration? stable-baselines3 vs RLlib vs CleanRL?
   - **Decision needed**: Choose based on ease of use for students, Gazebo Gym integration quality
   - **Research**: Test Gymnasium environment setup, policy export/deployment

6. **Performance Profiling Tools**: How to measure and optimize Gazebo RTF and Unity FPS for student hardware?
   - **Decision needed**: Provide profiling tools and optimization guidelines
   - **Research**: Identify bottlenecks, recommend hardware-specific settings

**Consolidation Format** (to be in research.md):
- Decision: [what was chosen]
- Rationale: [why chosen over alternatives]
- Alternatives considered: [what else evaluated]
- References: [official docs, benchmarks, community discussions]

## Phase 1: Design & Contracts (To be completed)

### 1. Data Model (`data-model.md`)

**Key Entities to Define**:

- **Chapter**: Represents one of the four distinct chapters in Module 2
  - Fields: chapter_number (1-4), title, learning_objectives (array), sections (array), code_examples (array), exercises (array), metadata (frontmatter)
  - Relationships: Each chapter independently testable, sequential dependencies (Chapter 4 depends on Chapters 1-3)

- **Learning Objective**: Measurable outcome using Bloom's taxonomy action verbs
  - Fields: id, chapter_number, description, bloom_level (understand/apply/analyze/create)
  - Validation: Must use action verbs (explain, configure, implement, demonstrate)

- **Code Example (Gazebo)**: Executable simulation demonstration
  - Fields: id, chapter_number, example_number, title, type (world_file/sdf_model/launch_file/python_script), file_paths (array), dependencies, README, test_script
  - Validation: Must include launch file, README with prerequisites, test.sh that validates execution

- **Code Example (Unity)**: Unity project demonstration
  - Fields: id, chapter_number, example_number, title, unity_version, required_packages (array), scene_paths (array), script_paths (array), README, screenshots
  - Validation: Must include Unity project structure, README with Unity version, screenshots of expected output

- **Sensor Model**: Simulated sensor configuration
  - Fields: id, sensor_type (lidar/depth_camera/imu), platform (gazebo/unity), parameters (resolution/noise/update_rate), sdf_config (if Gazebo), unity_script (if Unity)
  - Relationships: References Chapter 3 examples

- **Integration Pipeline**: Architecture connecting Gazebo and Unity
  - Fields: id, name, architecture_type (realtime_sync/unidirectional/offline), message_flow (ROS 2 topics/services), synchronization_method, performance_metrics (latency/rtf/fps)
  - Relationships: References Chapter 4 examples

- **Exercise**: Progressive challenge with acceptance criteria
  - Fields: id, chapter_number, difficulty (beginner/intermediate/advanced), description, acceptance_criteria (array), hints (array), solution_reference (optional)
  - Validation: Each chapter must have 5 exercises (2 beginner, 2 intermediate, 1 advanced)

- **Chapter Metadata**: Docusaurus frontmatter
  - Fields: id, title, sidebar_label, sidebar_position, description, keywords (array), tags (array)
  - Validation: Must align with Docusaurus plugin-content-docs structure

### 2. API Contracts (`contracts/chapter-content-contract.md`)

**Subagent Input Contract**:
- spec.md (functional requirements for assigned chapter)
- data-model.md (Chapter Template Structure)
- configuration JSON (chapter number, target versions: Gazebo Classic 11, Unity 2022 LTS, ROS 2 Humble)
- cross-reference data (for Chapters 2-4: imports from prior chapters)
- templates (content-template.md, code-example-template for Gazebo/Unity, test-template.sh)

**Subagent Output Contract**:
- Chapter content (`.mdx` file, 12,000-20,000 words)
- Gazebo examples directory (if applicable: Chapter 1, 3, 4): ≥3 examples, each with launch file, README, test.sh
- Unity project directory (if applicable: Chapter 2, 3, 4): ≥3 projects, each with Assets/, Packages/, README, screenshots
- Test report (execution status, Docker/Unity build logs, performance metrics: RTF, FPS, latency)
- Diagram sources (Mermaid files for architectures, flowcharts)

**Quality Gates** (20 gates per chapter):
- **Content Quality** (10 gates):
  1. All 9 sections present (Overview, Learning Objectives, Theoretical Foundations, Hands-On Implementation, Practical Examples, Exercises, Further Reading, Summary, Troubleshooting)
  2. 3-5 learning objectives with measurable action verbs
  3. At least 1 mathematical formulation with KaTeX notation
  4. At least 3 diagrams (Mermaid or screenshots)
  5. At least 5 exercises (2 beginner, 2 intermediate, 1 advanced)
  6. At least 5 references in "Further Reading" section
  7. Flesch-Kincaid grade level 12-16
  8. No broken internal links (cross-references to Module 1, within Module 2)
  9. Proper MDX formatting (code blocks, callouts, images)
  10. All images have descriptive alt text

- **Code Quality (Gazebo)** (6 gates, if applicable):
  11. All Gazebo worlds load without errors in Gazebo Classic 11
  12. All SDF models validate with `gz sdf` tool
  13. All launch files execute in Docker (osrf/ros:humble-desktop)
  14. RTF ≥ 0.9 on reference hardware
  15. All examples include README with prerequisites and test.sh
  16. Test report documents execution results

- **Code Quality (Unity)** (6 gates, if applicable):
  17. All Unity projects build without errors in Unity 2022 LTS
  18. All C# scripts compile without warnings
  19. FPS ≥30 on reference hardware (URP, 1080p)
  20. All projects include README with Unity version, packages, and screenshots
  21. Screenshots show expected visual output
  22. Manual test checklist provided (test-all.md)

- **Consistency** (4 gates):
  23. Terminology aligns with Module 1 and official documentation
  24. Cross-references to Module 1 chapters correct (FR-041)
  25. Code style consistent (Python: PEP 8, C#: Unity conventions)
  26. File paths follow conventions (`examples/module-2/chapter-N/`)

**Delivery Checklist**: Subagent provides delivery report with gate pass/fail counts, recommended fixes, and suggestions for next chapter.

### 3. Developer Guide (`quickstart.md`)

**Covered Topics**:
- Environment setup (Node.js, Docker, Gazebo, Unity)
- Manual chapter creation (following template)
- Subagent invocation (command-line examples for each of 4 agents)
- Gazebo example validation (Docker testing, RTF measurement)
- Unity example validation (build testing, FPS measurement, screenshot capture)
- Integration testing (Gazebo-Unity ROS 2 bridge, latency measurement)
- Docusaurus preview (npm run start)
- Quality gates execution (build, test, lint)
- Troubleshooting (common Gazebo issues, Unity import errors, ROS 2 bridge failures)
- Best practices (simulation content writing, physics parameter tuning, rendering optimization)
- Common tasks (add Gazebo world, add Unity project, update after spec changes, add callout boxes)

**Output**: data-model.md, /contracts/chapter-content-contract.md, quickstart.md

## Phase 2: Implementation Architecture

### Subagent Specifications

#### 1. GazeboPhysicsAgent

**Responsibility**: Generate Chapter 1 - Gazebo Physics Simulation (Gravity, Collisions, Contact Dynamics)

**Inputs**:
- `spec.md` (FR-001 to FR-009)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 1, module_number: 2, gazebo_version: "classic-11", ros2_distribution: "humble" }`

**Outputs**:
- `docs/module-2/chapter-1.mdx` (Gazebo Physics content)
- `examples/module-2/chapter-1/` (≥5 Gazebo examples: world creation, humanoid spawning, collision config, physics tuning, contact forces)
- `examples/module-2/chapter-1/test-report.md`
- `docs/module-2/_diagrams/chapter-1/` (Mermaid diagrams: Gazebo client-server architecture, physics engine pipeline, collision detection flow, ODE/Bullet/DART comparisons)
- `docs/module-2/_screenshots/chapter-1/` (Gazebo GUI screenshots, physics behavior visualizations)

**Key Content Sections**:
1. **Theoretical Foundations**: Rigid body dynamics (Newton-Euler equations), contact mechanics (Hertz contact theory, Coulomb friction), numerical integration (Euler, RK4, symplectic), physics engine algorithms (constraint-based solvers, impulse-based collision response)
2. **Hands-On Implementation**: Gazebo installation tutorial, world file creation (.world SDF format), URDF spawning with `spawn_entity.py`, physics parameter tuning (gravity, timestep, solver iterations, ERP/CFM for ODE)
3. **Practical Examples**: Humanoid standing/walking in Gazebo, object manipulation with contact forces, terrain interaction (stairs, slopes, uneven ground), external force application (`gz model` commands, ROS 2 force/torque services)
4. **Exercises**: Beginner (modify gravity, spawn custom models), Intermediate (configure collision geometries for stable grasping, tune friction coefficients), Advanced (create bipedal standing scenario with contact-based stabilization, measure ground reaction forces)

**Validation**:
- All Gazebo examples tested in Docker (osrf/ros:humble-desktop + Gazebo Classic 11)
- RTF ≥ 0.9 measured and documented
- Physics parameters validated against real-world benchmarks (e.g., gravity = 9.81 m/s², typical friction coefficients)
- `check_urdf` and `gz sdf` validation pass for all models

**Exports for Later Chapters**:
- `physics_fundamentals`: /module-2/chapter-1/#theoretical-foundations (referenced by Chapter 3 sensor placement, Chapter 4 integration)
- `gazebo_basics`: /module-2/chapter-1/#hands-on-implementation (referenced by Chapter 3 Gazebo sensor simulation, Chapter 4 Gazebo headless mode)

---

#### 2. UnityRenderingAgent

**Responsibility**: Generate Chapter 2 - Unity Rendering & Human-Robot Interaction (PBR Materials, Lighting, UI)

**Inputs**:
- `spec.md` (FR-010 to FR-019)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 2, module_number: 2, unity_version: "2022 LTS", rendering_pipeline: "URP" }`
- Cross-references: Module 1 Chapter 3 (URDF modeling)

**Outputs**:
- `docs/module-2/chapter-2.mdx` (Unity Rendering content)
- `examples/module-2/chapter-2/` (≥5 Unity projects: Unity basics, URDF import, PBR materials, UI interaction, Timeline animations)
- `examples/module-2/chapter-2/test-all.md` (manual test checklist)
- `docs/module-2/_diagrams/chapter-2/` (Mermaid diagrams: Unity Editor architecture, URP rendering pipeline, UI event flow, articulation body hierarchy)
- `docs/module-2/_screenshots/chapter-2/` (Unity Editor screenshots, rendered outputs, UI panels)

**Key Content Sections**:
1. **Theoretical Foundations**: PBR theory (Cook-Torrance BRDF, Fresnel equations, microfacet distribution), Unity rendering pipeline architecture (Built-in vs URP vs HDRP), real-time lighting (shadow mapping, SSAO, reflections), HDR imaging (tone mapping, color grading)
2. **Hands-On Implementation**: Unity Hub installation, project creation (URP template), URDF Importer package setup, articulation body configuration (joint types, drives, limits), material creation (albedo, metallic-smoothness workflow, normal maps), HDRI environment setup
3. **Practical Examples**: Humanoid robot rendering with realistic materials (metal, plastic, rubber textures), joint animation synchronized with simulation data, UI panel creation (Canvas, EventSystem, TextMeshPro), keyboard/gamepad teleoperation, Timeline cinematic sequences
4. **Exercises**: Beginner (import robot, apply single material), Intermediate (create multi-material robot with PBR textures, implement keyboard control for 3 joints), Advanced (build interactive demo with UI dashboard, Timeline animation, export video)

**Validation**:
- All Unity projects build successfully in Unity 2022 LTS on Windows 11 and Ubuntu 22.04
- FPS ≥30 measured at 1080p with URP on reference hardware (integrated graphics)
- Screenshots demonstrate photorealistic rendering quality
- C# scripts compile without errors or warnings

**Exports for Later Chapters**:
- `unity_basics`: /module-2/chapter-2/#hands-on-implementation (referenced by Chapter 3 Unity sensor simulation, Chapter 4 Unity visualization)
- `pbr_workflow`: /module-2/chapter-2/#pbr-materials (referenced by Chapter 3 camera-based sensors requiring realistic scenes)

---

#### 3. SensorSimulationAgent

**Responsibility**: Generate Chapter 3 - Sensor Simulation (LiDAR, Depth Cameras, IMUs) in Gazebo and Unity

**Inputs**:
- `spec.md` (FR-020 to FR-028)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 3, module_number: 2, gazebo_version: "classic-11", unity_version: "2022 LTS", ros2_distribution: "humble" }`
- Cross-references: Chapter 1 (Gazebo physics), Chapter 2 (Unity rendering), Module 1 Chapter 1 (ROS 2 topics)

**Outputs**:
- `docs/module-2/chapter-3.mdx` (Sensor Simulation content)
- `examples/module-2/chapter-3/` (≥6 examples: 3 Gazebo sensor examples (LiDAR, depth camera, IMU), 2 Unity sensor examples (LiDAR raycasting, Perception package), 1 sensor noise example)
- `examples/module-2/chapter-3/test-report.md`
- `docs/module-2/_diagrams/chapter-3/` (Mermaid diagrams: LiDAR scanning patterns, depth camera principles, IMU sensor fusion, ROS 2 sensor message flow)
- `docs/module-2/_screenshots/chapter-3/` (RViz point cloud visualizations, depth images, IMU plots)

**Key Content Sections**:
1. **Theoretical Foundations**: LiDAR physics (time-of-flight, phase shift, triangulation, scanning mechanisms), depth camera technologies (stereo vision geometry, structured light patterns, ToF imaging), IMU sensor fusion (complementary filters, EKF, UKF), sensor noise models (Gaussian noise, systematic bias, drift models)
2. **Hands-On Implementation (Gazebo)**: GPU ray sensor plugin configuration (SDF parameters: scan rate, horizontal/vertical samples, range min/max, noise stddev), depth camera plugin (image_width/height, FOV, clip distances, ROS 2 topic publishing), IMU sensor plugin (noise parameters: accelerometer/gyroscope bias_mean/bias_stddev, noise_density)
3. **Hands-On Implementation (Unity)**: Unity Perception package installation and setup (semantic segmentation labelers, bounding box labelers, depth rendering), custom LiDAR raycasting script (Physics.Raycast in spherical coordinates, point cloud generation, ROS 2 LaserScan message publishing), IMU simulation (Rigidbody.velocity, angularVelocity → sensor_msgs/Imu)
4. **Practical Examples**: Visual-inertial odometry (camera + IMU fusion), 3D mapping (LiDAR + depth camera point cloud merging), sensor fusion for localization (EKF with simulated GPS, IMU, wheel odometry), ground truth validation (compare sensor output to simulation state)

**Validation**:
- All Gazebo sensor examples tested in Docker with sensor data published to ROS 2 topics
- Unity Perception examples validated with semantic segmentation output
- Point cloud visualization in RViz successful for LiDAR examples
- Sensor noise parameters tuned to realistic values (citing real sensor datasheets: Velodyne VLP-16, Intel RealSense D435, Bosch BMI088)

**Exports for Later Chapters**:
- `sensor_configs`: /module-2/chapter-3/#sensor-configuration (referenced by Chapter 4 integration examples needing sensor data)

---

#### 4. IntegrationAgent

**Responsibility**: Generate Chapter 4 - Gazebo-Unity Integration Best Practices for Physical AI

**Inputs**:
- `spec.md` (FR-029 to FR-039 including FR-038 Module 2 summary and Module 3 preview)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 4, module_number: 2, gazebo_version: "classic-11", unity_version: "2022 LTS", ros2_distribution: "humble" }`
- Cross-references: Chapter 1 (Gazebo headless mode, physics), Chapter 2 (Unity rendering), Chapter 3 (sensor data), Module 1 Chapter 1 (ROS 2 topics/services)

**Outputs**:
- `docs/module-2/chapter-4.mdx` (Integration content)
- `examples/module-2/chapter-4/` (≥3 integration examples: ROS 2 bridge setup, realtime sync, RL training workflow)
- `examples/module-2/chapter-4/test-all.md` (manual integration test checklist)
- `docs/module-2/_diagrams/chapter-4/` (Mermaid diagrams: integration architecture patterns, ROS 2 bridge message flow, RL training pipeline, distributed simulation architecture)
- `docs/module-2/_screenshots/chapter-4/` (Synchronized Gazebo-Unity screenshots, RL training plots)

**Key Content Sections**:
1. **Theoretical Foundations**: Integration architecture patterns (realtime bidirectional sync, unidirectional data flow, offline replay, co-simulation timestep synchronization), ROS 2 DDS middleware (RTPS protocol, QoS policies for cross-machine communication), RL for robotics (Markov Decision Processes, policy gradient methods, sim-to-real transfer challenges, domain randomization)
2. **Hands-On Implementation**: ROS-TCP-Connector installation (Unity Package Manager, Unity Robotics Hub GitHub), ROS 2 endpoint configuration (IP addresses, ports, ROS_DOMAIN_ID), joint_states topic subscription (Unity C# subscriber, articulation body synchronization), tf transform synchronization (coordinate frame conversions), sensor data streaming (Gazebo sensor plugins → Unity visualization)
3. **Practical Examples**: Gazebo headless physics + Unity real-time rendering (measuring latency with timestamps), RL training with Gazebo Gym + stable-baselines3 (PPO algorithm, hyperparameter tuning, policy export to ONNX, Unity deployment for validation), human-in-the-loop teleoperation (Unity UI → Gazebo robot control, haptic feedback simulation), cloud deployment (Gazebo on AWS EC2 with Docker, Unity client connecting via VPN)
4. **Exercises**: Beginner (set up ROS 2 bridge, sync single joint), Intermediate (implement full robot state sync with sensor visualization, measure latency), Advanced (create RL training pipeline: Gazebo Gym environment, train PPO policy for 1M steps, deploy in Unity, visualize learned behavior, compare sim-to-real transfer)
5. **Module 2 Summary**: Recap key Digital Twin skills (physics simulation for accurate dynamics, photorealistic rendering for visualization, sensor modeling for perception testing, integration for hybrid workflows), emphasize sim-to-real transfer importance
6. **Module 3 Preview**: Introduce NVIDIA Isaac Sim (GPU-accelerated physics, ray-traced rendering, synthetic data generation), Vision-Language-Action models (embodied AI, language-conditioned policies), set expectations for advanced Physical AI topics

**Validation**:
- Integration examples tested on Linux-Linux (Ubuntu 22.04 Gazebo + Unity) and Windows-Linux (Windows 11 Unity + WSL2 Gazebo)
- Latency <100ms measured with synchronized timestamps
- RL training example successfully trains policy and deploys in Unity
- Docker Gazebo containerization validated

**Module 2 Finalization**:
- Module 2 summary section (200-300 words) per FR-038
- Module 3 preview section (150-200 words) per FR-038
- All cross-references validated (no broken links)

---

### Subagent Orchestration Workflow

**Master Orchestrator** (automated or manual coordination):

```plaintext
1. Phase 0: Research Complete (research.md generated by /sp.plan)
2. Phase 1: Design Complete (data-model.md, contracts/, quickstart.md generated by /sp.plan)
3. Phase 2: Task Generation (/sp.tasks creates tasks.md)
4. Phase 3: Implementation (Sequential execution with /sp.implement)

   Step 1: Invoke GazeboPhysicsAgent
      Input: spec.md (FR-001 to FR-009), config, templates, Module 1 URDF references
      Output: chapter-1.mdx, examples/chapter-1/, test-report, diagrams, screenshots
      Validation: Run quality gates, Docker tests, RTF measurement
      Register Exports: physics_fundamentals, gazebo_basics

   Step 2: Invoke UnityRenderingAgent
      Input: spec.md (FR-010 to FR-019), config, templates, Module 1 URDF references
      Output: chapter-2.mdx, examples/chapter-2/, test checklist, diagrams, screenshots
      Validation: Run quality gates, Unity builds, FPS measurement
      Register Exports: unity_basics, pbr_workflow

   Step 3: Invoke SensorSimulationAgent (depends on Steps 1 and 2)
      Input: spec.md (FR-020 to FR-028), config, templates, Chapter 1/2 exports, Module 1 ROS 2 topic references
      Output: chapter-3.mdx, examples/chapter-3/, test-report, diagrams, screenshots
      Validation: Run quality gates, Gazebo sensor tests, Unity Perception validation, RViz visualization
      Register Exports: sensor_configs

   Step 4: Invoke IntegrationAgent (depends on Steps 1-3)
      Input: spec.md (FR-029 to FR-039), config, templates, Chapters 1-3 exports, Module 1 ROS 2 references
      Output: chapter-4.mdx, examples/chapter-4/, integration test checklist, diagrams, screenshots
      Validation: Run quality gates, integration testing (Gazebo-Unity sync, RL training, latency measurement)
      Finalize: Module 2 summary + Module 3 preview per FR-038

5. Phase 4: Final Validation
   - Run full Docusaurus build (detect broken links, MDX errors)
   - Test all Gazebo examples (examples/module-2/chapter-1/test-all.sh, chapter-3/test-all.sh)
   - Test all Unity builds (manual checklist execution)
   - Test integration examples (manual latency/sync validation)
   - Manual QA: Readability, accessibility, cross-references, screenshot accuracy
   - Performance audit: Lighthouse score, page load times

6. Phase 5: Deployment
   - Merge to main branch
   - Trigger Vercel/GitHub Pages deployment
   - Verify production site
```

### Shared Templates for Consistency

**Content Template** (`templates/content-template-module2.md`):
```markdown
# Chapter {N}: {Title}

## Overview
[200-400 word introduction explaining the chapter's role in Digital Twin workflow]

**Estimated Time**: {6-8} hours
**Prerequisites**: Module 1 completion (ROS 2, URDF), [previous Module 2 chapters if applicable]

## Learning Objectives
- **LO-2.{N}.1**: [Action verb + outcome, e.g., "Explain Gazebo client-server architecture"]
- **LO-2.{N}.2**: [Action verb + outcome, e.g., "Configure physics parameters for stable simulation"]
- **LO-2.{N}.3**: [Action verb + outcome, e.g., "Demonstrate humanoid standing in Gazebo"]

[... rest of template follows data-model.md Chapter Template Structure ...]
```

**Gazebo Example Template** (`templates/gazebo-example-template/`):
```text
example-XX-name/
├── worlds/
│   └── demo.world              # SDF world file
├── models/
│   └── custom_model.sdf        # Custom SDF models
├── launch/
│   └── demo.launch.py          # ROS 2 Python launch file
├── scripts/
│   └── helper.py               # Optional helper scripts
├── README.md
└── test.sh                     # Validation script
```

README.md format:
```markdown
# Example {N}.{NN}: {Title}

## Description
{1-2 sentence description of what this example demonstrates}

## Prerequisites
- ROS 2 Humble installed
- Gazebo Classic 11 installed
- [Any additional dependencies]

## Usage
1. Source ROS 2: `source /opt/ros/humble/setup.bash`
2. Launch example: `ros2 launch {package_name} demo.launch.py`
3. Expected behavior: {describe what student should see}

## Testing
Run automated test: `./test.sh`

## Troubleshooting
- [Common issue 1]: [Solution]
- [Common issue 2]: [Solution]
```

**Unity Example Template** (`templates/unity-example-template/`):
```text
ExampleProject/
├── Assets/
│   ├── Scenes/
│   │   └── DemoScene.unity
│   ├── Scripts/
│   │   └── DemoScript.cs
│   ├── Materials/
│   ├── Prefabs/
│   └── URDF/ (if applicable)
├── Packages/
│   └── manifest.json           # Unity packages (URDF Importer, ROS-TCP, etc.)
├── ProjectSettings/
│   └── [Unity project settings]
├── README.md
└── screenshots/
    └── expected_output.png
```

README.md format:
```markdown
# Example {N}.{NN}: {Title}

## Description
{1-2 sentence description}

## Unity Version
- Unity 2022 LTS (2022.3.X)
- Required Packages:
  - [Package 1] (version)
  - [Package 2] (version)

## Setup
1. Open Unity Hub
2. Add project from disk: `[path]`
3. Open in Unity 2022 LTS
4. Install required packages if prompted

## Usage
1. Open DemoScene in Assets/Scenes/
2. Press Play
3. Expected output: {describe expected visual/interactive behavior}

## Expected Output
See `screenshots/expected_output.png`

## Troubleshooting
- [Common import issue]: [Solution]
- [Common script error]: [Solution]
```

**Test Script Template (Gazebo)** (`templates/test-template-gazebo.sh`):
```bash
#!/bin/bash
# Test script for Gazebo Example {N}.{NN}

set -e  # Exit on error

echo "Testing Gazebo Example {N}.{NN}: {Title}"

# 1. Check prerequisites
echo "Checking ROS 2 Humble..."
source /opt/ros/humble/setup.bash || { echo "ROS 2 Humble not found"; exit 1; }

echo "Checking Gazebo..."
which gzserver || { echo "Gazebo not found"; exit 1; }

# 2. Launch simulation in background
echo "Launching simulation..."
timeout 30s ros2 launch {package_name} demo.launch.py &
LAUNCH_PID=$!

# 3. Wait for simulation to stabilize
sleep 10

# 4. Validate (check if topics are published, measure RTF, etc.)
echo "Validating ROS 2 topics..."
ros2 topic list | grep "/robot_state_publisher" || { echo "Topic check failed"; kill $LAUNCH_PID; exit 1; }

# 5. Cleanup
echo "Cleaning up..."
kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null

echo "✓ All tests passed!"
```

## Success Metrics

Module 2 is complete when:

- [x] All 4 chapters delivered (chapter-1.mdx through chapter-4.mdx)
- [x] All Gazebo examples tested in Docker with 100% pass rate (SC-006)
- [x] All Unity projects build successfully in Unity 2022 LTS (SC-007)
- [x] Quality gates pass for all chapters (20 gates × 4 chapters = 80 total)
- [x] Docusaurus builds without errors
- [x] Cross-references validated (no broken links to Module 1, within Module 2)
- [x] Learning effectiveness criteria met (SC-011 to SC-014: stable humanoid standing, photorealistic rendering, sensor visualization, synchronized integration)
- [x] UX criteria met (SC-016 to SC-019: page load <2s, Lighthouse >90, responsive design, screenshots included)
- [x] Technical accuracy criteria met (SC-020 to SC-022: physics equations cited, realistic parameters, ROS 2 message compatibility)
- [x] Deployed to Vercel/GitHub Pages and accessible

## Risks and Mitigations

### Risk: Gazebo version fragmentation (Classic vs Fortress/Garden)

**Mitigation**: Primary focus on Gazebo Classic 11 for stability and wide adoption. Provide clear callout boxes for Gazebo Fortress/Garden differences. Test examples in both versions where feasible. Include migration guide in research.md.

### Risk: Unity version API changes break examples

**Mitigation**: Target Unity 2022 LTS for 2-year support window. Document Unity 6 compatibility notes. Pin Unity Robotics Hub package versions. Test on both Windows and Ubuntu to catch platform-specific issues.

### Risk: ROS 2 bridge reliability issues (DDS/network)

**Mitigation**: Comprehensive troubleshooting section in Chapter 4. Test ROS-TCP-Connector on localhost, cross-machine, and cloud configurations. Provide alternative methods (ROS Bridge over WebSocket). Document firewall rules and DDS domain configuration.

### Risk: Student hardware cannot run Gazebo + Unity simultaneously

**Mitigation**: Define reference hardware clearly (8GB RAM, 4-core CPU, integrated graphics). Provide performance profiling tools. Recommend offline integration workflow (record Gazebo rosbag, replay in Unity). Offer cloud simulation options (AWS/GCP with Docker Gazebo).

### Risk: RL training example too complex for students

**Mitigation**: Keep RL content at introductory level (FR-033). Use simple task (robot standing, ball balancing). Provide pre-trained policy for quick validation. Link to dedicated ML courses for deeper RL theory. Focus on workflow (Gazebo Gym → training → Unity deployment) rather than algorithm tuning.

### Risk: Physics simulation accuracy not matching real-world data

**Mitigation**: Validate physics parameters against real robot benchmarks (cite sources in SC-020, SC-021). Include "sim-to-real gap" discussion in Chapter 1 and Chapter 4. Recommend domain randomization techniques. Note that simulation is an approximation (set expectations).

## Next Steps

1. **Complete Phase 0**: Generate `research.md` answering all 6 research questions
2. **Complete Phase 1**: Generate `data-model.md`, `contracts/chapter-content-contract.md`, `quickstart.md`
3. **Re-check Constitution**: Ensure no violations introduced during design
4. **Run `/sp.tasks`**: Generate actionable task list organized by chapter and user story
5. **Run `/sp.implement`**: Execute tasks, invoking subagents sequentially (GazeboPhysicsAgent → UnityRenderingAgent → SensorSimulationAgent → IntegrationAgent)
6. **Validate outputs**: Run quality gates, Docker tests, Unity builds, integration tests
7. **Manual QA**: Review content for readability, accuracy, cross-references, screenshot quality
8. **Deploy**: Merge to main, trigger Vercel/GitHub Pages deployment
9. **Verify production**: Test Module 2 chapters in production environment

---

**Plan Status**: ✅ **PHASE 2 COMPLETE** - Constitution Check passed. Ready for Phase 0 (research.md generation) and Phase 1 (data-model.md, contracts/, quickstart.md generation).

**Next Command**: Agent will now proceed with Phase 0 research and Phase 1 design artifacts generation.
