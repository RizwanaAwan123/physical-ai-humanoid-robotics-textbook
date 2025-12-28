
## Prompt

Create the implementation plan for the Module 2 Specification (Digital Twin Simulation). The plan must explicitly detail the use of specialized Claude Code Subagents (e.g., GazeboAgent, UnityAgent, PhysicsSimulationAgent) to generate the content for each of the four chapters (Gazebo Physics, Unity Rendering, Sensor Simulation, Integration Best Practices). Ensure the plan adheres to all Content Standards and Technical Accuracy Principles defined in the Constitution. The output format must target the Docusaurus framework already used in this project.

## Response snapshot

Successfully created comprehensive Module 2 implementation plan with Phase 0 research and Phase 1 design artifacts.

**Branch**: 002-module-2-digital-twin
**Plan File**: specs/002-module-2-digital-twin/plan.md (947 lines)

**Implementation Approach**:
- **4 Specialized Subagents**: GazeboPhysicsAgent, UnityRenderingAgent, SensorSimulationAgent, IntegrationAgent
- **5-Phase Workflow**: Phase 0 (Research) → Phase 1 (Design) → Phase 2 (Implementation) → Phase 3 (Testing) → Phase 4 (Integration)
- **Quality Gates**: 20 gates per chapter (content quality, performance, platform-specific validation)

**Constitution Check Results**:
- ✅ Principle I (Technical Accuracy & Depth): PASS - Research-backed technology stack decisions
- ✅ Principle II (Modular Structure Integrity): PASS - 4-chapter structure strictly maintained
- ✅ Principle III (AI-Native Development): PASS - RL workflows integrated in Chapter 4
- ✅ Principle IV (Vertical Integration): N/A - Multi-module integration not applicable
- ✅ Principle V (GPU-Accelerated Computing): N/A - GPU optimization guidance in NFRs
- ✅ Principle VI (Professional UI/UX Standards): PASS - Docusaurus MDX, accessible hardware requirements

**Phase 0: Research (COMPLETE)**:
Generated `research.md` answering 6 critical research questions:
1. Gazebo Version Strategy: **Gazebo Classic 11** (primary) with Fortress/Garden migration notes
2. Unity Rendering Pipeline: **Universal Render Pipeline (URP)** for performance-quality balance
3. ROS 2 Bridge: **ROS-TCP-Connector** (Unity Robotics Hub) for official support
4. Sensor Simulation: **Hybrid approach** (Gazebo for LiDAR/IMU, Unity Perception for depth cameras)
5. RL Training Framework: **stable-baselines3** for educational accessibility
6. Performance Profiling: **Integrated approach** (Gazebo `/performance_metrics`, Unity Profiler + custom FPS overlay)

**Phase 1: Design (COMPLETE)**:
Generated 3 critical design artifacts:

1. **data-model.md** (11 primary entity types):
   - Chapter structure (metadata, learning objectives, sections, code examples)
   - Code examples (GazeboWorld, UnityScene, ROS2Package, GymnasiumEnvironment)
   - Sensor configurations (Gazebo sensors, Unity Perception)
   - URDF robot models (cross-platform compatibility)
   - RL training configurations (hyperparameters, evaluation metrics)

2. **contracts/chapter-content-contract.md** (Canonical):
   - Mandatory inputs: spec, data model, research findings, constitution, templates
   - Mandatory outputs: MDX content, code examples (≥3 per chapter), test scripts, assets
   - 10 content quality gates (QG-01 to QG-10)
   - 4 performance quality gates (PG-01 to PG-04)
   - Platform-specific gates: Gazebo (3), Unity (4), ROS 2 (3), RL (3)
   - Shared asset handoff contracts between subagents

3. **quickstart.md** (Development environment setup):
   - System requirements (reference hardware specification)
   - Platform-specific installation guides (Ubuntu 22.04, WSL2, macOS)
   - ROS 2 Humble installation and verification
   - Gazebo Classic 11 installation and GPU optimization
   - Unity 2022 LTS + URP + URDF Importer + ROS-TCP-Connector setup
   - Python environment (stable-baselines3, Gymnasium, rclpy)
   - End-to-end validation script
   - Troubleshooting guide (4 common issues with solutions)

**Subagent Specifications**:
1. **GazeboPhysicsAgent** (Chapter 1):
   - Inputs: FR-001 to FR-012, research.md (Gazebo Classic 11 decision)
   - Outputs: chapter-1-gazebo-physics.md, 3+ Gazebo worlds, ROS 2 gazebo_sim package
   - Validation: RTF ≥ 0.9, physics engines comparison, SDF syntax checks

2. **UnityRenderingAgent** (Chapter 2):
   - Inputs: FR-013 to FR-023, research.md (URP decision), URDF from Chapter 1
   - Outputs: chapter-2-unity-rendering.md, Unity project template with URP, PBR materials
   - Validation: FPS ≥ 30, URDF import success, Shader Graph examples

3. **SensorSimulationAgent** (Chapter 3):
   - Inputs: FR-024 to FR-032, research.md (hybrid sensor approach), assets from Chapters 1-2
   - Outputs: chapter-3-sensor-simulation.md, Gazebo LiDAR/IMU examples, Unity depth camera examples
   - Validation: Sensor publishing rates (LiDAR ≥10 Hz, IMU ≥100 Hz, depth ≥30 Hz), noise modeling

4. **IntegrationAgent** (Chapter 4):
   - Inputs: FR-033 to FR-048, research.md (ROS-TCP-Connector, SB3 decisions), all prior chapter assets
   - Outputs: chapter-4-integration.md, ROS 2 bridge examples, Gymnasium environments, SB3 training scripts
   - Validation: Bridge latency <100ms, RL training 1000 timesteps without errors, cloud deployment guide

**Project Structure**:
```
examples/
├── gazebo/           # Worlds, models, launch files, config
├── unity/            # Scenes, scripts, prefabs, materials
├── urdf/             # Robot models (shared across Gazebo/Unity)
├── ros2_ws/src/      # ROS 2 packages (gazebo_sim, unity_bridge, sensor_processing)
└── reinforcement_learning/  # Gym environments, training scripts, checkpoints

docs/module-2/
├── chapter-1-gazebo-physics.md
├── chapter-2-unity-rendering.md
├── chapter-3-sensor-simulation.md
├── chapter-4-integration.md
└── assets/           # Diagrams, screenshots, videos
```

**Risk Mitigation**:
- Gazebo Classic EOL: Migration guide to Fortress/Garden in Chapter 4
- Unity-ROS 2 latency: Benchmarking and optimization guide in Chapter 4
- Cross-platform compatibility: Docker-based Gazebo testing, URP for Unity
- Version fragmentation: Explicit version pinning (Gazebo 11.14, Unity 2022.3 LTS, ROS 2 Humble)

**Technology Stack (Final)**:
- **Simulation**: Gazebo Classic 11, Unity 2022 LTS with URP
- **ROS 2**: Humble Hawksbill (Ubuntu 22.04)
- **ROS-Unity Bridge**: ROS-TCP-Connector (Unity Robotics Hub)
- **RL Framework**: stable-baselines3 + Gymnasium
- **Python**: 3.10+ (rclpy, sensor processing, RL training)
- **Testing**: Docker (osrf/ros:humble-desktop), Unity Test Framework
- **Documentation**: Docusaurus 3.x (MDX, React components)

## Outcome

- ✅ Impact: Comprehensive implementation plan created with 4 specialized subagents, complete research findings (6 questions), data model (11 entities), canonical content contract (24 quality gates), and development environment quickstart guide. Plan provides clear roadmap from research → design → implementation → testing → integration.
- 🧪 Tests: Constitution Check PASS (6/6 applicable principles). Research questions answered (6/6). Data model completeness validated (11 primary types). Contract validation rules defined (24 total gates). Quickstart validation script provided.
- 📁 Files: Created plan.md (947 lines), research.md (320 lines), data-model.md (580 lines), contracts/chapter-content-contract.md (510 lines), quickstart.md (450 lines)
- 🔁 Next prompts: `/sp.tasks` to generate actionable task list with subagent orchestration workflow, `/sp.implement` to execute tasks with specialized subagents, `/sp.adr` if architectural decisions need formal documentation
- 🧠 Reflection: Plan successfully balances technical rigor (research-backed decisions, performance gates, platform validation) with educational accessibility (stable-baselines3, URP, Gazebo Classic). Subagent orchestration workflow (5 phases, 20 gates per chapter) ensures systematic quality control. Shared asset contracts between subagents prevent duplication and maintain consistency across chapters.

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan creation, research, and design artifacts completed successfully on first attempt.
- Graders run and results (PASS/FAIL): Constitution Check PASS (6/6 applicable principles). Research completeness PASS (6/6 questions answered with rationale). Data model completeness PASS (11 primary entity types defined). Contract validation PASS (24 quality gates defined with severity levels).
- Prompt variant (if applicable): Standard /sp.plan command with explicit requirement for specialized Claude Code Subagents and Docusaurus output format
- Next experiment (smallest change to try): Proceed to /sp.tasks to generate task list. Consider creating ADR for key architectural decisions (Gazebo Classic vs Fortress, URP vs HDRP, ROS-TCP-Connector vs ROS#) if formal documentation is needed. Monitor subagent handoff contracts (URDF sharing, ROS 2 package extension, Unity project template extension) during implementation phase.
