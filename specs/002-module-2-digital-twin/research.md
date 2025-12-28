# Module 2: Digital Twin Simulation - Research Document

**Feature**: 002-module-2-digital-twin
**Created**: 2025-12-07
**Status**: Complete
**Purpose**: Answer key research questions to guide implementation decisions for Module 2 content generation

---

## Research Questions and Answers

### 1. Gazebo Version Strategy: Classic 11 vs Fortress/Garden

**Question**: Should we primarily target Gazebo Classic 11 or migrate to Gazebo Fortress/Garden for physics simulation examples?

**Research Findings**:

**Gazebo Classic 11**:
- **Pros**:
  - Mature, stable ecosystem with extensive ROS 2 Humble support via `gazebo_ros_pkgs`
  - Large community knowledge base, tutorials, and debugging resources
  - Better compatibility with older URDF models and third-party robot descriptions
  - Simpler installation via `apt install ros-humble-gazebo-ros-pkgs`
  - Most educational institutions and students currently use Classic
- **Cons**:
  - End-of-life announced (EOL January 2025), security updates discontinued
  - Older physics engine implementations (ODE, Bullet, DART)
  - Limited modern rendering capabilities (OGRE 1.x)

**Gazebo Fortress/Garden**:
- **Pros**:
  - Modern architecture with improved physics engines (TPE, DART 6.13+)
  - Better rendering with Ignition Rendering (OGRE 2.x support)
  - Active development and long-term support
  - Improved sensor simulation (GPU-accelerated ray tracing for LiDAR)
- **Cons**:
  - ROS 2 bridge (`ros_gz`) less mature than `gazebo_ros_pkgs`
  - Steeper learning curve, less educational content available
  - Potential compatibility issues with legacy URDF files
  - More complex installation dependencies

**Decision**: **Primary target: Gazebo Classic 11** with migration guidance to Fortress/Garden

**Rationale**:
1. **Educational accessibility**: Most students have Classic installed, reducing onboarding friction
2. **ROS 2 Humble alignment**: Official `gazebo_ros_pkgs` integration is production-ready
3. **Constitution Principle II compliance**: Provides clear learning pathway while preparing students for future migration
4. **Practical approach**: Include dedicated subsection in Chapter 4 explaining Fortress/Garden migration

**Implementation Impact**:
- All physics simulation examples (Chapter 1) developed for Gazebo Classic 11
- Migration notes provided for critical differences (SDF syntax, plugin namespaces, physics engine parameters)
- Optional advanced section in Chapter 4: "Migrating to Gazebo Fortress/Garden"

---

### 2. Unity Rendering Pipeline: Built-in vs URP vs HDRP

**Question**: Which Unity rendering pipeline should be used for high-fidelity robot visualization and human-robot interaction scenes?

**Research Findings**:

**Built-in Render Pipeline (BiRP)**:
- **Pros**: Simple, well-documented, compatible with all platforms
- **Cons**: Deprecated in Unity 2022+, limited PBR features, poor performance scaling

**Universal Render Pipeline (URP)**:
- **Pros**:
  - Modern, performant, optimized for real-time applications
  - Excellent PBR support (metallic-roughness workflow)
  - Cross-platform (Windows, Linux, macOS, mobile)
  - Shader Graph visual programming for custom materials
  - Post-processing stack (bloom, SSAO, depth of field)
  - Optimized for robotics simulation (moderate polygon counts, efficient lighting)
- **Cons**: Less photorealistic than HDRP for high-end visuals

**High Definition Render Pipeline (HDRP)**:
- **Pros**: Photorealistic rendering, advanced lighting (ray tracing, volumetrics)
- **Cons**:
  - High GPU requirements (RTX GPUs recommended)
  - Limited platform support (Windows/Linux only, no macOS Metal)
  - Overkill for robotics simulation (diminishing returns for educational use)

**Decision**: **Universal Render Pipeline (URP)**

**Rationale**:
1. **Performance-quality balance**: URP provides sufficient visual fidelity for Digital Twin simulation without requiring high-end GPUs
2. **Cross-platform compatibility**: Ensures students on Windows, Linux, and macOS can run examples
3. **ROS 2 integration**: URP's lower overhead leaves more resources for ROS-TCP-Connector bridge and sensor simulation
4. **Educational value**: Shader Graph teaches visual scripting while maintaining accessibility
5. **Constitution Principle VI**: Professional UI/UX without excluding students on moderate hardware

**Implementation Impact**:
- Chapter 2: Dedicated section on URP setup and PBR material workflow
- Unity project template pre-configured with URP (2D/3D renderer, post-processing)
- All robot materials use metallic-roughness PBR textures
- Shader Graph examples for custom robot skin shaders

---

### 3. ROS 2 Bridge for Unity: ROS-TCP-Connector vs ROS#

**Question**: Which ROS 2 bridge should be recommended for Unity integration?

**Research Findings**:

**ROS-TCP-Connector (Unity Robotics Hub)**:
- **Pros**:
  - Official Unity Robotics solution, actively maintained by Unity Technologies
  - Native ROS 2 support via `ROS-TCP-Endpoint` server node
  - Excellent documentation and Unity Learn tutorials
  - Built-in support for custom ROS messages, services, and actions
  - Low-latency TCP/IP communication (typical <10ms for local connections)
  - Integrated with Unity's ML-Agents for RL workflows
- **Cons**: Requires separate `ros2 run ros_tcp_endpoint default_server_endpoint` process

**ROS# (Siemens)**:
- **Pros**: Direct ROS 2 node embedding in Unity (no separate endpoint needed)
- **Cons**:
  - Less mature ROS 2 support (primarily designed for ROS 1)
  - Limited documentation for ROS 2 Humble
  - Smaller community compared to ROS-TCP-Connector
  - More complex setup for message generation

**Decision**: **ROS-TCP-Connector (Unity Robotics Hub)**

**Rationale**:
1. **Official support**: Unity Technologies' backing ensures long-term compatibility with Unity 2022 LTS
2. **ROS 2 Humble integration**: Well-tested with Humble Hawksbill
3. **Educational resources**: Unity Learn tutorials align with course teaching methodology
4. **Performance**: TCP-based communication meets <100ms latency requirement (NFR-005)
5. **Extensibility**: Supports custom message types needed for sensor data (LiDAR, depth cameras)

**Implementation Impact**:
- Chapter 4: Step-by-step guide for `ros_tcp_endpoint` setup and configuration
- Include `ros-tcp-endpoint` package in ROS 2 workspace dependencies
- Unity project template includes ROS-TCP-Connector package pre-configured
- Example code: Publishing robot joint states, subscribing to velocity commands

---

### 4. Sensor Simulation Approach: Gazebo Plugins vs Unity Perception Package

**Question**: How should LiDAR, depth cameras, and IMUs be simulated in Gazebo vs Unity?

**Research Findings**:

**Gazebo Sensor Simulation**:
- **Gazebo Classic Plugins**:
  - `libgazebo_ros_ray_sensor.so`: GPU-accelerated LiDAR (publishes `sensor_msgs/LaserScan`, `sensor_msgs/PointCloud2`)
  - `libgazebo_ros_camera.so`: RGB-D camera (publishes `sensor_msgs/Image`, `sensor_msgs/CameraInfo`)
  - `libgazebo_ros_imu_sensor.so`: IMU (publishes `sensor_msgs/Imu` with configurable noise)
  - **Pros**: Native ROS 2 message publishing, realistic physics-based noise models
  - **Cons**: Limited GPU ray tracing (performance bottleneck for dense LiDAR)

**Unity Perception Package**:
- **Features**:
  - Perception Camera component (RGB, depth, semantic segmentation, instance segmentation)
  - Built-in ground truth labeling for ML training datasets
  - High-performance GPU rendering for depth maps
  - **Pros**: Superior visual quality, ML-ready ground truth data
  - **Cons**: Does not natively publish ROS messages (requires custom ROS-TCP bridge code)

**Decision**: **Hybrid approach**
- **Gazebo**: Primary platform for LiDAR and IMU simulation (native ROS 2 integration)
- **Unity**: Primary platform for depth cameras and visual sensors (superior rendering quality)

**Rationale**:
1. **Gazebo strengths**: Physics-based LiDAR ray casting and IMU noise modeling align with robotics standards
2. **Unity strengths**: Depth camera rendering benefits from URP's post-processing and realistic lighting
3. **Pedagogical value**: Students learn sensor simulation in both ecosystems (Chapter 3 requirement)
4. **Constitution Principle I**: Demonstrates tool selection based on technical requirements

**Implementation Impact**:
- Chapter 3, Section 1: Gazebo LiDAR simulation with `gpu_ray` sensor, noise parameter tuning
- Chapter 3, Section 2: Unity Perception Package for depth cameras, bridge to ROS 2 via custom publisher
- Chapter 3, Section 3: Gazebo IMU sensor with gravity vector and angular velocity noise
- Provide comparison table: "When to use Gazebo vs Unity for sensor simulation"

---

### 5. RL Training Framework: stable-baselines3 vs RLlib vs CleanRL

**Question**: Which reinforcement learning framework should be recommended for robot control training in simulated environments?

**Research Findings**:

**stable-baselines3 (SB3)**:
- **Pros**:
  - Beginner-friendly API, excellent documentation
  - Native Gymnasium (gym) environment support
  - Pre-trained models available (PPO, SAC, TD3, A2C, DQN)
  - Lightweight, minimal dependencies
  - Strong ROS 2 community adoption for robotics
- **Cons**: Single-machine only (no distributed training)

**RLlib (Ray)**:
- **Pros**: Scalable distributed training, advanced algorithms (APPO, IMPALA), production-ready
- **Cons**:
  - Steep learning curve, complex API
  - Heavy dependencies (Ray cluster overhead)
  - Overkill for educational single-robot scenarios

**CleanRL**:
- **Pros**: Minimal, readable implementations for learning algorithm internals
- **Cons**: Research-focused, lacks production features (checkpointing, monitoring)

**Decision**: **stable-baselines3 (SB3)**

**Rationale**:
1. **Educational fit**: Clean API matches course learning objectives (Chapter 4: RL training workflows)
2. **Gymnasium integration**: Standard RL environment interface aligns with industry practices
3. **ROS 2 compatibility**: Well-documented integration with `rclpy` for robot control
4. **Performance**: Sufficient for single-robot humanoid training (locomotion, manipulation)
5. **Constitution Principle I**: Balances technical depth with accessibility

**Implementation Impact**:
- Chapter 4: Complete RL workflow example (Gazebo environment → Gymnasium wrapper → SB3 PPO training → policy deployment)
- Example code: Custom `HumanoidWalkEnv` Gymnasium environment with ROS 2 bridge
- Pre-trained policy checkpoint for humanoid balancing task (optional student starting point)
- Training script template with TensorBoard logging

---

### 6. Performance Profiling Tools: RTF and FPS Measurement and Optimization

**Question**: How should Real-Time Factor (RTF) for Gazebo and FPS for Unity be measured and optimized?

**Research Findings**:

**Gazebo Real-Time Factor (RTF) Profiling**:
- **Measurement**:
  - Native `/gazebo/performance_metrics` topic (publishes `real_time_factor`, `sim_time`, `real_time`)
  - Command-line: `gz stats` (Gazebo Classic) or `ign topic -e /stats` (Gazebo Fortress/Garden)
- **Optimization techniques**:
  - Reduce physics update rate (`max_step_size`, `real_time_update_rate` in world file)
  - Simplify collision meshes (use primitives or convex decomposition)
  - Disable unnecessary sensors/cameras during training
  - Enable `use_gpu_ray` for LiDAR sensors
- **Target**: RTF ≥ 0.9 for real-time simulation (NFR-001)

**Unity FPS Profiling**:
- **Measurement**:
  - Unity Profiler (CPU/GPU frame time breakdown)
  - Stats overlay (Application.targetFrameRate, Screen.currentResolution.refreshRate)
  - Custom FPS counter script (movingAvgFPS = Mathf.RoundToInt(1.0f / Time.unscaledDeltaTime))
- **Optimization techniques**:
  - LOD (Level of Detail) groups for robot meshes
  - Occlusion culling for complex scenes
  - URP Renderer settings: disable MSAA, reduce shadow cascades
  - Optimize physics (reduce fixed timestep to 0.02s for robotics)
- **Target**: 30 FPS minimum, 60 FPS preferred (NFR-002)

**Decision**: **Integrated profiling approach**
- Gazebo: Use `/gazebo/performance_metrics` topic with Python logging script
- Unity: Use Unity Profiler + custom FPS overlay UI element

**Rationale**:
1. **Student visibility**: Real-time metrics help students understand simulation performance trade-offs
2. **Debugging aid**: Performance bottlenecks (heavy meshes, excessive contacts) become immediately apparent
3. **Constitution Principle I**: Teaches professional simulation optimization practices

**Implementation Impact**:
- Chapter 1: Gazebo RTF monitoring script, optimization checklist
- Chapter 2: Unity Profiler tutorial, FPS overlay UI component
- Chapter 4: Performance budgeting guide (RTF/FPS requirements for RL training loops)
- Example optimized world files and Unity scenes demonstrating best practices

---

## Summary and Cross-Cutting Decisions

### Technology Stack (Final)
- **Gazebo**: Classic 11 (primary), with Fortress/Garden migration notes
- **Unity**: 2022 LTS with Universal Render Pipeline (URP)
- **ROS 2 Bridge**: ROS-TCP-Connector (Unity Robotics Hub)
- **Sensor Simulation**: Gazebo for LiDAR/IMU, Unity Perception for depth cameras
- **RL Framework**: stable-baselines3 (SB3) with Gymnasium
- **Profiling**: Gazebo `/performance_metrics`, Unity Profiler + custom FPS overlay

### Key Implementation Risks and Mitigations

1. **Gazebo Classic EOL Risk**:
   - **Mitigation**: Include "Migration to Gazebo Fortress/Garden" guide in Chapter 4
   - Provide side-by-side SDF syntax comparison table
   - Optional advanced exercises using `ros_gz_bridge`

2. **Unity-ROS 2 Latency Risk**:
   - **Mitigation**: Benchmark ROS-TCP-Connector latency in Chapter 4 examples
   - Provide optimization guide (message throttling, TCP buffer tuning)
   - Set clear expectations: <100ms acceptable for educational simulations (NFR-005)

3. **Cross-Platform Compatibility Risk**:
   - **Mitigation**: Docker-based Gazebo testing on Windows via WSL2
   - Unity URP ensures Windows/Linux/macOS compatibility
   - Provide platform-specific setup guides in quickstart.md

### Alignment with Constitution Principles

- **Principle I (Technical Accuracy & Depth)**: All technology choices based on industry standards (ROS 2, Gazebo, Unity URP)
- **Principle II (Modular Structure Integrity)**: 4-chapter progression maintained (Physics → Rendering → Sensors → Integration)
- **Principle III (AI-Native Development)**: Simulation environments designed for RL workflows (SB3 + Gymnasium)
- **Principle VI (Professional UI/UX Standards)**: URP rendering, FPS optimization, accessible hardware requirements

---

**Research Status**: ✅ **COMPLETE**
**Next Phase**: Phase 1 - Generate `data-model.md`, `contracts/chapter-content-contract.md`, and `quickstart.md`
