# Feature Specification: Module 2 - Digital Twin Simulation (Gazebo & Unity)

**Feature Branch**: `002-module-2-digital-twin`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "Create the Specification for the entire Module 2 of the Physical AI & Humanoid Robotics Textbook. This module focuses on Digital Twin Simulation (Gazebo & Unity). The specification must ensure the generation of four distinct chapters: 1) Simulating Physics, Gravity, and Collisions in Gazebo, 2) High-fidelity Rendering and Human-Robot Interaction in Unity, 3) Simulating Sensors (LiDAR, Depth Cameras, and IMUs), 4) Gazebo and Unity Integration Best Practices for Physical AI."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Mastering Physics Simulation in Gazebo (Priority: P1)

As a robotics engineer or researcher, I want to understand how to simulate realistic physics, gravity, and collision dynamics in Gazebo so that I can create accurate digital twins of humanoid robots for testing control algorithms before physical deployment.

**Why this priority**: Physics simulation is the foundation of digital twin development. Without accurate physics modeling (gravity, inertia, friction, collisions), all subsequent simulation work (rendering, sensors, integration) will produce unrealistic results. This must be mastered first.

**Independent Test**: Can be fully tested by having a student read Chapter 1, create a Gazebo world with custom physics parameters, spawn a humanoid URDF model from Module 1, configure collision geometries and contact dynamics, and demonstrate the robot responding realistically to gravity, ground contact, and external forces.

**Acceptance Scenarios**:

1. **Given** no prior Gazebo experience, **When** a student completes Chapter 1 learning objectives, **Then** they can explain Gazebo architecture (client-server model), the ODE/Bullet/Simbody/DART physics engines, and configure world files with gravity, timesteps, and solver parameters
2. **Given** a humanoid URDF from Module 1, **When** a student follows the Gazebo spawn tutorials, **Then** they can launch Gazebo, load the robot model, visualize it in the 3D environment, and observe correct physics behavior (standing, falling, joint movements)
3. **Given** collision geometry examples, **When** a student modifies URDF collision tags, **Then** they can define primitive shapes (box, cylinder, sphere) and mesh-based collision geometries, configure surface properties (friction coefficients, restitution, contact stiffness/damping), and test collision detection with obstacles
4. **Given** force/torque application techniques, **When** a student experiments with external forces, **Then** they can apply impulses to links, simulate contact forces, and observe realistic robot responses (e.g., pushing the robot causes it to tip or step)
5. **Given** performance optimization guidance, **When** a student builds complex scenes, **Then** they can balance simulation accuracy vs real-time performance by adjusting physics engine parameters, collision simplification, and update rates

---

### User Story 2 - Creating High-Fidelity Visualizations in Unity (Priority: P2)

As a robotics developer creating demonstrations or human-robot interaction scenarios, I want to learn how to render high-fidelity visualizations of humanoid robots in Unity with realistic materials, lighting, and animation so that I can create compelling visual representations and interactive applications.

**Why this priority**: While Gazebo excels at physics, Unity provides photorealistic rendering and rich UI/UX capabilities essential for demonstrations, user studies, and human-robot interaction research. This builds on P1 by adding the visual fidelity layer.

**Independent Test**: Can be fully tested by having a student complete Chapter 2, import a humanoid robot model into Unity, configure realistic materials and lighting (PBR workflow), implement basic human-robot interaction (keyboard/mouse control, UI panels), and produce a high-quality rendered scene or video demonstration.

**Acceptance Scenarios**:

1. **Given** Unity fundamentals covered in the chapter, **When** a student completes the installation and setup, **Then** they can install Unity Hub, create a new 3D project with HDRP/URP rendering pipeline, and navigate the Unity Editor interface
2. **Given** robot model import tutorials, **When** a student follows the URDF-to-Unity conversion steps, **Then** they can import URDF files (using tools like URDF Importer or manual conversion), configure articulation bodies for joints, and verify kinematic/dynamic properties match the original model
3. **Given** PBR material and lighting examples, **When** a student applies rendering techniques, **Then** they can create realistic materials (metallic, roughness, normal maps), set up HDRI environment lighting, configure shadows and reflections, and achieve photorealistic output
4. **Given** human-robot interaction patterns, **When** a student implements UI elements, **Then** they can create canvas-based UI panels for robot status display, implement keyboard/gamepad control for robot teleoperation, and add interactive elements (buttons, sliders for joint control)
5. **Given** animation and timeline tools, **When** a student creates demonstrations, **Then** they can record robot movements, create cinematic camera paths, use the Timeline editor for sequenced animations, and export high-quality videos or interactive WebGL builds

---

### User Story 3 - Simulating Realistic Sensor Data (Priority: P3)

As a perception engineer or autonomy researcher, I want to master the simulation of robotic sensors (LiDAR, depth cameras, IMUs) in both Gazebo and Unity so that I can test perception algorithms and sensor fusion systems in realistic virtual environments before deploying to hardware.

**Why this priority**: Sensor simulation is critical for testing perception and navigation algorithms. It depends on physics (P1) for accurate sensor placement and movement, and benefits from high-fidelity rendering (P2) for camera-based sensors. This chapter integrates both platforms.

**Independent Test**: Can be fully tested by having a student complete Chapter 3, add LiDAR, depth camera, and IMU sensors to a simulated humanoid robot in Gazebo and Unity, visualize sensor data (point clouds, depth images, IMU readings), and process the data with ROS 2 nodes to demonstrate working perception.

**Acceptance Scenarios**:

1. **Given** sensor fundamentals explained in the chapter, **When** a student studies sensor types, **Then** they can describe LiDAR principles (time-of-flight, scanning patterns), depth camera technologies (stereo, structured light, ToF), and IMU components (accelerometer, gyroscope, magnetometer)
2. **Given** Gazebo sensor plugin tutorials, **When** a student adds sensors to URDF/SDF models, **Then** they can configure GPU-based ray sensor for LiDAR (scan rate, resolution, range, noise parameters), depth camera plugin (resolution, FOV, clip distances), and IMU plugin (noise models, update rate), and verify sensor data publications via ROS 2 topics
3. **Given** Unity sensor simulation examples, **When** a student implements sensors in Unity, **Then** they can use the Perception package for camera sensors (semantic segmentation, bounding boxes, depth), implement custom LiDAR raycasting scripts, and simulate IMU using Unity's physics system
4. **Given** sensor noise and calibration sections, **When** a student configures realistic sensors, **Then** they can add Gaussian noise to sensor readings, simulate calibration errors (bias, scale factors), model environmental effects (sunlight on cameras, rain on LiDAR), and test perception algorithm robustness
5. **Given** sensor fusion scenarios, **When** a student integrates multiple sensors, **Then** they can synchronize sensor data timestamps, fuse IMU and camera data for visual-inertial odometry, combine LiDAR and depth camera for 3D mapping, and validate against ground truth from simulation

---

### User Story 4 - Integrating Gazebo and Unity for Physical AI Workflows (Priority: P4)

As a Physical AI practitioner, I want to learn best practices for integrating Gazebo (physics) and Unity (visualization/interaction) in a unified workflow so that I can leverage the strengths of both platforms for training AI agents, conducting user studies, and deploying embodied AI systems.

**Why this priority**: This is the capstone of Module 2, combining physics simulation (Gazebo) with high-fidelity rendering (Unity) for advanced workflows like reinforcement learning training (physics in Gazebo) with visual rendering (Unity), or bidirectional sync for demonstrations. Requires solid understanding of P1-P3.

**Independent Test**: Can be fully tested by having a student complete Chapter 4, set up a synchronized Gazebo-Unity pipeline (e.g., using ROS 2 bridges), demonstrate a working integration example (such as physics simulation in Gazebo driving Unity visualization in real-time, or training an RL agent with Gazebo physics and Unity rendering), and document the integration architecture.

**Acceptance Scenarios**:

1. **Given** integration architecture patterns, **When** a student studies Chapter 4, **Then** they can explain different integration approaches (ROS 2 bridge for real-time sync, offline data transfer, co-simulation frameworks), understand trade-offs (latency vs fidelity, performance vs complexity), and select appropriate methods for different use cases
2. **Given** ROS 2 bridge setup tutorials, **When** a student configures synchronization, **Then** they can install and configure ros2-unity or ros#, set up topic/service communication between Gazebo and Unity, synchronize robot state (joint positions, velocities) and sensor data in real-time, and handle network latency
3. **Given** RL training workflow examples, **When** a student implements Physical AI pipelines, **Then** they can use Gazebo for physics-based training (running RL policy in Gazebo with fast simulation speeds), export trained policies, deploy in Unity for high-fidelity visualization/validation, and iterate on the training-deployment cycle
4. **Given** human-in-the-loop integration patterns, **When** a student creates interactive demonstrations, **Then** they can implement scenarios where human operators interact via Unity UI while physics runs in Gazebo, teleoperate robots with haptic feedback, and conduct user studies with realistic physics and appealing visuals
5. **Given** performance optimization strategies, **When** a student deploys integrated systems, **Then** they can profile simulation performance (Gazebo RTF - Real-Time Factor, Unity FPS), optimize message passing (compression, selective updates), run headless Gazebo for faster training, and use Unity for on-demand visualization

---

### Edge Cases

- **Gazebo vs Gazebo Ignition/Fortress/Garden**: How does content address the Gazebo Classic to Gazebo (new generation) transition? (Approach: Primary focus on Gazebo Classic for stability, with callout boxes for Gazebo Fortress/Garden migration paths and feature comparisons)
- **Unity Version Compatibility**: How are differences between Unity 2021 LTS, 2022 LTS, and Unity 6 handled? (Approach: Target Unity 2022 LTS for balance of stability and modern features, note version-specific differences in installation section)
- **URDF Import Limitations**: What if complex URDF models fail to import into Unity due to joint/constraint limitations? (Approach: Troubleshooting section covering common issues - mesh format conversion, joint type mapping, manual articulation body configuration)
- **Sensor Simulation Accuracy**: How realistic are simulated sensors compared to hardware? (Approach: Dedicated section on simulation-to-real gap, sensor noise modeling best practices, validation techniques using real sensor data when available)
- **Platform-Specific Issues**: How are Linux (Gazebo native) vs Windows (Unity native) workflow differences handled? (Approach: Provide setup instructions for both platforms, recommend Linux for Gazebo + WSL2 for Unity on Windows, or dual-boot/VM configurations)
- **Performance Constraints**: What if student hardware cannot run Gazebo + Unity simultaneously in real-time? (Approach: Provide performance profiling guidance, recommend offline integration (record Gazebo data, replay in Unity), and cloud/remote simulation options)
- **Missing ROS 2 Integration**: What if Unity-ROS 2 communication fails due to DDS/network issues? (Approach: Detailed troubleshooting section for DDS domain configuration, firewall rules, and alternative communication methods like ROS Bridge)

## Requirements *(mandatory)*

### Functional Requirements

#### Chapter 1: Gazebo Physics Simulation Requirements

- **FR-001**: Chapter 1 MUST include 3-5 learning objectives covering Gazebo architecture, physics engines (ODE, Bullet, DART), world configuration, and collision dynamics
- **FR-002**: Chapter 1 MUST explain theoretical foundations of rigid body dynamics, contact mechanics, numerical integration methods (Euler, RK4), and physics engine solver algorithms
- **FR-003**: Chapter 1 MUST provide step-by-step tutorials for installing Gazebo, creating world files (.world SDF format), spawning URDF robots, and configuring physics parameters (gravity, timestep, solver iterations)
- **FR-004**: Chapter 1 MUST include executable examples demonstrating gravity effects on humanoid robots, collision detection between robot and environment, joint friction and damping, and external force application
- **FR-005**: Chapter 1 MUST cover URDF/SDF collision geometry configuration (primitive shapes, mesh-based collisions), surface contact properties (friction coefficients: mu, mu2; restitution; contact stiffness/damping), and collision optimization techniques
- **FR-006**: Chapter 1 MUST provide practical examples including humanoid robot standing/walking in Gazebo, object manipulation with contact forces, and terrain interaction (stairs, slopes, uneven ground)
- **FR-007**: Chapter 1 MUST include exercises ranging from beginner (modify gravity, spawn models) to advanced (tune contact parameters for stable bipedal standing, create custom physics scenarios)
- **FR-008**: Chapter 1 MUST explain Gazebo client-server architecture, gzserver (physics simulation) vs gzclient (visualization), headless mode for faster-than-real-time simulation, and ROS 2 integration via gazebo_ros_pkgs
- **FR-009**: Chapter 1 MUST include a "Further Reading" section with references to Gazebo documentation, ODE/Bullet/DART engine docs, and research papers on physics-based robot simulation

#### Chapter 2: Unity Rendering and Interaction Requirements

- **FR-010**: Chapter 2 MUST include 3-5 learning objectives covering Unity basics, URDF import workflows, PBR materials/lighting, and human-robot interaction UI development
- **FR-011**: Chapter 2 MUST explain Unity fundamentals: GameObjects, Components, Prefabs, Scenes, the Unity Editor interface, and rendering pipelines (Built-in, URP, HDRP)
- **FR-012**: Chapter 2 MUST provide installation and setup tutorials for Unity Hub, Unity Editor (2022 LTS recommended), and relevant packages (URDF Importer, Articulation Body extensions, ROS-TCP-Connector)
- **FR-013**: Chapter 2 MUST demonstrate URDF-to-Unity import workflows using Unity Robotics Hub URDF Importer package, including mesh file conversion (COLLADA, STL, OBJ), material assignment, and articulation body configuration for joints
- **FR-014**: Chapter 2 MUST cover PBR (Physically Based Rendering) material creation: albedo, metallic, smoothness, normal maps, emission; HDRI environment lighting setup; real-time shadows and reflections; and post-processing effects (bloom, color grading, ambient occlusion)
- **FR-015**: Chapter 2 MUST include tutorials for implementing human-robot interaction: Canvas UI for robot status displays, keyboard/gamepad input for teleoperation, interactive UI elements (buttons, sliders, dropdowns) controlling robot parameters, and camera controls (orbit, zoom, first-person view)
- **FR-016**: Chapter 2 MUST provide examples of humanoid robot visualization in Unity with realistic materials (metal chassis, rubber grippers, LED indicators), animated joints synchronized with simulation data, and cinematic camera sequences
- **FR-017**: Chapter 2 MUST cover Unity Timeline editor for creating animated demonstrations, Cinemachine for cinematic camera paths, and exporting options (standalone builds, WebGL for web deployment, video recording)
- **FR-018**: Chapter 2 MUST include exercises ranging from beginner (import robot, apply materials) to advanced (create interactive demo with UI, implement gesture-based robot control)
- **FR-019**: Chapter 2 MUST include a "Further Reading" section with Unity Learn tutorials, PBR material guides, and human-robot interaction research references

#### Chapter 3: Sensor Simulation Requirements

- **FR-020**: Chapter 3 MUST include 3-5 learning objectives covering sensor types (LiDAR, depth cameras, IMUs), sensor simulation in Gazebo and Unity, noise modeling, and ROS 2 sensor data integration
- **FR-021**: Chapter 3 MUST explain theoretical foundations of robotic sensors: LiDAR principles (time-of-flight, phase shift, triangulation), depth camera technologies (stereo vision, structured light, time-of-flight), IMU sensor fusion (complementary filters, Kalman filters), and sensor noise characteristics (Gaussian noise, bias, drift)
- **FR-022**: Chapter 3 MUST provide Gazebo sensor simulation tutorials covering: GPU-based ray sensor for LiDAR (with SDF configuration for scan parameters, range, resolution, noise), depth camera plugin (image width/height, FOV, clip distances, ROS 2 topic publishing), and IMU sensor plugin (noise parameters: accelerometer/gyroscope bias and noise density)
- **FR-023**: Chapter 3 MUST include Unity sensor simulation examples using Unity Perception package for camera-based sensors (RGB cameras, semantic segmentation, instance segmentation, bounding boxes, keypoints), custom LiDAR implementation via raycasting (Physics.Raycast for 3D scanning patterns), and IMU simulation using Unity's physics system (Rigidbody acceleration/angular velocity)
- **FR-024**: Chapter 3 MUST demonstrate sensor data visualization: point cloud visualization in RViz from LiDAR topics, depth image display from depth camera topics, IMU data plotting (acceleration, gyroscope readings), and ROS 2 topic inspection using ros2 topic echo
- **FR-025**: Chapter 3 MUST cover sensor noise modeling: adding Gaussian noise to sensor readings, simulating systematic errors (bias, scale factors, misalignment), environmental effects (sunlight glare on cameras, rain/fog on LiDAR), and validation against real sensor data distributions
- **FR-026**: Chapter 3 MUST provide practical examples of multi-sensor systems: visual-inertial odometry using camera + IMU, 3D mapping with LiDAR + depth camera, sensor fusion for localization (EKF, particle filters), and ground truth comparison from simulation state
- **FR-027**: Chapter 3 MUST include exercises ranging from beginner (add LiDAR to robot, visualize point cloud) to advanced (implement sensor fusion algorithm, calibrate simulated sensors to match real sensor characteristics)
- **FR-028**: Chapter 3 MUST include a "Further Reading" section with references to sensor datasheets (Velodyne LiDAR, Intel RealSense, Ouster, Bosch IMU), ROS 2 sensor message types (sensor_msgs), and sensor simulation validation research

#### Chapter 4: Gazebo-Unity Integration Requirements

- **FR-029**: Chapter 4 MUST include 3-5 learning objectives covering integration architectures, ROS 2 bridge setup, real-time synchronization, RL training workflows, and performance optimization
- **FR-030**: Chapter 4 MUST explain integration architecture patterns: real-time bidirectional sync (Gazebo physics ↔ Unity rendering), unidirectional pipelines (Gazebo → Unity for visualization, Unity → Gazebo for human commands), offline data transfer (record Gazebo bag files, replay in Unity), and co-simulation frameworks (combining timesteps)
- **FR-031**: Chapter 4 MUST provide ROS 2 bridge setup tutorials: installing ROS-TCP-Connector for Unity, configuring ROS 2 endpoints (IP addresses, ports), setting up message publishers/subscribers for robot state (joint_states, tf transforms), and synchronizing simulation time
- **FR-032**: Chapter 4 MUST demonstrate synchronized Gazebo-Unity examples: running Gazebo physics simulation with headless mode (gzserver only), streaming robot joint states and sensor data to Unity via ROS 2 topics, rendering real-time visualization in Unity, and measuring synchronization latency
- **FR-033**: Chapter 4 MUST cover RL (Reinforcement Learning) training workflows for Physical AI: using Gazebo for fast physics-based training with OpenAI Gym / Gymnasium interface, training policies with PPO/SAC algorithms (stable-baselines3, RLlib), exporting trained models (ONNX, PyTorch SavedModel), deploying in Unity for high-fidelity validation and demonstration
- **FR-034**: Chapter 4 MUST include human-in-the-loop integration examples: implementing teleoperation where human controls robot via Unity UI and physics runs in Gazebo, collecting human demonstration data for imitation learning, and conducting user studies with Unity frontend and Gazebo backend
- **FR-035**: Chapter 4 MUST provide performance optimization strategies: profiling Gazebo RTF (Real-Time Factor) and Unity FPS, optimizing ROS 2 message frequency (throttling high-rate topics), using message compression, running headless Gazebo for maximum training speed, and selective rendering in Unity (on-demand screenshot capture vs continuous visualization)
- **FR-036**: Chapter 4 MUST explain cloud and distributed simulation: running Gazebo on remote servers (AWS, GCP, Azure), connecting Unity clients over network, Docker containerization for Gazebo simulation, and scaling with multiple parallel simulation instances for RL training
- **FR-037**: Chapter 4 MUST include exercises ranging from beginner (set up ROS 2 bridge, sync robot state) to advanced (implement full RL training pipeline with Gazebo+Unity, deploy trained policy, measure sim-to-real transfer)
- **FR-038**: Chapter 4 MUST include Module 2 summary section recapping Digital Twin capabilities (physics, rendering, sensors, integration) and preview Module 3 (AI-Robot Brain with NVIDIA Isaac Sim and VLA models)
- **FR-039**: Chapter 4 MUST include a "Further Reading" section with references to ROS 2 bridge documentation, RL for robotics papers (sim-to-real transfer, domain randomization), and co-simulation frameworks

### Key Entities *(feature involves educational content structure)*

- **Chapter**: Represents one of the four distinct chapters in Module 2. Contains learning objectives, theoretical foundations, hands-on tutorials, code examples, exercises, and references. Each chapter is independently testable.
- **Learning Objective**: Measurable outcome using Bloom's taxonomy action verbs (understand, apply, analyze, create). Must be technology-specific (unlike success criteria) but testable.
- **Code Example**: Executable demonstration (Gazebo launch files, Unity C# scripts, URDF/SDF models, ROS 2 Python nodes). Includes README, dependencies, and test instructions.
- **Sensor Model**: Simulated sensor configuration with parameters (resolution, noise, update rate). Includes Gazebo plugin config (SDF) and Unity component scripts.
- **Integration Pipeline**: Architecture connecting Gazebo and Unity. Defined by message flow (ROS 2 topics/services), synchronization method, and performance characteristics.
- **Exercise**: Progressive challenge (beginner/intermediate/advanced) with clear acceptance criteria. Students demonstrate competency by completing exercises.

## Success Criteria *(mandatory)*

### Measurable Outcomes

#### Chapter Completion Criteria

- **SC-001**: Each of the 4 chapters includes exactly 3-5 learning objectives using measurable action verbs (explain, configure, implement, demonstrate)
- **SC-002**: Each chapter includes at least 3 executable code examples or simulation scenarios (Gazebo worlds, Unity projects, sensor configs) tested in the respective platform
- **SC-003**: Each chapter provides at least 5 exercises with 2 beginner, 2 intermediate, and 1 advanced exercise, each with clear acceptance criteria
- **SC-004**: Each chapter includes at least 1 mathematical formulation or physical principle (e.g., rigid body dynamics equations, sensor noise models, rendering equations)
- **SC-005**: Each chapter includes a "Further Reading" section with at least 5 curated references to official documentation, research papers, or tutorials

#### Content Quality Criteria

- **SC-006**: 100% of Gazebo examples successfully load and simulate when tested in Gazebo Classic 11 or Gazebo Fortress/Garden (latest stable versions)
- **SC-007**: 100% of Unity projects successfully build and run when tested in Unity 2022 LTS on Windows 10/11 and Ubuntu 22.04/24.04
- **SC-008**: All explanatory text maintains Flesch-Kincaid grade level between 12-16 (college/undergraduate level), balancing technical accuracy with readability
- **SC-009**: All diagrams (simulation architecture diagrams, sensor raycast illustrations, integration pipelines) include descriptive alt text for accessibility
- **SC-010**: All Gazebo SDF/URDF code examples follow proper XML formatting, validate with check_urdf/gz sdf tools, and include inline comments explaining key parameters

#### Learning Effectiveness Criteria

- **SC-011**: Students completing Chapter 1 can configure Gazebo physics parameters to achieve stable humanoid standing (>5 seconds without falling) in simulation
- **SC-012**: Students completing Chapter 2 can import a humanoid URDF into Unity and produce a rendered image or video with photorealistic materials and lighting (subjectively realistic)
- **SC-013**: Students completing Chapter 3 can configure LiDAR and depth camera sensors in Gazebo or Unity, visualize sensor data in RViz, and process point clouds with ROS 2 nodes
- **SC-014**: Students completing Chapter 4 can demonstrate a working Gazebo-Unity integration with synchronized robot state visualization achieving <100ms latency between physics update and visual rendering
- **SC-015**: 80% of students completing exercises achieve correct implementations on first or second attempt (validated through automated tests or manual grading rubrics)

#### UX and Accessibility Criteria

- **SC-016**: Docusaurus site pages for Module 2 load in under 2 seconds on standard broadband connections and achieve Lighthouse performance score >90
- **SC-017**: All Module 2 pages are responsive and functional on viewport widths from 320px (mobile) to 1920px (desktop), with code examples horizontally scrollable
- **SC-018**: All Gazebo tutorials include screenshots showing expected visual output at key steps (model spawning, physics behavior, sensor visualization)
- **SC-019**: All Unity tutorials include screenshots or embedded videos demonstrating final visual output and interactive UI elements

#### Technical Accuracy Criteria

- **SC-020**: All physics equations and sensor models cite authoritative sources (textbooks, research papers, official documentation) and match real-world physical principles within engineering approximation bounds
- **SC-021**: All Gazebo physics parameters (gravity, friction, damping, contact stiffness) use realistic values validated against real robot data or standard simulation benchmarks
- **SC-022**: All Unity sensor simulations (LiDAR raycasting, depth cameras) produce data formats compatible with ROS 2 standard message types (sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/PointCloud2)

## Assumptions *(mandatory)*

- **Prerequisite Knowledge**: Students have completed Module 1 (ROS 2 Fundamentals) or possess equivalent knowledge: ROS 2 basics, URDF modeling, package management, and command-line proficiency
- **Software Availability**: Students have access to Ubuntu 22.04/24.04 (native or VM/WSL2) for Gazebo, and Windows 10/11 or Ubuntu for Unity development. Gazebo Classic 11 or Gazebo Fortress/Garden is installable.
- **Hardware Requirements**: Students have a computer with at least 8GB RAM, 4 CPU cores, dedicated GPU recommended for Unity rendering (but not required; integrated graphics supported with reduced visual quality). No physical robot hardware required.
- **Gazebo-Unity Versions**: Primary target versions are Gazebo Classic 11 (widely adopted, stable) with notes for Gazebo Fortress/Garden migration, and Unity 2022 LTS for long-term stability. Version-specific instructions provided where behavior differs.
- **ROS 2 Integration**: Students have ROS 2 Humble installed (from Module 1) and can install additional packages: gazebo_ros_pkgs for Gazebo, and Unity Robotics Hub packages for Unity-ROS 2 communication.
- **Internet Access**: Students can download Unity Hub, Unity Editor packages, Gazebo models from online repositories, and access documentation/tutorials during learning.
- **Simulation-to-Real Gap**: Students understand that simulated physics/sensors are approximations of reality. The module focuses on simulation skills; sim-to-real transfer techniques are noted but detailed treatment is deferred to advanced courses.
- **No Prior Gazebo/Unity Experience**: Content assumes students are new to both Gazebo and Unity but have general software development skills (editing configuration files, running commands, basic debugging).
- **English Proficiency**: Content is written in English at undergraduate technical level. Assumes students can read technical documentation and follow written tutorials.
- **Open-Source Focus**: All required software (Gazebo, ROS 2) is open-source and free. Unity Personal Edition is free for students/hobbyists; Unity Pro is optional for advanced features but not required for course content.

## Out of Scope *(mandatory)*

- **Physical Hardware Deployment**: Deploying simulated robots to real humanoid hardware, hardware-in-the-loop (HIL) simulation, and motor driver integration are out of scope. Module 2 focuses on pure simulation; hardware deployment is covered in future modules.
- **Advanced RL Algorithms**: Deep dive into reinforcement learning theory (PPO, SAC, TD3 algorithms) is out of scope. Chapter 4 introduces RL workflows at a high level; students should take dedicated ML courses for algorithm details.
- **Custom Physics Engine Development**: Implementing custom physics solvers, collision detection algorithms, or extending ODE/Bullet/DART source code is out of scope. Students use existing physics engines as black boxes with configurable parameters.
- **Photogrammetry and 3D Scanning**: Capturing real-world objects via photogrammetry, LiDAR scanning, or depth camera reconstruction for simulation assets is out of scope. Students use provided 3D models or simple primitive shapes.
- **Multiplayer/Networked Simulation**: Implementing multi-user collaborative simulation environments, distributed physics computation, or MMO-style robot simulations is out of scope. Focus is single-user, single-robot workflows with optional cloud deployment.
- **VR/AR Integration**: Virtual Reality (VR) or Augmented Reality (AR) interfaces for robot teleoperation or visualization are out of scope. Standard desktop/laptop interaction only.
- **Gazebo Plugins in C++**: Developing custom Gazebo sensor plugins, model plugins, or world plugins in C++ is out of scope. Students use existing plugins configured via SDF/URDF XML; custom plugin development is an advanced topic.
- **Unity Shader Programming**: Writing custom Unity shaders (HLSL/ShaderLab), render pipeline modifications, or GPU optimization is out of scope. Students use Unity's built-in PBR shaders and post-processing stack.
- **Sensor Calibration Algorithms**: Implementing camera calibration (intrinsic/extrinsic parameters), LiDAR-camera calibration, or IMU calibration routines is out of scope. Students configure sensor parameters but do not develop calibration algorithms.
- **SLAM and Localization Algorithms**: Simultaneous Localization and Mapping (SLAM), Monte Carlo Localization (MCL), or visual odometry implementation is out of scope. Sensor simulation enables these algorithms (students can run existing SLAM packages), but algorithm development is covered in perception courses.
- **Real-Time Operating Systems**: Deploying simulations on RTOS, real-time scheduling, or deterministic simulation guarantees are out of scope. Gazebo/Unity run on standard Linux/Windows without hard real-time constraints.
- **Cloud Simulation at Scale**: Enterprise-level cloud orchestration (Kubernetes for simulation clusters), cost optimization for AWS/GCP/Azure, or distributed training with 1000+ parallel Gazebo instances is mentioned but detailed setup is out of scope.

## Dependencies *(mandatory)*

### External Dependencies

- **ROS 2 Humble Hawksbill**: Required for Gazebo-ROS 2 integration (gazebo_ros_pkgs) and Unity-ROS 2 communication (ROS-TCP-Connector). Students must have ROS 2 Humble installed from Module 1.
- **Gazebo Classic 11 or Gazebo Fortress/Garden**: Physics simulation platform. Installation instructions provided for Ubuntu 22.04/24.04 via apt packages or source builds.
- **Unity 2022 LTS**: Rendering and interaction platform. Students download Unity Hub and Unity Editor 2022 LTS (free Personal Edition sufficient).
- **Unity Packages**: URDF Importer, Articulation Body extensions, ROS-TCP-Connector (from Unity Robotics Hub GitHub). Installation via Unity Package Manager.
- **Python 3.10+**: Required for ROS 2 nodes, sensor processing scripts, and RL training code. Should already be installed from Module 1.
- **Gazebo Models and Plugins**: Standard Gazebo model database (downloaded automatically), sensor plugins (libgazebo_ros_ray_sensor.so, libgazebo_ros_camera.so, libgazebo_ros_imu_sensor.so) from gazebo_ros_pkgs.
- **Docker (Optional)**: For cloud deployment scenarios and reproducible Gazebo environments. Installation instructions provided but not mandatory for core learning.
- **RL Libraries (Chapter 4 only)**: stable-baselines3, Gymnasium, PyTorch (CPU version sufficient). Installed via pip with requirements.txt provided.

### Content Dependencies

- **Module 1 Completion**: Students must complete Module 1 (ROS 2 Fundamentals) before starting Module 2, or demonstrate equivalent proficiency in:
  - ROS 2 topics, services, actions (Chapter 1 prerequisite for all Gazebo-ROS 2 integration)
  - URDF modeling (Chapter 3 prerequisite for importing robots into Gazebo and Unity)
  - ROS 2 package management (Chapter 4 prerequisite for organizing simulation workspaces)
- **Chapter Sequential Dependencies within Module 2**:
  - Chapter 2 (Unity) has weak dependency on Chapter 1 (Gazebo) — students should complete Chapter 1 first to understand physics fundamentals before adding visual rendering, but Unity can be learned independently if needed
  - Chapter 3 (Sensors) depends on both Chapter 1 (for Gazebo sensor simulation) and Chapter 2 (for Unity sensor simulation) — students must complete at least one of the two platforms before attempting sensor simulation
  - Chapter 4 (Integration) strongly depends on Chapters 1 and 2 — students must complete both Gazebo and Unity chapters before attempting integration workflows

### Platform Dependencies

- **Operating System**: Linux (Ubuntu 22.04/24.04 recommended) for Gazebo Classic. Unity runs on Linux, Windows 10/11, or macOS. For Gazebo on Windows, WSL2 with Ubuntu is supported but performance may vary.
- **Graphics Drivers**: Up-to-date GPU drivers for Unity rendering (NVIDIA, AMD, Intel). Gazebo visualization (gzclient) requires OpenGL support but headless mode (gzserver only) works without GPU.
- **Network Configuration**: For Gazebo-Unity ROS 2 bridge, proper network configuration allowing inter-process communication (firewall rules, DDS domain ID). Localhost setup works by default; cross-machine setup requires network troubleshooting.

## Cross-Chapter Requirements *(mandatory)*

- **FR-040**: All chapters MUST follow the 9-section template structure defined in Module 1: Overview, Learning Objectives, Theoretical Foundations, Hands-On Implementation, Practical Examples, Exercises, Further Reading, Summary, Troubleshooting
- **FR-041**: All chapters MUST include cross-references to relevant Module 1 content (e.g., Chapter 1 references URDF from Module 1 Chapter 3; Chapter 3 references ROS 2 topics from Module 1 Chapter 1)
- **FR-042**: All code examples MUST follow consistent file path conventions: `examples/module-2/chapter-N/example-XX-name/` with README.md, launch files, and test scripts
- **FR-043**: All Gazebo examples MUST include:
  - Launch file (.launch.py for ROS 2) to spawn the simulation
  - World file (.world) or SDF model files
  - README.md with prerequisites, execution instructions, and expected behavior
  - Test script verifying the example runs without errors
- **FR-044**: All Unity examples MUST include:
  - Unity project folder structure (Assets/, Packages/, ProjectSettings/)
  - README.md with Unity version, required packages, and import instructions
  - Screenshots or video demonstration of expected output
  - Script files (.cs) with inline comments explaining key logic
- **FR-045**: All chapters MUST use consistent terminology aligned with Module 1 and official documentation (e.g., "topic" not "stream", "node" not "process", "URDF" not "robot description file")
- **FR-046**: All mathematical formulations MUST use KaTeX/LaTeX notation for rendering in Docusaurus
- **FR-047**: All diagrams MUST be created using Mermaid syntax (for architecture/flowcharts) or SVG/PNG files (for screenshots and visual outputs)
- **FR-048**: Chapter 4 MUST include a Module 2 summary section (200-300 words) recapping key skills (physics simulation, high-fidelity rendering, sensor modeling, platform integration) and preview Module 3 content (AI-Robot Brain with NVIDIA Isaac Sim and Vision-Language-Action models)

## Non-Functional Requirements *(include if relevant)*

### Performance Requirements

- **NFR-001**: Gazebo simulation examples MUST achieve Real-Time Factor (RTF) ≥ 0.9 (90% of real-time) on reference hardware (8GB RAM, 4-core CPU, integrated graphics) with default physics parameters
- **NFR-002**: Unity rendering examples MUST achieve ≥30 FPS on reference hardware (8GB RAM, 4-core CPU, integrated graphics with URP pipeline) at 1080p resolution
- **NFR-003**: Gazebo-Unity synchronized examples (Chapter 4) MUST achieve end-to-end latency <100ms (physics update to visual rendering) on localhost setup

### Reliability Requirements

- **NFR-004**: All Gazebo launch files MUST include proper error handling and graceful shutdown (e.g., Ctrl+C handling, cleanup of spawned models)
- **NFR-005**: Unity projects MUST not crash or freeze during normal operation (importing URDF, running simulation, UI interaction) for at least 10 consecutive runs

### Usability Requirements

- **NFR-006**: All tutorials MUST include screenshots at decision points (e.g., "Click this button", "You should see this output") to guide visual learners
- **NFR-007**: All error messages in custom scripts MUST be descriptive, indicating the cause (e.g., "URDF file not found at path X" rather than "FileNotFoundError")
- **NFR-008**: All configuration files (Gazebo world files, Unity project settings) MUST include inline comments explaining non-obvious parameters

### Compatibility Requirements

- **NFR-009**: Gazebo examples MUST support both Gazebo Classic 11 (Ubuntu 22.04 apt default) and Gazebo Fortress/Garden (Ubuntu 22.04/24.04 via official PPA), with migration notes provided
- **NFR-010**: Unity projects MUST be compatible with Unity 2022 LTS and Unity 6 (latest), with version-specific notes for API changes

### Security Requirements

- **NFR-011**: All network communication examples (ROS 2 bridges) MUST use localhost by default, with clear warnings before exposing services to external networks
- **NFR-012**: All example code MUST not include hardcoded credentials, API keys, or sensitive information. Configuration files with secrets MUST be in .gitignore with template files provided (.env.example)

## Risks and Assumptions *(optional)*

### Identified Risks

- **Platform Fragmentation Risk**: Gazebo Classic vs Gazebo (new generation) creates two divergent ecosystems. Mitigation: Focus on Gazebo Classic for stability, provide clear migration path, monitor Gazebo Fortress/Garden adoption.
- **Unity Version Churn Risk**: Unity releases break backward compatibility (e.g., rendering pipeline changes, Articulation Body API changes). Mitigation: Target Unity 2022 LTS for 2-year support window, document version-specific workarounds.
- **ROS 2 Bridge Reliability Risk**: Unity-ROS 2 communication may fail due to DDS configuration, network issues, or platform differences. Mitigation: Comprehensive troubleshooting guide, alternative methods (ROS Bridge over WebSocket), test on multiple platforms.
- **Student Hardware Variability Risk**: Wide range of student hardware (low-end laptops to high-end desktops) affects simulation performance. Mitigation: Define reference hardware, provide performance tuning guidance, offer cloud simulation alternatives.
- **Simulation Complexity Risk**: Students may struggle with combined Gazebo+Unity complexity. Mitigation: Chapters 1-3 teach each platform independently before integration in Chapter 4; clear separation of concerns.

### Validation Strategy

- **Gazebo Example Testing**: All Gazebo examples tested in Docker container with Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11. Automated CI checks for launch file errors and RTF performance.
- **Unity Example Testing**: All Unity projects tested in Unity 2022 LTS on Windows 11 and Ubuntu 22.04. Build success, runtime errors, and performance (FPS) monitored.
- **Cross-Platform Testing**: Integration examples tested on Linux-Linux (Gazebo+Unity both on Ubuntu), Windows-Linux (Unity on Windows, Gazebo on WSL2), and cloud configurations (Gazebo on AWS, Unity local).
- **Student Pilot Testing**: Beta test with 5-10 students before finalization. Collect feedback on tutorial clarity, time to completion, and error frequency.
