# Tasks: Module 2 - Digital Twin Simulation (Gazebo & Unity)

**Input**: Design documents from `/specs/002-module-2-digital-twin/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chapter-content-contract.md

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

**Tests**: No automated tests requested in specification. Manual validation via Docker/Unity builds and quality gates.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Task IDs are sequential (T001, T002, ...) in execution order

## Path Conventions

This is a Docusaurus documentation site with simulation examples:
- Content: `docs/module-2/`
- Examples: `examples/module-2/chapter-N/`
- Assets: `docs/module-2/assets/`
- ROS 2 workspace: `examples/ros2_ws/`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Module 2 directory structure and development environment

- [X] T001 Create Module 2 directory structure (docs/module-2/, examples/module-2/, docs/module-2/assets/)
- [X] T002 [P] Create examples subdirectories (chapter-1/, chapter-2/, chapter-3/, chapter-4/)
- [X] T003 [P] Create assets subdirectories (_diagrams/chapter-N/, _screenshots/chapter-N/ for N=1,2,3,4)
- [X] T004 [P] Initialize ROS 2 workspace at examples/ros2_ws/ with src/ subdirectory
- [X] T005 Configure Docusaurus sidebars.js to include Module 2 chapters
- [ ] T006 Update docusaurus.config.js with KaTeX for math rendering (if not already configured)
- [ ] T007 [P] Create validation script validate_module2.sh for environment checks

---

## Phase 2: Foundational (Shared Resources - BLOCKING)

**Purpose**: Core shared assets that ALL chapters depend on - MUST complete before any chapter work begins

**⚠️ CRITICAL**: No chapter implementation can begin until this phase is complete

- [X] T008 Create canonical URDF model examples/module-2/shared/urdf/simple_humanoid.urdf from Module 1
- [ ] T009 Validate URDF with check_urdf command (must pass)
- [ ] T010 Validate URDF Gazebo compatibility with gz sdf -p command (must pass)
- [X] T011 [P] Create ROS 2 package gazebo_sim in examples/ros2_ws/src/ (ament_python)
- [X] T012 [P] Create package.xml for gazebo_sim with dependencies (ros-humble-gazebo-ros-pkgs, sensor_msgs, geometry_msgs)
- [X] T013 [P] Create setup.py for gazebo_sim package
- [ ] T014 Build ROS 2 workspace with colcon build (must succeed)
- [ ] T015 Source workspace install/setup.bash and verify package visible with ros2 pkg list

**Checkpoint**: Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 (Chapter 1 - Gazebo Physics Simulation) - Priority P1 🎯 MVP

**Goal**: Students master physics simulation in Gazebo (gravity, collisions, contact dynamics) to create accurate digital twins

**Independent Test**: Student can create Gazebo world, spawn humanoid robot, configure physics parameters, and demonstrate stable standing (RTF ≥ 0.9)

**Functional Requirements**: FR-001 to FR-009
**Success Criteria**: SC-001, SC-005, SC-006, SC-010, SC-011, SC-020, SC-021

### Content Creation for Chapter 1

- [X] T016 [US1] Create chapter-1.mdx frontmatter in docs/module-2/ (id, title, sidebar_position: 1)
- [X] T017 [US1] Write Overview section (200-400 words: Gazebo role in digital twin workflow)
- [X] T018 [US1] Write Learning Objectives section (3-5 objectives using Bloom taxonomy verbs)
- [X] T019 [US1] Write Theoretical Foundations section in docs/module-2/chapter-1.mdx:
  - Rigid body dynamics (Newton-Euler equations in KaTeX)
  - Contact mechanics (Hertz contact theory, Coulomb friction)
  - Numerical integration methods (Euler, RK4, symplectic)
  - Physics engine algorithms (ODE, Bullet, DART comparison)
- [X] T020 [US1] Write Hands-On Implementation section:
  - Gazebo installation tutorial (Ubuntu 22.04)
  - World file creation (.world SDF format)
  - URDF spawning with spawn_entity.py
  - Physics parameter tuning (gravity, timestep, solver iterations, ERP/CFM)

### Example 1: Basic World Creation (Beginner)

- [X] T021 [P] [US1] Create example-01-world-creation/ directory in examples/module-2/chapter-1/
- [X] T022 [P] [US1] Create empty_world.world in examples/module-2/chapter-1/example-01-world-creation/worlds/
- [X] T023 [US1] Configure physics engine (ODE), gravity=-9.81, max_step_size=0.001, real_time_update_rate=1000
- [X] T024 [US1] Add ground plane model reference
- [X] T025 [P] [US1] Create world.launch.py in examples/module-2/chapter-1/example-01-world-creation/launch/
- [X] T026 [P] [US1] Create README.md with prerequisites, usage instructions, expected RTF ≥ 0.9
- [ ] T027 [US1] Create test.sh script to launch Gazebo headless, measure RTF for 30s, exit code 0 if RTF ≥ 0.9

### Example 2: Spawn Humanoid Robot (Beginner)

- [X] T028 [P] [US1] Create example-02-spawn-humanoid/ directory in examples/module-2/chapter-1/
- [X] T029 [P] [US1] Copy simple_humanoid.urdf to examples/module-2/chapter-1/example-02-spawn-humanoid/urdf/
- [X] T030 [US1] Create spawn_robot.launch.py to launch Gazebo + spawn simple_humanoid at origin
- [X] T031 [P] [US1] Create README.md documenting spawn_entity.py parameters
- [ ] T032 [US1] Create test.sh to verify robot spawns without errors, joints visible

### Example 3: Collision Configuration (Intermediate)

- [X] T033 [P] [US1] Create example-03-collision-config/ directory in examples/module-2/chapter-1/
- [ ] T034 [P] [US1] Create obstacle.sdf with box collision geometry in examples/module-2/chapter-1/example-03-collision-config/models/
- [ ] T035 [US1] Configure surface properties: mu1=1.0, mu2=1.0 (friction), kp=1e6 (contact stiffness), kd=1.0 (damping)
- [ ] T036 [US1] Create collision_test.launch.py to spawn humanoid + obstacle
- [X] T037 [P] [US1] Create README.md explaining collision parameters
- [ ] T038 [US1] Create test.sh to verify collision detection (robot stops when contacting obstacle)

### Example 4: Physics Parameter Tuning (Intermediate)

- [X] T039 [P] [US1] Create example-04-physics-tuning/ directory in examples/module-2/chapter-1/
- [ ] T040 [P] [US1] Create physics_demo.world with custom physics parameters in examples/module-2/chapter-1/example-04-physics-tuning/worlds/
- [ ] T041 [US1] Demonstrate gravity modification (gravity=0 for zero-G, gravity=-3.71 for Mars)
- [ ] T042 [US1] Demonstrate solver iteration tuning (iters=50 vs iters=500, measure stability vs RTF trade-off)
- [ ] T043 [US1] Create physics.launch.py
- [X] T044 [P] [US1] Create README.md with parameter recommendations
- [ ] T045 [US1] Create test.sh to measure RTF at different solver settings

### Example 5: External Force Application (Advanced)

- [X] T046 [P] [US1] Create example-05-contact-forces/ directory in examples/module-2/chapter-1/
- [ ] T047 [P] [US1] Create apply_force.py ROS 2 node to apply forces via /apply_body_wrench service
- [ ] T048 [US1] Implement force application to humanoid torso (100N impulse in +X direction)
- [ ] T049 [US1] Create force_demo.launch.py launching Gazebo + force application node
- [X] T050 [P] [US1] Create README.md documenting ApplyBodyWrench service interface
- [ ] T051 [US1] Create test.sh to verify force application (robot moves when force applied)

### Diagrams and Assets for Chapter 1

- [X] T052 [P] [US1] Create Mermaid diagram: Gazebo client-server architecture in docs/module-2/assets/_diagrams/chapter-1/gazebo_architecture.mmd
- [X] T053 [P] [US1] Create Mermaid diagram: Physics engine pipeline (simulation loop) in docs/module-2/assets/_diagrams/chapter-1/physics_pipeline.mmd
- [X] T054 [P] [US1] Create Mermaid diagram: Collision detection flowchart in docs/module-2/assets/_diagrams/chapter-1/collision_flow.mmd
- [X] T055 [P] [US1] Create comparison table: ODE vs Bullet vs DART physics engines (performance, stability, features)
- [ ] T056 [P] [US1] Capture screenshot: Gazebo GUI with simple_humanoid loaded in docs/module-2/assets/_screenshots/chapter-1/gazebo_gui_humanoid.png
- [ ] T057 [P] [US1] Capture screenshot: Physics behavior visualization (robot falling due to gravity) in docs/module-2/assets/_screenshots/chapter-1/gravity_demo.png

### Exercises and Resources for Chapter 1

- [X] T058 [US1] Write Exercises section in docs/module-2/chapter-1.mdx:
  - Beginner: Modify gravity to -1.62 (Moon), spawn model, observe
  - Beginner: Spawn custom box model with mass=10kg
  - Intermediate: Configure collision geometries for stable grasping scenario
  - Intermediate: Tune friction coefficients for climbing stairs scenario
  - Advanced: Create bipedal standing scenario with contact-based stabilization, measure ground reaction forces
- [X] T059 [US1] Write Summary section (recap physics fundamentals, Gazebo capabilities)
- [X] T060 [US1] Write Troubleshooting section (common issues: low RTF, collision failures, spawn errors)
- [X] T061 [US1] Write Further Reading section with 5+ references:
  - Gazebo Classic documentation
  - ODE/Bullet/DART official docs
  - Research papers on physics-based robot simulation

### Validation for Chapter 1

- [ ] T062 [US1] Run all test.sh scripts in Docker (osrf/ros:humble-desktop + Gazebo Classic 11)
- [ ] T063 [US1] Verify all examples achieve RTF ≥ 0.9 on reference hardware
- [ ] T064 [US1] Validate all .world and .sdf files with gz sdf -k command
- [ ] T065 [US1] Validate simple_humanoid.urdf with check_urdf command
- [ ] T066 [US1] Build Docusaurus site, verify chapter-1.mdx renders without errors
- [ ] T067 [US1] Verify all cross-references to Module 1 are correct (URDF from Module 1 Chapter 3)
- [ ] T068 [US1] Manual QA: Read through chapter, verify clarity and technical accuracy

**Checkpoint**: Chapter 1 (Gazebo Physics) is independently complete and testable

---

## Phase 4: User Story 2 (Chapter 2 - Unity Rendering & HRI) - Priority P2

**Goal**: Students create high-fidelity visualizations in Unity with PBR materials, lighting, and human-robot interaction UI

**Independent Test**: Student can import humanoid URDF into Unity, apply PBR materials, implement keyboard control UI, and produce photorealistic rendered image (FPS ≥ 30)

**Functional Requirements**: FR-010 to FR-019
**Success Criteria**: SC-002, SC-007, SC-008, SC-012, SC-013, SC-018, SC-019

### Content Creation for Chapter 2

- [ ] T069 [US2] Create chapter-2.mdx frontmatter in docs/module-2/ (id, title, sidebar_position: 2)
- [ ] T070 [US2] Write Overview section (200-400 words: Unity role in digital twin visualization)
- [ ] T071 [US2] Write Learning Objectives section (3-5 objectives: Unity basics, URDF import, PBR, HRI UI)
- [ ] T072 [US2] Write Theoretical Foundations section in docs/module-2/chapter-2.mdx:
  - PBR theory (Cook-Torrance BRDF, Fresnel equations, microfacet distribution) with KaTeX formulas
  - Unity rendering pipeline architecture (Built-in vs URP vs HDRP comparison)
  - Real-time lighting (shadow mapping, SSAO, reflections)
  - HDR imaging (tone mapping, color grading)
- [ ] T073 [US2] Write Hands-On Implementation section:
  - Unity Hub installation (Windows/Linux/macOS)
  - Project creation with URP template
  - URDF Importer package setup
  - Articulation body configuration (joint types, drives, limits)
  - PBR material creation (albedo, metallic-smoothness workflow, normal maps)
  - HDRI environment setup

### Example 1: Unity Basics & Project Setup (Beginner)

- [ ] T074 [P] [US2] Create example-01-unity-basics/ directory in examples/module-2/chapter-2/
- [ ] T075 [P] [US2] Create Unity project UnityBasicsProject/ with URP template (Unity 2022.3 LTS)
- [ ] T076 [US2] Create IntroScene.unity with basic GameObjects (plane, directional light, camera)
- [ ] T077 [US2] Configure URP asset: Forward renderer, MSAA off, shadow cascades=2
- [ ] T078 [US2] Create HelloUnity.cs script demonstrating Debug.Log() and Update() loop
- [ ] T079 [P] [US2] Create README.md with Unity version, project setup steps
- [ ] T080 [P] [US2] Capture screenshot: Unity Editor with IntroScene loaded in examples/module-2/chapter-2/example-01-unity-basics/screenshots/editor_view.png
- [ ] T081 [US2] Document manual test: Open project in Unity 2022 LTS, press Play, verify no errors

### Example 2: URDF Import & Articulation Bodies (Intermediate)

- [ ] T082 [P] [US2] Create example-02-urdf-import/ directory in examples/module-2/chapter-2/
- [ ] T083 [P] [US2] Create Unity project URDFImportProject/ with URP template
- [ ] T084 [US2] Install URDF Importer package (com.unity.robotics.urdf-importer version 0.5.2+)
- [ ] T085 [US2] Copy simple_humanoid.urdf to URDFImportProject/Assets/URDF/
- [ ] T086 [US2] Import URDF via Assets → Import Robot from URDF
- [ ] T087 [US2] Create ArticulationConfig.cs script to configure joint drives (stiffness, damping, force limits)
- [ ] T088 [US2] Create URDFScene.unity with imported robot
- [ ] T089 [P] [US2] Create README.md documenting URDF import process, package installation
- [ ] T090 [P] [US2] Capture screenshot: Imported humanoid in Unity Scene view in examples/module-2/chapter-2/example-02-urdf-import/screenshots/urdf_imported.png
- [ ] T091 [US2] Document manual test: Robot articulations visible in Hierarchy, joints configurable

### Example 3: PBR Materials & Lighting (Intermediate)

- [ ] T092 [P] [US2] Create example-03-pbr-materials/ directory in examples/module-2/chapter-2/
- [ ] T093 [P] [US2] Create Unity project PBRRenderingProject/ with URP template
- [ ] T094 [US2] Create Metal.mat material (metallic=1.0, smoothness=0.9, albedo=gray)
- [ ] T095 [US2] Create Rubber.mat material (metallic=0.0, smoothness=0.4, albedo=black)
- [ ] T096 [US2] Import HDRI skybox studio_env.hdr to PBRRenderingProject/Assets/HDRI/
- [ ] T097 [US2] Configure Lighting settings: Skybox, ambient intensity=1.0, reflection probes
- [ ] T098 [US2] Create RenderingScene.unity with humanoid robot, apply Metal.mat to chassis, Rubber.mat to grippers
- [ ] T099 [P] [US2] Create README.md explaining PBR metallic-roughness workflow
- [ ] T100 [P] [US2] Capture screenshot: Photorealistic rendered robot with PBR materials in examples/module-2/chapter-2/example-03-pbr-materials/screenshots/pbr_render.png
- [ ] T101 [US2] Document manual test: FPS ≥ 30 at 1080p on reference hardware (integrated graphics + URP)

### Example 4: UI Interaction & Teleoperation (Advanced)

- [ ] T102 [P] [US2] Create example-04-ui-interaction/ directory in examples/module-2/chapter-2/
- [ ] T103 [P] [US2] Create Unity project HRIProject/ with URP template
- [ ] T104 [US2] Create RobotController.cs script for keyboard control (WASD for base movement, arrow keys for joint control)
- [ ] T105 [US2] Create UIManager.cs script managing Canvas UI with TextMeshPro labels (joint angles, robot status)
- [ ] T106 [US2] Create Canvas.prefab with UI panel (joint sliders, status display)
- [ ] T107 [US2] Create HRIScene.unity integrating robot + UI + controls
- [ ] T108 [P] [US2] Create README.md documenting keyboard controls, UI setup
- [ ] T109 [P] [US2] Capture screenshot: Unity scene with UI overlay showing robot status in examples/module-2/chapter-2/example-04-ui-interaction/screenshots/ui_demo.png
- [ ] T110 [US2] Document manual test: Press Play, use keyboard to control robot, UI updates in real-time

### Example 5: Timeline Animations (Advanced - Optional)

- [ ] T111 [P] [US2] Create example-05-timeline/ directory in examples/module-2/chapter-2/
- [ ] T112 [P] [US2] Create Unity project TimelineProject/ with URP template
- [ ] T113 [US2] Create Timeline asset robot_demo.playable
- [ ] T114 [US2] Animate robot joints over 10 seconds (hip flexion, knee extension, arm raise)
- [ ] T115 [US2] Add Cinemachine virtual cameras for cinematic views
- [ ] T116 [US2] Create TimelineScene.unity with Timeline playback
- [ ] T117 [P] [US2] Create README.md explaining Timeline editor usage
- [ ] T118 [P] [US2] Capture screenshot or export video (mp4): Robot animation sequence in examples/module-2/chapter-2/example-05-timeline/screenshots/timeline_render.mp4
- [ ] T119 [US2] Document manual test: Play Timeline, verify smooth animation

### Diagrams and Assets for Chapter 2

- [ ] T120 [P] [US2] Create Mermaid diagram: Unity Editor architecture in docs/module-2/assets/_diagrams/chapter-2/unity_architecture.mmd
- [ ] T121 [P] [US2] Create Mermaid diagram: URP rendering pipeline flowchart in docs/module-2/assets/_diagrams/chapter-2/urp_pipeline.mmd
- [ ] T122 [P] [US2] Create Mermaid diagram: UI event flow (keyboard input → RobotController → articulation body) in docs/module-2/assets/_diagrams/chapter-2/ui_event_flow.mmd
- [ ] T123 [P] [US2] Create Mermaid diagram: Articulation body hierarchy (root link → child links) in docs/module-2/assets/_diagrams/chapter-2/articulation_hierarchy.mmd

### Exercises and Resources for Chapter 2

- [ ] T124 [US2] Write Exercises section in docs/module-2/chapter-2.mdx:
  - Beginner: Import simple_humanoid.urdf, apply single albedo material
  - Intermediate: Create multi-material robot with PBR textures (metal chassis, rubber feet)
  - Intermediate: Implement keyboard control for 3 joints (hip, knee, ankle)
  - Advanced: Build interactive demo with UI dashboard showing joint angles, Timeline cinematic sequence, export 30s video
- [ ] T125 [US2] Write Summary section (recap Unity rendering, PBR, HRI capabilities)
- [ ] T126 [US2] Write Troubleshooting section (URDF import failures, material rendering issues, performance problems)
- [ ] T127 [US2] Write Further Reading section with 5+ references:
  - Unity Learn tutorials
  - PBR material guides
  - Unity Robotics Hub GitHub

### Validation for Chapter 2

- [ ] T128 [US2] Build all Unity projects in Unity 2022.3 LTS on Windows 11 and Ubuntu 22.04
- [ ] T129 [US2] Verify all C# scripts compile without errors
- [ ] T130 [US2] Measure FPS in all scenes (must achieve ≥ 30 FPS at 1080p on reference hardware)
- [ ] T131 [US2] Verify URDF import succeeds for simple_humanoid.urdf
- [ ] T132 [US2] Build Docusaurus site, verify chapter-2.mdx renders without errors
- [ ] T133 [US2] Verify cross-references to Module 1 Chapter 3 (URDF) are correct
- [ ] T134 [US2] Manual QA: Read through chapter, verify Unity tutorial clarity

**Checkpoint**: Chapter 2 (Unity Rendering) is independently complete and testable

---

## Phase 5: User Story 3 (Chapter 3 - Sensor Simulation) - Priority P3

**Goal**: Students master LiDAR, depth camera, and IMU simulation in Gazebo and Unity for perception testing

**Independent Test**: Student can add sensors to simulated robot (Gazebo LiDAR, Unity depth camera, Gazebo IMU), visualize data in RViz, and process with ROS 2 nodes

**Functional Requirements**: FR-020 to FR-028
**Success Criteria**: SC-003, SC-009, SC-014, SC-022

**Dependencies**: Requires Chapter 1 (Gazebo) and Chapter 2 (Unity) completion for sensor platforms

### Content Creation for Chapter 3

- [ ] T135 [US3] Create chapter-3.mdx frontmatter in docs/module-2/ (id, title, sidebar_position: 3)
- [ ] T136 [US3] Write Overview section (sensor simulation importance for perception algorithms)
- [ ] T137 [US3] Write Learning Objectives section (3-5 objectives: LiDAR, depth cameras, IMUs, noise modeling)
- [ ] T138 [US3] Write Theoretical Foundations section in docs/module-2/chapter-3.mdx:
  - LiDAR physics (time-of-flight, phase shift, scanning mechanisms) with formulas
  - Depth camera technologies (stereo vision geometry, structured light, ToF imaging)
  - IMU sensor fusion (complementary filters, EKF, UKF) with state equations
  - Sensor noise models (Gaussian noise, systematic bias, drift models)
- [ ] T139 [US3] Write Hands-On Implementation (Gazebo) section:
  - GPU ray sensor plugin configuration (scan rate, horizontal/vertical samples, range, noise)
  - Depth camera plugin (image resolution, FOV, clip distances, ROS 2 topic publishing)
  - IMU sensor plugin (noise parameters: accelerometer/gyroscope bias, noise density)
- [ ] T140 [US3] Write Hands-On Implementation (Unity) section:
  - Unity Perception package installation
  - Perception Camera component (semantic segmentation, depth rendering)
  - Custom LiDAR raycasting script (Physics.Raycast, point cloud generation)
  - IMU simulation (Rigidbody velocity → sensor_msgs/Imu)

### Example 1: Gazebo LiDAR Simulation (Intermediate)

- [ ] T141 [P] [US3] Create example-01-gazebo-lidar/ directory in examples/module-2/chapter-3/
- [ ] T142 [P] [US3] Create robot_with_lidar.sdf in examples/module-2/chapter-3/example-01-gazebo-lidar/models/
- [ ] T143 [US3] Add gpu_ray sensor: horizontal samples=1875, resolution=1.0, min_angle=-π, max_angle=+π, range 0.1-100m
- [ ] T144 [US3] Configure sensor plugin to publish to /scan topic (sensor_msgs/LaserScan)
- [ ] T145 [US3] Add noise: type=gaussian, mean=0.0, stddev=0.01
- [ ] T146 [US3] Create lidar_sim.launch.py launching Gazebo + RViz
- [ ] T147 [US3] Create visualize_pointcloud.py ROS 2 node subscribing to /scan, visualizing in RViz
- [ ] T148 [P] [US3] Create README.md documenting LiDAR sensor parameters
- [ ] T149 [US3] Create test.sh verifying /scan topic publishes at ≥10 Hz

### Example 2: Gazebo Depth Camera Simulation (Intermediate)

- [ ] T150 [P] [US3] Create example-02-gazebo-depth-camera/ directory in examples/module-2/chapter-3/
- [ ] T151 [P] [US3] Create robot_with_camera.sdf in examples/module-2/chapter-3/example-02-gazebo-depth-camera/models/
- [ ] T152 [US3] Add depth_camera sensor: resolution 640x480, horizontal_fov=1.5708 (90°), clip_near=0.3m, clip_far=10.0m
- [ ] T153 [US3] Configure camera plugin to publish /camera/depth/image_raw (sensor_msgs/Image)
- [ ] T154 [US3] Create depth_camera_sim.launch.py
- [ ] T155 [US3] Create process_depth_image.py ROS 2 node subscribing to depth images, displaying with cv_bridge
- [ ] T156 [P] [US3] Create README.md
- [ ] T157 [US3] Create test.sh verifying depth images publish at ≥30 Hz

### Example 3: Gazebo IMU Simulation (Beginner)

- [ ] T158 [P] [US3] Create example-03-gazebo-imu/ directory in examples/module-2/chapter-3/
- [ ] T159 [P] [US3] Create robot_with_imu.sdf in examples/module-2/chapter-3/example-03-gazebo-imu/models/
- [ ] T160 [US3] Add imu_sensor plugin: linear_acceleration noise (mean=0.0, stddev=0.017), angular_velocity noise (mean=0.0, stddev=0.00017)
- [ ] T161 [US3] Configure IMU to publish to /imu/data (sensor_msgs/Imu)
- [ ] T162 [US3] Create imu_sim.launch.py
- [ ] T163 [US3] Create plot_imu_data.py ROS 2 node subscribing to IMU, plotting acceleration/gyroscope with matplotlib
- [ ] T164 [P] [US3] Create README.md
- [ ] T165 [US3] Create test.sh verifying IMU publishes at ≥100 Hz

### Example 4: Unity LiDAR Raycasting (Advanced)

- [ ] T166 [P] [US3] Create example-04-unity-lidar/ directory in examples/module-2/chapter-3/
- [ ] T167 [P] [US3] Create Unity project UnityLiDARProject/ with URP template
- [ ] T168 [US3] Create LiDARRaycaster.cs script implementing Physics.Raycast in spherical coordinates (360° horizontal scan, 16 vertical layers)
- [ ] T169 [US3] Generate point cloud data structure (List<Vector3>)
- [ ] T170 [US3] Visualize point cloud with Unity Gizmos.DrawSphere() in Scene view
- [ ] T171 [US3] (Optional) Integrate ROS-TCP-Connector to publish sensor_msgs/PointCloud2
- [ ] T172 [US3] Create LiDARScene.unity with robot + LiDAR raycaster
- [ ] T173 [P] [US3] Create README.md
- [ ] T174 [P] [US3] Capture screenshot: Point cloud visualization in Unity in examples/module-2/chapter-3/example-04-unity-lidar/screenshots/lidar_pointcloud.png
- [ ] T175 [US3] Document manual test: Play scene, verify raycast visualization appears

### Example 5: Unity Perception Package (Advanced)

- [ ] T176 [P] [US3] Create example-05-unity-perception/ directory in examples/module-2/chapter-3/
- [ ] T177 [P] [US3] Create Unity project PerceptionProject/ with URP template
- [ ] T178 [US3] Install Unity Perception package (com.unity.perception version 1.0.0+)
- [ ] T179 [US3] Add PerceptionCamera component to Main Camera
- [ ] T180 [US3] Configure labelers: SemanticSegmentationLabeler, DepthLabeler, BoundingBox2DLabeler
- [ ] T181 [US3] Create label config: "robot", "ground", "obstacle" with color mappings
- [ ] T182 [US3] Create PerceptionConfig.cs script to configure perception camera output
- [ ] T183 [US3] Create PerceptionScene.unity
- [ ] T184 [P] [US3] Create README.md
- [ ] T185 [P] [US3] Capture screenshots: RGB, depth, semantic segmentation outputs in examples/module-2/chapter-3/example-05-unity-perception/screenshots/
- [ ] T186 [US3] Document manual test: Verify perception outputs generate correctly

### Example 6: Sensor Noise Modeling (Intermediate)

- [ ] T187 [P] [US3] Create example-06-sensor-noise/ directory in examples/module-2/chapter-3/
- [ ] T188 [P] [US3] Create add_noise.py ROS 2 node in examples/ros2_ws/src/gazebo_sim/gazebo_sim/
- [ ] T189 [US3] Subscribe to /scan (clean LiDAR data), add Gaussian noise (stddev=0.05), publish to /scan_noisy
- [ ] T190 [US3] Subscribe to /imu/data, add bias (accel bias=0.1 m/s², gyro bias=0.01 rad/s), publish to /imu/data_noisy
- [ ] T191 [US3] Create noisy_sensors.launch.py
- [ ] T192 [P] [US3] Create README.md explaining noise model parameters
- [ ] T193 [US3] Create test.sh comparing clean vs noisy topic data

### Diagrams and Assets for Chapter 3

- [ ] T194 [P] [US3] Create Mermaid diagram: LiDAR scanning patterns (360° horizontal, multi-layer vertical) in docs/module-2/assets/_diagrams/chapter-3/lidar_scanning.mmd
- [ ] T195 [P] [US3] Create Mermaid diagram: Depth camera principles (stereo vs ToF) in docs/module-2/assets/_diagrams/chapter-3/depth_camera.mmd
- [ ] T196 [P] [US3] Create Mermaid diagram: IMU sensor fusion architecture (EKF state estimation) in docs/module-2/assets/_diagrams/chapter-3/imu_fusion.mmd
- [ ] T197 [P] [US3] Create Mermaid diagram: ROS 2 sensor message flow (Gazebo → ROS 2 topics → RViz/processing nodes) in docs/module-2/assets/_diagrams/chapter-3/sensor_dataflow.mmd
- [ ] T198 [P] [US3] Capture screenshot: RViz showing LiDAR point cloud in docs/module-2/assets/_screenshots/chapter-3/rviz_pointcloud.png
- [ ] T199 [P] [US3] Capture screenshot: Depth image visualization in docs/module-2/assets/_screenshots/chapter-3/depth_image.png
- [ ] T200 [P] [US3] Capture screenshot: IMU data plot (acceleration, gyroscope over time) in docs/module-2/assets/_screenshots/chapter-3/imu_plot.png

### Exercises and Resources for Chapter 3

- [ ] T201 [US3] Write Exercises section in docs/module-2/chapter-3.mdx:
  - Beginner: Add LiDAR to robot in Gazebo, visualize point cloud in RViz
  - Intermediate: Configure depth camera with custom resolution (1280x720), verify in RViz
  - Intermediate: Add IMU to robot, plot acceleration data during simulated movement
  - Advanced: Implement sensor fusion algorithm (EKF with IMU + odometry), compare with ground truth
  - Advanced: Calibrate simulated LiDAR to match real sensor noise characteristics (cite Velodyne VLP-16 datasheet)
- [ ] T202 [US3] Write Summary section (recap sensor types, Gazebo vs Unity strengths)
- [ ] T203 [US3] Write Troubleshooting section (LiDAR not publishing, depth camera blank images, IMU noise too high)
- [ ] T204 [US3] Write Further Reading section with 5+ references:
  - Sensor datasheets (Velodyne VLP-16, Intel RealSense D435, Bosch BMI088)
  - ROS 2 sensor_msgs documentation
  - Unity Perception package documentation
  - Sensor simulation validation research papers

### Validation for Chapter 3

- [ ] T205 [US3] Run all Gazebo sensor examples in Docker, verify topics publish at expected rates
- [ ] T206 [US3] Verify LiDAR publishes at ≥10 Hz, depth camera at ≥30 Hz, IMU at ≥100 Hz
- [ ] T207 [US3] Verify Unity Perception examples generate outputs
- [ ] T208 [US3] Validate sensor noise parameters are realistic (cite real sensor datasheets)
- [ ] T209 [US3] Build Docusaurus site, verify chapter-3.mdx renders without errors
- [ ] T210 [US3] Verify cross-references to Chapter 1 (Gazebo), Chapter 2 (Unity), Module 1 Chapter 1 (ROS 2 topics)
- [ ] T211 [US3] Manual QA: Read through chapter, verify sensor simulation clarity

**Checkpoint**: Chapter 3 (Sensor Simulation) is independently complete and testable

---

## Phase 6: User Story 4 (Chapter 4 - Gazebo-Unity Integration) - Priority P4

**Goal**: Students integrate Gazebo (physics) and Unity (visualization) for Physical AI workflows (RL training, demos, hybrid simulation)

**Independent Test**: Student can set up ROS 2 bridge, synchronize Gazebo physics with Unity rendering (latency <100ms), and demonstrate RL training workflow (Gazebo Gym environment → SB3 training → Unity deployment)

**Functional Requirements**: FR-029 to FR-039
**Success Criteria**: SC-004, SC-008, SC-009, SC-015

**Dependencies**: Requires Chapters 1, 2, 3 completion (Gazebo physics, Unity rendering, sensor simulation)

### Content Creation for Chapter 4

- [ ] T212 [US4] Create chapter-4.mdx frontmatter in docs/module-2/ (id, title, sidebar_position: 4)
- [ ] T213 [US4] Write Overview section (integration patterns, Physical AI workflows)
- [ ] T214 [US4] Write Learning Objectives section (3-5 objectives: ROS 2 bridge, realtime sync, RL workflows, performance optimization)
- [ ] T215 [US4] Write Theoretical Foundations section in docs/module-2/chapter-4.mdx:
  - Integration architecture patterns (realtime bidirectional, unidirectional, offline, co-simulation)
  - ROS 2 DDS middleware (RTPS protocol, QoS policies for cross-machine communication)
  - RL for robotics (Markov Decision Processes, policy gradient methods, sim-to-real transfer, domain randomization) with MDP formulation
- [ ] T216 [US4] Write Hands-On Implementation section:
  - ROS-TCP-Connector installation (Unity Package Manager)
  - ROS 2 endpoint configuration (IP addresses, ports, ROS_DOMAIN_ID)
  - joint_states topic subscription in Unity (C# subscriber, articulation body synchronization)
  - tf transform synchronization (coordinate frame conversions)
  - Sensor data streaming (Gazebo sensor plugins → Unity visualization)

### Example 1: ROS 2 Bridge Setup (Beginner)

- [ ] T217 [P] [US4] Create example-01-ros2-bridge-setup/ directory in examples/module-2/chapter-4/
- [ ] T218 [P] [US4] Create Unity project ROSBridgeProject/ with URP template in examples/module-2/chapter-4/example-01-ros2-bridge-setup/unity/
- [ ] T219 [US4] Install ROS-TCP-Connector package (com.unity.robotics.ros-tcp-connector version 0.7.0+)
- [ ] T220 [US4] Create ROSTCPConnector.cs script configuring ROS IP (127.0.0.1), port (10000)
- [ ] T221 [US4] Create ROSConnectionScene.unity with ROS connection test
- [ ] T222 [US4] Create gazebo_headless.launch.py in examples/module-2/chapter-4/example-01-ros2-bridge-setup/gazebo/launch/ (launches Gazebo without GUI for performance)
- [ ] T223 [US4] Add ros_tcp_endpoint to examples/ros2_ws/src/ (install via pip: pip install ros-tcp-endpoint)
- [ ] T224 [US4] Create launch file ros_tcp_endpoint.launch.py starting ROS-TCP server
- [ ] T225 [P] [US4] Create README.md documenting ROS 2 bridge setup steps, firewall configuration
- [ ] T226 [US4] Create test.md manual checklist: Launch endpoint, launch Unity, verify connection handshake in Unity console

### Example 2: Realtime Gazebo-Unity Synchronization (Intermediate)

- [ ] T227 [P] [US4] Create example-02-realtime-sync/ directory in examples/module-2/chapter-4/
- [ ] T228 [P] [US4] Create Unity project SyncProject/ in examples/module-2/chapter-4/example-02-realtime-sync/unity/
- [ ] T229 [US4] Create JointStateSubscriber.cs script subscribing to /joint_states (sensor_msgs/JointState)
- [ ] T230 [US4] Implement articulation body synchronization (Unity joint positions ← Gazebo joint_states)
- [ ] T231 [US4] Create state_publisher.py ROS 2 node in examples/module-2/chapter-4/example-02-realtime-sync/gazebo/scripts/
- [ ] T232 [US4] Publish /joint_states at 50 Hz from Gazebo physics simulation
- [ ] T233 [US4] Add timestamp synchronization (measure latency: Gazebo publish time → Unity receive time)
- [ ] T234 [US4] Create SyncScene.unity importing simple_humanoid, adding JointStateSubscriber component
- [ ] T235 [US4] Create sync_gazebo.launch.py launching Gazebo + state_publisher + ros_tcp_endpoint
- [ ] T236 [P] [US4] Create README.md
- [ ] T237 [US4] Create test.md manual checklist: Launch Gazebo, launch Unity, verify robot moves in Unity synchronized with Gazebo (measure latency <100ms)

### Example 3: RL Training Workflow (Advanced)

- [ ] T238 [P] [US4] Create example-03-rl-training/ directory in examples/module-2/chapter-4/
- [ ] T239 [P] [US4] Create rl_env.py Gymnasium environment in examples/module-2/chapter-4/example-03-rl-training/gazebo/scripts/
- [ ] T240 [US4] Define observation space: Box (joint positions, velocities, IMU data) shape=(24,)
- [ ] T241 [US4] Define action space: Box (joint torques) shape=(12,)
- [ ] T242 [US4] Implement reset(): Reset Gazebo simulation, randomize joint positions ±5°
- [ ] T243 [US4] Implement step(): Apply actions, read observations, compute reward (forward velocity + balance penalty - energy cost)
- [ ] T244 [US4] Integrate with Gazebo via ROS 2 (/joint_commands publisher, /joint_states subscriber)
- [ ] T245 [US4] Create train_ppo.py in examples/module-2/chapter-4/example-03-rl-training/training/
- [ ] T246 [US4] Configure stable-baselines3 PPO: learning_rate=3e-4, n_steps=2048, batch_size=64, n_epochs=10
- [ ] T247 [US4] Train for 100,000 timesteps (fast validation), save checkpoints every 10,000 steps
- [ ] T248 [US4] Create requirements.txt (stable-baselines3, gymnasium, rclpy, numpy)
- [ ] T249 [US4] Create Unity project RLVisualization/ in examples/module-2/chapter-4/example-03-rl-training/unity/
- [ ] T250 [US4] Create PolicyDeployer.cs script loading trained policy (ONNX format), running inference in Unity
- [ ] T251 [US4] Create RLDemoScene.unity visualizing trained policy behavior
- [ ] T252 [P] [US4] Create README.md documenting RL workflow (Gazebo training → policy export → Unity deployment)
- [ ] T253 [US4] Create test.md manual checklist: Run training for 1000 steps (quick test), verify policy saves, load in Unity, verify robot behavior

### Integration Architecture Diagrams

- [ ] T254 [P] [US4] Create Mermaid diagram: Integration patterns (realtime sync, unidirectional, offline) in docs/module-2/assets/_diagrams/chapter-4/integration_patterns.mmd
- [ ] T255 [P] [US4] Create Mermaid diagram: ROS 2 bridge message flow (Gazebo → ros_tcp_endpoint → Unity) in docs/module-2/assets/_diagrams/chapter-4/ros2_bridge_flow.mmd
- [ ] T256 [P] [US4] Create Mermaid diagram: RL training pipeline (Gymnasium env → SB3 PPO → policy checkpoint → Unity deployment) in docs/module-2/assets/_diagrams/chapter-4/rl_pipeline.mmd
- [ ] T257 [P] [US4] Create Mermaid diagram: Distributed simulation architecture (Gazebo headless on cloud, Unity client local) in docs/module-2/assets/_diagrams/chapter-4/distributed_sim.mmd

### Performance Optimization and Cloud Deployment

- [ ] T258 [US4] Write Performance Optimization section in docs/module-2/chapter-4.mdx:
  - Profiling Gazebo RTF (use /gazebo/performance_metrics topic)
  - Profiling Unity FPS (Unity Profiler, custom FPS counter)
  - Optimizing ROS 2 message frequency (throttle high-rate topics, use QoS settings)
  - Message compression techniques
  - Running headless Gazebo for maximum training speed (gzserver only, no GUI)
  - Selective Unity rendering (on-demand screenshot capture vs continuous visualization)
- [ ] T259 [US4] Write Cloud Deployment section in docs/module-2/chapter-4.mdx:
  - Running Gazebo on remote servers (AWS EC2, GCP Compute Engine)
  - Docker containerization for Gazebo (.docker/gazebo-humble.Dockerfile example)
  - Unity client connecting over network (VPN, SSH tunneling)
  - Scaling with multiple parallel Gazebo instances for RL training

### Gazebo-Unity Screenshots

- [ ] T260 [P] [US4] Capture screenshot: Synchronized Gazebo and Unity side-by-side (robot in same pose) in docs/module-2/assets/_screenshots/chapter-4/gazebo_unity_sync.png
- [ ] T261 [P] [US4] Capture screenshot: RL training plot (TensorBoard reward curve over timesteps) in docs/module-2/assets/_screenshots/chapter-4/rl_training_curve.png
- [ ] T262 [P] [US4] Capture screenshot: Unity deployed policy (robot walking) in docs/module-2/assets/_screenshots/chapter-4/unity_policy_demo.png

### Module 2 Summary and Module 3 Preview

- [ ] T263 [US4] Write Module 2 Summary section in docs/module-2/chapter-4.mdx (200-300 words):
  - Recap key Digital Twin skills (Gazebo physics for accurate dynamics, Unity rendering for visualization, sensor modeling for perception testing, integration for hybrid workflows)
  - Emphasize sim-to-real transfer importance
  - Note limitations: Simulation is approximation, domain randomization helps bridge gap
- [ ] T264 [US4] Write Module 3 Preview section in docs/module-2/chapter-4.mdx (150-200 words):
  - Introduce NVIDIA Isaac Sim (GPU-accelerated physics, ray-traced rendering, synthetic data generation)
  - Vision-Language-Action models (embodied AI, language-conditioned policies)
  - Set expectations for advanced Physical AI topics in Module 3

### Exercises and Resources for Chapter 4

- [ ] T265 [US4] Write Exercises section in docs/module-2/chapter-4.mdx:
  - Beginner: Set up ROS 2 bridge, synchronize single joint between Gazebo and Unity
  - Intermediate: Implement full robot state synchronization with sensor data visualization, measure latency (<100ms target)
  - Advanced: Create RL training pipeline - Gazebo Gym environment for robot balancing, train PPO policy for 1M steps, deploy in Unity, visualize learned behavior
  - Advanced: Compare sim-to-real transfer - train policy in Gazebo, measure performance drop in Unity with different physics parameters
- [ ] T266 [US4] Write Summary section (recap integration patterns, RL workflows, Physical AI capabilities)
- [ ] T267 [US4] Write Troubleshooting section (ROS 2 bridge connection failures, DDS domain issues, latency problems, RL training instability)
- [ ] T268 [US4] Write Further Reading section with 5+ references:
  - ROS-TCP-Connector documentation (Unity Robotics Hub)
  - stable-baselines3 documentation
  - RL for robotics papers (sim-to-real transfer, domain randomization, PPO algorithm)
  - Gazebo-Unity co-simulation frameworks

### Validation for Chapter 4

- [ ] T269 [US4] Test ROS 2 bridge on Linux-Linux (Ubuntu 22.04 Gazebo + Unity)
- [ ] T270 [US4] Test ROS 2 bridge on Windows-Linux (Windows 11 Unity + WSL2 Gazebo)
- [ ] T271 [US4] Measure latency in realtime sync example (must be <100ms average)
- [ ] T272 [US4] Run RL training example for 1000 timesteps (smoke test), verify policy saves successfully
- [ ] T273 [US4] Load trained policy in Unity, verify inference runs
- [ ] T274 [US4] Build Docusaurus site, verify chapter-4.mdx renders without errors
- [ ] T275 [US4] Verify all cross-references to Chapters 1-3, Module 1 are correct
- [ ] T276 [US4] Manual QA: Read through chapter, verify integration workflow clarity

**Checkpoint**: Chapter 4 (Integration) is independently complete and testable

---

## Phase 7: Module 2 Finalization & Cross-Chapter Polish

**Purpose**: Improvements that affect multiple chapters and final quality assurance

- [ ] T277 [P] Update sidebars.js to ensure correct chapter ordering (1→2→3→4)
- [ ] T278 [P] Verify all internal links between chapters work (Chapter 4 references Chapters 1-3)
- [ ] T279 [P] Verify all cross-references to Module 1 are correct and linked
- [ ] T280 Run full Docusaurus build (npm run build), verify zero errors
- [ ] T281 Run Docusaurus link checker, verify no broken links
- [ ] T282 [P] Performance audit: Lighthouse score on each chapter page (target >90)
- [ ] T283 [P] Accessibility audit: Verify all images have alt text, code blocks have language labels
- [ ] T284 Validate all Gazebo examples in Docker (osrf/ros:humble-desktop + Gazebo Classic 11)
- [ ] T285 Measure RTF for all Gazebo examples (must be ≥ 0.9 on reference hardware)
- [ ] T286 Build all Unity projects on Windows 11 and Ubuntu 22.04
- [ ] T287 Measure FPS for all Unity scenes (must be ≥ 30 at 1080p on reference hardware)
- [ ] T288 [P] Code cleanup: Remove any [TODO], [FIXME], placeholder comments from all files
- [ ] T289 [P] Ensure all example READMEs follow consistent format (Prerequisites, Usage, Testing, Troubleshooting)
- [ ] T290 Run validate_module2.sh environment validation script
- [ ] T291 Final manual QA: Read through all 4 chapters sequentially, verify narrative flow and technical coherence

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all chapters
- **User Story 1 / Chapter 1 (Phase 3)**: Depends on Foundational (Phase 2) - No dependencies on other chapters
- **User Story 2 / Chapter 2 (Phase 4)**: Depends on Foundational (Phase 2) - Weak dependency on Chapter 1 (for understanding physics context, but can be developed independently)
- **User Story 3 / Chapter 3 (Phase 5)**: Depends on Foundational (Phase 2), Chapter 1 (Gazebo platform), Chapter 2 (Unity platform)
- **User Story 4 / Chapter 4 (Phase 6)**: Depends on Foundational (Phase 2), Chapters 1, 2, 3 (requires all prior chapters for integration examples)
- **Finalization (Phase 7)**: Depends on all 4 chapters being complete

### User Story Dependencies

**Recommended Execution Order**:
1. **Sequential by Priority (Recommended for Solo Developer)**:
   - Complete Phases 1 → 2 → 3 (Chapter 1) → 4 (Chapter 2) → 5 (Chapter 3) → 6 (Chapter 4) → 7
   - This ensures each chapter builds on prior knowledge

2. **Parallel with Team (If Multiple Developers)**:
   - Complete Phases 1 → 2 (all developers together)
   - Then split:
     - Developer A: Phase 3 (Chapter 1 - Gazebo)
     - Developer B: Phase 4 (Chapter 2 - Unity)
   - After Chapters 1 & 2 complete:
     - Developer C: Phase 5 (Chapter 3 - Sensors, requires 1 & 2)
   - After Chapters 1, 2, 3 complete:
     - Developer D: Phase 6 (Chapter 4 - Integration, requires 1, 2, 3)
   - Finally: Phase 7 (all developers for QA)

### Within Each Chapter

**General Pattern** (applies to all chapters):
1. Content structure first (MDX frontmatter, sections)
2. Examples in parallel (marked [P] can run simultaneously)
3. Diagrams in parallel (marked [P])
4. Exercises and resources
5. Validation and testing

**Specific Dependencies**:
- Chapter 1 Example 2 depends on T008-T009 (URDF validation)
- Chapter 2 all examples depend on T008-T010 (URDF ready for Unity import)
- Chapter 3 Gazebo examples depend on Chapter 1 completion (Gazebo platform ready)
- Chapter 3 Unity examples depend on Chapter 2 completion (Unity platform ready)
- Chapter 4 all examples depend on Chapters 1, 2, 3 (integration requires all platforms)

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T002, T003, T004 can run in parallel (different directories)

**Foundational Phase (Phase 2)**:
- T011, T012, T013 can run in parallel (ROS 2 package creation tasks)

**Chapter 1 (Phase 3)**:
- Within each example: Directory creation [P], README [P], test script creation [P] can parallelize
- T052-T057 (all diagrams/screenshots) can run in parallel
- All 5 examples (T021-T051) can be created in parallel by different developers

**Chapter 2 (Phase 4)**:
- T074-T119 (all Unity examples) can be created in parallel
- T120-T123 (all diagrams) can run in parallel

**Chapter 3 (Phase 5)**:
- T141-T193 (all sensor examples) can be created in parallel
- T194-T200 (all diagrams/screenshots) can run in parallel

**Chapter 4 (Phase 6)**:
- T217-T253 (all integration examples) can be created in parallel (once dependencies are met)
- T254-T257 (all diagrams) can run in parallel

**Finalization Phase (Phase 7)**:
- T277-T290 (most QA tasks) can run in parallel

---

## Implementation Strategy

### MVP First (User Story 1 / Chapter 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all chapters)
3. Complete Phase 3: Chapter 1 (Gazebo Physics Simulation)
4. **STOP and VALIDATE**: Test all Chapter 1 examples in Docker, verify RTF ≥ 0.9
5. Deploy Docusaurus site with Chapter 1 only
6. **Outcome**: Students can learn Gazebo physics simulation (MVP delivered!)

### Incremental Delivery (Chapter by Chapter)

1. Complete Setup + Foundational → Foundation ready
2. Add Chapter 1 → Test independently → Deploy (MVP: Gazebo Physics!)
3. Add Chapter 2 → Test independently → Deploy (Unity Rendering added!)
4. Add Chapter 3 → Test independently → Deploy (Sensor Simulation added!)
5. Add Chapter 4 → Test independently → Deploy (Integration added - Module 2 complete!)
6. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy (4 Developers)

With 4 developers (GazeboPhysicsAgent, UnityRenderingAgent, SensorSimulationAgent, IntegrationAgent):

1. **All developers**: Complete Setup + Foundational together (T001-T015)
2. **Once Foundational is done**:
   - **Developer A (GazeboPhysicsAgent)**: Phase 3 (Chapter 1)
   - **Developer B (UnityRenderingAgent)**: Phase 4 (Chapter 2)
   - **Developer C waits** (Chapter 3 needs 1 & 2)
   - **Developer D waits** (Chapter 4 needs 1, 2, 3)
3. **After Chapters 1 & 2 complete**:
   - **Developer C (SensorSimulationAgent)**: Phase 5 (Chapter 3)
4. **After Chapters 1, 2, 3 complete**:
   - **Developer D (IntegrationAgent)**: Phase 6 (Chapter 4)
5. **All developers**: Phase 7 (Finalization & QA)

---

## Task Summary

**Total Tasks**: 291
- **Setup (Phase 1)**: 7 tasks
- **Foundational (Phase 2)**: 8 tasks (BLOCKING)
- **Chapter 1 (Phase 3)**: 53 tasks
- **Chapter 2 (Phase 4)**: 66 tasks
- **Chapter 3 (Phase 5)**: 77 tasks
- **Chapter 4 (Phase 6)**: 65 tasks
- **Finalization (Phase 7)**: 15 tasks

**Estimated Effort** (assuming single developer):
- Setup + Foundational: 1-2 days
- Chapter 1: 1 week
- Chapter 2: 1 week
- Chapter 3: 1.5 weeks
- Chapter 4: 1.5 weeks
- Finalization: 2-3 days
- **Total**: ~6-7 weeks

**Parallel Opportunities**:
- 127 tasks marked [P] can run in parallel (44% of tasks)
- With 4 specialized developers (subagents): Estimated ~3-4 weeks

**Independent Test Criteria per User Story**:
- **Chapter 1**: RTF ≥ 0.9 for all Gazebo examples, robot stands stably for >5s
- **Chapter 2**: FPS ≥ 30 for all Unity scenes, photorealistic rendering achieved, keyboard control works
- **Chapter 3**: LiDAR ≥10 Hz, depth camera ≥30 Hz, IMU ≥100 Hz, RViz visualization successful
- **Chapter 4**: ROS 2 bridge latency <100ms, RL training completes 1000 steps, policy deploys in Unity

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (Setup + Foundational + Chapter 1) = **68 tasks** (~2 weeks)

---

## Notes

- **[P] tasks**: Different files, no dependencies - safe to parallelize
- **[Story] labels**: Map tasks to specific chapters for traceability (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3, US4=Chapter 4)
- **Tests**: No automated pytest/unittest tests requested. Validation via Docker builds, manual testing, and quality gates.
- **Quality Gates**: Each chapter must pass contract validation (see contracts/chapter-content-contract.md)
- **Checkpoint after each chapter**: Ensure chapter is independently functional before proceeding
- **Commit frequently**: After each task or logical group (e.g., after completing an example)
- **Stop at checkpoints**: Validate independently to catch issues early

---

**Tasks Status**: ✅ **READY FOR EXECUTION**
**Next Command**: `/sp.implement` to begin implementation with specialized subagents
**Recommended First Milestone**: Complete MVP (Phases 1-3) to deliver Chapter 1 (Gazebo Physics Simulation)
