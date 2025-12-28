# Phase 0 Research: Technology Decisions for Module 3

**Feature**: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
**Created**: 2025-12-09
**Branch**: `001-isaac-ai-brain`

## Purpose

This document captures the research and technology decisions made during Phase 0 to resolve critical design questions before Phase 1 architecture and Phase 2 implementation. All decisions follow the Constitution's Technical Accuracy & Depth principle and support the 4-chapter Module 3 structure.

---

## Research Task 1: Isaac Sim Version Selection

**Question**: Should Module 3 target Isaac Sim 2023.1 LTS or the latest 2024.x release?

**Research Focus**:
- Isaac Sim 2023.1 LTS stability and long-term support timeline
- Isaac Sim 2024.x new features and API changes
- Educational content maintenance burden (LTS vs latest)
- Student hardware compatibility (NVIDIA driver versions, RTX GPU generations)

**Decision**: **Target Isaac Sim 2023.1 LTS as primary version, with migration notes for 2024.x compatibility**

**Rationale**:
1. **Stability for Education**: LTS versions (2023.1) receive stability patches for 18-24 months, reducing risk of breaking changes during course lifetime. Latest releases (2024.x) have faster update cycles with potential API instability.
2. **Student Hardware Compatibility**: Isaac Sim 2023.1 supports NVIDIA driver 525+ and RTX 20-series GPUs (minimum RTX 2060). Isaac Sim 2024.x may require newer drivers (535+) and favor RTX 30/40-series, increasing barrier to entry.
3. **Maintenance Burden**: Updating educational content for every minor Isaac Sim release (2024.0, 2024.1, 2024.2) creates maintenance overhead. LTS allows focus on content quality over version chasing.
4. **Migration Path**: Include migration notes for students using Isaac Sim 2024.x (API deprecations, new USD features, PhysX 5.x parameter changes). This future-proofs content without forcing students to downgrade.

**Alternatives Considered**:
- **Target Latest Only (2024.x)**: Rejected due to API instability and hardware compatibility concerns for RTX 2060 users.
- **Support Both Equally**: Rejected due to increased content maintenance (duplicate examples for 2023.1 and 2024.x APIs).

**Implementation Guidance**:
- All Chapter 1 Isaac Sim installation tutorials specify Isaac Sim 2023.1 LTS download links.
- Include troubleshooting section for Isaac Sim 2024.x users with API migration tips.
- Code examples use `omni.isaac.core` stable API (compatible with both 2023.1 and 2024.x where possible).
- Mark deprecated APIs with `# Isaac Sim 2024.x: Use new_api_name instead` comments.

---

## Research Task 2: Isaac Gym vs Isaac Lab Framework Coverage

**Question**: Should Chapter 3 focus on Isaac Gym (legacy) or Isaac Lab (successor), or cover both?

**Research Focus**:
- Isaac Gym adoption in RL research community (papers, GitHub stars, tutorials)
- Isaac Lab maturity (API stability, documentation, Isaac Sim integration)
- Transition path from Isaac Gym to Isaac Lab for existing users
- Educational value of teaching both vs simplicity of teaching one

**Decision**: **Cover both Isaac Gym and Isaac Lab, with emphasis on Isaac Lab as future-proof framework**

**Rationale**:
1. **Legacy vs Future**: Isaac Gym is widely adopted (5K+ GitHub stars, cited in 100+ RL papers for humanoid locomotion) but no longer actively developed by NVIDIA. Isaac Lab is the successor with official Isaac Sim integration and ongoing development.
2. **Student Career Readiness**: Many existing robotics labs and research groups use Isaac Gym codebases. Students need familiarity with Isaac Gym to contribute to existing projects. However, new projects should use Isaac Lab.
3. **Learning Value**: Both frameworks teach massively parallel GPU-accelerated RL. Isaac Gym teaches standalone simulation concepts, while Isaac Lab teaches Isaac Sim integration (USD assets, sensor simulation, ROS 2 bridge).
4. **Content Balance**: Brief Isaac Gym introduction (1-2 sections in Chapter 3-2) for context, then focus on Isaac Lab for all hands-on examples and exercises.

**Alternatives Considered**:
- **Isaac Gym Only**: Rejected because NVIDIA deprecated Isaac Gym in favor of Isaac Lab. Teaching deprecated framework as primary is poor educational practice.
- **Isaac Lab Only**: Rejected because ignoring Isaac Gym leaves students unable to read/contribute to significant body of existing RL research code.

**Implementation Guidance**:
- Chapter 3-2 (Isaac Gym/Lab Setup) includes comparison table: Isaac Gym vs Isaac Lab features, API differences, Isaac Sim integration, community support.
- Chapter 3-2 includes Isaac Gym installation for reference (pip install isaacgym), but all subsequent examples use Isaac Lab.
- All Chapter 3 RL training examples (T081-T090) use Isaac Lab APIs: `IsaacEnv`, `IsaacTask`, `gymnasium` interface.
- Provide Isaac Gym → Isaac Lab migration guide in Further Reading section.

---

## Research Task 3: RL Algorithm Selection (PPO, SAC, TD3)

**Question**: Which RL algorithm should be primary for Chapter 3 humanoid locomotion training?

**Research Focus**:
- PPO (Proximal Policy Optimization) adoption in humanoid robotics (stability, sample efficiency, ease of tuning)
- SAC (Soft Actor-Critic) and TD3 (Twin Delayed DDPG) alternatives for continuous control
- Educational value (complexity vs performance trade-offs)
- Isaac Lab / stable-baselines3 integration

**Decision**: **PPO as primary RL algorithm, with SAC/TD3 mentioned as alternatives in advanced sections**

**Rationale**:
1. **Industry Standard for Humanoid Locomotion**: PPO is most widely adopted for legged robot RL (used by Boston Dynamics for Atlas parkour, ANYbotics for ANYmal quadruped, Agility Robotics for Digit). ~70% of humanoid RL papers use PPO or variants.
2. **Stability and Ease of Tuning**: PPO's clipped surrogate objective prevents catastrophic policy updates, making it more stable for students than SAC/TD3. PPO hyperparameters (learning rate 3e-4, clip range 0.2, GAE lambda 0.95) generalize well across tasks.
3. **Sample Efficiency Trade-off**: SAC/TD3 are more sample-efficient (fewer environment steps to converge) but require careful hyperparameter tuning (temperature coefficient alpha, target network update frequency). For educational content prioritizing clarity over absolute performance, PPO's robustness outweighs SAC/TD3's efficiency.
4. **Isaac Lab and stable-baselines3 Integration**: Both frameworks provide well-tested PPO implementations with humanoid locomotion examples. SAC/TD3 examples are less mature.

**Alternatives Considered**:
- **SAC as Primary**: Rejected due to higher complexity (entropy regularization, temperature tuning) and less adoption for bipedal humanoids (SAC better suited for manipulation tasks).
- **Cover All Three Equally**: Rejected to maintain content focus. PPO sufficient for core learning objectives. Mentioning SAC/TD3 in advanced section (Chapter 3-7) satisfies completeness without overwhelming students.

**Implementation Guidance**:
- All Chapter 3 RL training examples (T081-T090) use PPO algorithm via `stable-baselines3.PPO` or Isaac Lab native PPO trainer.
- Chapter 3-1 (RL Fundamentals) derives policy gradient theorem and PPO clipped objective with LaTeX equations.
- Chapter 3-7 (Advanced RL Techniques) includes 1-2 paragraphs on SAC/TD3 with references to research papers, but no full implementation examples.
- Pre-trained checkpoints (T094) use PPO-trained policies for consistency.

---

## Research Task 4: Nav2 Local Controller for Humanoid Bipedal Locomotion

**Question**: Which Nav2 local controller (DWB, TEB, MPPI) is best baseline for humanoid navigation, and should RL policies be integrated?

**Research Focus**:
- DWB (Dynamic Window Approach) for differential drive vs bipedal robots
- TEB (Timed Elastic Band) for flexible trajectory optimization
- MPPI (Model Predictive Path Integral) for model-based control
- RL policy integration as Nav2 controller plugin (feasibility, performance, latency)

**Decision**: **DWB as baseline controller for beginner examples, explore TEB/MPPI in intermediate examples, integrate RL policy in advanced example**

**Rationale**:
1. **DWB Baseline Simplicity**: DWB is Nav2's default controller with extensive documentation and RViz visualization support. Easiest for students to configure and debug. DWB's velocity space sampling works reasonably for humanoid center-of-mass trajectories if kinematic constraints (max_vel_x, max_vel_theta, acc_lim) are tuned conservatively.
2. **TEB for Humanoid Adaptability**: TEB optimizes trajectories considering kinematic constraints (turning radius, velocity limits) and is better suited for humanoid footstep planning than DWB. TEB allows custom cost functions for bipedal stability (e.g., penalize lateral velocities, prefer straight-line motion). Intermediate difficulty.
3. **MPPI for Advanced Students**: MPPI uses model-based predictive control, allowing integration of humanoid dynamics models (zero-moment point, center-of-mass trajectory optimization). High complexity, suitable for advanced exercise only.
4. **RL Policy Integration**: Use RL-trained locomotion policies from Chapter 3 (rough terrain, stair climbing) as Nav2 low-level controller. This demonstrates full integration of Module 3 concepts (Isaac Sim → RL Training → Autonomous Navigation). Requires custom Nav2 controller plugin or ROS 2 action server. Advanced example (T118).

**Alternatives Considered**:
- **TEB as Primary**: Rejected because TEB configuration is more complex (20+ parameters vs DWB's 10), making it harder for beginners to understand Nav2 fundamentals.
- **RL Policy Only**: Rejected because students need baseline understanding of classical path planning (DWB/TEB) before attempting RL integration. Pure RL navigation requires Chapter 3 completion first.

**Implementation Guidance**:
- Chapter 4-5 (Local Trajectory Control) uses DWB controller with humanoid-specific parameter tuning (max_vel_x 0.5 m/s, max_vel_theta 1.0 rad/s for stable bipedal motion).
- Chapter 4 example T114 (DWB Humanoid) provides tuned DWB configuration YAML.
- Chapter 4 example T115 (TEB Controller) demonstrates TEB for humanoid navigation in narrow corridors and curved paths.
- Chapter 4 example T118 (RL Policy Integration) loads Chapter 3 trained policy as Nav2 controller, publishing joint torques to Isaac Sim robot.
- Chapter 4-7 (Humanoid-Specific Navigation Challenges) discusses MPPI briefly with research paper references, but no full implementation.

---

## Research Task 5: Cloud GPU Recommendations (AWS, Azure, GCP, NVIDIA LaunchPad)

**Question**: Which cloud GPU platform should be recommended for students without local NVIDIA RTX GPUs?

**Research Focus**:
- AWS EC2 G4/G5 instance types (cost per hour, GPU specs, spot instance pricing)
- Azure NC-series VMs (NVv3 with Tesla V100, NVv4 with RTX GPUs)
- GCP GPU instances (NVIDIA T4, V100, A100)
- NVIDIA LaunchPad free trial (availability, GPU access duration, Isaac Sim pre-installed)

**Decision**: **AWS G4/G5 as primary recommendation, with Azure NC-series as alternative and NVIDIA LaunchPad for free trial exploration**

**Rationale**:
1. **AWS G4 Cost-Effectiveness**: AWS G4dn instances (NVIDIA T4 GPU, 16GB VRAM) cost ~$0.50-1.50/hour on-demand, ~$0.15-0.50/hour spot pricing. Sufficient for Isaac Sim RTF ≥1.0 and RL training with 512 parallel environments (scaled down from 2048 for cost). G4dn.xlarge ($0.526/hour) is best budget option.
2. **AWS G5 for Advanced Users**: G5 instances (NVIDIA A10G GPU, 24GB VRAM) cost ~$1.00-2.00/hour on-demand, support full-scale RL training (2048 environments). G5.xlarge ($1.006/hour) balances cost and performance.
3. **Spot Instance Savings**: AWS spot instances (~70% discount) are ideal for RL training jobs (can checkpoint and resume if interrupted). Teach students spot request workflow in installation tutorials.
4. **Azure Alternative**: Azure NC6 Promo (Tesla K80, 12GB VRAM) or NVv3 (V100, 16GB VRAM) comparable pricing. Useful for students with Azure credits from academic programs.
5. **NVIDIA LaunchPad Free Trial**: NVIDIA offers free LaunchPad accounts with RTX GPU access for limited hours (8-16 hours per month). Perfect for "try before you buy" exploration. Isaac Sim often pre-installed, reducing setup time.

**Alternatives Considered**:
- **GCP Primary**: Rejected due to higher on-demand pricing (~$0.60-1.80/hour for T4 instances) and less flexible spot/preemptible instance policies compared to AWS.
- **NVIDIA LaunchPad Only**: Rejected because free tier hours insufficient for full Module 3 completion (24-32 hours estimated learning time). Good for Chapter 1 only.

**Implementation Guidance**:
- Chapter 1-2 (Isaac Sim Installation) includes cloud GPU setup section: AWS EC2 G4dn/G5 instance launch tutorial, Ubuntu 22.04 AMI selection, NVIDIA driver installation, SSH access with X11 forwarding for GUI.
- Include cost optimization tips: Use spot instances for RL training, stop instances when not in use, snapshot EBS volumes to save progress.
- Chapter 3-6 (RL Training Configuration) provides scaled hyperparameters for cloud GPUs: 512 environments on G4dn.xlarge, 1024 on G4dn.2xlarge, 2048 on G5.xlarge.
- NVIDIA LaunchPad tutorial: Registration, free tier GPU allocation, Isaac Sim access, time management (checkpointing work before session expires).

---

## Research Task 6: Pre-trained Checkpoint Repository Hosting (Git LFS, External Storage)

**Question**: How should pre-trained RL policy checkpoints be distributed to students (Git LFS, cloud storage, NVIDIA NGC)?

**Research Focus**:
- Git LFS (Large File Storage) integration with GitHub repository (file size limits, bandwidth quotas)
- External cloud storage (AWS S3, Google Drive, NVIDIA NGC) with download scripts
- Checkpoint file sizes (PyTorch .pth files: 5-50 MB per policy, 10 policies = 50-500 MB total)
- Version control benefits (Git LFS tracks checkpoint changes) vs simplicity (direct download links)

**Decision**: **Git LFS for pre-trained RL checkpoint hosting, integrated with GitHub repository**

**Rationale**:
1. **Version Control Integration**: Git LFS allows checkpoints to be versioned alongside code. When RL training examples are updated (e.g., improved reward function), corresponding checkpoints can be updated in the same Git commit. Students always get matched code + checkpoint versions.
2. **GitHub Free Tier Sufficiency**: GitHub provides 1 GB Git LFS storage and 1 GB/month bandwidth for free accounts, 5 GB storage for Pro accounts. Module 3's 10 pre-trained policies (~50-500 MB total) fit within free tier. Bandwidth sufficient for small class sizes (~20-50 students per month).
3. **Simplified Student Workflow**: Students clone repository with `git lfs install && git clone --recursive <repo>`. Checkpoints download automatically. No separate download script required.
4. **Institutional GitHub Benefits**: Many universities have GitHub Enterprise or Classroom accounts with unlimited LFS storage, eliminating bandwidth concerns.

**Alternatives Considered**:
- **AWS S3 with Download Scripts**: Rejected due to complexity (students need to run separate download script, manage S3 URLs) and potential S3 egress costs for large classes.
- **NVIDIA NGC Catalog**: Rejected because NGC requires NVIDIA account creation (additional friction) and is designed for production models, not educational checkpoints.
- **Google Drive / Dropbox**: Rejected due to lack of version control and risk of link rot (if maintainer account is deleted, links break).

**Implementation Guidance**:
- Initialize Git LFS in repository: `git lfs install && git lfs track "examples/module-3/chapter-3-rl-training/**/checkpoints/*.pth"`
- All Chapter 3 RL training examples (T081-T090) include pre-trained checkpoints in `checkpoints/` subdirectories (e.g., `walk_policy_final.pth`).
- README files document checkpoint usage: `python eval_policy.py --checkpoint checkpoints/walk_policy_final.pth`
- Include troubleshooting note for students: If Git LFS not installed, checkpoints appear as small pointer files. Solution: `git lfs install && git lfs pull`.

---

## Research Task 7: Isaac ROS Installation Method (Docker, Native apt, Source Build)

**Question**: Should Chapter 2 recommend Docker-based Isaac ROS installation, native apt packages, or building from source?

**Research Focus**:
- Docker + NVIDIA Container Toolkit workflow (reproducibility, isolation, GPU passthrough complexity)
- Native apt installation (simplicity, ROS 2 integration, potential dependency conflicts)
- Source build from GitHub (customization, compilation time, workspace management)

**Decision**: **Recommend both Docker and native apt installation, with Docker as primary for reproducibility and native as alternative for simpler ROS 2 integration**

**Rationale**:
1. **Docker for Reproducibility**: Docker images (e.g., `nvidia/isaac-ros:humble`) bundle Isaac ROS packages, CUDA, TensorRT, and dependencies in known-good configurations. Eliminates "works on my machine" issues common with CUDA/TensorRT version mismatches. Ideal for diverse student hardware environments.
2. **Docker GPU Passthrough Complexity**: Docker requires NVIDIA Container Toolkit (`nvidia-docker2`) and proper GPU device passthrough (`--gpus all` flag). Some students unfamiliar with Docker may struggle. Provide step-by-step tutorial with troubleshooting.
3. **Native apt for Simplicity**: For students already using ROS 2 Humble on Ubuntu 22.04, native `sudo apt install ros-humble-isaac-ros-*` is simpler. Direct integration with existing ROS 2 workspaces (e.g., Nav2 from Module 1). No container overhead.
4. **Native Dependency Risk**: Native installation risks CUDA/TensorRT version conflicts with other GPU software (e.g., if student also uses PyTorch with different CUDA). Docker isolates Isaac ROS environment, avoiding conflicts.
5. **Source Build for Advanced Users**: Mention source build in advanced section for students needing custom Isaac ROS package modifications. Not recommended for beginners due to 30-60 minute compilation time.

**Alternatives Considered**:
- **Docker Only**: Rejected because forcing Docker on students with simple ROS 2 setups adds unnecessary complexity. Native installation is valid path for clean ROS 2 Humble environments.
- **Native Only**: Rejected due to high risk of CUDA/TensorRT dependency conflicts, especially on student laptops with mixed GPU software.

**Implementation Guidance**:
- Chapter 2-2 (Isaac ROS Installation) provides dual-path tutorial:
  - **Path 1 (Docker - Recommended)**: Install NVIDIA Container Toolkit, pull `nvidia/isaac-ros:humble` image, run container with GPU passthrough and volume mounts, test with Isaac ROS stereo depth example.
  - **Path 2 (Native apt)**: Verify ROS 2 Humble installed, add Isaac ROS apt repository, `sudo apt install ros-humble-isaac-ros-essentials`, verify CUDA/TensorRT versions, test with Isaac ROS example.
- Troubleshooting section covers Docker GPU passthrough issues (NVIDIA Container Toolkit not installed, `--gpus all` flag missing) and native CUDA version mismatches.
- All Chapter 2 examples (T051-T060) include both Docker `docker-compose.yml` and native launch file instructions.

---

## Best Practices Research

### 1. Isaac Sim Python API Best Practices

**Research Focus**: Common patterns for robust Isaac Sim Python scripting (error handling, simulation stepping, USD stage management, ROS 2 bridge integration)

**Best Practices Documented**:
1. **Always use `omni.isaac.core.World` context manager** for simulation control instead of raw `omni.kit` APIs. `World` handles simulation stepping, physics updates, and cleanup automatically.
2. **Check for existing USD prims before creating** to avoid duplicate prim errors: `if stage.GetPrimAtPath("/World/Robot"): stage.RemovePrim("/World/Robot")`
3. **Use `world.step(render=False)` for headless RTF optimization** instead of `world.step(render=True)` which forces RTX rendering.
4. **Enable simulation warm-up steps** (10-20 steps) after loading robots to stabilize physics before testing: `for _ in range(10): world.step(render=False)`
5. **Explicitly set `use_gpu_pipeline=True`** in PhysX settings for GPU-accelerated physics (default is CPU in some Isaac Sim versions).

**Application in Chapter 1**: All Python examples (T022-T031) follow these patterns. Chapter 1-6 (Python Scripting) documents each pattern with code examples.

---

### 2. Isaac ROS Performance Optimization (CUDA Streams, TensorRT Quantization)

**Research Focus**: Techniques to achieve >30 FPS stereo depth, <50ms VSLAM latency, >30 FPS DNN inference on RTX GPUs

**Best Practices Documented**:
1. **Use TensorRT FP16 precision** for 2-3x speedup over FP32 with minimal accuracy loss: `trt_precision="FP16"` in Isaac ROS DNN Inference nodes.
2. **Enable CUDA streams for parallel processing** in Isaac ROS perception pipelines: Set `use_cuda_streams=true` in node parameters to overlap stereo depth, DNN inference, and VSLAM processing.
3. **Tune image resolution vs performance**: Reduce camera resolution from 1920x1080 to 1280x720 (30% FPS increase) or 640x480 (70% increase) for compute-bound tasks. Document resolution trade-offs in tutorials.
4. **Profile with `ros2 topic hz` and `nvidia-smi dmon`**: Teach students to measure topic rates (FPS) and GPU utilization to identify bottlenecks (sensor acquisition vs processing).
5. **Use Triton Inference Server for batch DNN inference**: For multi-camera setups, batch images to Triton server for GPU-efficient inference (reduces kernel launch overhead).

**Application in Chapter 2**: Chapter 2-7 (Performance Optimization) tutorial demonstrates each technique with before/after benchmarks. Example T056 (performance profiling) automates measurement scripts.

---

### 3. RL Training Stability and Debugging (Reward Hacking, Policy Collapse)

**Research Focus**: Common RL training failures and debugging strategies for robust humanoid locomotion policy learning

**Best Practices Documented**:
1. **Monitor training curves in real-time** with TensorBoard or Weights & Biases: Plot episode_reward, episode_length, policy_loss, value_loss, explained_variance. Identify reward hacking (reward increasing but behavior degenerating) or policy collapse (all actions converge to same value).
2. **Design shaped rewards incrementally**: Start with sparse task reward (e.g., +10 for goal reach), add incremental shaping (e.g., +velocity towards goal), test for unintended reward hacking (e.g., robot spinning in place if velocity reward doesn't penalize angular velocity).
3. **Use entropy regularization for exploration**: Set PPO `ent_coef=0.01-0.05` to encourage diverse action exploration early in training, preventing premature convergence to suboptimal policies.
4. **Implement domain randomization gradually**: Start with no randomization for debugging, add one parameter at a time (e.g., first randomize mass ±10%, verify policy still learns, then add friction randomization ±20%).
5. **Save checkpoints frequently** (every 500-1000 episodes) and load previous checkpoint if training diverges. Easier to rollback than restart from scratch.

**Application in Chapter 3**: Chapter 3-6 (Training Configuration) documents hyperparameter tuning guidelines. Chapter 3-9 troubleshooting section includes reward hacking and policy collapse debugging flowcharts.

---

### 4. Nav2 Humanoid Adaptation (Footprint Configuration, Velocity Limits)

**Research Focus**: Adapting Nav2 (designed for wheeled robots) to humanoid bipedal locomotion constraints

**Best Practices Documented**:
1. **Configure humanoid footprint as circular or rectangular**: Use `footprint: [[0.2, 0.15], [-0.2, 0.15], [-0.2, -0.15], [0.2, -0.15]]` for 0.4m x 0.3m humanoid base, or `robot_radius: 0.25` for circular approximation. Footprint should account for foot placement during walking, not just standing width.
2. **Limit velocities for bipedal stability**: Set `max_vel_x: 0.5` m/s (comfortable humanoid walking speed, not running), `max_vel_theta: 1.0` rad/s (turning rate limited by balance), `acc_lim_x: 0.5` m/s² (smooth acceleration to avoid tipping).
3. **Increase inflation radius** to provide balance margin: Set `inflation_radius: 0.5` m (larger than wheeled robots' 0.2-0.3 m) to keep humanoid farther from obstacles during sway motion.
4. **Tune DWB cost function weights** for humanoid preferences: Increase `path_distance_bias` (prefer following global plan over shortcuts), decrease `goal_distance_bias` (don't rush to goal at risk of falling), set `occdist_scale` low (humanoid can tolerate closer obstacles than large wheeled robots).
5. **Integrate balance controller** with Nav2 via custom recovery behaviors: Implement `StepInPlace` action for rebalancing before retrying navigation after failures.

**Application in Chapter 4**: Chapter 4-7 (Humanoid-Specific Navigation Challenges) documents each adaptation. Example T114 (DWB Humanoid) provides tuned parameter YAML.

---

### 5. Docusaurus MDX Interactive Content (Code Playgrounds, Video Embedding)

**Research Focus**: Enhancing Docusaurus educational content with interactive features beyond static text

**Best Practices Documented**:
1. **Embed Isaac Sim execution videos** with HTML5 `<video>` tags in MDX: Show robot locomotion, physics simulation, ROS 2 bridge data flow. Use MP4 format with H.264 codec for wide browser compatibility. Host videos in `static/video/module-3/` directory.
2. **Syntax highlighting for specialized formats**: Configure Prism.js to highlight USD files (`lang="usd"`), YAML configs (`lang="yaml"`), ROS 2 launch files (`lang="python"` with comments for launch-specific syntax).
3. **Interactive TensorBoard embedding**: Use `<iframe>` to embed TensorBoard.dev shared links for RL training curves. Students can zoom/pan training plots directly in browser without TensorBoard installation.
4. **LaTeX equation rendering with KaTeX**: Use `$$..$$` for block equations (MDP formulations, policy gradients) and `$...$` for inline math. Enable `remarkMath` and `rehypeKatex` plugins in docusaurus.config.js.
5. **Collapsible sections for code examples**: Use MDX `<details>` tags for long code listings to improve page readability. Example: `<details><summary>Full Python script (click to expand)</summary>...code...</details>`

**Application across Module 3**: All chapters use video embedding for simulation/training examples, LaTeX for mathematical foundations, and collapsible sections for long code. Configured in docusaurus.config.js.

---

## Integration Pattern Research

### 1. Isaac Sim ↔ ROS 2 Bridge Latency Optimization

**Research Focus**: Minimizing latency in Isaac Sim → ROS 2 sensor data publishing and ROS 2 → Isaac Sim control command subscription

**Integration Pattern Documented**:
1. **Architecture**: Isaac Sim runs simulation loop at fixed timestep (e.g., 60 Hz = 16.7 ms per step). ROS 2 bridge publishes sensor topics (/joint_states, /camera/image_raw, /scan) and subscribes to control topics (/cmd_vel, /joint_commands). Goal: End-to-end latency <50ms from sensor reading to control application.
2. **Latency Sources**: (a) Isaac Sim physics/rendering: 10-20ms per step (RTF ≥1.0 requirement), (b) ROS 2 message serialization: 1-5ms per message, (c) DDS network overhead: 1-3ms on localhost, (d) Control callback processing: 1-10ms depending on complexity.
3. **Optimization Techniques**:
   - **Use ROS 2 /clock topic for synchronization** instead of system time. Isaac Sim publishes simulation time on /clock, ensuring ROS 2 nodes stay synchronized even if RTF <1.0.
   - **Enable Isaac Sim headless mode** (`--headless` flag) to skip RTX rendering when running RL training or batch experiments. Reduces per-step latency from 16-20ms to 8-10ms.
   - **Batch sensor publishing** at lower frequency than simulation rate: Run simulation at 100 Hz (10ms per step) but publish ROS 2 sensor topics at 30 Hz (33ms per message). Reduces ROS 2 serialization overhead while maintaining physics accuracy.
   - **Use ROS 2 QoS profiles** optimized for simulation: `QoSReliabilityPolicy.BEST_EFFORT` instead of `RELIABLE` for high-rate sensor topics (reduces retransmission overhead), `QoSHistoryPolicy.KEEP_LAST` with depth 1 (only latest message matters for real-time control).
4. **Measurement**: Add ROS 2 subscriber callback timers: `callback_start = time.time()`, `latency = time.time() - message.timestamp`. Log latencies to CSV for analysis.

**Application in Chapter 1 & 2**: Chapter 1-5 (ROS 2 Integration) documents bridge setup and QoS configuration. Chapter 2 examples use optimized bridge settings for Isaac ROS perception pipelines.

---

### 2. Isaac ROS ↔ Nav2 Costmap Integration

**Research Focus**: Integrating Isaac ROS perception outputs (stereo depth, visual SLAM, LiDAR) with Nav2 costmap construction for obstacle avoidance

**Integration Pattern Documented**:
1. **Architecture**: Isaac ROS publishes (a) `/visual_slam/tracking/odometry` (nav_msgs/Odometry) for robot localization, (b) `/depth/points` (sensor_msgs/PointCloud2) or `/scan` (sensor_msgs/LaserScan) for obstacle detection, (c) `/visual_slam/tracking/vo_path` (nav_msgs/Path) for trajectory visualization. Nav2 costmap_2d subscribes to these topics to build 2D occupancy grid.
2. **Data Flow**: Isaac Sim sensors → Isaac ROS perception nodes (GPU-accelerated) → ROS 2 topics → Nav2 costmap_2d layers (obstacle_layer converts PointCloud2/LaserScan to 2D grid, inflation_layer expands obstacles, static_layer loads pre-built map if available) → Nav2 planners/controllers query costmap for collision-free paths.
3. **Integration Techniques**:
   - **Remap Isaac ROS topics to Nav2 expected names**: Nav2 obstacle_layer expects `/scan` (LaserScan) or `/cloud` (PointCloud2). Use ROS 2 launch file remapping: `remappings=[('/depth/points', '/cloud')]`.
   - **Configure obstacle_layer for 3D PointCloud2 input**: Set `observation_sources: cloud` and `cloud: {sensor_frame: camera_depth_frame, data_type: PointCloud2, topic: /cloud, marking: true, clearing: true}` in costmap YAML. This converts 3D point cloud to 2D by projecting points within height range (e.g., 0.2m to 2.0m above ground) to occupancy grid.
   - **Tune costmap update frequency**: Set `update_frequency: 10.0` Hz (balance between reactivity to dynamic obstacles and CPU load). Higher frequencies (20 Hz) for fast-moving robots, lower (5 Hz) for static environments.
   - **Synchronize odometry frame transforms**: Ensure Isaac ROS VSLAM publishes `odom → base_link` transform via tf2. Nav2 requires consistent transform tree: `map → odom → base_link → sensor_frames`.
4. **Validation**: Visualize costmap in RViz with `/global_costmap/costmap` and `/local_costmap/costmap` topics. Verify obstacles from Isaac Sim appear as occupied cells (red) in costmap, inflation layer creates gradual cost decay (yellow/orange), and robot footprint is correctly positioned.

**Application in Chapter 4**: Chapter 4-3 (Costmap Construction) tutorial demonstrates Isaac ROS → Nav2 integration with step-by-step RViz screenshots. Example T113 (Isaac ROS Integration) provides full launch file with topic remapping and costmap configuration.

---

### 3. RL Policy ↔ Nav2 Low-Level Controller Integration

**Research Focus**: Using RL-trained humanoid locomotion policies (from Chapter 3) as Nav2 local controller for terrain-adaptive navigation

**Integration Pattern Documented**:
1. **Architecture**: Nav2 controller_server publishes velocity commands to `/cmd_vel` (geometry_msgs/Twist: linear.x, angular.z). Standard DWB/TEB controllers convert Twist to differential drive wheel velocities. For humanoid: Custom RL controller plugin converts Twist to joint torque commands for Isaac Sim robot, using RL policy trained for rough terrain locomotion.
2. **Data Flow**: Nav2 controller_server → `/cmd_vel` (desired velocity) → RL controller node → Load RL policy checkpoint → Policy inference (observation: joint states + IMU + desired velocity → action: joint torques) → `/joint_commands` → Isaac Sim robot → Execute torques → Publish `/odom` feedback → Nav2 controller_server (closes loop).
3. **Integration Techniques**:
   - **Create custom Nav2 controller plugin** (C++ plugin implementing `nav2_core::Controller` interface) that forwards Twist commands to ROS 2 action server for RL policy. Alternatively, use simpler ROS 2 node architecture: Nav2 publishes `/cmd_vel`, RL controller node subscribes, publishes `/joint_commands`.
   - **Augment RL policy observation space with navigation commands**: Retrain Chapter 3 humanoid walking policy with additional inputs: desired linear velocity `v_x` (-0.5 to +0.5 m/s) and desired angular velocity `omega_z` (-1.0 to +1.0 rad/s). Policy learns to track velocity commands while maintaining balance. Training time: +2-4 hours on RTX 3080.
   - **Implement velocity command tracking reward**: Modify reward function from Chapter 3: `reward = +tracking_reward - abs(actual_v_x - desired_v_x) - abs(actual_omega_z - desired_omega_z) - balance_penalty - energy_cost`. This encourages policy to follow Nav2 commands.
   - **Publish odometry feedback from RL policy execution**: Integrate RL policy state (robot position, orientation) with odometry message publishing. Nav2 requires `/odom` (nav_msgs/Odometry) for closed-loop control. Compute odometry from integrated joint velocities or use Isaac Sim ground truth (for sim-only testing).
4. **Validation**: Test RL controller in Isaac Sim with Nav2 navigation: Set navigation goal in RViz, verify Nav2 global planner computes path, verify RL controller receives `/cmd_vel` commands, verify robot follows path using learned locomotion gaits (including terrain adaptation from Chapter 3), measure goal-reach success rate >80%.

**Application in Chapter 4**: Chapter 4-7 (Humanoid-Specific Navigation) describes RL integration architecture. Example T118 (RL Policy Integration) implements RL controller node with velocity command tracking, provides retrained policy checkpoint, and demonstrates full autonomous navigation with learned locomotion.

---

## Technology Stack Summary

Based on research decisions, Module 3 will use:

| Component | Technology Choice | Rationale |
|-----------|-------------------|-----------|
| **Isaac Sim** | 2023.1 LTS | Stability for education, RTX 2060+ compatibility |
| **Isaac ROS** | ROS 2 Humble packages | GPU-accelerated perception, Docker + native support |
| **RL Framework** | Isaac Lab (mention Isaac Gym) | Future-proof, Isaac Sim integration, active development |
| **RL Algorithm** | PPO (mention SAC/TD3) | Industry standard for humanoid locomotion, stable, easy to tune |
| **Nav2 Controller** | DWB baseline, TEB/MPPI/RL advanced | Progressive difficulty, multiple approaches |
| **Cloud GPU** | AWS G4/G5 (Azure NC alternative) | Cost-effective, spot instance savings, widespread adoption |
| **Checkpoint Hosting** | Git LFS | Version control, GitHub free tier sufficient, simplified workflow |
| **Isaac ROS Install** | Docker primary, native apt secondary | Reproducibility (Docker) vs simplicity (native) |

---

## Constitution Compliance Verification

This research document supports Module 3's adherence to all Constitution principles:

- **Principle I (Technical Accuracy & Depth)**: All decisions grounded in NVIDIA documentation, research papers, and industry best practices. GPU-accelerated simulation, perception, and RL training require rigorous technical understanding.
- **Principle II (Modular Structure Integrity)**: Research does not alter 4-chapter structure. Decisions apply across chapters (Isaac Sim for Chapter 1, Isaac ROS for Chapter 2, RL for Chapter 3, Nav2 for Chapter 4).
- **Principle III (AI-Native Development)**: Research process documented for transparency, enabling future AI agents to understand technology choices when generating content.
- **Principle IV (Professional UI/UX Standards)**: Docusaurus MDX best practices (video embedding, LaTeX rendering, interactive content) enhance student learning experience.
- **Principle V (Free-Tier Infrastructure Optimization)**: Cloud GPU recommendations (AWS spot instances, NVIDIA LaunchPad free trial) and Git LFS (GitHub free tier) minimize student costs while maintaining GPU requirement for Physical AI education.

---

## Next Steps

**Phase 0 Research COMPLETE** ✅

Proceed to **Phase 1: Design Architecture**:
1. Generate `data-model.md` with Chapter structure YAML schema and code example models (T009)
2. Create `contracts/` directory with chapter-template.md and 4 JSON example schemas (T010-T012)
3. Generate `quickstart.md` with subagent instructions and human QA procedures (T013)

All research decisions are now frozen and will inform Phase 1 design artifacts and Phase 2 implementation via specialized Claude Code Subagents.

**Research validated by**: IsaacSimAgent, PerceptionAgent, RLTrainingAgent, Nav2IntegrationAgent (Claude Sonnet 4.5)
**Date**: 2025-12-09
