# Module 2: Digital Twin Simulation - Data Model

**Feature**: 002-module-2-digital-twin
**Created**: 2025-12-07
**Status**: Complete
**Purpose**: Define canonical data structures for educational content, simulation examples, and cross-chapter shared resources

---

## 1. Educational Content Data Model

### 1.1 Chapter Structure

```typescript
interface Chapter {
  id: string;                    // "chapter-1-gazebo-physics", "chapter-2-unity-rendering", etc.
  title: string;                 // "Simulating Physics, Gravity, and Collisions in Gazebo"
  moduleId: string;              // "module-2-digital-twin"
  orderIndex: number;            // 1, 2, 3, 4 (sequential)

  metadata: ChapterMetadata;
  learningObjectives: LearningObjective[];
  prerequisites: Prerequisite[];
  sections: Section[];
  codeExamples: CodeExample[];
  exercises: Exercise[];
  assessments: Assessment[];
  resources: Resource[];
}

interface ChapterMetadata {
  estimatedDuration: string;     // "4-6 hours"
  difficultyLevel: "beginner" | "intermediate" | "advanced";
  tags: string[];                // ["gazebo", "physics", "simulation", "ros2"]
  lastUpdated: string;           // ISO 8601 date
  contributors: string[];        // ["alishba-nasir"]
}
```

### 1.2 Learning Objectives

```typescript
interface LearningObjective {
  id: string;                    // "LO-2.1.1"
  chapterId: string;             // "chapter-1-gazebo-physics"
  order: number;                 // 1, 2, 3, ...
  description: string;           // "Understand Gazebo architecture (Client, Server, Physics, Rendering)"
  bloomLevel: BloomLevel;        // "understand", "apply", "analyze", "create"
  assessmentCriteria: string[];  // How students demonstrate mastery
}

enum BloomLevel {
  Remember = "remember",
  Understand = "understand",
  Apply = "apply",
  Analyze = "analyze",
  Evaluate = "evaluate",
  Create = "create"
}
```

### 1.3 Section Structure

```typescript
interface Section {
  id: string;                    // "section-1.1-gazebo-architecture"
  chapterId: string;
  title: string;                 // "Gazebo Architecture and Components"
  orderIndex: number;            // 1.1, 1.2, 1.3, ...

  content: ContentBlock[];       // Markdown text, diagrams, callouts
  codeExampleRefs: string[];     // References to CodeExample.id
  exerciseRefs: string[];        // References to Exercise.id
  estimatedDuration: string;     // "30-45 minutes"
}

interface ContentBlock {
  type: "markdown" | "diagram" | "callout" | "video" | "interactive";
  content: string;               // Markdown text or diagram URL
  metadata?: {
    diagramType?: "architecture" | "flowchart" | "sequence" | "class";
    calloutType?: "info" | "warning" | "tip" | "danger";
    videoUrl?: string;
    interactiveWidget?: string;  // e.g., "gazebo-world-visualizer"
  };
}
```

---

## 2. Code Example Data Model

### 2.1 Code Example Structure

```typescript
interface CodeExample {
  id: string;                    // "gazebo-basic-world", "unity-urdf-import", etc.
  chapterId: string;
  title: string;                 // "Basic Gazebo World with Gravity Simulation"
  description: string;           // Purpose and learning outcome

  exampleType: ExampleType;
  difficulty: "beginner" | "intermediate" | "advanced";
  estimatedCompletionTime: string; // "15-20 minutes"

  files: CodeFile[];
  runInstructions: RunInstructions;
  expectedOutput: ExpectedOutput;
  troubleshooting: TroubleshootingGuide[];

  dependencies: Dependency[];
  testValidation: TestValidation;
}

enum ExampleType {
  GazeboWorld = "gazebo-world",           // SDF world file
  GazeboPlugin = "gazebo-plugin",         // ROS 2 Gazebo plugin configuration
  UnityScene = "unity-scene",             // Unity scene setup
  UnityScript = "unity-script",           // C# scripts for Unity
  ROSPackage = "ros-package",             // ROS 2 package with nodes
  PythonScript = "python-script",         // Standalone Python (RL training, sensor processing)
  IntegrationExample = "integration"      // Multi-system example (Gazebo + Unity + ROS 2)
}
```

### 2.2 Code Files

```typescript
interface CodeFile {
  id: string;                    // "basic_world.sdf", "robot_controller.py", etc.
  path: string;                  // Relative path in examples/ directory
  language: "xml" | "python" | "csharp" | "yaml" | "bash";
  content: string;               // Full file content
  highlights: CodeHighlight[];   // Lines to emphasize in documentation
  annotations: Annotation[];     // Inline explanations
}

interface CodeHighlight {
  lineStart: number;
  lineEnd: number;
  description: string;           // What this code block does
}

interface Annotation {
  line: number;
  type: "explanation" | "warning" | "best-practice";
  text: string;
}
```

### 2.3 Run Instructions

```typescript
interface RunInstructions {
  setup: SetupStep[];            // Pre-run setup (install packages, build workspace)
  execution: ExecutionStep[];    // Commands to run the example
  verification: VerificationStep[]; // How to verify success
}

interface SetupStep {
  order: number;
  description: string;           // "Install Gazebo ROS packages"
  commands: string[];            // ["sudo apt install ros-humble-gazebo-ros-pkgs"]
  platform: "linux" | "windows" | "macos" | "all";
}

interface ExecutionStep {
  order: number;
  description: string;           // "Launch Gazebo with the basic world"
  commands: string[];            // ["ros2 launch gazebo_sim basic_world.launch.py"]
  expectedDuration: string;      // "5-10 seconds to load"
}

interface VerificationStep {
  order: number;
  description: string;           // "Verify physics simulation is running"
  method: "visual" | "command" | "ros-topic" | "log-check";
  details: string;               // "Box should fall due to gravity. Check /gazebo/performance_metrics topic for RTF ≥ 0.9"
}
```

### 2.4 Expected Output

```typescript
interface ExpectedOutput {
  type: "terminal" | "visual" | "ros-topic" | "file";
  description: string;           // What students should observe
  successCriteria: SuccessCriterion[];
  screenshots?: string[];        // Paths to reference screenshots
  logs?: string[];               // Sample terminal output
}

interface SuccessCriterion {
  id: string;
  description: string;           // "RTF should be ≥ 0.9"
  validationMethod: string;      // "Check /gazebo/performance_metrics topic"
  passingValue: string;          // "real_time_factor >= 0.9"
}
```

---

## 3. Simulation Asset Data Model

### 3.1 Gazebo Assets

```typescript
interface GazeboWorld {
  id: string;                    // "humanoid-physics-test-world"
  name: string;                  // "Humanoid Physics Testing Environment"
  sdfVersion: string;            // "1.9"

  physics: PhysicsConfig;
  scene: SceneConfig;
  models: ModelReference[];
  plugins: PluginConfig[];

  filePath: string;              // "examples/gazebo/worlds/humanoid_physics.world"
}

interface PhysicsConfig {
  engine: "ode" | "bullet" | "dart" | "simbody";
  maxStepSize: number;           // 0.001 (1ms)
  realTimeUpdateRate: number;    // 1000 Hz
  gravity: [number, number, number]; // [0, 0, -9.81]

  solverConfig?: {
    iters?: number;              // ODE: solver iterations
    sor?: number;                // ODE: successive over-relaxation
  };
}

interface SceneConfig {
  ambient: [number, number, number, number]; // RGBA
  background: [number, number, number, number]; // RGBA
  shadows: boolean;
  grid: boolean;
}

interface ModelReference {
  name: string;                  // "ground_plane", "humanoid_robot"
  uri: string;                   // "model://ground_plane" or path to custom model
  pose: Pose;
}

interface Pose {
  position: [number, number, number]; // [x, y, z]
  orientation: [number, number, number, number]; // [roll, pitch, yaw, w] (quaternion)
}

interface PluginConfig {
  name: string;                  // "gazebo_ros_state", "gazebo_ros_ray_sensor"
  filename: string;              // "libgazebo_ros_state.so"
  parameters: Record<string, any>; // Plugin-specific parameters
}
```

### 3.2 Unity Assets

```typescript
interface UnityScene {
  id: string;                    // "humanoid-urdf-visualization"
  name: string;                  // "Humanoid Robot URDF Visualization"
  unityVersion: string;          // "2022.3 LTS"
  renderPipeline: "URP" | "HDRP" | "Built-in";

  gameObjects: GameObjectConfig[];
  lighting: LightingConfig;
  postProcessing: PostProcessingConfig;

  sceneAssetPath: string;        // "examples/unity/Scenes/HumanoidVisualization.unity"
}

interface GameObjectConfig {
  name: string;                  // "HumanoidRobot", "DirectionalLight", "Main Camera"
  type: "urdf-robot" | "camera" | "light" | "ui-canvas" | "custom";
  transform: Transform;
  components: ComponentConfig[];
}

interface Transform {
  position: [number, number, number];
  rotation: [number, number, number, number]; // Quaternion
  scale: [number, number, number];
}

interface ComponentConfig {
  type: string;                  // "ArticulationBody", "Camera", "Light", "RosSubscriber"
  parameters: Record<string, any>;
}

interface LightingConfig {
  skybox: string;                // Asset path or "Procedural"
  ambientMode: "Skybox" | "Trilight" | "Flat";
  ambientIntensity: number;      // 0.0 - 1.0
  directionalLights: DirectionalLightConfig[];
}

interface DirectionalLightConfig {
  name: string;
  color: [number, number, number]; // RGB
  intensity: number;             // Lux (URP)
  rotation: [number, number, number]; // Euler angles
  shadows: "None" | "Hard" | "Soft";
}

interface PostProcessingConfig {
  enabled: boolean;
  effects: {
    bloom?: { enabled: boolean; intensity: number; threshold: number; };
    depthOfField?: { enabled: boolean; focusDistance: number; aperture: number; };
    ambientOcclusion?: { enabled: boolean; intensity: number; radius: number; };
    colorGrading?: { enabled: boolean; temperature: number; saturation: number; };
  };
}
```

---

## 4. Sensor Simulation Data Model

### 4.1 Gazebo Sensor Configuration

```typescript
interface GazeboSensor {
  id: string;                    // "lidar-sensor-vlp16", "imu-sensor-bno055"
  type: "ray" | "camera" | "depth_camera" | "imu" | "contact" | "force_torque";
  name: string;                  // "VLP-16 LiDAR Sensor"

  updateRate: number;            // Hz (e.g., 10 for LiDAR, 100 for IMU)
  topic: string;                 // ROS 2 topic name (e.g., "/scan", "/imu/data")
  frameId: string;               // TF frame (e.g., "lidar_link", "imu_link")

  plugin: PluginConfig;
  sensorSpecificConfig: RaySensorConfig | CameraConfig | IMUConfig;

  noiseModel?: NoiseModel;
}

interface RaySensorConfig {
  scanType: "horizontal" | "vertical" | "3d";
  horizontal: {
    samples: number;             // 1875 for VLP-16
    resolution: number;          // 1.0
    minAngle: number;            // -3.14159 (radians)
    maxAngle: number;            // 3.14159
  };
  vertical?: {
    samples: number;             // 16 for VLP-16
    resolution: number;
    minAngle: number;            // -0.2618 (radians, -15°)
    maxAngle: number;            // 0.2618 (+15°)
  };
  range: {
    min: number;                 // 0.1 meters
    max: number;                 // 100.0 meters
    resolution: number;          // 0.01 meters
  };
}

interface CameraConfig {
  imageFormat: "R8G8B8" | "L8" | "R16";
  width: number;                 // 640, 1280, etc.
  height: number;                // 480, 720, etc.
  fov: number;                   // Field of view (radians)

  clip: {
    near: number;                // 0.1 meters
    far: number;                 // 100.0 meters
  };
}

interface IMUConfig {
  linearAcceleration: {
    x: AxisConfig;
    y: AxisConfig;
    z: AxisConfig;
  };
  angularVelocity: {
    x: AxisConfig;
    y: AxisConfig;
    z: AxisConfig;
  };
  orientation: {
    x: AxisConfig;
    y: AxisConfig;
    z: AxisConfig;
  };
}

interface AxisConfig {
  noise: NoiseModel;
  bias?: number;                 // Constant bias offset
}

interface NoiseModel {
  type: "gaussian" | "gaussian_quantized";
  mean: number;
  stddev: number;
}
```

### 4.2 Unity Sensor Configuration

```typescript
interface UnitySensor {
  id: string;                    // "depth-camera-realsense-d435"
  type: "rgb-camera" | "depth-camera" | "semantic-segmentation" | "instance-segmentation";
  name: string;                  // "Intel RealSense D435 Depth Camera"

  perceptionCamera: PerceptionCameraConfig;
  rosPublisher: ROSPublisherConfig;
}

interface PerceptionCameraConfig {
  outputFormat: "Color" | "Depth" | "Segmentation";
  width: number;                 // 640, 1280
  height: number;                // 480, 720
  captureRate: number;           // Hz (e.g., 30)

  fieldOfView: number;           // Degrees
  nearClipPlane: number;         // 0.3 meters
  farClipPlane: number;          // 10.0 meters

  depthEncoding?: "DepthPerspective" | "DepthOrthographic";
  segmentationLabels?: SegmentationLabel[];
}

interface SegmentationLabel {
  labelId: number;
  labelName: string;             // "robot", "ground", "obstacle"
  pixelValue: [number, number, number]; // RGB color
}

interface ROSPublisherConfig {
  topic: string;                 // "/camera/depth/image_raw"
  messageType: string;           // "sensor_msgs/Image"
  frameId: string;               // "camera_depth_optical_frame"
  publishRate: number;           // Hz (should match captureRate)
}
```

---

## 5. ROS 2 Integration Data Model

### 5.1 ROS 2 Package Structure

```typescript
interface ROS2Package {
  id: string;                    // "gazebo_sim", "unity_bridge"
  name: string;                  // Package name
  version: string;               // "0.1.0"
  description: string;

  maintainers: Maintainer[];
  license: string;               // "Apache-2.0"

  buildType: "ament_cmake" | "ament_python";
  dependencies: Dependency[];

  nodes: ROS2Node[];
  launchFiles: LaunchFile[];
  messages: CustomMessage[];
  services: CustomService[];
}

interface ROS2Node {
  id: string;                    // "robot_state_publisher", "joint_controller"
  name: string;
  executable: string;            // Python script or C++ binary name

  publishers: Topic[];
  subscribers: Topic[];
  services: Service[];
  parameters: Parameter[];
}

interface Topic {
  name: string;                  // "/joint_states", "/cmd_vel"
  messageType: string;           // "sensor_msgs/JointState", "geometry_msgs/Twist"
  qos: QoSProfile;
}

interface QoSProfile {
  reliability: "reliable" | "best_effort";
  durability: "volatile" | "transient_local";
  history: "keep_last" | "keep_all";
  depth: number;                 // For keep_last
}

interface Service {
  name: string;                  // "/spawn_entity", "/reset_simulation"
  serviceType: string;           // "gazebo_msgs/SpawnEntity", "std_srvs/Empty"
}

interface Parameter {
  name: string;                  // "robot_description", "use_sim_time"
  type: "string" | "int" | "double" | "bool" | "string_array";
  defaultValue: any;
}

interface LaunchFile {
  id: string;                    // "gazebo_world.launch.py"
  path: string;                  // "launch/gazebo_world.launch.py"
  description: string;

  arguments: LaunchArgument[];
  includedLaunchFiles: string[]; // Other launch files to include
  nodes: string[];               // Node IDs launched by this file
}

interface LaunchArgument {
  name: string;                  // "world_file", "gui"
  type: "string" | "bool";
  defaultValue: string;
  description: string;
}
```

### 5.2 Custom Message Definitions

```typescript
interface CustomMessage {
  id: string;                    // "PerformanceMetrics.msg"
  packageName: string;           // "gazebo_sim_msgs"
  messageName: string;           // "PerformanceMetrics"

  fields: MessageField[];
  constants?: MessageConstant[];

  filePath: string;              // "gazebo_sim_msgs/msg/PerformanceMetrics.msg"
}

interface MessageField {
  type: string;                  // "float64", "string", "sensor_msgs/Image"
  name: string;                  // "real_time_factor", "timestamp"
  isArray: boolean;
  arraySize?: number;            // Fixed size or -1 for dynamic
  defaultValue?: any;
}

interface MessageConstant {
  type: string;                  // "uint8"
  name: string;                  // "STATUS_RUNNING"
  value: any;                    // 1
}
```

---

## 6. Reinforcement Learning Data Model

### 6.1 Gymnasium Environment

```typescript
interface GymnasiumEnvironment {
  id: string;                    // "HumanoidWalk-v0"
  name: string;                  // "Humanoid Walking Environment"
  description: string;

  observationSpace: Space;
  actionSpace: Space;
  rewardFunction: RewardFunction;

  resetBehavior: ResetBehavior;
  stepBehavior: StepBehavior;

  rosIntegration: ROSIntegrationConfig;
}

interface Space {
  type: "Box" | "Discrete" | "MultiDiscrete" | "MultiBinary" | "Dict" | "Tuple";
  shape?: number[];              // For Box: [observation_dim]
  low?: number | number[];       // For Box: minimum values
  high?: number | number[];      // For Box: maximum values
  n?: number;                    // For Discrete: number of discrete values

  description: string;           // What this space represents
  example?: any;                 // Example observation/action
}

interface RewardFunction {
  components: RewardComponent[];
  totalRewardFormula: string;    // "forward_progress + balance_penalty - energy_cost"
}

interface RewardComponent {
  name: string;                  // "forward_progress", "balance_penalty"
  weight: number;                // Scalar multiplier
  formula: string;               // "velocity_x * dt"
  range: [number, number];       // Expected reward range
}

interface ResetBehavior {
  triggerConditions: string[];   // ["robot_falls", "episode_timeout", "out_of_bounds"]
  resetProcedure: string;        // "Reset robot to origin, zero velocities, randomize joint positions ±5°"
  maxEpisodeSteps: number;       // 1000
}

interface StepBehavior {
  actionProcessing: string;      // "Map actions to joint torques, publish to /joint_commands"
  observationCollection: string; // "Read /joint_states, /imu/data, /base_link pose"
  stepDuration: number;          // Simulation timestep (seconds)
}

interface ROSIntegrationConfig {
  commandTopics: string[];       // ["/joint_commands"]
  observationTopics: string[];   // ["/joint_states", "/imu/data"]
  resetService: string;          // "/gazebo/reset_world"
}
```

### 6.2 Training Configuration

```typescript
interface TrainingConfig {
  id: string;                    // "humanoid-walk-ppo-v1"
  environmentId: string;         // "HumanoidWalk-v0"
  algorithm: "PPO" | "SAC" | "TD3" | "A2C" | "DQN";

  hyperparameters: Hyperparameters;
  trainingProcedure: TrainingProcedure;
  evaluation: EvaluationConfig;
}

interface Hyperparameters {
  learningRate: number;          // 3e-4
  batchSize: number;             // 64
  nSteps: number;                // 2048 (for PPO)
  gamma: number;                 // 0.99 (discount factor)

  algorithmSpecific?: {
    // PPO
    nEpochs?: number;            // 10
    clipRange?: number;          // 0.2
    entCoef?: number;            // 0.01 (entropy coefficient)
    vfCoef?: number;             // 0.5 (value function coefficient)
    maxGradNorm?: number;        // 0.5

    // SAC
    tau?: number;                // 0.005 (soft update coefficient)
    bufferSize?: number;         // 1000000 (replay buffer)
    trainFreq?: number;          // 1
  };
}

interface TrainingProcedure {
  totalTimesteps: number;        // 1000000
  logInterval: number;           // 100 (timesteps between logs)
  saveInterval: number;          // 10000 (timesteps between checkpoints)
  evalInterval: number;          // 5000 (timesteps between evaluations)

  checkpointDir: string;         // "checkpoints/humanoid-walk-ppo-v1"
  tensorboardLogDir: string;     // "logs/tensorboard/humanoid-walk-ppo-v1"
}

interface EvaluationConfig {
  nEvalEpisodes: number;         // 10
  evalFreq: number;              // 5000 (timesteps)
  successThreshold: number;      // Mean reward threshold for success

  metrics: EvaluationMetric[];
}

interface EvaluationMetric {
  name: string;                  // "mean_reward", "episode_length", "success_rate"
  loggingMethod: "tensorboard" | "csv" | "wandb";
}
```

---

## 7. Cross-Chapter Shared Resources

### 7.1 URDF Robot Model

```typescript
interface URDFRobotModel {
  id: string;                    // "simple-humanoid-v1"
  name: string;                  // "Simple Humanoid Robot"
  description: string;

  urdfFilePath: string;          // "examples/urdf/simple_humanoid.urdf"
  meshDirectories: string[];     // ["meshes/visual/", "meshes/collision/"]

  links: URDFLink[];
  joints: URDFJoint[];

  gazeboCompatible: boolean;
  unityCompatible: boolean;      // Requires URDF Importer package
}

interface URDFLink {
  name: string;                  // "base_link", "torso", "left_leg", etc.
  inertial?: Inertial;
  visual?: Visual[];
  collision?: Collision[];
}

interface Inertial {
  mass: number;                  // kg
  origin: Pose;
  inertia: {
    ixx: number; ixy: number; ixz: number;
    iyy: number; iyz: number; izz: number;
  };
}

interface Visual {
  name?: string;
  origin: Pose;
  geometry: Geometry;
  material?: Material;
}

interface Collision {
  name?: string;
  origin: Pose;
  geometry: Geometry;
}

interface Geometry {
  type: "box" | "cylinder" | "sphere" | "mesh";
  size?: [number, number, number]; // For box
  radius?: number;               // For cylinder, sphere
  length?: number;               // For cylinder
  meshFilename?: string;         // For mesh
  meshScale?: [number, number, number];
}

interface Material {
  name: string;
  color?: [number, number, number, number]; // RGBA
  texture?: string;              // Path to texture file
}

interface URDFJoint {
  name: string;                  // "hip_joint", "knee_joint"
  type: "revolute" | "continuous" | "prismatic" | "fixed" | "floating" | "planar";
  parent: string;                // Link name
  child: string;                 // Link name
  origin: Pose;
  axis?: [number, number, number]; // Rotation/translation axis

  limit?: {
    lower: number;               // Radians or meters
    upper: number;
    effort: number;              // Max force/torque
    velocity: number;            // Max velocity
  };

  dynamics?: {
    damping: number;
    friction: number;
  };
}
```

### 7.2 Dependency Management

```typescript
interface Dependency {
  name: string;                  // "ros-humble-gazebo-ros-pkgs"
  type: "apt" | "pip" | "unity-package" | "git-repo";
  version?: string;              // ">=2.0.0"
  platform: "linux" | "windows" | "macos" | "all";

  installCommand: string;        // "sudo apt install ros-humble-gazebo-ros-pkgs"
  verificationCommand?: string;  // "ros2 pkg list | grep gazebo_ros"

  isOptional: boolean;
  purpose: string;               // "Required for Gazebo physics simulation"
}
```

---

## 8. Data Validation and Quality

### 8.1 Content Validation Rules

```typescript
interface ContentValidationRule {
  id: string;                    // "code-example-runnable"
  name: string;                  // "All code examples must be runnable"
  description: string;
  severity: "error" | "warning" | "info";

  validationFunction: (content: any) => ValidationResult;
}

interface ValidationResult {
  passed: boolean;
  message?: string;
  affectedItems?: string[];      // IDs of content items that failed
}

// Example validation rules:
const ValidationRules = {
  codeExampleRunnable: {
    id: "code-example-runnable",
    description: "All code examples must include run instructions and expected output",
    validate: (example: CodeExample) => {
      return example.runInstructions && example.expectedOutput;
    }
  },

  learningObjectivesCount: {
    id: "learning-objectives-count",
    description: "Each chapter must have 3-5 learning objectives (FR-001, FR-013, FR-022, FR-033)",
    validate: (chapter: Chapter) => {
      const count = chapter.learningObjectives.length;
      return count >= 3 && count <= 5;
    }
  },

  codeExamplesMinimum: {
    id: "code-examples-minimum",
    description: "Each chapter must have at least 3 executable code examples (FR-003, FR-015, FR-024, FR-035)",
    validate: (chapter: Chapter) => {
      return chapter.codeExamples.length >= 3;
    }
  }
};
```

---

## 9. File System Organization

```
examples/
├── gazebo/
│   ├── worlds/
│   │   ├── basic_world.world                  # GazeboWorld instances
│   │   ├── physics_test.world
│   │   └── sensor_test.world
│   ├── models/
│   │   └── simple_humanoid/
│   │       ├── model.sdf
│   │       └── meshes/
│   ├── launch/
│   │   └── *.launch.py                        # LaunchFile instances
│   └── config/
│       └── *.yaml                             # ROS 2 parameters
├── unity/
│   ├── Scenes/
│   │   └── *.unity                            # UnityScene instances
│   ├── Scripts/
│   │   └── *.cs                               # Unity C# scripts
│   ├── Prefabs/
│   │   └── *.prefab                           # Robot prefabs
│   └── Materials/
│       └── *.mat                              # PBR materials
├── urdf/
│   └── *.urdf                                 # URDFRobotModel instances
├── ros2_ws/
│   └── src/
│       ├── gazebo_sim/                        # ROS2Package instances
│       ├── unity_bridge/
│       └── sensor_processing/
└── reinforcement_learning/
    ├── environments/
    │   └── *.py                               # GymnasiumEnvironment implementations
    ├── training/
    │   └── train_*.py                         # Training scripts with TrainingConfig
    └── checkpoints/
        └── */                                 # Saved model checkpoints

docs/module-2/
├── chapter-1-gazebo-physics.md                # Chapter content (Markdown)
├── chapter-2-unity-rendering.md
├── chapter-3-sensor-simulation.md
├── chapter-4-integration.md
└── assets/
    ├── diagrams/
    └── screenshots/
```

---

## 10. Data Model Summary and Usage

### 10.1 Primary Entity Relationships

```
Module 2
├── Chapter (4)
│   ├── LearningObjectives (3-5 per chapter)
│   ├── Sections (multiple)
│   │   └── ContentBlocks (markdown, diagrams, callouts)
│   ├── CodeExamples (≥3 per chapter)
│   │   ├── CodeFiles (SDF, Python, C#, YAML)
│   │   ├── RunInstructions
│   │   └── ExpectedOutput
│   ├── Exercises
│   └── Assessments
├── Shared Resources
│   ├── URDFRobotModel (used across Gazebo, Unity, ROS 2)
│   ├── GazeboWorlds (Chapter 1, 3, 4)
│   ├── UnityScenes (Chapter 2, 3, 4)
│   ├── ROS2Packages (all chapters)
│   └── GymnasiumEnvironments (Chapter 4)
└── Dependencies
    ├── System packages (apt, pip)
    └── Unity packages
```

### 10.2 Key Contracts

1. **Chapter → CodeExample Contract**: Each chapter MUST have ≥3 runnable code examples (FR-003, FR-015, FR-024, FR-035)
2. **CodeExample → TestValidation Contract**: Each code example MUST pass validation tests (SC-001, SC-002, SC-003, SC-004)
3. **GazeboWorld → PhysicsConfig Contract**: RTF ≥ 0.9 for all physics simulations (NFR-001)
4. **UnityScene → PerformanceConfig Contract**: FPS ≥ 30 for all rendering scenes (NFR-002)
5. **URDF → Cross-Platform Contract**: URDF models MUST be compatible with both Gazebo and Unity (FR-041)

---

**Data Model Status**: ✅ **COMPLETE**
**Next Artifact**: `contracts/chapter-content-contract.md`
