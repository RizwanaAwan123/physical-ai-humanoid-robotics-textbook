# Chapter Content Contract

**Feature**: 002-module-2-digital-twin
**Created**: 2025-12-07
**Status**: Canonical
**Purpose**: Define mandatory inputs, outputs, validation rules, and quality gates for all chapter content generation

---

## 1. Contract Overview

This contract governs the interface between specialized subagents (GazeboPhysicsAgent, UnityRenderingAgent, SensorSimulationAgent, IntegrationAgent) and the orchestrating system. All chapter content MUST adhere to these specifications.

### 1.1 Contract Parties

- **Content Producer**: Specialized subagent responsible for generating chapter content
- **Content Consumer**: Docusaurus build system and end users (students)
- **Orchestrator**: Main planning agent coordinating subagent execution
- **Validator**: Quality gate system enforcing validation rules

---

## 2. Mandatory Inputs (Pre-Conditions)

Each subagent MUST receive the following inputs before generating chapter content:

### 2.1 Specification Inputs

```yaml
required_inputs:
  - spec_file: "specs/002-module-2-digital-twin/spec.md"
    sections:
      - User Scenarios (relevant priority)
      - Functional Requirements (chapter-specific FRs)
      - Non-Functional Requirements (all NFRs)
      - Success Criteria (chapter-specific SCs)

  - data_model: "specs/002-module-2-digital-twin/data-model.md"
    entities:
      - Chapter structure
      - CodeExample structure
      - Asset types (GazeboWorld, UnityScene, ROS2Package)

  - research_findings: "specs/002-module-2-digital-twin/research.md"
    decisions:
      - Technology stack choices
      - Platform versions
      - Integration approaches

  - constitution: ".specify/memory/constitution.md"
    principles:
      - Technical Accuracy & Depth (Principle I)
      - Modular Structure Integrity (Principle II)
      - AI-Native Development (Principle III)
      - Professional UI/UX Standards (Principle VI)
```

### 2.2 Context Inputs

```yaml
chapter_context:
  chapter_id: string              # e.g., "chapter-1-gazebo-physics"
  chapter_number: number          # 1, 2, 3, 4
  chapter_title: string           # Full chapter title from spec
  user_story_id: string           # P1, P2, P3, P4
  functional_requirements: string[] # ["FR-001", "FR-002", ...]
  success_criteria: string[]      # ["SC-001", "SC-002", ...]

dependencies:
  prior_chapters: string[]        # [] for Chapter 1, ["chapter-1"] for Chapter 2, etc.
  module_1_topics: string[]       # ["ROS 2 Fundamentals", "URDF Modeling", ...]
  shared_assets: string[]         # ["simple_humanoid.urdf"]
```

### 2.3 Template Inputs

```yaml
templates:
  - content_template: "templates/chapter-content.template.md"
  - gazebo_example_template: "templates/gazebo-example.template.md"  # For Chapters 1, 3, 4
  - unity_example_template: "templates/unity-example.template.md"    # For Chapters 2, 3, 4
  - test_script_template: "templates/test-script.template.sh"
```

---

## 3. Mandatory Outputs (Post-Conditions)

Each subagent MUST produce the following outputs:

### 3.1 Primary Content Output

```yaml
chapter_content:
  file_path: "docs/module-2/{chapter-filename}.md"
  format: "MDX (Docusaurus-compatible Markdown)"

  structure:
    frontmatter:
      - id: string
      - title: string
      - sidebar_position: number
      - tags: string[]
      - keywords: string[]

    sections:
      - introduction:
          required: true
          min_paragraphs: 2
          max_paragraphs: 4
          must_include:
            - Chapter learning objectives (3-5, matching spec FR-001/013/022/033)
            - Prerequisites from Module 1
            - Estimated completion time

      - learning_objectives:
          required: true
          format: "Numbered list"
          count: 3-5
          bloom_taxonomy: ["understand", "apply", "analyze", "create"]

      - main_content_sections:
          required: true
          count: 3-6
          per_section:
            - heading_level: "## (H2)"
            - min_words: 500
            - max_words: 1500
            - must_include:
              - Conceptual explanation
              - Code example reference (at least 1)
              - Diagrams or callouts (at least 1)

      - exercises:
          required: true
          count: 2-4
          per_exercise:
            - difficulty: "beginner | intermediate | advanced"
            - estimated_time: string
            - step_by_step_instructions: true
            - expected_outcome: string

      - summary:
          required: true
          min_paragraphs: 2
          max_paragraphs: 3
          must_include:
            - Key concepts recap
            - Connection to next chapter (if applicable)

      - additional_resources:
          required: true
          min_links: 3
          max_links: 8
          link_types: ["official_docs", "tutorials", "research_papers"]
```

### 3.2 Code Examples Output

```yaml
code_examples:
  location: "examples/{platform}/{chapter-id}/"
  minimum_count: 3  # Per FR-003, FR-015, FR-024, FR-035

  per_example:
    structure:
      - example_id: string
      - title: string
      - description: string
      - difficulty: "beginner | intermediate | advanced"
      - estimated_time: string
      - files: CodeFile[]  # As defined in data-model.md
      - run_instructions: RunInstructions
      - expected_output: ExpectedOutput
      - troubleshooting: TroubleshootingGuide[]

    file_organization:
      gazebo_examples:
        - worlds/*.world
        - launch/*.launch.py
        - config/*.yaml
        - README.md

      unity_examples:
        - Scenes/*.unity
        - Scripts/*.cs
        - Prefabs/*.prefab
        - Materials/*.mat
        - README.md

      ros2_examples:
        - package.xml
        - CMakeLists.txt (if ament_cmake)
        - setup.py (if ament_python)
        - {package_name}/*.py or src/*.cpp
        - launch/*.launch.py
        - README.md

      rl_examples:
        - environments/*.py
        - training/train_*.py
        - config/*.yaml
        - requirements.txt
        - README.md

    validation:
      - must_be_runnable: true
      - must_include_dependencies: true
      - must_include_verification_steps: true
      - must_pass_automated_tests: true  # See Section 4
```

### 3.3 Test Scripts Output

```yaml
test_scripts:
  location: "examples/{platform}/{chapter-id}/tests/"
  minimum_coverage: "100% of code examples"

  per_test_script:
    - file_name: "test_{example_id}.sh" or "test_{example_id}.py"
    - test_type: "smoke_test | integration_test | performance_test"
    - exit_codes:
        - 0: All tests passed
        - 1: Tests failed
        - 2: Environment setup error

    test_cases:
      smoke_tests:
        - file_existence_checks: "Verify all required files present"
        - syntax_validation: "Python/C#/XML syntax checks"
        - dependency_checks: "Verify ROS 2 packages, Unity packages installed"

      integration_tests:
        - gazebo_launch_test: "Launch Gazebo world, verify no errors, check RTF ≥ 0.9"
        - unity_scene_test: "Load Unity scene (headless mode), verify no errors"
        - ros2_topic_test: "Verify topics publishing at expected rates"

      performance_tests:
        - rtf_measurement: "Measure Gazebo RTF over 30s"
        - fps_measurement: "Measure Unity FPS over 30s (headless)"
        - latency_measurement: "Measure ROS 2 bridge latency (Unity ↔ ROS 2)"
```

### 3.4 Assets Output

```yaml
assets:
  diagrams:
    location: "docs/module-2/assets/diagrams/{chapter-id}/"
    formats: ["svg", "png"]
    minimum_count: 2
    types:
      - architecture_diagrams: "System components and connections"
      - flowcharts: "Process flows (e.g., physics engine pipeline)"
      - sequence_diagrams: "Interaction sequences (e.g., ROS 2 message flow)"

  screenshots:
    location: "docs/module-2/assets/screenshots/{chapter-id}/"
    format: "png"
    resolution: "1920x1080 or higher"
    minimum_count: 3
    must_show:
      - Example running successfully
      - Key UI elements (Gazebo GUI, Unity Scene view)
      - Expected output (visualizations, terminal logs)

  videos (optional):
    location: "docs/module-2/assets/videos/{chapter-id}/"
    format: "mp4"
    max_duration: "5 minutes"
    use_cases: ["Complex setup procedures", "Interactive demonstrations"]
```

---

## 4. Validation Rules (Quality Gates)

All outputs MUST pass these validation rules before acceptance:

### 4.1 Content Quality Gates

```yaml
quality_gates:
  - gate_id: "QG-01"
    name: "Learning Objectives Count"
    rule: "3 ≤ learning_objectives.length ≤ 5"
    severity: "error"
    reference: "FR-001, FR-013, FR-022, FR-033"

  - gate_id: "QG-02"
    name: "Code Examples Minimum"
    rule: "code_examples.length ≥ 3"
    severity: "error"
    reference: "FR-003, FR-015, FR-024, FR-035"

  - gate_id: "QG-03"
    name: "Code Examples Runnability"
    rule: "ALL code_examples MUST pass test scripts (exit code 0)"
    severity: "error"
    reference: "SC-001, SC-002, SC-003, SC-004"

  - gate_id: "QG-04"
    name: "Exercises Count"
    rule: "exercises.length ≥ 2"
    severity: "error"
    reference: "FR-004, FR-016, FR-025, FR-036"

  - gate_id: "QG-05"
    name: "Constitution Compliance"
    rule: "Content MUST cite sources, avoid jargon without explanation, use professional tone"
    severity: "error"
    reference: "Constitution Principle I, VI"

  - gate_id: "QG-06"
    name: "Markdown Validity"
    rule: "Content MUST be valid MDX (no syntax errors)"
    severity: "error"
    reference: "NFR-007"

  - gate_id: "QG-07"
    name: "Diagrams Presence"
    rule: "diagrams.length ≥ 2"
    severity: "warning"
    reference: "FR-002, FR-014, FR-023, FR-034"

  - gate_id: "QG-08"
    name: "Technical Accuracy"
    rule: "All code snippets, commands, and configurations MUST be technically accurate"
    severity: "error"
    reference: "Constitution Principle I"

  - gate_id: "QG-09"
    name: "Cross-References"
    rule: "Content MUST reference Module 1 prerequisites appropriately"
    severity: "error"
    reference: "FR-005, FR-017, FR-026, FR-037"

  - gate_id: "QG-10"
    name: "Accessibility"
    rule: "Images MUST have alt text, code blocks MUST have language labels"
    severity: "error"
    reference: "NFR-007, Constitution Principle VI"
```

### 4.2 Performance Quality Gates

```yaml
performance_gates:
  - gate_id: "PG-01"
    name: "Gazebo RTF Performance"
    rule: "RTF ≥ 0.9 for all Gazebo examples"
    measurement: "Average RTF over 30s of simulation"
    severity: "error"
    reference: "NFR-001"

  - gate_id: "PG-02"
    name: "Unity FPS Performance"
    rule: "FPS ≥ 30 for all Unity examples (headless mode)"
    measurement: "Average FPS over 30s of runtime"
    severity: "error"
    reference: "NFR-002"

  - gate_id: "PG-03"
    name: "ROS 2 Bridge Latency"
    rule: "Round-trip latency < 100ms (Unity ↔ ROS 2)"
    measurement: "Average latency over 100 message exchanges"
    severity: "error"
    reference: "NFR-005"

  - gate_id: "PG-04"
    name: "Example Load Time"
    rule: "Examples MUST load in < 30s on reference hardware"
    measurement: "Time from launch command to ready state"
    severity: "warning"
    reference: "NFR-008"
```

### 4.3 Platform-Specific Quality Gates

```yaml
gazebo_gates:
  - gate_id: "GZ-01"
    name: "SDF Syntax Validation"
    rule: "All .world and .sdf files MUST pass `gz sdf -k` validation"
    severity: "error"

  - gate_id: "GZ-02"
    name: "Gazebo Plugin Loading"
    rule: "All plugins MUST load without errors (check Gazebo server logs)"
    severity: "error"

  - gate_id: "GZ-03"
    name: "ROS 2 Topic Publishing"
    rule: "All expected ROS 2 topics MUST be active (verified via `ros2 topic list`)"
    severity: "error"

unity_gates:
  - gate_id: "UN-01"
    name: "Unity Scene Validity"
    rule: "All .unity scenes MUST open without errors in Unity 2022 LTS"
    severity: "error"

  - gate_id: "UN-02"
    name: "C# Script Compilation"
    rule: "All C# scripts MUST compile without errors"
    severity: "error"

  - gate_id: "UN-03"
    name: "URDF Import Success"
    rule: "URDF Importer MUST successfully import robot models without errors"
    severity: "error"

  - gate_id: "UN-04"
    name: "ROS-TCP-Connector Connection"
    rule: "Unity MUST successfully connect to ros_tcp_endpoint (verify handshake)"
    severity: "error"

ros2_gates:
  - gate_id: "ROS-01"
    name: "Package Build Success"
    rule: "All ROS 2 packages MUST build with `colcon build` (exit code 0)"
    severity: "error"

  - gate_id: "ROS-02"
    name: "Launch File Validation"
    rule: "All .launch.py files MUST execute without immediate errors"
    severity: "error"

  - gate_id: "ROS-03"
    name: "Custom Message Generation"
    rule: "All custom messages MUST generate Python/C++ interfaces successfully"
    severity: "error"

rl_gates:
  - gate_id: "RL-01"
    name: "Gymnasium Environment Registration"
    rule: "Custom environments MUST register with `gymnasium.make()` successfully"
    severity: "error"

  - gate_id: "RL-02"
    name: "SB3 Training Script Execution"
    rule: "Training scripts MUST run for at least 100 timesteps without errors"
    severity: "error"

  - gate_id: "RL-03"
    name: "Policy Checkpoint Saving"
    rule: "Trained policies MUST save and load successfully"
    severity: "error"
```

---

## 5. Acceptance Criteria

### 5.1 Chapter Completion Criteria

A chapter is considered **COMPLETE** when:

1. ✅ All mandatory outputs (Section 3) are present
2. ✅ All quality gates (Section 4.1) return PASS
3. ✅ All performance gates (Section 4.2) return PASS
4. ✅ All platform-specific gates (Section 4.3) return PASS for relevant platforms
5. ✅ Manual review confirms content matches spec requirements
6. ✅ Docusaurus build succeeds with zero warnings related to the chapter

### 5.2 Success Criteria Mapping

Each chapter MUST satisfy its Success Criteria from `spec.md`:

**Chapter 1 (Gazebo Physics)**:
- SC-001: All Gazebo examples load successfully (100% success rate)
- SC-005: Physics simulations achieve RTF ≥ 0.9
- SC-012: Technical accuracy verified (Gazebo architecture, physics engines, SDF syntax)

**Chapter 2 (Unity Rendering)**:
- SC-002: All Unity examples load successfully (100% success rate)
- SC-006: Rendering achieves FPS ≥ 30
- SC-013: Technical accuracy verified (Unity URP, URDF import, PBR materials)

**Chapter 3 (Sensor Simulation)**:
- SC-003: All sensor simulation examples load successfully (100% success rate)
- SC-007: Sensor data publishes at expected rates (LiDAR ≥10 Hz, IMU ≥100 Hz, depth camera ≥30 Hz)
- SC-014: Technical accuracy verified (sensor models, noise parameters, ROS 2 integration)

**Chapter 4 (Integration)**:
- SC-004: All integration examples load successfully (100% success rate)
- SC-008: ROS 2 bridge latency < 100ms
- SC-009: RL training completes 1000 timesteps without errors
- SC-015: Technical accuracy verified (ROS-TCP-Connector, Gymnasium environments, SB3 algorithms)

### 5.3 Regression Prevention

Once a chapter passes acceptance:
- ✅ All test scripts MUST be committed to version control
- ✅ CI/CD pipeline MUST run test scripts on every commit
- ✅ Any failing test MUST block merging to main branch

---

## 6. Interface Contracts Between Subagents

### 6.1 Shared Asset Handoff

**Contract**: When Chapter 1 (GazeboPhysicsAgent) creates a URDF model, Chapters 2, 3, 4 MUST reuse it.

```yaml
shared_asset_contract:
  producer: "GazeboPhysicsAgent (Chapter 1)"
  consumers: ["UnityRenderingAgent (Chapter 2)", "SensorSimulationAgent (Chapter 3)", "IntegrationAgent (Chapter 4)"]

  handoff_artifact:
    file: "examples/urdf/simple_humanoid.urdf"
    validation:
      - URDF syntax valid (check_urdf)
      - Gazebo-compatible (gz sdf -p)
      - Unity URDF Importer compatible (manual verification)

  consumer_obligations:
    - MUST NOT modify the canonical URDF file
    - MAY create chapter-specific variants (e.g., add sensors) in separate files
    - MUST reference original URDF in documentation
```

### 6.2 ROS 2 Package Handoff

**Contract**: When Chapter 1 creates `gazebo_sim` package, Chapters 3 and 4 MUST extend it.

```yaml
ros2_package_contract:
  producer: "GazeboPhysicsAgent (Chapter 1)"
  consumers: ["SensorSimulationAgent (Chapter 3)", "IntegrationAgent (Chapter 4)"]

  handoff_artifact:
    package_name: "gazebo_sim"
    location: "examples/ros2_ws/src/gazebo_sim/"
    initial_nodes: ["state_publisher", "joint_controller"]

  consumer_obligations:
    - MUST NOT break existing nodes
    - MAY add new nodes (e.g., "sensor_aggregator" in Chapter 3)
    - MUST update package.xml dependencies
    - MUST ensure backward compatibility with Chapter 1 examples
```

### 6.3 Unity Project Handoff

**Contract**: When Chapter 2 creates Unity project template, Chapters 3 and 4 MUST extend it.

```yaml
unity_project_contract:
  producer: "UnityRenderingAgent (Chapter 2)"
  consumers: ["SensorSimulationAgent (Chapter 3)", "IntegrationAgent (Chapter 4)"]

  handoff_artifact:
    project_name: "HumanoidDigitalTwin"
    location: "examples/unity/HumanoidDigitalTwin/"
    initial_scenes: ["URDFVisualization.unity", "PBRMaterialDemo.unity"]
    packages: ["URDF Importer", "URP"]

  consumer_obligations:
    - MUST NOT remove existing scenes
    - MAY add new scenes (e.g., "SensorVisualization.unity" in Chapter 3)
    - MUST maintain URP settings
    - MUST ensure Unity 2022 LTS compatibility
```

---

## 7. Error Handling and Rollback

### 7.1 Validation Failure Handling

When a quality gate fails:

1. **Severity: Error**
   - Subagent execution MUST halt immediately
   - Orchestrator logs failure details (gate ID, rule, affected files)
   - Subagent receives failure report and MUST fix before re-execution
   - No partial outputs are accepted

2. **Severity: Warning**
   - Subagent execution continues
   - Warning logged for manual review
   - Subagent receives warning report for optional improvement

### 7.2 Rollback Procedure

If a chapter fails acceptance after initial approval (regression):

1. Identify changed files via Git diff
2. Revert changes to last passing commit
3. Re-run all quality gates to confirm passing state
4. Log incident for post-mortem analysis

---

## 8. Versioning and Change Management

### 8.1 Contract Versioning

This contract follows semantic versioning:
- **Major version**: Breaking changes to inputs/outputs (requires subagent refactoring)
- **Minor version**: Additive changes (new optional fields, additional quality gates)
- **Patch version**: Clarifications, typo fixes

**Current Version**: 1.0.0

### 8.2 Contract Amendment Process

To amend this contract:

1. Propose change via GitHub issue (tag: `contract-amendment`)
2. Assess impact on existing subagents
3. Update contract document with version bump
4. Notify all subagents of changes
5. Provide migration guide if breaking change

---

## 9. Appendix: Reference Hardware Specification

All performance gates (Section 4.2) assume this reference hardware:

```yaml
reference_hardware:
  cpu: "Intel i5-8th gen or AMD Ryzen 5 3600 equivalent"
  ram: "16 GB DDR4"
  gpu: "NVIDIA GTX 1060 6GB or AMD RX 580 8GB"
  os: "Ubuntu 22.04 LTS"
  ros2_version: "Humble Hawksbill"
  gazebo_version: "Gazebo Classic 11.14.0"
  unity_version: "2022.3 LTS"
  python_version: "3.10+"
```

Performance on lower-spec hardware may degrade (acceptable if documented).

---

## 10. Summary and Compliance Checklist

Before submitting chapter content, subagents MUST verify:

- [ ] All mandatory inputs received (Section 2)
- [ ] All mandatory outputs produced (Section 3)
- [ ] All quality gates passed (Section 4.1)
- [ ] All performance gates passed (Section 4.2)
- [ ] All platform-specific gates passed (Section 4.3)
- [ ] All acceptance criteria met (Section 5)
- [ ] Shared assets properly handed off (Section 6)
- [ ] Test scripts committed and passing in CI/CD
- [ ] Documentation follows Docusaurus MDX standards
- [ ] No [TODO], [FIXME], or placeholder content remains

**Contract Status**: ✅ **CANONICAL** - Ready for subagent implementation

---

**Next Steps**:
1. Generate `quickstart.md` (development environment setup guide)
2. Re-check Constitution compliance after Phase 1 artifacts
3. Proceed to `/sp.tasks` for task generation
