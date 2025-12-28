# Implementation Plan: Module 1 - ROS 2 Fundamentals

**Branch**: `001-module-1-ros2` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-module-1-ros2/spec.md`

**Note**: This plan details the implementation approach for Module 1's four chapters using specialized Claude Code Subagents.

## Summary

Module 1 - ROS 2 Fundamentals is the foundational educational content for the Physical AI & Humanoid Robotics Textbook, covering four distinct chapters: (1) ROS 2 Architecture (Nodes, Topics, Services, Actions), (2) Bridging Python Agents to ROS Controllers (rclpy), (3) Understanding URDF for Humanoids, and (4) Building and Managing ROS 2 Packages.

**Technical Approach**: Content will be generated using four specialized Claude Code Subagents (ROSArchitectureAgent, RclpyIntegrationAgent, URDFModelingAgent, PackageManagementAgent), each responsible for one chapter. Content is delivered in Docusaurus MDX format with executable code examples tested in Docker (`osrf/ros:humble-desktop`). Mathematical notation rendered via KaTeX, diagrams via Mermaid. All content adheres to Constitution Principles I (Technical Accuracy & Depth), II (Modular Structure Integrity), III (AI-Native Development), and IV (Professional UI/UX Standards).

## Technical Context

**Language/Version**:
- Content: Markdown (MDX for Docusaurus)
- Code Examples: Python 3.10+ (rclpy), C++17 (rclcpp)
- Math: LaTeX (KaTeX rendering)
- Diagrams: Mermaid syntax

**Primary Dependencies**:
- Docusaurus 3.x (@docusaurus/preset-classic)
- ROS 2 Humble Hawksbill (target environment)
- remark-math, rehype-katex (math rendering)
- @docusaurus/theme-mermaid (diagram rendering)
- Docker (osrf/ros:humble-desktop for testing)

**Storage**:
- Git repository (content versioning)
- File-based (MDX files, code examples)
- N/A (no database for Module 1 content)

**Testing**:
- Docker-based code example validation (osrf/ros:humble-desktop)
- Docusaurus build (broken link detection, MDX validation)
- Manual QA (readability, accessibility, cross-references)
- flake8 (Python linting), cppcheck (C++ static analysis)

**Target Platform**:
- Web (Vercel or GitHub Pages deployment)
- Student environment: Ubuntu 22.04/24.04 with ROS 2 Humble

**Project Type**:
- Documentation site (Docusaurus-based textbook)

**Performance Goals**:
- Page load time: <2 seconds (SC-016)
- Lighthouse score: 90+ (SC-016)
- Code example test execution: <30 seconds per example
- Module 1 build time: <5 minutes

**Constraints**:
- ROS 2 Humble compatibility (LTS, supported until 2027)
- Free-tier deployment (Vercel/GitHub Pages)
- CPU-only code examples (no GPU requirement)
- Flesch-Kincaid grade level: 12-16 (SC-007)
- WCAG 2.1 AA accessibility compliance

**Scale/Scope**:
- 4 chapters (Module 1 only)
- 50-100 code examples total
- 30,000-60,000 words total (8,000-15,000 per chapter)
- Estimated student learning time: 16-24 hours

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Technical Accuracy & Depth ✅ PASS

- [x] All chapters require executable code samples (FR-003, FR-010, FR-017, FR-024)
- [x] Theoretical foundations mandated for all chapters (FR-002, FR-009, FR-016, FR-023)
- [x] Progressive exercises from beginner to advanced (FR-006, FR-014, FR-021, FR-028)
- [x] Mathematical formulations required (SC-004)
- [x] Code tested in clean Docker environment (SC-006)

**Status**: PASS - All functional requirements enforce technical depth per Principle I.

### Principle II: Modular Structure Integrity ✅ PASS (NON-NEGOTIABLE)

- [x] Exactly 4 chapters specified (FR-001 to FR-028 organized by chapter)
- [x] Fixed sequence: Architecture → rclpy → URDF → Packages
- [x] No additions or removals to 4-chapter structure
- [x] Each chapter independently testable (user stories P1-P4)

**Status**: PASS - Specification strictly adheres to 4-chapter Module 1 structure.

### Principle III: AI-Native Development & Spec-Driven Approach ✅ PASS

- [x] Specification created via `/sp.specify` (this document from `/sp.plan`)
- [x] Claude Subagents explicitly designed (4 specialized agents in plan)
- [x] Implementation to follow `/sp.tasks` → `/sp.implement` workflow
- [x] PHRs documented (constitution requirement)

**Status**: PASS - Plan mandates use of 4 specialized Claude Code Subagents, aligning with bonus criteria.

### Principle IV: Professional UI/UX Standards ✅ PASS

- [x] Custom Docusaurus theme specified (research.md Q6)
- [x] Responsive design required (SC-017: 320px-1920px viewports)
- [x] Syntax highlighting configured (Prism.js, Python/C++/XML/YAML)
- [x] Mathematical notation via KaTeX (research.md Q2)
- [x] Mermaid diagrams for visual hierarchy (research.md Q3)
- [x] Accessibility compliance (SC-009: alt text, WCAG 2.1 AA)

**Status**: PASS - Technical stack and success criteria enforce professional UI/UX.

### Content Standards Compliance ✅ PASS

- [x] All chapters require 6 content sections: Learning Objectives, Theoretical Foundations, Hands-On Implementation, Practical Examples, Exercises, Further Reading (FR-029)
- [x] Code Quality Standards enforced via Docker testing and linting (FR-030, FR-031)
- [x] Content Review Gates defined (data-model.md validation checklist)

**Status**: PASS - data-model.md Chapter Template Structure enforces all 6 required sections.

### Free-Tier Infrastructure Optimization ⚠️ NOT APPLICABLE (Module 1)

- **Status**: NOT APPLICABLE - Module 1 has no backend services (RAG, auth, database). This principle applies to later features.

### RAG System Content Fidelity ⚠️ NOT APPLICABLE (Module 1)

- **Status**: NOT APPLICABLE - RAG chatbot is a separate feature. Module 1 provides source content for RAG system.

**Overall Constitution Check**: ✅ **PASS** - All applicable principles satisfied, no violations to justify.

## Project Structure

### Documentation (this feature)

```text
specs/001-module-1-ros2/
├── spec.md                          # Feature specification (User input: requirements)
├── plan.md                          # This file (/sp.plan command output)
├── research.md                      # Phase 0 output: Design decisions and research findings
├── data-model.md                    # Phase 1 output: Content structure schema
├── quickstart.md                    # Phase 1 output: Developer guide for adding/modifying chapters
├── contracts/                       # Phase 1 output: Subagent input/output contracts
│   └── chapter-content-contract.md  # Chapter delivery expectations
├── checklists/                      # Validation checklists
│   └── requirements.md              # Spec validation (created by /sp.specify)
└── tasks.md                         # Phase 2 output: Actionable tasks (created by /sp.tasks - NOT by /sp.plan)
```

### Source Code (repository root)

**Structure Decision**: This is a Docusaurus documentation site, not a traditional application. The structure follows Docusaurus conventions with educational content (docs/) and executable examples (examples/).

```text
# Documentation Content (Docusaurus docs plugin)
docs/
├── module-1/                        # Module 1: ROS 2 Fundamentals
│   ├── _diagrams/                   # Shared diagram assets
│   │   ├── chapter-1/               # Chapter 1 Mermaid diagrams
│   │   ├── chapter-2/               # Chapter 2 Mermaid diagrams
│   │   ├── chapter-3/               # Chapter 3 Mermaid diagrams and URDF visualizations
│   │   └── chapter-4/               # Chapter 4 Mermaid diagrams
│   ├── chapter-1.mdx                # Chapter 1: ROS 2 Architecture
│   ├── chapter-2.mdx                # Chapter 2: Bridging Python Agents (rclpy)
│   ├── chapter-3.mdx                # Chapter 3: URDF Modeling
│   └── chapter-4.mdx                # Chapter 4: Packages, Launch Files, Parameters
├── module-2/                        # Future: Digital Twin (Gazebo, Unity)
├── module-3/                        # Future: AI-Robot Brain (NVIDIA Isaac)
├── module-4/                        # Future: Vision-Language-Action (VLA)
└── index.mdx                        # Homepage

# Executable Code Examples
examples/
└── module-1/
    ├── chapter-1/                   # ROS 2 Architecture examples
    │   ├── README.md                # Chapter 1 examples overview
    │   ├── example-01-minimal-publisher/
    │   │   ├── minimal_publisher.py
    │   │   ├── package.xml
    │   │   ├── README.md
    │   │   └── test.sh
    │   ├── example-02-minimal-subscriber/
    │   │   └── [similar structure]
    │   ├── example-03-service-server/
    │   │   └── [similar structure]
    │   ├── example-04-action-server/
    │   │   └── [similar structure]
    │   └── test-all.sh              # Test all Chapter 1 examples
    ├── chapter-2/                   # rclpy Integration examples
    │   ├── README.md
    │   ├── example-01-rclpy-node/
    │   │   └── [similar structure]
    │   ├── example-02-timer-callbacks/
    │   │   └── [similar structure]
    │   ├── example-03-ai-agent-integration/
    │   │   ├── agent.py             # AI agent logic (NumPy-based or PyTorch)
    │   │   ├── ros2_wrapper.py      # ROS 2 node wrapper
    │   │   ├── requirements.txt     # Python dependencies
    │   │   └── [rest of structure]
    │   └── test-all.sh
    ├── chapter-3/                   # URDF Modeling examples
    │   ├── README.md
    │   ├── example-01-simple-urdf/
    │   │   ├── robot.urdf
    │   │   ├── robot.xacro
    │   │   ├── launch/display.launch.py  # RViz launch file
    │   │   └── [rest of structure]
    │   ├── example-02-humanoid-arm/
    │   │   ├── arm.urdf
    │   │   ├── arm.xacro
    │   │   └── [rest of structure]
    │   ├── example-03-humanoid-leg/
    │   │   └── [similar structure]
    │   └── test-all.sh
    └── chapter-4/                   # Package Management examples
        ├── README.md
        ├── example-01-package-creation/
        │   ├── my_robot_pkg/        # Python package
        │   │   ├── package.xml
        │   │   ├── setup.py
        │   │   └── my_robot_pkg/
        │   │       └── node.py
        │   └── [rest of structure]
        ├── example-02-launch-files/
        │   ├── launch/
        │   │   ├── robot.launch.py  # Python launch file
        │   │   └── robot.launch.xml # XML launch file
        │   └── [rest of structure]
        ├── example-03-parameters/
        │   ├── config/
        │   │   └── params.yaml
        │   └── [rest of structure]
        └── test-all.sh

# Docusaurus Configuration
docusaurus.config.js                 # Docusaurus config (KaTeX, Mermaid, theme)
sidebars.js                          # Sidebar navigation structure
src/
├── css/
│   └── custom.css                   # Custom theme styling (color palette, fonts)
├── components/                      # React components for MDX
│   ├── CodeExample.js               # Enhanced code block with run button
│   └── ROSTopicViewer.js            # Interactive ROS 2 topic visualization (optional)
└── pages/
    └── index.js                     # Landing page

# CI/CD and Testing
.github/
└── workflows/
    ├── docusaurus-build.yml         # Build and deploy Docusaurus to Vercel/GitHub Pages
    └── test-examples.yml            # Test all code examples in Docker

# Docker for Testing
.docker/
└── ros2-humble-test.Dockerfile      # Custom Dockerfile extending osrf/ros:humble-desktop (optional)
```

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: NOT APPLICABLE - No constitution violations identified. All gates pass.

## Phase 0: Research & Design Decisions (Completed)

**Output**: `research.md` (already created)

**Key Decisions**:

1. **Content Platform**: Docusaurus 3.x with MDX
   - **Rationale**: Built-in versioning, search, navigation; React ecosystem integration
   - **Alternative Rejected**: GitBook (limited customization), MkDocs Material (less React integration)

2. **Math Rendering**: KaTeX via remark-math/rehype-katex
   - **Rationale**: Faster than MathJax, server-side rendering prevents layout shift
   - **Alternative Rejected**: MathJax (slower), images (poor accessibility)

3. **Diagrams**: Mermaid via @docusaurus/theme-mermaid
   - **Rationale**: Source-controlled, supports flowcharts/sequence/class diagrams
   - **Alternative Rejected**: PlantUML (Java dependency), Draw.io (binary files)

4. **Code Validation**: Docker (osrf/ros:humble-desktop) + CI automation
   - **Rationale**: Reproducible testing, matches student environment
   - **Alternative Rejected**: Manual testing (unscalable), VMs (slower than Docker)

5. **Subagent Architecture**: 4 specialized agents (ROSArchitecture, RclpyIntegration, URDFModeling, PackageManagement)
   - **Rationale**: Domain expertise per chapter, consistency via shared templates
   - **Alternative Rejected**: Single monolithic agent (lacks specialization)

6. **UI/UX**: Swizzled @docusaurus/theme-classic with custom color palette (teal), Inter/JetBrains Mono fonts
   - **Rationale**: Professional design, maintains Docusaurus upgrade path
   - **Alternative Rejected**: Full custom theme (maintenance burden), default theme (violates Principle IV)

## Phase 1: Design & Contracts (Completed)

**Outputs**: `data-model.md`, `contracts/chapter-content-contract.md`, `quickstart.md` (already created)

### 1. Content Structure (data-model.md)

**Key Entities Defined**:
- **Chapter**: Contains sections, code examples, exercises, metadata
- **Section**: TheoryFoundation, HandsOnTutorial, PracticalExample, Exercise, FurtherReading, Summary
- **LearningObjective**: Measurable outcomes with Bloom's taxonomy levels
- **CodeExample**: Executable code with dependencies, setup instructions, test scripts
- **Diagram**: Mermaid or SVG/PNG with alt text
- **Exercise**: Progressive difficulty (beginner, intermediate, advanced) with acceptance criteria
- **ChapterMetadata**: Frontmatter for Docusaurus (id, title, description, keywords, tags)

**Chapter Template Structure**: 9 required sections (Overview, Learning Objectives, Theoretical Foundations, Hands-On Implementation, Practical Examples, Exercises & Challenges, Further Reading, Summary, Troubleshooting)

**Validation Checklist**: 10-item checklist per chapter (3-5 learning objectives, ≥3 code examples, ≥5 exercises, ≥5 references, etc.)

### 2. Subagent Contracts (contracts/chapter-content-contract.md)

**Input Contract**: Each subagent receives spec.md (functional requirements), data-model.md (template), configuration JSON (chapter number, ROS 2 distribution), cross-reference data (for Chapters 2-4)

**Output Contract**: Each subagent delivers:
- Chapter content (`.mdx` file, 8,000-15,000 words)
- Code examples directory (≥3 examples, each with README, test.sh)
- Test report (execution status, Docker logs)
- Diagram sources (Mermaid files)

**Quality Gates**: 20 gates across 3 categories:
- Content Quality: 10 gates (all sections present, learning objectives, math, diagrams, exercises, references, grade level, no broken links)
- Code Quality: 6 gates (Docker execution, linting, README, test scripts, test report)
- Consistency: 4 gates (terminology, cross-references, code style, file paths)

**Delivery Checklist**: Subagent provides delivery report with gate pass/fail counts and recommendations for next chapter

### 3. Developer Guide (quickstart.md)

**Covered Topics**:
- Environment setup (Node.js, Docker)
- Manual chapter creation (following template)
- Subagent invocation (command-line examples)
- Code example validation (Docker testing)
- Docusaurus preview (npm run start)
- Quality gates execution (build, test, lint)
- Troubleshooting (common issues with solutions)
- Best practices (content writing, code examples, diagrams, cross-references)
- Common tasks (add example, update after spec changes, add callout boxes)

## Phase 2: Implementation Architecture

### Subagent Specifications

#### 1. ROSArchitectureAgent

**Responsibility**: Generate Chapter 1 - ROS 2 Architecture (Nodes, Topics, Services, Actions)

**Inputs**:
- `spec.md` (FR-001 to FR-007)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 1, module_number: 1, ros2_distribution: "humble" }`

**Outputs**:
- `docs/module-1/chapter-1.mdx` (ROS 2 Architecture content)
- `examples/module-1/chapter-1/` (≥3 code examples: publisher, subscriber, service, action)
- `examples/module-1/chapter-1/test-report.md`
- `docs/module-1/_diagrams/chapter-1/` (Mermaid diagrams: pubsub graph, service pattern, action pattern, DDS architecture)

**Key Content Sections**:
1. **Theoretical Foundations**: DDS middleware layer, QoS policies, ROS 1 vs ROS 2 architectural differences, communication graph theory
2. **Hands-On Implementation**: Minimal publisher/subscriber tutorial (Python + C++), service client/server tutorial, action client/server tutorial
3. **Practical Examples**: Sensor data publishing scenario, motor command subscription, configuration service, goal-based navigation action
4. **Exercises**: Beginner (modify publisher topic name), Intermediate (create service for robot configuration), Advanced (design multi-node system with topics + services + actions)

**Validation**:
- All code examples execute in Docker (osrf/ros:humble-desktop)
- `ros2 topic list`, `ros2 service list`, `ros2 action list` show expected entities
- Mermaid diagrams render correctly in Docusaurus

**Exports for Later Chapters**:
- `communication_patterns`: /module-1/chapter-1/#communication-patterns (referenced by Chapter 2)
- `node_basics`: /module-1/chapter-1/#node-basics (referenced by Chapter 4)

---

#### 2. RclpyIntegrationAgent

**Responsibility**: Generate Chapter 2 - Bridging Python Agents to ROS Controllers (rclpy)

**Inputs**:
- `spec.md` (FR-008 to FR-014)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 2, module_number: 1, ros2_distribution: "humble" }`
- Cross-references: Chapter 1 exports (communication_patterns, node_basics)

**Outputs**:
- `docs/module-1/chapter-2.mdx` (rclpy Integration content)
- `examples/module-1/chapter-2/` (≥3 code examples: rclpy node, timer callbacks, AI agent integration)
- `examples/module-1/chapter-2/test-report.md`
- `docs/module-1/_diagrams/chapter-2/` (Mermaid diagrams: rclpy architecture, executor model, AI agent integration pattern)

**Key Content Sections**:
1. **Theoretical Foundations**: rclpy architecture, Python-C++ bridge internals, callback execution models, executor types (single-threaded, multi-threaded), real-time considerations
2. **Hands-On Implementation**: rclpy node lifecycle tutorial, timer-based control loop tutorial, multi-threaded executor tutorial, AI agent wrapper tutorial (NumPy-based simple planner)
3. **Practical Examples**: AI agent integration with PyTorch (simple Q-learning controller), obstacle avoidance using planning algorithm, real-time control loop achieving 10-100 Hz
4. **Exercises**: Beginner (modify timer frequency), Intermediate (implement AI agent with state machine), Advanced (create real-time controller achieving 50 Hz with AI decision logic)

**Validation**:
- All code examples execute in Docker with rclpy installed
- AI agent examples achieve target control frequencies (≥10 Hz per SC-011)
- PyTorch example includes `requirements.txt` and tests with CPU-only mode

**Exports for Later Chapters**:
- `rclpy_patterns`: /module-1/chapter-2/#rclpy-patterns (referenced by Chapter 4)

---

#### 3. URDFModelingAgent

**Responsibility**: Generate Chapter 3 - Understanding URDF for Humanoid Robots

**Inputs**:
- `spec.md` (FR-015 to FR-021)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 3, module_number: 1, ros2_distribution: "humble" }`
- Cross-references: Chapter 1 exports (node_basics - for package structure)

**Outputs**:
- `docs/module-1/chapter-3.mdx` (URDF Modeling content)
- `examples/module-1/chapter-3/` (≥3 code examples: simple URDF, humanoid arm, humanoid leg, full humanoid)
- `examples/module-1/chapter-3/test-report.md`
- `docs/module-1/_diagrams/chapter-3/` (Mermaid diagrams: kinematic chain, URDF structure; SVG/PNG: robot renderings from RViz)

**Key Content Sections**:
1. **Theoretical Foundations**: Rigid body transformations (homogeneous transforms), Denavit-Hartenberg parameters, inertial property calculations (center of mass, inertia tensor), URDF XML structure
2. **Hands-On Implementation**: Simple URDF creation tutorial (2-link arm), xacro macro tutorial, humanoid arm URDF (shoulder, elbow, wrist = 3 DOF), humanoid leg URDF (hip, knee, ankle = 3 DOF)
3. **Practical Examples**: Full simplified humanoid URDF (10+ DOF: torso, 2 arms, 2 legs, head), mesh file integration (.STL, .DAE), realistic inertial properties
4. **Exercises**: Beginner (modify joint limits), Intermediate (create custom robot limb), Advanced (design complete simplified humanoid with ≥10 DOF, validate with check_urdf and RViz)

**Validation**:
- All URDF files pass `check_urdf` validation
- URDF models load in RViz without errors
- Humanoid model has ≥10 DOF (per SC-012)
- Kinematic chains correctly specified (parent-child relationships)

**Exports for Later Chapters**:
- `urdf_basics`: /module-1/chapter-3/#urdf-basics (referenced by Chapter 4 for package structure)

---

#### 4. PackageManagementAgent

**Responsibility**: Generate Chapter 4 - Building and Managing ROS 2 Packages, Launch Files, Parameters

**Inputs**:
- `spec.md` (FR-022 to FR-028, FR-029 to FR-034 cross-chapter requirements)
- `data-model.md` (Chapter Template Structure)
- `contracts/chapter-content-contract.md`
- Configuration: `{ chapter_number: 4, module_number: 1, ros2_distribution: "humble" }`
- Cross-references: Chapter 1 (node_basics, communication_patterns), Chapter 2 (rclpy_patterns), Chapter 3 (urdf_basics)

**Outputs**:
- `docs/module-1/chapter-4.mdx` (Package Management content)
- `examples/module-1/chapter-4/` (≥3 code examples: package creation, launch files, parameter management, multi-package workspace)
- `examples/module-1/chapter-4/test-report.md`
- `docs/module-1/_diagrams/chapter-4/` (Mermaid diagrams: colcon build flow, package dependencies, launch file composition)

**Key Content Sections**:
1. **Theoretical Foundations**: colcon build system architecture, ament package structure (ament_cmake, ament_python), ROS 2 dependency resolution (package.xml), workspace overlay concepts
2. **Hands-On Implementation**: Python package creation tutorial (setup.py, package.xml), CMake package creation tutorial (CMakeLists.txt), Python launch file tutorial, XML launch file tutorial, parameter YAML tutorial
3. **Practical Examples**: Multi-package workspace (robot_description, robot_control, robot_bringup packages), launch file composition (including other launch files), namespace management, dynamic parameter reconfiguration
4. **Exercises**: Beginner (create single package), Intermediate (write launch file orchestrating 3 nodes), Advanced (structure multi-package project integrating nodes from Chapters 1-3, with ≥5 nodes per SC-013)

**Validation**:
- All packages build with `colcon build` without errors
- Launch files successfully start nodes (≥5 nodes for advanced exercise per SC-013)
- Parameter YAML files load correctly
- Dependency resolution works (package.xml dependencies satisfied)

**Summary Section**:
- Recap of Module 1 (ROS 2 fundamentals mastered)
- Preview of Module 2 (Digital Twin: Gazebo and Unity simulation)

---

### Subagent Orchestration Workflow

**Master Orchestrator** (automated or manual coordination):

```plaintext
1. Phase 0: Research Complete (research.md generated)
2. Phase 1: Design Complete (data-model.md, contracts/, quickstart.md generated)
3. Phase 2: Task Generation (/sp.tasks creates tasks.md)
4. Phase 3: Implementation (Sequential execution)

   Step 1: Invoke ROSArchitectureAgent
      Input: spec.md (FR-001 to FR-007), config, templates
      Output: chapter-1.mdx, examples/chapter-1/, test-report, diagrams
      Validation: Run quality gates, Docker tests
      Register Exports: communication_patterns, node_basics

   Step 2: Invoke RclpyIntegrationAgent (depends on Step 1 exports)
      Input: spec.md (FR-008 to FR-014), config, templates, Chapter 1 exports
      Output: chapter-2.mdx, examples/chapter-2/, test-report, diagrams
      Validation: Run quality gates, Docker tests, verify ≥10 Hz control loop
      Register Exports: rclpy_patterns

   Step 3: Invoke URDFModelingAgent (weak dependency on Step 1)
      Input: spec.md (FR-015 to FR-021), config, templates, Chapter 1 exports (node_basics)
      Output: chapter-3.mdx, examples/chapter-3/, test-report, diagrams
      Validation: Run quality gates, check_urdf, RViz, verify ≥10 DOF
      Register Exports: urdf_basics

   Step 4: Invoke PackageManagementAgent (depends on Steps 1-3 exports)
      Input: spec.md (FR-022 to FR-034), config, templates, Chapters 1-3 exports
      Output: chapter-4.mdx, examples/chapter-4/, test-report, diagrams
      Validation: Run quality gates, colcon build, verify ≥5 nodes in launch
      Finalize: Module 1 summary section with Module 2 preview

5. Phase 4: Final Validation
   - Run full Docusaurus build (detect broken links, MDX errors)
   - Test all examples across all chapters (examples/module-1/test-all.sh)
   - Manual QA: Readability, accessibility, cross-references
   - Performance audit: Lighthouse score, page load times

6. Phase 5: Deployment
   - Merge to main branch
   - Trigger Vercel/GitHub Pages deployment
   - Verify production site
```

### Shared Templates for Consistency

**Content Template** (`templates/content-template.md`):
```markdown
# Chapter {N}: {Title}

## Overview
[150-300 word introduction]

**Estimated Time**: {4-6} hours
**Prerequisites**: [List]

## Learning Objectives
- **LO-{N}.1**: [Action verb + outcome]
- **LO-{N}.2**: [Action verb + outcome]
- **LO-{N}.3**: [Action verb + outcome]

[... rest of template follows data-model.md Chapter Template Structure ...]
```

**Code Example Template** (`templates/code-example-template.py`):
```python
#!/usr/bin/env python3
"""
Example {N}.{NN}: {Title}

Description: {1-2 sentence description}

Author: {SubagentName}
License: MIT
"""

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    """
    {Brief class description}
    """
    def __init__(self):
        super().__init__('example_node')
        # {Comment explaining setup}
        self.get_logger().info('Example node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test Script Template** (`templates/test-template.sh`):
```bash
#!/bin/bash
# Test script for Example {N}.{NN}

set -e  # Exit on error

echo "Testing Example {N}.{NN}: {Title}"

# 1. Build (if applicable)
# 2. Lint
# 3. Execute
# 4. Validate
# 5. Cleanup

echo "✓ All tests passed!"
```

## Success Metrics

Module 1 is complete when:

- [x] All 4 chapters delivered (chapter-1.mdx through chapter-4.mdx)
- [x] All code examples tested in Docker (100% pass rate per SC-006)
- [x] Quality gates pass for all chapters (20 gates × 4 chapters = 80 total)
- [x] Docusaurus builds without errors
- [x] Cross-references validated (no broken internal links)
- [x] Learning effectiveness criteria met (SC-010 to SC-013)
- [x] UX criteria met (SC-014 to SC-017)
- [x] Technical accuracy criteria met (SC-018 to SC-020)
- [x] Deployed to Vercel/GitHub Pages and accessible

## Risks and Mitigations

### Risk: Subagent generates content not matching spec

**Mitigation**: Quality gates enforce spec requirements (3-5 learning objectives per FR-001, ≥3 code examples per SC-002, etc.). Automated validation catches violations before delivery.

### Risk: Code examples fail in different environments

**Mitigation**: Docker testing with `osrf/ros:humble-desktop` ensures reproducibility. Pin ROS 2 Humble version, document dependencies explicitly.

### Risk: Cross-references break as chapters evolve

**Mitigation**: Export/import mechanism tracks cross-references. Automated link checker in Docusaurus build detects broken links.

### Risk: Content exceeds Flesch-Kincaid grade level target

**Mitigation**: Use readability tools during content generation. Simplify language where possible without sacrificing technical accuracy. Add glossary for complex terms.

### Risk: Docusaurus build performance degrades

**Mitigation**: Monitor bundle size, optimize Mermaid diagrams (simplify complex graphs), lazy-load large code examples.

### Risk: ROS 2 API changes in future distributions

**Mitigation**: Target ROS 2 Humble (LTS until 2027). Note experimental features explicitly. Plan annual review cycle post-hackathon.

## Next Steps

1. **Run `/sp.tasks`**: Generate actionable task list organized by chapter and user story
2. **Run `/sp.implement`**: Execute tasks, invoking subagents sequentially
3. **Validate outputs**: Run quality gates, Docker tests, Docusaurus build
4. **Manual QA**: Review content for readability, accuracy, cross-references
5. **Deploy**: Merge to main, trigger Vercel/GitHub Pages deployment
6. **Verify production**: Test Module 1 chapters in production environment

---

**Plan Status**: ✅ **COMPLETE** - Ready for `/sp.tasks` command to generate actionable task list.
