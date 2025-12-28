# Quickstart Guide: Module 3 Implementation

**Feature**: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
**Created**: 2025-12-09
**Branch**: `001-isaac-ai-brain`

## Purpose

This quickstart provides instructions for **specialized subagents** (IsaacSimAgent, PerceptionAgent, RLTrainingAgent, Nav2IntegrationAgent) and **human developers** to implement Module 3 content following the task breakdown in `tasks.md`.

---

## For Specialized Subagents

### Prerequisites

Before starting your assigned chapter, ensure you have loaded:

1. **Feature Context**:
   - Read `specs/001-isaac-ai-brain/spec.md` for requirements
   - Read `specs/001-isaac-ai-brain/plan.md` for architecture
   - Read `specs/001-isaac-ai-brain/tasks.md` for your assigned user story tasks
   - Read `specs/001-isaac-ai-brain/research.md` for technology decisions

2. **Design Artifacts** (Phase 2):
   - Read `specs/001-isaac-ai-brain/data-model.md` for chapter structure and example models
   - Read `specs/001-isaac-ai-brain/contracts/chapter-template.md` for MDX template
   - Read relevant JSON schemas in `contracts/` directory for code example validation

### Agent Assignments

| Agent | Chapter | User Story | Tasks | Files to Generate |
|-------|---------|------------|-------|-------------------|
| **IsaacSimAgent** | Chapter 1: Isaac Sim Setup | US1 (P1) | T014-T041 (42 tasks) | 7 MDX files, 10+ examples, diagrams, screenshots |
| **PerceptionAgent** | Chapter 2: Isaac ROS Perception | US2 (P2) | T042-T070 (43 tasks) | 8 MDX files, 10+ examples, diagrams, screenshots |
| **RLTrainingAgent** | Chapter 3: RL Training | US3 (P3) | T071-T101 (47 tasks) | 9 MDX files, 10+ examples, pre-trained checkpoints |
| **Nav2IntegrationAgent** | Chapter 4: Nav2 Integration | US4 (P4) | T102-T132 (42 tasks) | 9 MDX files, 10+ examples, diagrams, screenshots |

### Implementation Workflow

For each task in your assigned user story (USXN):

1. **Load Context**:
   - Identify task type: MDX content, code example, diagram, validation
   - Review chapter-template.md placeholders
   - Review data-model.md requirements for this task type

2. **Generate MDX Content** (for T014-T021, T042-T050, T071-T080, T102-T111):
   - Follow chapter-template.md 9-section structure
   - Replace ALL placeholders ({{PLACEHOLDER}}) with actual content
   - Include LaTeX equations using `$$..$$` for theory sections
   - Include Mermaid diagrams for architectures
   - Add screenshots using relative paths: `![Description](../../static/img/module-3/{{CHAPTER_SLUG}}/{{FILE}}.png)`
   - Ensure code blocks specify language for syntax highlighting

3. **Create Code Examples** (for T022-T031, T051-T060, T081-T090, T112-T121):
   - Follow relevant JSON schema (isaac-sim-example-schema.json, isaac-ros-example-schema.json, etc.)
   - Create directory: `examples/module-3/{{CHAPTER_SLUG}}/{{EXAMPLE_ID}}/`
   - Write main script files (Python, launch, YAML as needed)
   - Write comprehensive README.md following schema structure
   - **CRITICAL**: Examples must be syntactically valid and executable (or clearly documented as pseudo-code)
   - Include comments explaining key concepts

4. **Generate Diagrams** (for T032, T061, T091, T122-T123):
   - Use Mermaid syntax for architecture diagrams
   - Embed directly in MDX files using triple-backtick mermaid blocks
   - Ensure diagrams are readable and accurate

5. **Document Troubleshooting** (for T033, T062, T092, T124):
   - Include 5+ common issues with Symptom → Cause → Solution format
   - Cover hardware, software, and usage errors
   - Provide specific parameter tuning guidance

6. **Validation** (for T037-T041, T066-T070, T097-T101, T127-T132):
   - Verify all MDX files follow template structure
   - Check all code examples match JSON schemas
   - Ensure internal links are functional
   - Validate against functional requirements (FR-XXX) and success criteria (SC-XXX)
   - **NOTE**: If you cannot actually execute examples (no GPU access), document clearly in README that examples are designed for RTX 2060+ GPUs and mark as "Validated syntactically, execution validation required on target hardware"

7. **Report Completion**:
   - List all files created with absolute paths
   - Identify any blockers or dependencies
   - Confirm alignment with spec.md requirements

### Parallel Execution Strategy

Tasks marked `[P]` are parallelizable within your chapter. Group them for efficient generation:

- **MDX Content** (T014-T021): All 7 files can be generated in parallel
- **Code Examples** (T022-T031): All 10 examples can be generated in parallel
- **Diagrams/Screenshots** (T032-T034): Can be generated in parallel

**Sequential Dependencies**:
- Validation tasks (T037-T041) MUST run AFTER content/example generation completes
- MDX content should reference specific example IDs, so examples should be designed before MDX is finalized (or update MDX after examples are created)

---

## For Human Developers

### Setup

1. **Verify Prerequisites**:
   ```bash
   # Check Docusaurus installation
   npm list @docusaurus/core

   # Verify directory structure
   ls docs/textbook/module-3-ai-robot-brain/
   ls examples/module-3/
   ls static/img/module-3/
   ```

2. **Pull Latest from Feature Branch**:
   ```bash
   git checkout 001-isaac-ai-brain
   git pull origin 001-isaac-ai-brain
   ```

### Review Checklist

After subagents complete their chapters, perform manual QA:

- [ ] **Content Completeness**:
  - [ ] All 7-9 MDX files per chapter exist
  - [ ] All 10+ code examples per chapter exist
  - [ ] All placeholders {{...}} replaced with actual content
  - [ ] All sections from chapter-template.md present

- [ ] **Technical Accuracy**:
  - [ ] Code examples are syntactically valid (run linters)
  - [ ] YAML/JSON configuration files parse correctly
  - [ ] Commands and paths are correct for Linux/Ubuntu
  - [ ] Version numbers match research.md decisions (Isaac Sim 2023.1, ROS 2 Humble, etc.)

- [ ] **Links and References**:
  - [ ] Internal links to other chapters functional
  - [ ] External links to NVIDIA docs, ROS 2 docs accessible
  - [ ] Image paths correct and files exist in static/img/module-3/
  - [ ] Example file paths match actual repository structure

- [ ] **Accessibility**:
  - [ ] All images have alt text
  - [ ] Headings follow hierarchy (h1 → h2 → h3)
  - [ ] Code blocks have language specified

### GPU Testing Procedures

**Hardware**: NVIDIA RTX 3080 (or RTX 2060 minimum)

1. **Isaac Sim Examples (Chapter 1)**:
   ```bash
   cd examples/module-3/chapter-1-isaac-sim/01-hello-isaac-sim/
   python hello_isaac_sim.py
   # Verify: RTF ≥1.0, no errors
   ```

2. **Isaac ROS Examples (Chapter 2)**:
   ```bash
   cd examples/module-3/chapter-2-isaac-ros/01-stereo-depth/
   ros2 launch stereo_depth.launch.py
   # Verify: Stereo depth >30 FPS, RViz visualizes point cloud
   ```

3. **RL Training Examples (Chapter 3)**:
   ```bash
   cd examples/module-3/chapter-3-rl-training/02-humanoid-walk/
   python train_humanoid_walk.py
   # Run for 100 episodes (quick test), verify training starts without CUDA errors
   ```

4. **Nav2 Examples (Chapter 4)**:
   ```bash
   cd examples/module-3/chapter-4-nav2/01-nav2-basic/
   ros2 launch nav2_bringup.launch.py
   # Verify: Costmap publishes, robot navigates to RViz 2D Nav Goal
   ```

### Docusaurus Build

```bash
# Development build
npm run start

# Production build
npm run build

# Check for errors
npm run build 2>&1 | tee build.log
grep -i "error\|warning" build.log
```

### Manual QA Workflow

1. **Navigate to Module 3 landing page**: `/docs/module-3-ai-robot-brain`
2. **For each chapter (1-4)**:
   - Click through all 7-9 MDX sections
   - Test internal links
   - Verify diagrams render
   - Check code syntax highlighting
   - Verify images load
   - Test responsive design (mobile, tablet, desktop)

3. **Test examples**:
   - Clone repository fresh
   - Follow installation instructions in example READMEs
   - Execute examples on GPU hardware
   - Verify expected outputs

### Staging Deployment

```bash
# Deploy to Vercel staging
vercel --prod

# Or GitHub Pages
npm run build
gh-pages -d build
```

### Feedback Collection

Invite stakeholders to review:
- Content accuracy (subject matter experts in robotics, NVIDIA Isaac)
- Clarity and completeness (educators, students)
- Technical correctness (run examples, check for errors)

Document feedback in GitHub issues on feature branch.

---

## Success Criteria

### Phase 3 (US1) - Chapter 1 Complete:
- [ ] 7 MDX files in `docs/textbook/module-3-ai-robot-brain/chapter-1-isaac-sim-setup/`
- [ ] 10+ code examples in `examples/module-3/chapter-1-isaac-sim/`
- [ ] All examples run on RTX 2060+ with RTF ≥1.0
- [ ] 15+ screenshots in `static/img/module-3/isaac-sim-screenshots/`
- [ ] Independent testability verified (reviewer can complete chapter end-to-end)

### Phase 4 (US2) - Chapter 2 Complete:
- [ ] 8 MDX files in `docs/textbook/module-3-ai-robot-brain/chapter-2-isaac-ros-perception/`
- [ ] 10+ code examples in `examples/module-3/chapter-2-isaac-ros/`
- [ ] Perception performance targets met: >30 FPS stereo depth, <50ms VSLAM, >30 FPS DNN inference
- [ ] 20+ screenshots in `static/img/module-3/isaac-ros-rviz/`

### Phase 5 (US3) - Chapter 3 Complete:
- [ ] 9 MDX files in `docs/textbook/module-3-ai-robot-brain/chapter-3-rl-training/`
- [ ] 10+ code examples in `examples/module-3/chapter-3-rl-training/`
- [ ] Pre-trained checkpoints for 5-10 tasks (Git LFS tracked)
- [ ] RL training achieves >70% success within 4 hours on RTX 3080
- [ ] 15+ screenshots/plots in `static/img/module-3/rl-training-plots/`

### Phase 6 (US4) - Chapter 4 Complete:
- [ ] 9 MDX files in `docs/textbook/module-3-ai-robot-brain/chapter-4-nav2-integration/`
- [ ] 10+ code examples in `examples/module-3/chapter-4-nav2/`
- [ ] Navigation success >80% over 10 trials in Isaac Sim
- [ ] 20+ screenshots in `static/img/module-3/nav2-visualizations/`

### Phase 7 (Polish) - Module 3 Complete:
- [ ] Module 3 landing page created
- [ ] Cross-references validated across all 4 chapters
- [ ] 7 ADRs created for architectural decisions
- [ ] Docusaurus build succeeds with no errors
- [ ] Staging deployment accessible and reviewed

---

## Troubleshooting

### Subagent Issues

**Problem**: Subagent doesn't know specific NVIDIA Isaac API details

**Solution**: Subagents should use research.md best practices and reference official NVIDIA documentation. Pseudo-code is acceptable if marked clearly, with comments indicating "# TODO: Verify with Isaac Sim 2023.1 API documentation"

**Problem**: Subagent cannot generate screenshots (no GUI access)

**Solution**: Create placeholder images with descriptive filenames (e.g., `isaac-sim-urdf-import-step-3.png`) and document in task output that screenshots need to be captured by human developer with GPU access

**Problem**: Subagent cannot test RL training (no 2048 parallel envs available)

**Solution**: Provide training scripts with configurable `n_parallel_envs` parameter. Document recommended 2048 for RTX 3080, but script should work with 128 envs for testing

### Human Developer Issues

**Problem**: Examples fail to run due to missing dependencies

**Solution**: Ensure `requirements.txt` or `package.xml` files are complete. Check Isaac Sim, Isaac ROS, Nav2 versions match research.md specifications

**Problem**: Docusaurus build fails with MDX syntax errors

**Solution**: Run `npm run build` and check error log. Common issues:
- Unclosed JSX tags
- Missing front matter in MDX files
- Invalid Mermaid diagram syntax
- Broken image paths

**Problem**: Git LFS checkpoints not downloading

**Solution**:
```bash
git lfs install
git lfs pull
```

---

## Contact & Support

- **Feature Lead**: Specify Project Manager (AI Agent)
- **Technical SME**: NVIDIA Isaac Subject Matter Experts
- **Repository Issues**: https://github.com/{{ORG}}/{{REPO}}/issues

---

**Quickstart Version**: 1.0
**Last Updated**: 2025-12-09
