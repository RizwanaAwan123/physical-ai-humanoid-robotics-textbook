# Quickstart: Developing Module 1 Content

**Audience**: Developers extending or modifying Module 1 - ROS 2 Fundamentals
**Date**: 2025-12-06

## Overview

This guide explains how to:
1. Set up the development environment for Module 1
2. Use Claude Code Subagents to generate chapter content
3. Validate code examples using Docker
4. Deploy to Docusaurus for preview

## Prerequisites

- Git (for version control)
- Node.js 18+ and npm (for Docusaurus)
- Docker (for testing ROS 2 code examples)
- Claude Code CLI (for subagent orchestration)

## Directory Structure

```
hackathon1-humanoid-robotics-book/
├── docs/                          # Docusaurus content
│   └── module-1/                  # Module 1 chapters
│       ├── _diagrams/             # Shared diagrams
│       │   ├── chapter-1/
│       │   ├── chapter-2/
│       │   ├── chapter-3/
│       │   └── chapter-4/
│       ├── chapter-1.mdx          # Chapter 1 content
│       ├── chapter-2.mdx          # Chapter 2 content
│       ├── chapter-3.mdx          # Chapter 3 content
│       └── chapter-4.mdx          # Chapter 4 content
├── examples/                      # Executable code examples
│   └── module-1/
│       ├── chapter-1/
│       ├── chapter-2/
│       ├── chapter-3/
│       └── chapter-4/
├── specs/                         # Feature specifications and plans
│   └── 001-module-1-ros2/
│       ├── spec.md                # Requirements
│       ├── plan.md                # Implementation plan
│       ├── research.md            # Design decisions
│       ├── data-model.md          # Content structure
│       ├── quickstart.md          # This file
│       └── contracts/             # Subagent contracts
└── docusaurus.config.js           # Docusaurus configuration
```

## Step-by-Step Guide

### 1. Install Dependencies

```bash
# Install Node.js dependencies (Docusaurus)
npm install

# Verify Docker is running
docker --version
docker pull osrf/ros:humble-desktop
```

### 2. Create a New Chapter (Manual Approach)

If adding a chapter manually (not using subagents):

```bash
# 1. Create chapter content file
touch docs/module-1/chapter-5.mdx

# 2. Add frontmatter (see data-model.md for schema)
# 3. Follow Chapter Template Structure from data-model.md
# 4. Create examples directory
mkdir -p examples/module-1/chapter-5

# 5. Add code examples (see contracts/chapter-content-contract.md)
```

### 3. Use Claude Code Subagents (Recommended)

**Invoke a Subagent**:

```bash
# Example: Generate Chapter 1 using ROSArchitectureAgent

claude-agent run ROSArchitectureAgent \
  --input specs/001-module-1-ros2/spec.md \
  --config specs/001-module-1-ros2/data-model.md \
  --contract specs/001-module-1-ros2/contracts/chapter-content-contract.md \
  --output docs/module-1/chapter-1.mdx \
  --examples-output examples/module-1/chapter-1 \
  --validate docker
```

**Subagent Parameters**:
- `--input`: Path to spec.md with functional requirements
- `--config`: Path to data-model.md with content structure
- `--contract`: Path to chapter-content-contract.md with delivery expectations
- `--output`: Target path for chapter MDX file
- `--examples-output`: Target directory for code examples
- `--validate`: Enable Docker-based code example validation

**Available Subagents**:
- `ROSArchitectureAgent`: Chapter 1 (ROS 2 Architecture)
- `RclpyIntegrationAgent`: Chapter 2 (Python/rclpy Integration)
- `URDFModelingAgent`: Chapter 3 (URDF Modeling)
- `PackageManagementAgent`: Chapter 4 (Packages, Launch Files, Parameters)

### 4. Validate Code Examples

**Test a Single Example**:

```bash
cd examples/module-1/chapter-1/example-01-minimal-publisher

# Run test script
./test.sh
```

**Test All Chapter Examples**:

```bash
cd examples/module-1/chapter-1
./test-all.sh
```

**Manual Docker Testing**:

```bash
# Start interactive Docker container with ROS 2 Humble
docker run -it --rm \
  -v $PWD/examples/module-1/chapter-1:/workspace \
  osrf/ros:humble-desktop \
  bash

# Inside container:
cd /workspace/example-01-minimal-publisher
source /opt/ros/humble/setup.bash
python3 minimal_publisher.py  # or: colcon build && . install/setup.bash && ros2 run ...
```

### 5. Preview in Docusaurus

```bash
# Start Docusaurus development server
npm run start

# Open browser to http://localhost:3000
# Navigate to Module 1 → Chapter X
```

**Hot Reload**: Docusaurus auto-reloads when you save `.mdx` files.

### 6. Validate Content Quality

**Check Markdown Rendering**:
```bash
# Verify no broken links
npm run build  # Docusaurus will error on broken internal links
```

**Check Math Rendering (KaTeX)**:
```bash
# In browser, inspect equations
# Look for proper rendering, no "raw" LaTeX text
```

**Check Diagrams (Mermaid)**:
```bash
# In browser, verify Mermaid diagrams render
# If not, check syntax at https://mermaid.live/
```

**Check Readability**:
```bash
# Install readability tool (optional)
npm install -g readability-cli

# Test chapter readability (target: Flesch-Kincaid grade 12-16)
readability docs/module-1/chapter-1.mdx
```

### 7. Run Full Quality Gates

Before committing, run all quality checks:

```bash
# 1. Build Docusaurus (catches broken links, invalid MDX)
npm run build

# 2. Test all Module 1 code examples
cd examples/module-1
for chapter in chapter-*; do
  echo "Testing $chapter..."
  cd $chapter && ./test-all.sh && cd ..
done

# 3. Lint Python code (if applicable)
find examples/module-1 -name "*.py" -exec flake8 {} +

# 4. Check for unresolved TODOs or placeholders
grep -r "TODO\|FIXME\|PLACEHOLDER" docs/module-1/ examples/module-1/ || echo "No TODOs found ✓"

# 5. Verify frontmatter consistency
# (manual check or custom script)
```

## Troubleshooting

### Issue: Docusaurus Build Fails with "Cannot find module"

**Cause**: Missing npm dependencies

**Solution**:
```bash
rm -rf node_modules package-lock.json
npm install
```

### Issue: KaTeX Math Not Rendering

**Cause**: Invalid LaTeX syntax or missing KaTeX plugin

**Solution**:
1. Verify `remark-math` and `rehype-katex` installed: `npm list remark-math rehype-katex`
2. Test LaTeX syntax at https://katex.org/
3. Check `docusaurus.config.js` has KaTeX configured (see research.md Q2)

### Issue: Mermaid Diagram Not Rendering

**Cause**: Syntax error or missing Mermaid plugin

**Solution**:
1. Verify `@docusaurus/theme-mermaid` installed: `npm list @docusaurus/theme-mermaid`
2. Test diagram at https://mermaid.live/
3. Ensure code block uses ` ```mermaid ` (not ` ```diagram `)

### Issue: Code Example Fails in Docker

**Cause**: Missing dependencies or incorrect ROS 2 setup

**Solution**:
1. Check example's `README.md` for dependency list
2. Verify Docker image: `docker run -it osrf/ros:humble-desktop bash -c "ros2 --version"`
3. Add debug output to `test.sh`: `set -x` at top of script

### Issue: Subagent Fails to Generate Content

**Cause**: Invalid inputs, spec changes, or agent configuration error

**Solution**:
1. Verify `spec.md` functional requirements match expected format
2. Check `chapter-content-contract.md` for updated schema
3. Review subagent logs for specific error messages
4. Re-run with `--debug` flag (if supported)

## Best Practices

### Content Writing

1. **Start with Learning Objectives**: Write these first to guide content creation
2. **Theory Before Practice**: Introduce concepts theoretically before code examples
3. **Progressive Complexity**: Beginner → Intermediate → Advanced in exercises
4. **Real-World Relevance**: Tie examples to actual robotics applications

### Code Examples

1. **Keep Examples Focused**: One concept per example (e.g., "publisher-only", not "publisher + service + action")
2. **Comment Generously**: Assume students are new to ROS 2
3. **Test in Clean Environment**: Always test in Docker before committing
4. **Provide Expected Output**: Students should know what success looks like

### Diagrams

1. **Prefer Mermaid**: Source-controlled, easy to update
2. **Use Alt Text**: Accessibility requirement (SC-009)
3. **Keep Simple**: Avoid cluttered diagrams (max 10-12 nodes in a graph)

### Cross-References

1. **Link Early Content**: When introducing new concepts, link to where they were introduced
2. **Preview Later Content**: In summaries, tease next chapter's content
3. **Use Relative Links**: `../chapter-2/#section-id` (not absolute URLs)

## Common Tasks

### Add a New Code Example to Existing Chapter

1. Create example directory:
   ```bash
   mkdir examples/module-1/chapter-1/example-05-my-new-example
   cd examples/module-1/chapter-1/example-05-my-new-example
   ```

2. Create files:
   - `README.md` (follow template in `contracts/chapter-content-contract.md`)
   - Source code (`.py`, `.cpp`, etc.)
   - `test.sh` (follow template)

3. Add to chapter content:
   ```markdown
   ### Example 1.5: My New Example

   [Link to code](../../../../examples/module-1/chapter-1/example-05-my-new-example)

   [Description, setup, expected output]
   ```

4. Update `test-all.sh`:
   ```bash
   echo "Testing example-05-my-new-example..."
   cd example-05-my-new-example && ./test.sh && cd ..
   ```

5. Test and commit:
   ```bash
   ./test.sh
   git add .
   git commit -m "feat(module-1/ch1): add example 1.5 - my new example"
   ```

### Update a Chapter After Spec Changes

1. Review updated spec: `specs/001-module-1-ros2/spec.md`
2. Identify changed functional requirements (FR-XXX)
3. Update chapter content to align with new requirements
4. Re-run quality gates (especially code example tests)
5. Update test report if code changes

### Add a Callout Box (Info, Warning, Tip)

Docusaurus supports admonitions:

```markdown
:::info Prerequisites
You need ROS 2 Humble installed. See [Installation Guide](#installation).
:::

:::warning Common Pitfall
Don't forget to source ROS 2 before running examples:
`source /opt/ros/humble/setup.bash`
:::

:::tip Performance Optimization
Use multi-threaded executors for real-time control loops (see Chapter 2).
:::
```

## Resources

- **Spec**: `specs/001-module-1-ros2/spec.md` - Requirements for Module 1
- **Plan**: `specs/001-module-1-ros2/plan.md` - Implementation architecture
- **Data Model**: `specs/001-module-1-ros2/data-model.md` - Content structure schema
- **Contract**: `specs/001-module-1-ros2/contracts/chapter-content-contract.md` - Subagent expectations
- **Docusaurus Docs**: https://docusaurus.io/docs
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **KaTeX Docs**: https://katex.org/docs/supported.html
- **Mermaid Docs**: https://mermaid.js.org/

## Getting Help

- **Spec Questions**: Review `spec.md` and `plan.md` for design decisions
- **Code Issues**: Test in Docker with `osrf/ros:humble-desktop`
- **Docusaurus Issues**: Check https://docusaurus.io/docs or GitHub issues
- **ROS 2 Questions**: Official ROS 2 Discourse: https://discourse.ros.org/
