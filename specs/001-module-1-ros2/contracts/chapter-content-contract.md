# Chapter Content Delivery Contract

**Version**: 1.0.0
**Date**: 2025-12-06
**Scope**: Module 1 - ROS 2 Fundamentals (Chapters 1-4)

## Contract Purpose

This contract defines the expected inputs and outputs for Claude Code Subagents generating Module 1 chapters. It ensures consistency, quality, and compliance with Constitution principles.

## Subagent Input Contract

Each subagent receives:

### Required Inputs

1. **Specification Reference**
   - File: `specs/001-module-1-ros2/spec.md`
   - Section: Functional Requirements for specific chapter
   - Format: Markdown

2. **Content Template**
   - File: `specs/001-module-1-ros2/data-model.md` (Chapter Template Structure section)
   - Format: Markdown template with placeholder sections

3. **Configuration**
   ```json
   {
     "chapter_number": 1-4,
     "module_number": 1,
     "ros2_distribution": "humble",
     "ubuntu_version": "22.04",
     "target_audience": "undergraduate/graduate robotics students",
     "estimated_hours": 4-6,
     "code_languages": ["Python", "C++"],
     "output_format": "MDX"
   }
   ```

4. **Cross-Reference Data** (for Chapters 2-4)
   ```json
   {
     "previous_chapters": [
       {
         "chapter_number": 1,
         "export_sections": {
           "communication_patterns": "module-1/chapter-1/#communication-patterns",
           "node_basics": "module-1/chapter-1/#node-basics"
         }
       }
     ]
   }
   ```

### Optional Inputs

1. **Style Guide**
   - Tone: Professional but accessible
   - Voice: Second person ("you will learn")
   - Technical depth: Balance theory and practice

2. **Example Code Constraints**
   - Maximum lines per example: 150 (for readability)
   - Minimum comments: 1 comment per 10 lines (non-obvious logic)
   - Test execution environment: Docker (`osrf/ros:humble-desktop`)

## Subagent Output Contract

Each subagent delivers:

### Required Outputs

1. **Chapter Content File**
   - **Path**: `docs/module-1/chapter-{N}.mdx` (where N = 1-4)
   - **Format**: MDX (Markdown + JSX)
   - **Size**: 8,000-15,000 words (approximately 30-50 pages printed)
   - **Structure**: Must follow Chapter Template Structure from `data-model.md`
   - **Frontmatter**: YAML with ChapterMetadata (see schema below)

2. **Code Examples Directory**
   - **Path**: `examples/module-1/chapter-{N}/`
   - **Structure**: One subdirectory per example
   - **Contents per example**:
     - Source code files (`.py`, `.cpp`, `.urdf`, `.xacro`, etc.)
     - `README.md` (setup instructions, dependencies, expected output)
     - `test.sh` (automated validation script)
     - `package.xml` (if ROS 2 package)
     - `CMakeLists.txt` or `setup.py` (if applicable)

3. **Test Report**
   - **Path**: `examples/module-1/chapter-{N}/test-report.md`
   - **Contents**:
     - List of all code examples
     - Test execution status (PASS/FAIL)
     - Docker image used (`osrf/ros:humble-desktop`)
     - Execution logs (if failures)

4. **Diagram Sources**
   - **Path**: `docs/module-1/_diagrams/chapter-{N}/`
   - **Format**: Mermaid (`.mmd` files) or SVG (for complex diagrams)
   - **Naming**: Descriptive, e.g., `pubsub-communication-graph.mmd`

### Chapter Content Schema

**MDX Frontmatter**:
```yaml
---
id: [chapter-slug]                            # e.g., "ros2-architecture"
title: "Chapter {N}: {Title}"                 # e.g., "Chapter 1: ROS 2 Architecture"
sidebar_label: "Ch {N}: {Short Title}"        # e.g., "Ch 1: ROS 2 Architecture"
sidebar_position: {N}                         # 1, 2, 3, or 4
description: "{SEO-friendly description}"     # 150-160 chars
keywords: [{tag1}, {tag2}, ...]               # e.g., ["ROS 2", "architecture", "DDS"]
slug: /module-1/{chapter-slug}                # e.g., /module-1/ros2-architecture
tags:
  - {tag1}
  - {tag2}
---
```

**Required Sections** (in order):

1. **Overview** (200-300 words)
   - What this chapter covers
   - Why it matters for robotics
   - How it fits in Module 1 progression

2. **Learning Objectives** (3-5 items)
   - Action verb + measurable outcome
   - Bloom's taxonomy level indicated
   - Assessment method described

3. **Theoretical Foundations** (2,000-4,000 words)
   - Core concepts explained
   - ≥1 mathematical formulation (KaTeX)
   - ≥2 Mermaid diagrams
   - Citations to ROS 2 documentation

4. **Hands-On Implementation** (3,000-5,000 words)
   - ≥2 step-by-step tutorials
   - ≥3 executable code examples (Python and/or C++)
   - Setup instructions for each example
   - Expected output described

5. **Practical Examples** (1,500-2,500 words)
   - ≥2 real-world scenarios
   - Code implementations with explanations
   - Design choices justified

6. **Exercises & Challenges** (1,000-1,500 words)
   - ≥5 exercises total
   - Mix of beginner (2), intermediate (2), advanced (1)
   - Acceptance criteria for each

7. **Further Reading** (300-500 words)
   - ≥5 curated references
   - Mix of official docs, research papers, tutorials
   - Brief relevance description for each

8. **Summary** (300-500 words)
   - Key takeaways (3-5 bullet points)
   - Preview of next chapter

9. **Troubleshooting** (500-800 words)
   - ≥3 common issues with solutions
   - Platform-specific notes (Ubuntu 22.04 vs 24.04)

### Code Example Schema

**example-{NN}-{name}/** structure:

```
example-{NN}-{descriptive-name}/
├── README.md                 # Required
├── src/                      # Optional (if multi-file)
│   ├── main.py or main.cpp
│   └── helpers.py or helpers.cpp
├── package.xml               # Required (if ROS 2 package)
├── setup.py or CMakeLists.txt  # Required (build file)
├── config/                   # Optional (launch files, params)
│   ├── params.yaml
│   └── launch.py
├── test.sh                   # Required (validation script)
└── expected_output.txt       # Optional (for test.sh)
```

**README.md Template**:

```markdown
# Example {N}.{NN}: {Title}

## Description

{1-2 sentence description of what this example demonstrates}

## Dependencies

- ROS 2 Humble (`ros-humble-desktop`)
- Python 3.10+ (or C++17)
- Additional packages: `{package-list}`

## Setup

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install dependencies (if any)
{installation commands}

# Build (if necessary)
{build commands}
```

## Running the Example

```bash
{execution commands}
```

## Expected Output

```
{sample output}
```

## Validation

Run the test script:

```bash
./test.sh
```

Expected result: `All tests passed ✓`

## Learning Objectives

This example demonstrates:
- {objective 1}
- {objective 2}
- {objective 3}
```

**test.sh Template**:

```bash
#!/bin/bash

# Test script for Example {N}.{NN}

set -e  # Exit on error

echo "Testing Example {N}.{NN}: {Title}"

# 1. Build test (if applicable)
echo "Step 1: Building..."
{build commands}

# 2. Static analysis (if applicable)
echo "Step 2: Linting..."
{linting commands, e.g., flake8, cppcheck}

# 3. Execution test
echo "Step 3: Running example..."
timeout 10s {execution command} &
PID=$!

# 4. Validation (check expected topics/services/actions)
sleep 2
echo "Step 4: Validating ROS 2 entities..."
{validation commands, e.g., ros2 topic list | grep /expected_topic}

# 5. Cleanup
kill $PID 2>/dev/null || true

echo "✓ All tests passed!"
```

## Quality Gates

Before a subagent delivers outputs, it MUST validate:

### Content Quality Gates

- [ ] All 9 required sections present (Overview, Learning Objectives, ..., Troubleshooting)
- [ ] 3-5 learning objectives with action verbs
- [ ] ≥1 mathematical formulation in Theoretical Foundations (KaTeX syntax valid)
- [ ] ≥2 Mermaid diagrams with alt text
- [ ] ≥3 executable code examples (Python and/or C++)
- [ ] ≥5 exercises (2 beginner, 2 intermediate, 1 advanced)
- [ ] ≥5 further reading references with descriptions
- [ ] Summary includes next chapter preview
- [ ] Flesch-Kincaid grade level: 12-16 (measured via readability tool)
- [ ] No broken internal links (validated via link checker)

### Code Quality Gates

- [ ] All code examples execute without errors in Docker (`osrf/ros:humble-desktop`)
- [ ] Python code passes `flake8` linting (PEP 8 compliant)
- [ ] C++ code passes `cppcheck` static analysis
- [ ] All examples have `README.md` with setup instructions
- [ ] All examples have `test.sh` that exits 0 on success
- [ ] Test report documents all example execution results

### Consistency Gates

- [ ] Terminology consistent with `data-model.md` (Node, Topic, Service, Action, etc.)
- [ ] Cross-references to previous chapters valid (if Chapter 2-4)
- [ ] Code style consistent (CamelCase for classes, snake_case for functions/variables)
- [ ] File paths follow conventions (`examples/module-1/chapter-{N}/`)

## Delivery Checklist

When a subagent completes its chapter, it provides:

```markdown
## Subagent Delivery Report: Chapter {N}

**Subagent**: {AgentName}
**Date**: {YYYY-MM-DD}
**Status**: {Complete/Needs Review/Blocked}

### Deliverables

- [x] Chapter content: `docs/module-1/chapter-{N}.mdx` ({word-count} words)
- [x] Code examples: `examples/module-1/chapter-{N}/` ({example-count} examples)
- [x] Test report: `examples/module-1/chapter-{N}/test-report.md` ({pass-count}/{total-count} passed)
- [x] Diagrams: `docs/module-1/_diagrams/chapter-{N}/` ({diagram-count} diagrams)

### Quality Gate Results

**Content Quality**: {X}/10 gates passed
**Code Quality**: {Y}/6 gates passed
**Consistency**: {Z}/4 gates passed

**Overall**: {PASS/FAIL}

### Issues / Blockers

{List any issues encountered or blockers preventing completion}

### Recommendations for Next Chapter

{Suggestions for Chapter {N+1} based on learnings}
```

## Error Handling

If a subagent encounters issues:

### Content Issues

**Issue**: Unclear specification requirement
**Action**: Flag in delivery report, propose reasonable interpretation, request clarification

**Issue**: Exceeds word count limits
**Action**: Identify sections to condense, preserve essential content, maintain clarity

**Issue**: Missing prerequisite knowledge not in previous chapters
**Action**: Add to Prerequisites subsection, link to external resources

### Code Issues

**Issue**: Code example fails in Docker
**Action**: Debug in Docker environment, update code, re-test, document troubleshooting steps

**Issue**: ROS 2 API deprecation or change
**Action**: Use current Humble API, add callout box noting future changes in Jazzy

**Issue**: Example too complex for target audience
**Action**: Simplify example, move complexity to Advanced Exercises section

## Version History

- **v1.0.0** (2025-12-06): Initial contract for Module 1 chapters 1-4
