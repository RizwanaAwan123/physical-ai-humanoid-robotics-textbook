---
id: 001
title: Constitution v1.0.0 Physical AI Robotics Textbook
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "initial-setup", "project-principles", "hackathon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - N/A (constitution document)
---

## Prompt

```text
Project Name: Physical AI & Humanoid Robotics Textbook — The Embodied AI Course

Purpose:
Create a comprehensive, high-quality, and AI-Native textbook for teaching the Physical AI & Humanoid Robotics course. The book must be built using Docusaurus, possess a professional, attractive, and clean UI/UX, and feature all core and bonus requirements of the hackathon.

Scope:
- Full Textbook: 4 Modules, each with 4 detailed, technical chapters (Total 16 chapters).
    - Module 1: The Robotic Nervous System (ROS 2)
    - Module 2: The Digital Twin (Gazebo & Unity)
    - Module 3: The AI-Robot Brain (NVIDIA Isaac™)
    - Module 4: Vision-Language-Action (VLA)
- Full RAG Chatbot Integration (Qdrant, Neon, FastAPI, OpenAI ChatKit/Agents).
- Bonus Features Implementation: Signup/Signin (Better-Auth), Content Personalization, and Urdu Translation.

Core Principles (The 'Look'):
- **Professionalism & Depth:** Content must be technically accurate and deep, following the course details strictly.
- **Attractive & Descent UI:** Use a modern, clean Docusaurus theme with custom styling (e.g., color palette, typography) to match a high-end educational platform (like Panaversity's existing work).
- **AI-Driven & Reusable:** All development should prioritize AI/Spec-Driven methods, including the creation of Claude Subagents for bonus points.
- **Free-Tier Optimization:** The RAG system must be viable using free tiers (Qdrant Cloud Free Tier, Neon Serverless Postgres).

Success Criteria:
- Successful generation of all 16 chapters via Spec-Kit Plus.
- RAG Chatbot accurately answers questions based ONLY on book content.
- Functional implementation of Signup/Signin (Better-Auth).
- Functional **Personalization** and **Urdu Translation** buttons for logged-in users.
- Clean, smooth deployment to Vercel or GitHub Pages.

Constraints:
- **No deviations from the 4-Module, 4-Chapter structure.**
- The RAG backend must minimize complexity to remain within free-tier limits.
- The UI must feel modern and professional, not a default Docusaurus template.

Generate full constitution.
```

## Response snapshot

Created comprehensive constitution v1.0.0 for the Physical AI & Humanoid Robotics Textbook project with:

**Six Core Principles:**
1. Technical Accuracy & Depth - Enforces professional robotics engineering standards
2. Modular Structure Integrity (NON-NEGOTIABLE) - Fixed 4×4 module/chapter architecture
3. AI-Native Development & Spec-Driven Approach - SDD methodology mandatory
4. Professional UI/UX Standards - Custom Docusaurus theme with polished design
5. Free-Tier Infrastructure Optimization - Qdrant, Neon, OpenAI within free tiers
6. RAG System Content Fidelity - Chatbot answers exclusively from textbook content

**Additional Sections:**
- Content Standards (chapter requirements, code quality, review gates)
- Technical Architecture (Docusaurus config, RAG stack, auth, translation)
- Development Workflow (feature lifecycle, git workflow, quality gates, testing)
- Governance (amendment process, versioning policy, compliance)

**Technical Stack Specified:**
- Frontend: Docusaurus 3.x + custom theme + MDX + KaTeX + Mermaid
- Backend: FastAPI (Python 3.10+)
- Databases: Qdrant (vector), Neon Postgres (relational)
- AI: OpenAI text-embedding-3-small + GPT-4o-mini
- Auth: Better-Auth
- i18n: Docusaurus i18n plugin (Urdu support)

**Governance Established:**
- Semantic versioning (MAJOR.MINOR.PATCH)
- Amendment process with ADR creation for architectural decisions
- Compliance enforcement via code reviews and quality gates

## Outcome

- ✅ Impact: Established foundational governance for entire hackathon project lifecycle
- 🧪 Tests: No tests (governance document)
- 📁 Files: Created constitution.md (v1.0.0, 187 lines)
- 🔁 Next prompts: `/sp.specify` for first feature (likely "Docusaurus Setup" or "Module 1 Chapter 1")
- 🧠 Reflection: Constitution comprehensively addresses hackathon requirements while maintaining flexibility for educational content iteration

## Evaluation notes (flywheel)

- Failure modes observed: None (initial constitution creation)
- Graders run and results (PASS/FAIL): Manual review - PASS (all placeholders filled, principles clear)
- Prompt variant (if applicable): N/A (first constitution)
- Next experiment (smallest change to try): Monitor if free-tier limits need adjustment after RAG implementation begins
