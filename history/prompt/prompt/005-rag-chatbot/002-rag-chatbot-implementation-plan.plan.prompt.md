---
id: 002
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: 005-rag-chatbot
branch: 005-rag-chatbot
command: /sp.plan
labels: ["planning", "architecture", "rag", "backend", "frontend"]
links:
  spec: specs/005-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-rag-chatbot/plan.md
  - specs/005-rag-chatbot/research.md
  - specs/005-rag-chatbot/data-model.md
  - specs/005-rag-chatbot/contracts/api-spec.yaml
  - specs/005-rag-chatbot/contracts/test-scenarios.md
  - specs/005-rag-chatbot/quickstart.md
tests:
  - Architecture review
  - Constitution check validation
---

## Prompt

```text
/sp.plan

Create the implementation plan for the RAG Chatbot Specification. The plan must be purely technical, detailing the steps required to implement the FastAPI backend, Qdrant vector indexing, and Docusaurus frontend integration. Explicitly omit all non-technical, descriptive, or aesthetic details (e.g., UI/UX descriptions, deployment success criteria related to aesthetics). The plan must prioritize the core RAG functionality to minimize token usage.
```

## Response snapshot

Created comprehensive technical implementation plan for RAG Chatbot with focus on core functionality:

**Architecture**: Three-component system:
1. Content Indexing Pipeline (Markdown � Chunks � Qdrant)
2. FastAPI Backend (Vector search + LLM integration)
3. React Chat Widget (Docusaurus embedded component)

**Key Technical Decisions** (Phase 0 Research):
- Chunking: 800 tokens with 100-token overlap, semantic boundaries
- Qdrant: Cosine similarity, HNSW (m=16, ef_construct=100)
- Prompt: Structured system prompt with grounding rules
- Text Selection: Browser Selection API + React Context
- Context Window: Last 5 message pairs, sliding window
- Caching: In-memory LRU (100 embeddings, 50 vector results)

**Tech Stack**:
- Embeddings: text-embedding-3-small ($0.02/1M tokens)
- LLM: GPT-4o-mini ($0.15/1M tokens)
- Vector DB: Qdrant Cloud (free tier)
- Database: Neon Serverless Postgres (free tier)
- Backend: FastAPI + Python 3.11+
- Frontend: React 18 + TypeScript

**Phase 1 Artifacts**:
- Data Model: 4 entities (DocumentChunk, ConversationSession, Message, SourceCitation)
- API Contracts: OpenAPI 3.1 spec with 4 endpoints
- Test Scenarios: 5 core scenarios + performance targets
- Quickstart Guide: Setup and testing instructions

**Architecture Decisions (ADRs)**:
1. text-embedding-3-small over ada-002 (5x cheaper)
2. GPT-4o-mini over GPT-4 (60x cheaper)
3. Session-based history (no auth required)
4. React component over iframe (native integration)

**Constitution Check**: Aligned with free-tier optimization and content fidelity principles. No violations.

## Outcome

-  Impact: Complete technical plan ready for task generation via `/sp.tasks`
- >� Tests: Constitution check passed, architecture reviewed
- =� Files: plan.md, research.md, data-model.md, contracts/, quickstart.md
- = Next prompts: Run `/sp.tasks` to generate detailed implementation task breakdown
- >� Reflection: Focused on core RAG functionality with cost optimization (free tiers, cheaper OpenAI models). Research phase resolved all technical unknowns. Ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): PASS - All constitution checks passed, technical decisions documented
- Prompt variant (if applicable): Streamlined version focusing on technical details per user request
- Next experiment (smallest change to try): Monitor actual API costs during implementation to validate budget assumptions
