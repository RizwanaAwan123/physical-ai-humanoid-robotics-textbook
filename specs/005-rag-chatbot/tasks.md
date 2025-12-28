# Tasks: RAG Chatbot Integration

**Input**: Design documents from `/specs/005-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, US4)
- All tasks include exact file paths

## Path Conventions

Web application structure (from plan.md):
- Backend: `backend/src/`, `backend/tests/`
- Frontend: `src/components/`, `src/plugins/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependencies

- [X] T001 Create backend directory structure per plan.md (backend/src/{api,services,models,core,indexing}, backend/tests/)
- [X] T002 Create backend/requirements.txt with FastAPI 0.109+, qdrant-client 1.7+, openai 1.10+, sqlalchemy 2.0+, pydantic 2.5+, tiktoken, cachetools 5.3+, python-dotenv, uvicorn, pytest, pytest-asyncio, httpx
- [ ] T003 Initialize Python virtual environment and install dependencies from requirements.txt
- [X] T004 [P] Create backend/.env.example with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL placeholders
- [X] T005 [P] Create frontend directories (src/components/ChatWidget/, src/plugins/chat-widget-plugin/)
- [X] T006 [P] Add frontend dependencies to package.json: axios 1.6+, uuid
- [X] T007 [P] Create backend/src/core/config.py for settings management using Pydantic BaseSettings
- [X] T008 [P] Create backend/.gitignore with venv/, __pycache__/, .env, *.pyc, .pytest_cache/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure required before any user story implementation

**� CRITICAL**: Complete before starting user story phases

- [X] T009 Setup Qdrant collection using qdrant-client in backend/src/services/vector_store.py (cosine similarity, 1536 dimensions, HNSW m=16)
- [ ] T010 Create Neon Postgres schema in backend/scripts/setup_db.py (conversation_sessions, messages, source_citations tables per data-model.md)
- [X] T011 Implement backend/src/services/embedder.py with OpenAI text-embedding-3-small integration and LRU caching (100 queries)
- [X] T012 [P] Create Pydantic models in backend/src/models/chat.py (ChatRequest, ChatResponse, Citation)
- [X] T013 [P] Create Pydantic models in backend/src/models/document.py (DocumentChunk, ChunkMetadata)
- [X] T014 [P] Create Pydantic models in backend/src/models/session.py (ConversationSession, Message)
- [X] T015 [P] Setup FastAPI app in backend/src/main.py with CORS middleware and exception handlers
- [X] T016 [P] Implement database connection pooling in backend/src/core/dependencies.py using SQLAlchemy async engine

**Checkpoint**: Foundation complete - user stories can now be implemented in parallel

---

## Phase 3: User Story 1 - General Book Content Querying (Priority: P1) <� MVP

**Goal**: Students can ask questions and receive accurate answers with citations from textbook content

**Independent Test**: Ask "What is ROS 2?" � Receives answer with Module 1 Chapter 1 citation in <3 seconds

### Indexing Pipeline (US1)

- [X] T017 [US1] Implement markdown parser in backend/src/indexing/markdown_parser.py (parse Docusaurus MD files, extract content + metadata)
- [X] T018 [US1] Implement semantic chunker in backend/src/indexing/chunker.py (800-token chunks, 100-token overlap, preserve code blocks, use tiktoken)
- [X] T019 [US1] Create indexing script backend/scripts/index_content.py (CLI tool: parse docs/textbook/, chunk, generate embeddings, store in Qdrant)
- [ ] T020 [US1] Run indexing script to populate Qdrant with existing Module 1-4 content

### Core RAG Services (US1)

- [X] T021 [US1] Implement vector search in backend/src/services/vector_store.py (search method with top-k=5, relevance threshold=0.7)
- [X] T022 [US1] Implement LLM service in backend/src/services/llm_service.py (GPT-4o-mini integration, system prompt from research.md, temperature=0.3)
- [X] T023 [US1] Implement conversation service in backend/src/services/conversation.py (store/retrieve messages from Postgres, sliding window last 5 pairs)

### API Implementation (US1)

- [X] T024 [US1] Implement POST /api/chat/query in backend/src/api/routes/chat.py (integrate embedding, vector search, LLM, conversation storage)
- [X] T025 [US1] Add response time tracking and citation formatting to chat query endpoint
- [X] T026 [P] [US1] Implement GET /health in backend/src/api/routes/health.py (basic status check)
- [X] T027 [P] [US1] Implement GET /health/dependencies in backend/src/api/routes/health.py (check Qdrant, Neon, OpenAI connectivity)

### Testing & Validation (US1)

- [ ] T028 [P] [US1] Create unit test backend/tests/unit/test_chunker.py (validate 800-token chunks, overlap, code block preservation)
- [ ] T029 [P] [US1] Create unit test backend/tests/unit/test_embedder.py (validate caching, API integration)
- [ ] T030 [US1] Create integration test backend/tests/integration/test_chat_api.py (end-to-end query � response flow)
- [ ] T031 [US1] Manual test: Query "What is ROS 2?" and verify response quality, citations, <3s response time

**US1 Deliverable**: Working backend API that answers textbook questions with citations

---

## Phase 4: User Story 4 - Embedded UI Integration (Priority: P1) <� MVP

**Goal**: Chat widget seamlessly integrated in Docusaurus, accessible from all pages

**Independent Test**: Navigate to any book page � See chat button � Click � Chat window opens � Can submit query

**Note**: US4 is implemented before US2/US3 as it provides the basic UI needed for core functionality

### React Components (US4)

- [X] T032 [P] [US4] Create TypeScript interfaces in src/components/ChatWidget/types.ts (Message, Citation, ChatState)
- [X] T033 [US4] Implement ChatButton component in src/components/ChatWidget/ChatButton.tsx (floating button, bottom-right, toggle handler)
- [X] T034 [US4] Implement ChatWindow component in src/components/ChatWidget/ChatWindow.tsx (modal/drawer container, open/close state)
- [X] T035 [US4] Implement MessageList component in src/components/ChatWidget/MessageList.tsx (display user/assistant messages, citations, markdown rendering)
- [X] T036 [US4] Implement InputBar component in src/components/ChatWidget/InputBar.tsx (text input, submit button, loading state)
- [X] T037 [US4] Create useChat hook in src/components/ChatWidget/hooks/useChat.ts (API integration, session management with localStorage, message state)

### Docusaurus Integration (US4)

- [X] T038 [US4] Create main ChatWidget in src/components/ChatWidget/index.tsx (compose Button, Window, MessageList, InputBar)
- [X] T039 [US4] Create Docusaurus plugin in src/plugins/chat-widget-plugin/index.ts (inject ChatWidget into all pages)
- [X] T040 [US4] Add plugin configuration to docusaurus.config.js
- [X] T041 [P] [US4] Create styles in src/components/ChatWidget/styles.module.css (responsive design, theme integration)

### Testing (US4)

- [ ] T042 [P] [US4] Create component tests for ChatWidget using Jest + React Testing Library
- [ ] T043 [US4] Manual test: Navigate to book pages, verify chat button visibility, test open/close, verify responsiveness on mobile

**US4 Deliverable**: Functional chat widget embedded in Docusaurus

---

## Phase 5: User Story 2 - Text Selection Context Querying (Priority: P2)

**Goal**: Students can select text and immediately ask chatbot for clarification

**Independent Test**: Select "URDF" on page � Click "Ask Chatbot" � Chat opens with query "Explain 'URDF'" � Receives contextual answer

**Dependencies**: Requires US1 (backend) and US4 (chat UI) complete

### Text Selection Feature (US2)

- [ ] T044 [US2] Implement useTextSelection hook in src/components/ChatWidget/hooks/useTextSelection.ts (Selection API, mouseup handler, 500-char limit)
- [ ] T045 [US2] Add text selection state to ChatWidget context
- [ ] T046 [US2] Update ChatWindow to accept and display selected_text in query input
- [ ] T047 [US2] Update backend/src/api/routes/chat.py to handle context.selected_text in ChatRequest
- [ ] T048 [US2] Update LLM prompt in backend/src/services/llm_service.py to incorporate selected text as context

### Testing (US2)

- [ ] T049 [US2] Manual test: Select "visual SLAM" on Module 3 page � Trigger chatbot � Verify pre-filled query � Verify contextual response

**US2 Deliverable**: Text selection triggers contextual chatbot queries

---

## Phase 6: User Story 3 - Conversation History and Follow-up Questions (Priority: P3)

**Goal**: Students can ask follow-up questions and chatbot maintains context

**Independent Test**: Ask "What is Isaac Sim?" then "How do I install it?" � Chatbot understands "it" refers to Isaac Sim

**Dependencies**: Requires US1 (backend with conversation service) and US4 (chat UI) complete

### Conversation Context (US3)

- [ ] T050 [US3] Implement GET /api/chat/history/{session_id} in backend/src/api/routes/chat.py
- [ ] T051 [US3] Update useChat hook to load history on mount (restore conversation when reopening chat)
- [ ] T052 [US3] Update backend/src/services/conversation.py to include last 5 message pairs in LLM context (sliding window per research.md)
- [ ] T053 [US3] Update backend/src/services/llm_service.py to format conversation history for LLM prompt

### Testing (US3)

- [ ] T054 [US3] Create integration test backend/tests/integration/test_conversation_context.py (multi-turn conversation, verify context maintained)
- [ ] T055 [US3] Manual test: Multi-turn conversation ("What is RL?" � "Show example" � "How to train?") � Verify coherent responses

**US3 Deliverable**: Multi-turn conversations with context awareness

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Production readiness, performance, reliability

### Rate Limiting & Error Handling

- [ ] T056 [P] Implement rate limiting middleware in backend/src/api/middleware/rate_limit.py (15 queries/minute per session)
- [ ] T057 [P] Add comprehensive error handling to all API endpoints (400, 429, 500 responses)
- [ ] T058 [P] Implement frontend error states in ChatWidget (API errors, network failures, timeouts)

### Performance & Monitoring

- [ ] T059 [P] Add request logging to backend (query text, response time, token usage)
- [ ] T060 [P] Implement usage tracking for OpenAI API costs (log to file, alert at 80% budget)
- [ ] T061 [P] Optimize database queries with indexes (session_id, timestamp in messages table)

### Documentation

- [ ] T062 [P] Create backend/README.md (setup instructions, API documentation, troubleshooting)
- [ ] T063 [P] Update quickstart.md with complete setup and testing procedures
- [ ] T064 [P] Generate OpenAPI documentation from FastAPI schema (export to contracts/api-spec.yaml)

### Final Validation

- [ ] T065 Load test with 50 concurrent users (Apache Bench or Locust), verify <3s P95 latency
- [ ] T066 Validate all 5 test scenarios from contracts/test-scenarios.md
- [ ] T067 Cross-browser testing (Chrome, Firefox, Safari, Edge)
- [ ] T068 Mobile responsiveness testing (iOS Safari, Android Chrome)
- [ ] T069 Accessibility audit (keyboard navigation, screen reader compatibility)

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)
**Goal**: Core Q&A functionality with basic UI

**Phases to Complete**:
1.  Phase 1: Setup
2.  Phase 2: Foundational
3.  Phase 3: User Story 1 (General Querying)
4.  Phase 4: User Story 4 (UI Integration)

**MVP Deliverable**: Students can ask questions via chat widget and receive cited answers from textbook

**Estimated Effort**: T001-T043 (43 tasks)

### Full Feature Set
Add remaining user stories incrementally:
- Phase 5: Text Selection (US2) - 6 tasks
- Phase 6: Conversation History (US3) - 6 tasks
- Phase 7: Polish - 14 tasks

**Total Tasks**: 69

---

## Dependencies & Execution Order

### Critical Path
```
Setup (Phase 1)
  � Foundational (Phase 2)
    � US1 Backend (T017-T027)
      � US4 Frontend (T032-T041)
        � US2 Text Selection (T044-T049)
        � US3 Conversation (T050-T055)
          � Polish (T056-T069)
```

### User Story Dependencies
- **US1** (General Querying): No dependencies (can start after Phase 2)
- **US4** (UI Integration): No dependencies (can start after Phase 2)
- **US2** (Text Selection): Requires US1 + US4 complete
- **US3** (Conversation): Requires US1 + US4 complete

### Parallel Execution Opportunities

**Phase 2 Foundational** (after T008):
- Group A: T009 (Qdrant setup)
- Group B: T010 (Database setup)
- Group C: T011 (Embedder service)
- Group D: T012-T014 (Pydantic models) - can all run together
- Group E: T015-T016 (FastAPI setup) - can run together

**Phase 3 US1** (after T020 indexing):
- Group A: T021-T023 (Core services) - sequential within group
- Group B: T026-T027 (Health endpoints) - can run together
- Group C: T028-T029 (Unit tests) - can run together

**Phase 4 US4** (after T031 US1 complete):
- Group A: T033-T036 (React components) - can develop in parallel
- Group B: T041 (Styles) - can develop in parallel with components

**Phase 7 Polish**:
- All tasks T056-T064 can run in parallel (different files/concerns)
- T065-T069 must run sequentially (validation)

---

## Task Checklist Summary

- **Total Tasks**: 69
- **MVP Tasks**: 43 (Phases 1-4)
- **Parallelizable**: 28 tasks marked with [P]
- **User Story Breakdown**:
  - US1 (General Querying): 15 tasks
  - US4 (UI Integration): 12 tasks
  - US2 (Text Selection): 6 tasks
  - US3 (Conversation): 6 tasks
  - Setup: 8 tasks
  - Foundational: 8 tasks
  - Polish: 14 tasks

---

## Independent Test Criteria

### User Story 1 (General Querying)
**Test**: `curl -X POST http://localhost:8000/api/chat/query -d '{"session_id":"test-123","query":"What is ROS 2?"}'`
**Success**: Returns JSON response with answer, Module 1 citation, processing_time_ms < 3000

### User Story 4 (UI Integration)
**Test**: Visit http://localhost:3000/docs/module-1/chapter-1 � Click chat button
**Success**: Chat window opens, can type query, displays response

### User Story 2 (Text Selection)
**Test**: Select text "URDF" on page � Click selection � Chatbot opens with query
**Success**: Query pre-filled with "Explain 'URDF'", receives contextual answer

### User Story 3 (Conversation)
**Test**: Ask "What is Isaac Sim?" then "How do I install it?" in same session
**Success**: Second response understands "it" = Isaac Sim, provides installation steps

---

## Notes for Implementation

1. **Start with MVP**: Complete Phases 1-4 first for a working demo
2. **Parallel Development**: Backend (US1) and Frontend (US4) teams can work simultaneously after Phase 2
3. **Test Early**: Run T030-T031 after US1 to validate backend before frontend integration
4. **Incremental Delivery**: Each user story is independently testable and deliverable
5. **Cost Monitoring**: Track OpenAI API usage from day 1 (T060)
