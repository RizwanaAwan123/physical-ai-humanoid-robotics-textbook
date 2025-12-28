# Implementation Plan: RAG Chatbot Integration

**Branch**: `005-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-rag-chatbot/spec.md`

## Summary

Implement a Retrieval-Augmented Generation (RAG) chatbot system that enables students to query the Physical AI & Humanoid Robotics textbook content through natural language questions. The system consists of three technical components:

1. **Content Indexing Pipeline**: Parse Docusaurus markdown files, chunk text semantically, generate embeddings, and store in Qdrant vector database
2. **FastAPI Backend**: REST API for query processing, vector search, LLM response generation, and conversation history management
3. **React Chat Widget**: Embedded UI component in Docusaurus with text-selection query support

**Technical Approach**: Standard RAG architecture with OpenAI embeddings (text-embedding-3-small), Qdrant for vector storage, GPT-4o-mini for response generation, Neon Postgres for conversation persistence, and React component embedded via Docusaurus plugin.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 18 (frontend)
**Primary Dependencies**:
- Backend: FastAPI 0.109+, Qdrant Client 1.7+, OpenAI Python SDK 1.10+, SQLAlchemy 2.0+, Pydantic 2.5+
- Frontend: React 18, @docusaurus/core 3.x, axios 1.6+

**Storage**:
- Vector DB: Qdrant Cloud (free tier: 1GB, 100K vectors)
- Relational DB: Neon Serverless Postgres (free tier: 512MB)

**Testing**: pytest (backend), Jest + React Testing Library (frontend)
**Target Platform**: Linux server (backend), Web browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web application (separate backend + frontend integration)
**Performance Goals**:
- Query processing: <3 seconds end-to-end (P95)
- Vector search: <500ms (P95)
- LLM response: <2 seconds (P95)
- Concurrent users: 100

**Constraints**:
- OpenAI API budget: $50-100/month (~1000 queries)
- Qdrant free tier: 100K vectors, 1GB storage
- Neon free tier: 512MB storage
- Response must stay within 3-second requirement

**Scale/Scope**:
- Content corpus: ~16 chapters, est. 200K-300K tokens
- Chunk size: 500-1000 tokens with 100-token overlap
- Estimated vectors: ~300-600 chunks
- User sessions: ~50-100 concurrent (pilot phase)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Aligned Principles:**
- ✅ **V. Free-Tier Infrastructure Optimization**: Solution uses Qdrant Cloud free tier, Neon Postgres free tier, and optimizes OpenAI API usage through efficient chunking
- ✅ **VI. RAG System Content Fidelity**: System designed to answer from textbook content exclusively with citation support and grounding checks
- ✅ **III. AI-Native Development**: Following SDD methodology with spec → plan → tasks workflow

**Potential Concerns:**
- ⚠️ **OpenAI API Cost Control**: Need caching strategy and rate limiting to stay within budget
- ⚠️ **Embedding Quality for Technical Content**: Code blocks and mathematical notation may not embed well
- ⚠️ **Response Time Under Load**: 3-second requirement with 100 concurrent users requires optimization

**Mitigations:**
- Implement query result caching (Redis or in-memory LRU)
- Test chunking strategies to preserve code context
- Use connection pooling and async processing
- Monitor API usage with alerts at 80% budget threshold

**Re-check After Design**: Will validate that data model and API contracts maintain constitution alignment.

## Project Structure

###Documentation (this feature)

```text
specs/005-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── api-spec.yaml   # OpenAPI 3.1 specification
│   └── test-scenarios.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── routes/
│   │   │   ├── chat.py         # POST /api/chat/query, GET /api/chat/history
│   │   │   └── health.py       # GET /health, GET /health/dependencies
│   │   └── middleware/
│   │       ├── cors.py          # CORS configuration
│   │       └── rate_limit.py    # Rate limiting per session
│   ├── services/
│   │   ├── embedder.py          # OpenAI embedding generation
│   │   ├── vector_store.py      # Qdrant vector operations
│   │   ├── llm_service.py       # OpenAI chat completions
│   │   └── conversation.py      # Conversation history management
│   ├── models/
│   │   ├── chat.py              # Pydantic models: ChatRequest, ChatResponse
│   │   ├── document.py          # DocumentChunk, ChunkMetadata
│   │   └── session.py           # ConversationSession, Message
│   ├── core/
│   │   ├── config.py            # Settings from environment
│   │   └── dependencies.py      # FastAPI dependency injection
│   ├── indexing/
│   │   ├── markdown_parser.py   # Parse Docusaurus MD files
│   │   ├── chunker.py           # Semantic text chunking
│   │   └── indexer.py           # Build/update Qdrant index
│   └── main.py                  # FastAPI app entry point
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   └── test_embedder.py
│   ├── integration/
│   │   └── test_chat_api.py
│   └── fixtures/
│       └── sample_docs.md
├── scripts/
│   └── index_content.py         # CLI script to run indexing
├── requirements.txt
├── .env.example
└── README.md

src/
└── components/
    └── ChatWidget/
        ├── index.tsx                 # Main chat component
        ├── ChatButton.tsx            # Floating action button
        ├── ChatWindow.tsx            # Chat interface
        ├── MessageList.tsx           # Message display
        ├── InputBar.tsx              # User input + submit
        ├── hooks/
        │   ├── useChat.ts           # Chat API integration
        │   └── useTextSelection.ts  # Text selection handler
        ├── types.ts                  # TypeScript interfaces
        └── styles.module.css         # Component styles

src/
└── plugins/
    └── chat-widget-plugin/
        ├── index.ts                  # Docusaurus plugin entry
        └── client.ts                 # Browser-side initialization

.env
.gitignore
```

**Structure Decision**: Web application architecture selected due to:
- Clear separation: FastAPI backend handles RAG logic, React frontend handles UI
- Independent deployment: Backend can be hosted on any Python-capable platform
- Docusaurus integration: Chat widget integrated as plugin/component
- Testing isolation: Backend API tests separate from frontend component tests

## Complexity Tracking

No constitution violations requiring justification. The architecture aligns with free-tier optimization and content fidelity principles.

## Phase 0: Research

**Unknowns to Resolve:**

1. **Optimal Chunking Strategy for Technical Content**
   - Question: What chunk size and overlap preserve context for code examples and multi-step procedures?
   - Research: Test 500/1000/1500 token chunks with 50/100/200 token overlap on sample chapters
   - Deliverable: Recommended chunking parameters with sample output

2. **Qdrant Collection Configuration**
   - Question: What distance metric (cosine/euclidean/dot product) and index parameters optimize for textbook Q&A?
   - Research: Review Qdrant docs for educational content use cases, test retrieval quality
   - Deliverable: Collection configuration (distance metric, vector params, HNSW settings)

3. **Prompt Engineering for Grounded Responses**
   - Question: What system prompt structure minimizes hallucination and ensures citation?
   - Research: Test prompt templates with chain-of-thought, few-shot examples, citation requirements
   - Deliverable: System prompt template with examples

4. **Text Selection Integration Pattern**
   - Question: How to capture text selection events and trigger chatbot from Docusaurus pages?
   - Research: Browser Selection API, Docusaurus plugin hooks, event delegation patterns
   - Deliverable: Implementation approach with code snippets

5. **Conversation Context Window Management**
   - Question: How many messages to include in context while staying within token limits?
   - Research: Calculate token usage for typical conversations, test context window sizes
   - Deliverable: Context window strategy (e.g., last N messages, sliding window, summarization)

6. **Caching Strategy**
   - Question: What to cache (embeddings, vector results, LLM responses) and cache eviction policy?
   - Research: Analyze query patterns, estimate hit rates, evaluate caching libraries (aiocache, Redis)
   - Deliverable: Caching implementation plan

**Research Output**: All findings consolidated in `research.md`

## Phase 1: Data Model & Contracts

### Data Model (`data-model.md`)

**Entities:**

1. **DocumentChunk**
   - id: UUID
   - content: Text
   - embedding: Vector[1536]  # text-embedding-3-small dimension
   - metadata: ChunkMetadata
     - source_file: str (e.g., "docs/module-1/chapter-1.md")
     - module_name: str
     - chapter_title: str
     - chunk_index: int
     - token_count: int

2. **ConversationSession**
   - session_id: UUID (client-generated, stored in browser)
   - created_at: timestamp
   - last_activity: timestamp
   - messages: List[Message]

3. **Message**
   - message_id: UUID
   - session_id: UUID (foreign key)
   - role: enum (user, assistant, system)
   - content: Text
   - source_citations: List[SourceCitation] (optional, for assistant messages)
   - timestamp: timestamp

4. **SourceCitation**
   - chunk_id: UUID (references DocumentChunk)
   - relevance_score: float
   - excerpt: Text (retrieved chunk content)

**Relationships:**
- ConversationSession (1) → (N) Message
- Message (1) → (N) SourceCitation
- SourceCitation (N) → (1) DocumentChunk

### API Contracts (`contracts/api-spec.yaml`)

**Endpoints:**

1. **POST /api/chat/query**
   - Request:
     ```json
     {
       "session_id": "uuid",
       "query": "string",
       "context": {
         "current_page": "string (optional)",
         "selected_text": "string (optional)"
       }
     }
     ```
   - Response:
     ```json
     {
       "message_id": "uuid",
       "response": "string (markdown)",
       "citations": [
         {
           "source": "Module 1, Chapter 2",
           "excerpt": "string",
           "relevance": 0.95
         }
       ],
       "suggested_questions": ["string"],
       "processing_time_ms": 1250
     }
     ```
   - Errors: 400 (invalid request), 429 (rate limit), 500 (server error)

2. **GET /api/chat/history/{session_id}**
   - Response:
     ```json
     {
       "session_id": "uuid",
       "messages": [
         {
           "role": "user|assistant",
           "content": "string",
           "timestamp": "ISO 8601",
           "citations": []
         }
       ]
     }
     ```

3. **GET /health**
   - Response: `{"status": "healthy", "timestamp": "ISO 8601"}`

4. **GET /health/dependencies**
   - Response:
     ```json
     {
       "qdrant": {"status": "up", "latency_ms": 45},
       "neon_postgres": {"status": "up", "latency_ms": 23},
       "openai": {"status": "up", "latency_ms": 150}
     }
     ```

### Integration Scenarios (`contracts/test-scenarios.md`)

**Scenario 1: Simple Question**
- User asks: "What is ROS 2?"
- Expected: System retrieves chunks from Module 1, generates response with definition, returns citation to Chapter 1

**Scenario 2: Text Selection Query**
- User selects "URDF" on page, clicks "Ask Chatbot"
- Expected: Chatbot opens with query "Explain 'URDF'", processes with selected_text context, returns definition + related concepts

**Scenario 3: Follow-up Question**
- User asks: "What is Isaac Sim?" then "How do I install it?"
- Expected: System includes previous message in context, understands "it" refers to Isaac Sim, returns installation instructions

**Scenario 4: Out-of-Scope Query**
- User asks: "What is the capital of France?"
- Expected: System returns "I don't have information about this in the Physical AI & Humanoid Robotics textbook"

**Scenario 5: Rate Limiting**
- User sends 20 queries in 1 minute
- Expected: After 15th query, system returns 429 with retry-after header

### Quickstart Guide (`quickstart.md`)

**Backend Setup:**
```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with API keys: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL

# Index content
python scripts/index_content.py --source ../docs/textbook

# Run server
uvicorn src.main:app --reload --port 8000
```

**Frontend Integration:**
```bash
cd .. # to repo root
npm install

# Configure plugin in docusaurus.config.js
# (details in quickstart.md)

npm run start
# Visit http://localhost:3000, click chat button
```

**Testing:**
```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests
cd ..
npm test -- ChatWidget
```

## Phase 2 Preparation Notes

**Task Generation Guidance for `/sp.tasks`:**

1. **Setup Tasks**:
   - Install dependencies (backend requirements.txt, frontend package.json)
   - Configure environment variables
   - Set up Qdrant collection
   - Set up Neon Postgres tables

2. **Core RAG Implementation**:
   - Implement markdown parser and chunker
   - Implement embedding service
   - Implement vector store operations
   - Implement LLM service with prompt template
   - Create indexing script

3. **API Implementation**:
   - Implement POST /api/chat/query endpoint
   - Implement conversation history storage
   - Implement rate limiting middleware
   - Implement health check endpoints

4. **Frontend Implementation**:
   - Create ChatButton component
   - Create ChatWindow component
   - Create MessageList component
   - Implement useChat hook
   - Implement text selection handler
   - Create Docusaurus plugin integration

5. **Testing & Validation**:
   - Unit tests for chunker and embedder
   - Integration tests for chat API
   - Component tests for React widgets
   - End-to-end test for full query flow
   - Validate response time <3 seconds
   - Validate concurrent user handling

6. **Documentation**:
   - API documentation (OpenAPI spec)
   - Deployment guide
   - Troubleshooting guide

**Critical Path**: Indexing → Vector Search → LLM Integration → API → Frontend Widget

**Parallelizable**: API routes can be developed while frontend components are built (mock API during frontend dev)

## Architecture Decisions

### ADR-001: Use text-embedding-3-small over ada-002
**Decision**: Use OpenAI text-embedding-3-small for embeddings
**Rationale**:
- 5x cheaper than ada-002 ($0.02/1M tokens vs $0.10/1M)
- Same 1536 dimensions
- Better performance on recent benchmarks
- Budget constraint of $50-100/month requires cost optimization

**Alternatives Considered**:
- ada-002: More expensive, no significant quality advantage
- Open-source models (sentence-transformers): Hosting cost/complexity not worth savings at pilot scale

### ADR-002: Use GPT-4o-mini instead of GPT-4
**Decision**: Use GPT-4o-mini for response generation
**Rationale**:
- 60x cheaper than GPT-4 ($0.15/1M input tokens vs $30/1M)
- Sufficient quality for educational Q&A grounded in retrieved context
- Budget allows ~1000 queries/month at average 2K tokens per request
- Can upgrade to GPT-4 for complex queries if needed

**Alternatives Considered**:
- GPT-4: Better reasoning but cost prohibitive for pilot
- GPT-3.5-turbo: Cheaper but GPT-4o-mini provides better grounding

### ADR-003: Session-based conversation history (no user accounts required)
**Decision**: Store conversation history keyed by browser-generated session_id, no authentication required
**Rationale**:
- Spec assumes public book access model (no auth required for chatbot)
- Simplifies MVP implementation
- Session storage in browser (localStorage) sufficient for pilot
- Neon Postgres stores history for session continuity across page refreshes

**Alternatives Considered**:
- User accounts: Adds complexity, out of scope for P1
- No persistence: Poor UX, users lose context on refresh

### ADR-004: React component over iframe for chat widget
**Decision**: Implement chat widget as native React component integrated via Docusaurus plugin
**Rationale**:
- Native styling control (matches Docusaurus theme)
- Direct access to page context (current route, text selection events)
- Better performance than iframe
- Simpler state management

**Alternatives Considered**:
- Iframe: Isolation benefits not needed, styling complexity not worth it
- Web Component: Less ecosystem support for React integration

## Risk Mitigation

1. **OpenAI API Cost Overrun**
   - Mitigation: Implement usage tracking with alerts at 80% budget, cache common queries, rate limit to 15 queries/minute per session

2. **Vector Search Quality Issues**
   - Mitigation: Test multiple chunking strategies in Phase 0, implement re-ranking with cross-encoder if needed, collect feedback for iteration

3. **Response Time SLA Miss**
   - Mitigation: Implement async processing, optimize database queries with indexes, use connection pooling, monitor P95 latency

4. **Content Indexing Failures**
   - Mitigation: Validate markdown parsing on all existing chapters, implement error handling and partial indexing, log failures for manual review

## Next Steps

1. **Complete Phase 0**: Run research tasks, consolidate findings in research.md
2. **Complete Phase 1**: Generate data-model.md, contracts/api-spec.yaml, quickstart.md
3. **Run `/sp.tasks`**: Generate detailed task breakdown for implementation
4. **Begin Implementation**: Execute tasks following TDD approach (tests before code)
