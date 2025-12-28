# Research: RAG Chatbot Technical Decisions

**Feature**: 005-rag-chatbot
**Date**: 2025-12-10
**Purpose**: Resolve technical unknowns from Phase 0 planning

## 1. Optimal Chunking Strategy for Technical Content

### Decision
- **Chunk size**: 800 tokens
- **Overlap**: 100 tokens
- **Strategy**: Semantic chunking with boundary detection

### Rationale
- **800 tokens** balances context preservation with embedding quality
  - Large enough to capture multi-step procedures and code examples
  - Small enough to maintain focused semantic meaning
  - Fits comfortably within embedding model limits (8191 tokens for text-embedding-3-small)
- **100-token overlap** ensures context continuity across chunks
  - Prevents information loss at chunk boundaries
  - Allows retrieval of adjacent chunks when needed
- **Semantic boundaries**: Split on section headers, code blocks, and paragraph breaks rather than arbitrary token counts

### Alternatives Considered
- **500 tokens**: Too small, fragments multi-step tutorials
- **1500 tokens**: Too large, dilutes semantic focus, reduces retrieval precision
- **No overlap**: Risks losing context at boundaries, especially for cross-referencing content

### Implementation Notes
- Use `tiktoken` library for accurate token counting (OpenAI's tokenizer)
- Preserve markdown structure: keep code blocks intact, don't split mid-code
- Add metadata tags to chunks: `contains_code`, `is_definition`, `is_procedure`

---

## 2. Qdrant Collection Configuration

### Decision
- **Distance Metric**: Cosine similarity
- **Vector Size**: 1536 (text-embedding-3-small)
- **HNSW Parameters**:
  - `m`: 16 (number of edges per node)
  - `ef_construct`: 100 (construction time accuracy)
  - `ef`: 64 (search time accuracy)

### Rationale
- **Cosine similarity** is standard for text embeddings
  - Normalizes vector magnitude, focuses on direction
  - Better for semantic similarity than euclidean distance
  - OpenAI embeddings are optimized for cosine
- **HNSW parameters** balance speed and accuracy for educational Q&A
  - `m=16`: Good balance between index size and search speed
  - `ef_construct=100`: Higher quality index (worth construction time for static content)
  - `ef=64`: Fast search with acceptable recall (target: >95%)

### Alternatives Considered
- **Dot product**: Requires normalized vectors, no advantage over cosine
- **Euclidean distance**: Less effective for high-dimensional embeddings
- **Higher `ef`**: Slower search with minimal recall improvement for our corpus size

### Implementation Notes
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(
        size=1536,
        distance=Distance.COSINE
    ),
    hnsw_config={
        "m": 16,
        "ef_construct": 100
    }
)

# At search time
search_params = {"hnsw_ef": 64}
```

---

## 3. Prompt Engineering for Grounded Responses

### Decision
Use structured system prompt with explicit grounding instructions, citation requirements, and scope boundaries.

### System Prompt Template
```
You are an AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook. Your purpose is to help students understand concepts from the book.

CRITICAL RULES:
1. Answer ONLY based on the retrieved context provided below
2. If the context doesn't contain relevant information, respond: "I don't have information about this in the Physical AI & Humanoid Robotics textbook"
3. ALWAYS cite your sources using this format: [Source: Module X, Chapter Y]
4. Use Markdown formatting for code snippets, lists, and emphasis
5. Keep responses concise (2-4 paragraphs max) unless the question requires detail

RETRIEVED CONTEXT:
{retrieved_chunks}

CONVERSATION HISTORY:
{conversation_history}

STUDENT QUESTION:
{user_query}

Respond to the student's question using only the information in the retrieved context. Cite specific chapters/sections.
```

### Rationale
- **Explicit grounding** reduces hallucination by setting clear boundaries
- **Citation requirement** enforces source tracking and enables verification
- **Markdown formatting** improves readability for technical content
- **Length constraint** keeps responses focused and reduces token cost
- **Conversation history** enables context awareness for follow-up questions

### Alternatives Considered
- **Few-shot examples**: Adds token cost without significant quality improvement for simple Q&A
- **Chain-of-thought**: Unnecessary complexity for factual questions, increases latency
- **JSON response format**: More rigid, less natural for educational context

### Implementation Notes
- Truncate conversation history to last 3-5 message pairs to stay within context window
- Include chunk source metadata (file path, module, chapter) in retrieved context
- Use `temperature=0.3` for more deterministic, grounded responses

---

## 4. Text Selection Integration Pattern

### Decision
Use browser Selection API with event delegation and React Context for state management.

### Implementation Approach

**1. Text Selection Detection**
```typescript
// In Docusaurus client module
useEffect(() => {
  const handleSelection = () => {
    const selection = window.getSelection();
    const selectedText = selection?.toString().trim();

    if (selectedText && selectedText.length > 0 && selectedText.length < 500) {
      // Store selection and show "Ask Chatbot" tooltip/button
      setSelectedTextContext(selectedText);
    }
  };

  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, []);
```

**2. Context Menu or Floating Button**
- Show small floating button near selection cursor
- Click opens chatbot with pre-filled query: `Explain "${selectedText}"`
- Auto-submit query when opened via text selection

**3. React Context for Global State**
```typescript
interface ChatContextType {
  isOpen: boolean;
  selectedText: string | null;
  openWithQuery: (query: string, context?: string) => void;
}

const ChatContext = createContext<ChatContextType>({...});
```

### Rationale
- **Selection API** is native, no external dependencies
- **Event delegation** efficient for dynamic content (Docusaurus SPA)
- **React Context** provides clean state sharing between selection handler and chat widget
- **Auto-submit** improves UX for text selection flow (one click instead of two)

### Alternatives Considered
- **Context menu override**: Browser compatibility issues, poor UX
- **Tooltip library**: Unnecessary dependency for simple button
- **Global event bus**: Over-engineered for single widget

### Implementation Notes
- Debounce selection handler (300ms) to avoid excessive updates
- Limit selectedText to 500 chars to prevent abuse
- Clear selection state when chat is closed

---

## 5. Conversation Context Window Management

### Decision
Include last 5 message pairs (10 messages total) in LLM context with sliding window strategy.

### Strategy
1. **Always include**: Current query + system prompt + retrieved chunks
2. **Context window**: Last 5 user queries and 5 assistant responses
3. **Token budget**: Reserve 4000 tokens for conversation history
   - Average message: ~200 tokens
   - 10 messages H 2000 tokens
   - Safety margin: 2000 tokens
4. **Truncation**: If history exceeds budget, remove oldest messages first

### Rationale
- **5 message pairs** sufficient for typical educational Q&A sessions
  - Handles follow-up questions ("What about X?", "Can you explain that further?")
  - Doesn't overload context with irrelevant old messages
- **Sliding window** simple and predictable
  - Alternative (summarization) adds latency and cost
- **Token budget** keeps total request under OpenAI limits
  - GPT-4o-mini context: 128K tokens
  - Typical request: 500 (system) + 2000 (history) + 1500 (chunks) + 100 (query) = 4100 tokens
  - Well within limits with room for response

### Alternatives Considered
- **Full history**: Wastes tokens, reduces retrieval space
- **Summarization**: Adds API call cost and latency
- **3 message pairs**: Too limiting for multi-turn clarifications

### Implementation Notes
```python
def prepare_conversation_context(session_id: str, max_pairs: int = 5):
    messages = get_recent_messages(session_id, limit=max_pairs * 2)

    context_str = ""
    for msg in messages:
        role = "Student" if msg.role == "user" else "Assistant"
        context_str += f"{role}: {msg.content}\n\n"

    return context_str
```

---

## 6. Caching Strategy

### Decision
Multi-level caching:
1. **L1 - In-Memory LRU Cache**: Query embeddings (100 most recent queries)
2. **L2 - In-Memory**: Vector search results (50 most common queries)
3. **No LLM response caching** (responses should be grounded, vary with context)

### Rationale
- **Embedding cache** reduces OpenAI API calls for repeated/similar queries
  - Hit rate estimate: 20-30% (common questions: "What is ROS 2?", "How to install Isaac Sim?")
  - Savings: $0.02/1M tokens, ~$0.10-$0.20/month at 1000 queries
- **Vector search cache** speeds up exact duplicate queries
  - Hit rate estimate: 10-15%
  - Latency reduction: 300-500ms per hit
- **No LLM caching** because:
  - Responses vary based on conversation history
  - Grounding in retrieved context may differ
  - Marginal savings not worth stale response risk

### Implementation
```python
from cachetools import LRUCache

embedding_cache = LRUCache(maxsize=100)  # query -> embedding
vector_result_cache = LRUCache(maxsize=50)  # query -> list of chunk_ids

async def get_query_embedding(query: str):
    if query in embedding_cache:
        return embedding_cache[query]

    embedding = await openai_client.embeddings.create(
        input=query,
        model="text-embedding-3-small"
    )
    embedding_cache[query] = embedding
    return embedding
```

### Alternatives Considered
- **Redis**: Over-engineered for pilot scale, adds infrastructure complexity
- **LLM response caching**: Risks serving stale/incorrect responses
- **Larger cache**: Diminishing returns beyond 100 embeddings, minimal memory cost

### Cache Invalidation
- **Embedding cache**: LRU eviction (automatic)
- **Vector result cache**: Clear on content re-indexing
- **Monitoring**: Log hit rates to validate assumptions

---

## Technology Stack Summary

| Component | Technology | Version | Justification |
|-----------|-----------|---------|---------------|
| Embeddings | OpenAI text-embedding-3-small | Latest | 5x cheaper than ada-002, same quality |
| LLM | OpenAI GPT-4o-mini | Latest | 60x cheaper than GPT-4, sufficient for grounded Q&A |
| Vector DB | Qdrant Cloud | Free Tier | Easy setup, good Python SDK, free tier adequate |
| Database | Neon Serverless Postgres | Free Tier | Serverless, free tier, good for conversation history |
| Backend | FastAPI | 0.109+ | Fast, modern, great OpenAPI support |
| Frontend | React + TypeScript | 18.x | Docusaurus native, type safety |
| Chunking | tiktoken + custom logic | Latest | OpenAI's tokenizer, accurate token counting |
| Caching | cachetools | 5.3+ | Simple in-memory caching, no Redis needed |

---

## Open Questions Resolved

1.  **Chunking**: 800 tokens with 100-token overlap, semantic boundaries
2.  **Qdrant Config**: Cosine similarity, HNSW (m=16, ef_construct=100, ef=64)
3.  **Prompting**: Structured system prompt with grounding rules and citation requirements
4.  **Text Selection**: Browser Selection API with React Context
5.  **Context Window**: Last 5 message pairs, sliding window
6.  **Caching**: In-memory LRU for embeddings (100) and vector results (50)

---

## Next Steps

1. **Proceed to Phase 1**: Generate `data-model.md` and `contracts/`
2. **Validate assumptions**: Test chunking strategy on sample chapters
3. **Prototype prompt**: Test grounding effectiveness with sample queries
4. **Set up infrastructure**: Create Qdrant collection, Neon database schema
