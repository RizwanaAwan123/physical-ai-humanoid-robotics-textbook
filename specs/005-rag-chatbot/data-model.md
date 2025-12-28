# Data Model: RAG Chatbot

**Feature**: 005-rag-chatbot
**Date**: 2025-12-10

## Entities

### 1. DocumentChunk (Qdrant)
Represents a semantically chunked segment of textbook content with embedding.

**Fields**:
- `id`: UUID (Qdrant point ID)
- `vector`: float[1536] (text-embedding-3-small)
- `payload`:
  - `content`: string (chunk text, 800 tokens avg)
  - `source_file`: string (`docs/textbook/module-1/chapter-1.md`)
  - `module_name`: string (`Module 1: ROS 2 Fundamentals`)
  - `chapter_title`: string (`Chapter 1: ROS 2 Architecture`)
  - `chunk_index`: int (position in source file)
  - `token_count`: int

**Indexes**: Vector index (HNSW), payload index on `module_name`

---

### 2. ConversationSession (Neon Postgres)
Represents a chat session with message history.

**Schema**:
```sql
CREATE TABLE conversation_sessions (
    session_id UUID PRIMARY KEY,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    last_activity TIMESTAMP NOT NULL DEFAULT NOW()
);
```

**Relationships**: 1-to-many with Messages

---

### 3. Message (Neon Postgres)
Represents a single message in a conversation.

**Schema**:
```sql
CREATE TABLE messages (
    message_id UUID PRIMARY KEY,
    session_id UUID NOT NULL REFERENCES conversation_sessions(session_id) ON DELETE CASCADE,
    role VARCHAR(10) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    CONSTRAINT messages_session_fk FOREIGN KEY (session_id) REFERENCES conversation_sessions(session_id)
);

CREATE INDEX idx_messages_session_timestamp ON messages(session_id, timestamp DESC);
```

**Relationships**: Many-to-one with ConversationSession, One-to-many with SourceCitations

---

### 4. SourceCitation (Neon Postgres)
Links assistant messages to source chunks.

**Schema**:
```sql
CREATE TABLE source_citations (
    citation_id UUID PRIMARY KEY,
    message_id UUID NOT NULL REFERENCES messages(message_id) ON DELETE CASCADE,
    chunk_id UUID NOT NULL,
    relevance_score FLOAT NOT NULL,
    excerpt TEXT NOT NULL,
    CONSTRAINT citations_message_fk FOREIGN KEY (message_id) REFERENCES messages(message_id)
);

CREATE INDEX idx_citations_message ON source_citations(message_id);
```

**Relationships**: Many-to-one with Message

---

## Entity Relationships

```
ConversationSession (1) --< (N) Message
Message (1) --< (N) SourceCitation
SourceCitation (N) --> (1) DocumentChunk (via chunk_id reference to Qdrant)
```

---

## Data Flow

1. **Indexing**: Markdown ’ DocumentChunks ’ Qdrant
2. **Query**: User query ’ Embedding ’ Vector search ’ DocumentChunks
3. **Response**: Chunks + Query ’ LLM ’ Message (with SourceCitations)
4. **Storage**: Message ’ Neon Postgres

---

## Validation Rules

- **DocumentChunk**: `token_count` must be 100-1500
- **Message**: `role` must be user|assistant|system
- **SourceCitation**: `relevance_score` must be 0.0-1.0
- **ConversationSession**: Auto-expire after 24 hours of inactivity (optional cleanup job)
