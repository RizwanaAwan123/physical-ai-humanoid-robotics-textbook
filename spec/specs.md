# SpecKit Specifications

## Project: AI Book Generator and RAG Chatbot

### Core Components

#### 1. Book Generation Service
```
Component: BookGenerator
Purpose: Generate comprehensive AI book content
Inputs: Topic, target audience, word count requirements
Outputs: Structured book content with chapters and sections
Dependencies: OpenAI API, Content templates
```

**Specification:**
- Generate books of 20,000-50,000 words
- Create 8-15 chapters with hierarchical structure
- Include consistent formatting and style
- Support multiple topic domains
- Validate content quality before completion

#### 2. Content Chunking System
```
Component: ContentChunker
Purpose: Split book content into searchable chunks
Inputs: Complete book content
Outputs: Array of content chunks with metadata
Dependencies: Text processing libraries
```

**Specification:**
- Chunk size: 300-500 words per segment
- Preserve semantic boundaries
- Include chapter/section metadata
- Maintain context coherence
- Generate unique identifiers for each chunk

#### 3. Qdrant Vector Storage
```
Component: QdrantManager
Purpose: Store and retrieve vector embeddings of book content
Inputs: Content chunks, query strings
Outputs: Similarity search results
Dependencies: Qdrant database, Embedding models
```

**Specification:**
- Vector dimension: 1536 (compatible with text-embedding-ada-002)
- Similarity metric: Cosine similarity
- Support for metadata filtering
- Batch ingestion capabilities
- Efficient similarity search (top-k retrieval)

#### 4. RAG Query Processor
```
Component: QueryProcessor
Purpose: Process user queries and retrieve relevant content
Inputs: User query, conversation context
Outputs: Retrieved context and generated response
Dependencies: QdrantManager, LLM service
```

**Specification:**
- Query understanding and expansion
- Multi-turn conversation support
- Context window management (4096 tokens)
- Response quality validation
- Source attribution in responses

### API Endpoints

#### Book Generation
```
POST /api/book/generate
Request: { topic: string, length: number, audience: string }
Response: { bookId: string, status: "processing" | "completed", content: BookStructure }
```

#### Content Search
```
GET /api/search
Request: { query: string, bookId: string, limit: number }
Response: { results: Array<SearchResult>, queryTime: number }
```

#### Chat Interface
```
POST /api/chat
Request: { message: string, bookId: string, sessionId: string }
Response: { response: string, sources: Array<Source>, timestamp: Date }
```

### Data Models

#### Book Structure
```json
{
  "id": "string",
  "title": "string",
  "topic": "string",
  "chapters": [
    {
      "id": "string",
      "title": "string",
      "sections": [
        {
          "id": "string",
          "title": "string",
          "content": "string",
          "wordCount": "number"
        }
      ]
    }
  ],
  "metadata": {
    "wordCount": "number",
    "generatedAt": "Date",
    "audience": "string"
  }
}
```

#### Content Chunk
```json
{
  "id": "string",
  "bookId": "string",
  "chapterId": "string",
  "sectionId": "string",
  "content": "string",
  "embedding": "Array<number>",
  "metadata": {
    "chapterTitle": "string",
    "sectionTitle": "string",
    "position": "number"
  }
}
```

#### Search Result
```json
{
  "chunkId": "string",
  "content": "string",
  "similarity": "number",
  "metadata": {
    "chapterTitle": "string",
    "sectionTitle": "string"
  }
}
```

### Configuration Requirements

#### Environment Variables
```
OPENAI_API_KEY: API key for OpenAI services
QDRANT_URL: URL for Qdrant vector database
QDRANT_API_KEY: Authentication key for Qdrant
EMBEDDING_MODEL: Name of the embedding model to use
LLM_MODEL: Name of the language model for responses
MAX_TOKENS: Maximum tokens for LLM responses
QUERY_TIMEOUT: Timeout for search operations
```

#### Performance Targets
- Query response time: < 3 seconds
- Content ingestion rate: 1000 chunks/minute
- Accuracy threshold: > 0.7 similarity for valid results
- Uptime: 99.5%
- Concurrent users: Support 100+ simultaneous sessions

### Quality Assurance

#### Content Quality
- Generated content must pass coherence checks
- Responses must cite specific book sections
- Factual accuracy validation against source material
- Style consistency across all generated content

#### System Quality
- 99.5% API endpoint availability
- < 100ms response time for cached queries
- Proper error handling and graceful degradation
- Comprehensive logging for debugging