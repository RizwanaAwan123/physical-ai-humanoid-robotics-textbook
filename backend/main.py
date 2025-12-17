

from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel
from typing import List, Dict, Optional, Any
from passlib.context import CryptContext
from jose import JWTError, jwt
from datetime import datetime, timedelta
import uuid
import asyncio
import asyncpg
import os
from dotenv import load_dotenv
import logging

# Optional Qdrant imports (if you plan to use Qdrant)
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models as qdrant_models
except Exception:
    QdrantClient = None

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# App
app = FastAPI(title="Physical AI & Humanoid Robotics RAG Chatbot", version="1.0.0")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # change in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------
# AUTH CONFIG
# ----------------------
SECRET_KEY = os.getenv("JWT_SECRET", "mysecretkey123")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/auth/login")
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Simple in-memory user store (replace with DB in production)
fake_users_db: Dict[str, Dict[str, Any]] = {}

def get_password_hash(password: str):
    return pwd_context.hash(password)

def verify_password(plain: str, hashed: str):
    return pwd_context.verify(plain, hashed)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(status_code=401, detail="Could not validate credentials")
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
    except JWTError:
        raise credentials_exception
    user = fake_users_db.get(username)
    if user is None:
        raise credentials_exception
    return {"username": username}

# ----------------------
# DB / Clients
# ----------------------
DB_POOL = None
QDRANT_CLIENT = None
OPENAI_CLIENT = None  # placeholder to avoid undefined variable in stats
COLLECTION_NAME = "physical_ai_textbook"

def initialize_clients():
    global QDRANT_CLIENT
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if QdrantClient:
        try:
            if qdrant_url:
                # Use cloud Qdrant with provided credentials
                QDRANT_CLIENT = QdrantClient(
                    url=qdrant_url,
                    api_key=qdrant_api_key,
                    prefer_grpc=False  # Use HTTP for cloud connection
                )
                logger.info("Qdrant client initialized with cloud configuration")
            else:
                # Use in-memory Qdrant for local development
                QDRANT_CLIENT = QdrantClient(":memory:")
                logger.info("Qdrant client initialized with in-memory configuration")
        except Exception as e:
            logger.error(f"Failed to init Qdrant client: {e}")
            QDRANT_CLIENT = None

async def get_db_pool():
    global DB_POOL
    if DB_POOL is None:
        neon_url = os.getenv("NEON_DATABASE_URL")
        # Clean up the neon_url to remove any potential whitespace
        if neon_url:
            neon_url = neon_url.strip()
        if not neon_url:
            # If no DB configured, raise so endpoints using DB will inform caller
            raise HTTPException(status_code=500, detail="NEON_DATABASE_URL not configured")
        DB_POOL = await asyncpg.create_pool(neon_url, min_size=1, max_size=10, command_timeout=60)
    return DB_POOL

# ----------------------
# Pydantic models
# ----------------------
class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    max_context_chunks: int = 5

class ChatResponse(BaseModel):
    response: str
    session_id: str
    context_chunks: List[Dict[str, Any]]

class ChatHistoryRequest(BaseModel):
    session_id: str

class ChatHistoryResponse(BaseModel):
    messages: List[ChatMessage]
    session_id: str

class NewChatRequest(BaseModel):
    title: Optional[str] = "New Chat"

class NewChatResponse(BaseModel):
    session_id: str
    title: str

class ChatSession(BaseModel):
    id: str
    title: str
    created_at: datetime

class ChatSessionsResponse(BaseModel):
    sessions: List[ChatSession]

# ----------------------
# Real search/generation using Qdrant
# ----------------------
async def get_mock_embedding(text: str) -> List[float]:
    """Get mock embedding matching the ingestion script"""
    import hashlib
    try:
        hash_obj = hashlib.md5(text.encode())
        hash_bytes = hash_obj.digest()
        embedding = []
        for i in range(1536):
            byte_idx = i % len(hash_bytes)
            embedding.append(float(hash_bytes[byte_idx]) / 255.0)
        return embedding
    except Exception as e:
        logger.error(f"Error getting mock embeddings: {e}")
        return [0.0] * 1536


def get_simple_embedding(text: str) -> List[float]:
    """Simple embedding function using TF-IDF-like approach"""
    import hashlib
    import math

    # Create a simple embedding based on character n-grams
    text_lower = text.lower()
    embedding = [0.0] * 1536

    # Add character-level features
    for i, char in enumerate(text_lower[:500]):  # Limit to first 500 chars
        hash_val = int(hashlib.md5((char + str(i)).encode()).hexdigest(), 16)
        idx = hash_val % 1536
        embedding[idx] += 1.0

    # Add word-level features
    words = text_lower.split()
    for i, word in enumerate(words[:100]):  # Limit to first 100 words
        hash_val = int(hashlib.md5((word + str(i)).encode()).hexdigest(), 16)
        idx = hash_val % 1536
        embedding[idx] += 2.0

    # Normalize the embedding
    norm = math.sqrt(sum(x * x for x in embedding))
    if norm > 0:
        embedding = [x / norm for x in embedding]

    return embedding


async def get_embedding(text: str) -> List[float]:
    """Get embedding for text - using local implementation"""
    return get_simple_embedding(text)

async def search_documents(query: str, limit: int = 5) -> List[Dict[str, Any]]:
    """Search documents using Qdrant vector database"""
    if not QDRANT_CLIENT:
        logger.warning("Qdrant client not initialized, falling back to mock search")
        # Fallback mock responses
        mock_responses = [
            {
                'content': "Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world.",
                'metadata': {'filename': 'physical_ai_intro.md', 'filepath': 'docs/chapter1/physical_ai_intro.md'},
                'score': 0.95
            },
            {
                'content': "Humanoid robotics focuses on creating robots with human-like form and capabilities, including bipedal locomotion and dexterous manipulation.",
                'metadata': {'filename': 'humanoid_robotics.md', 'filepath': 'docs/chapter1/humanoid_robotics.md'},
                'score': 0.90
            },
            {
                'content': "Kinematics studies the motion of robotic systems without considering forces. Forward kinematics determines end-effector position from joint angles.",
                'metadata': {'filename': 'kinematics.md', 'filepath': 'docs/chapter3/kinematics.md'},
                'score': 0.85
            },
        ]
        query_lower = query.lower()
        filtered = [r for r in mock_responses if any(k in r['content'].lower() for k in query_lower.split())]
        return filtered[:limit] if filtered else mock_responses[:limit]

    try:
        # Get embedding for the query using the new function
        query_embedding = await get_embedding(query)

        # Search in Qdrant
        search_result = QDRANT_CLIENT.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True,
            with_vectors=False,
            score_threshold=0.1  # Only return results with minimum relevance
        )

        # Format results
        results = []
        for hit in search_result:
            payload = hit.payload
            # Extract content from payload - assuming it's stored in the payload
            content = payload.get('content', '')
            if not content and 'text' in payload:
                content = payload['text']

            result = {
                'content': content,
                'metadata': {k: v for k, v in payload.items() if k != 'content' and k != 'text'},
                'score': hit.score
            }
            results.append(result)

        logger.info(f"Found {len(results)} documents for query: {query}")
        return results

    except Exception as e:
        logger.error(f"Error searching documents: {e}")
        # Fallback to mock search
        mock_responses = [
            {
                'content': "Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world.",
                'metadata': {'filename': 'physical_ai_intro.md', 'filepath': 'docs/chapter1/physical_ai_intro.md'},
                'score': 0.95
            }
        ]
        return mock_responses[:limit]

async def generate_response(query: str, context: str) -> str:
    """Generate a response based on the query and context using simple text processing"""
    import re

    # If we have context, try to extract relevant information
    if context.strip():
        # Clean up the context to extract the most relevant parts
        context_parts = context.split('\n\n')  # Split by paragraphs
        relevant_parts = []

        # Look for parts that are most relevant to the query
        query_lower = query.lower()
        for part in context_parts:
            if len(part.strip()) > 10:  # Skip very short parts
                # Score based on keyword matches
                score = sum(1 for word in query_lower.split() if word in part.lower())
                if score > 0 or len(relevant_parts) == 0:  # Include first part or if it matches
                    relevant_parts.append((score, part))

        # Sort by relevance and take top parts
        relevant_parts.sort(key=lambda x: x[0], reverse=True)
        top_context = ' '.join([part[1] for part in relevant_parts[:2]])  # Take top 2 relevant parts

        # Generate a response based on the context
        if top_context:
            # Extract key information from context
            sentences = re.split(r'[.!?]+', top_context)
            relevant_sentences = []

            for sentence in sentences:
                if any(word in sentence.lower() for word in query_lower.split()):
                    relevant_sentences.append(sentence.strip())

            if relevant_sentences:
                response = f"Based on the textbook: {'. '.join(relevant_sentences[:2])}"
            else:
                response = f"Based on the textbook: {top_context[:500]}..."  # Truncate if too long
        else:
            response = f"According to the textbook: {context[:300]}..."
    else:
        # Fallback if no context is provided
        q = query.lower()
        if "physical ai" in q or "physical" in q:
            response = "Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world. Unlike traditional AI that operates purely in digital domains, Physical AI is embodied and interacts directly with the physical environment."
        elif "humanoid" in q:
            response = "Humanoid robotics focuses on creating robots with human-like form and capabilities, including bipedal locomotion and dexterous manipulation. These robots are designed to operate in human environments and interact with humans effectively."
        elif "kinematics" in q:
            response = "Kinematics studies the motion of robotic systems without considering forces. Forward kinematics determines end-effector position from joint angles, while inverse kinematics solves for joint angles needed to achieve a desired end-effector position."
        else:
            response = f"I found some information in the textbook related to your query: '{query}'. Please check the chatbot interface for the specific context snippets."

    return response

# ----------------------
# DB helpers (these expect a table schema in your DB)
# ----------------------
async def create_session(db_pool) -> str:
    async with db_pool.acquire() as conn:
        # Attempt to insert a new session - adjust SQL to your schema
        row = await conn.fetchrow("INSERT INTO chat_sessions DEFAULT VALUES RETURNING id")
        return str(row['id'])

async def update_session_title(db_pool, session_id: str, title: str):
    async with db_pool.acquire() as conn:
        await conn.execute("UPDATE chat_sessions SET title = $1 WHERE id = $2", title, session_id)

async def save_message(db_pool, session_id: str, role: str, content: str, context_chunks: Optional[List[Dict[str, Any]]] = None):
    async with db_pool.acquire() as conn:
        await conn.execute(
            """
            INSERT INTO chat_messages (session_id, role, content, context_chunks)
            VALUES ($1, $2, $3, $4)
            """,
            session_id, role, content, context_chunks
        )

async def get_chat_history(db_pool, session_id: str) -> List[Dict[str, Any]]:
    async with db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT role, content, created_at, context_chunks
            FROM chat_messages
            WHERE session_id = $1
            ORDER BY created_at ASC
            """,
            session_id
        )
        return [
            {
                'role': row['role'],
                'content': row['content'],
                'created_at': row['created_at'],
                'context_chunks': row['context_chunks']
            }
            for row in rows
        ]

async def get_chat_sessions(db_pool) -> List[Dict[str, Any]]:
    async with db_pool.acquire() as conn:
        rows = await conn.fetch(
            """
            SELECT id, title, created_at
            FROM chat_sessions
            ORDER BY updated_at DESC
            LIMIT 50
            """
        )
        return [
            {'id': str(r['id']), 'title': r['title'], 'created_at': r['created_at']}
            for r in rows
        ]

# ----------------------
# Startup / Shutdown
# ----------------------
@app.on_event("startup")
async def startup_event():
    global DB_POOL
    initialize_clients()

    # Always attempt to ingest documents when QDRANT_CLIENT is initialized
    if QDRANT_CLIENT:
        logger.info("Qdrant client initialized, loading documents...")
        try:
            # Import and run the ingestion function to populate the database
            from ingestion.ingest_documents import load_documents_to_qdrant
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")
            docs_dir = os.getenv("DOCS_DIR", "./docs")
            load_documents_to_qdrant(qdrant_url, qdrant_api_key, COLLECTION_NAME, docs_dir=docs_dir)
        except Exception as e:
            logger.error(f"Error loading documents to Qdrant: {e}")

    neon_url = os.getenv("NEON_DATABASE_URL")
    if not neon_url:
        logger.warning("NEON_DATABASE_URL not configured - DB features disabled for now")
        return
    try:
        DB_POOL = await asyncpg.create_pool(neon_url, min_size=1, max_size=10, command_timeout=60)
        logger.info("DB pool initialized")
    except Exception as e:
        logger.error(f"Failed to initialize DB pool: {e}")

@app.on_event("shutdown")
async def shutdown_event():
    global DB_POOL
    if DB_POOL:
        await DB_POOL.close()
        logger.info("DB pool closed")

# ----------------------
# Health & Auth endpoints
# ----------------------
@app.get("/health")
async def health_check():
    return {"status": "healthy", "qdrant_client": QDRANT_CLIENT is not None, "openai_client": OPENAI_CLIENT is not None}

# Public route to check auth status OR to return a guest user when no token
@app.get("/api/auth")
async def api_auth_optional(request: Request, token: Optional[str] = Depends(oauth2_scheme)):
    # If token given and valid -> return user
    try:
        if token:
            payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
            user = payload.get("sub")
            return {"authenticated": True, "user": user}
    except Exception:
        pass
    # Fallback guest user
    return {"authenticated": False, "user": {"id": "guest", "name": "Guest User"}}

@app.post("/api/auth/signup")
async def signup(form: OAuth2PasswordRequestForm = Depends()):
    if form.username in fake_users_db:
        raise HTTPException(status_code=400, detail="User already exists")
    fake_users_db[form.username] = {"username": form.username, "password": get_password_hash(form.password)}
    return {"success": True, "message": "User created"}

@app.post("/api/auth/login")
async def login(form: OAuth2PasswordRequestForm = Depends()):
    user = fake_users_db.get(form.username)
    if not user or not verify_password(form.password, user["password"]):
        raise HTTPException(status_code=401, detail="Invalid credentials")
    token = create_access_token({"sub": form.username})
    return {"access_token": token, "token_type": "bearer"}

@app.get("/api/protected")
async def protected_route(current_user: dict = Depends(get_current_user)):
    return {"message": "You are authenticated", "user": current_user}

# ----------------------
# Chat & RAG endpoints
# ----------------------
@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        # If DB not configured, create local session id
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        if not request.session_id:
            if db_pool:
                session_id = await create_session(db_pool)
            else:
                session_id = str(uuid.uuid4())
        else:
            session_id = request.session_id

        # Get context chunks
        context_chunks = await search_documents(request.message, request.max_context_chunks)
        if not context_chunks:
            response_text = "Not found in textbook"
            context_chunks = []
        else:
            context_text = "\n\n".join([c['content'] for c in context_chunks])
            response_text = await generate_response(request.message, context_text)

        # Save messages if DB available
        if db_pool:
            try:
                await save_message(db_pool, session_id, "user", request.message, None)
                await save_message(db_pool, session_id, "assistant", response_text, context_chunks)

                # update title for new sessions
                history = await get_chat_history(db_pool, session_id)
                if len(history) <= 2:
                    title = request.message.strip()[:50] or "New Chat"
                    await update_session_title(db_pool, session_id, title)
            except Exception as e:
                logger.warning(f"Could not save message to database: {e}. Continuing without DB storage.")
                # Continue without database storage - this is non-critical for RAG functionality

        return ChatResponse(response=response_text, session_id=session_id, context_chunks=context_chunks)

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/chat/new", response_model=NewChatResponse)
async def create_new_chat(request: NewChatRequest):
    try:
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        if db_pool:
            session_id = await create_session(db_pool)
            if request.title:
                await update_session_title(db_pool, session_id, request.title)
        else:
            session_id = str(uuid.uuid4())

        return NewChatResponse(session_id=session_id, title=request.title or "New Chat")
    except Exception as e:
        logger.error(f"Error creating new chat: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/chat/history", response_model=ChatHistoryResponse)
async def get_history(session_id: str):
    try:
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        if not db_pool:
            raise HTTPException(status_code=500, detail="DB not configured")

        history = await get_chat_history(db_pool, session_id)
        messages = [ChatMessage(role=item['role'], content=item['content']) for item in history]
        return ChatHistoryResponse(messages=messages, session_id=session_id)
    except Exception as e:
        logger.error(f"Error getting chat history: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/chat/sessions", response_model=ChatSessionsResponse)
async def get_sessions():
    try:
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        if not db_pool:
            raise HTTPException(status_code=500, detail="DB not configured")

        sessions = await get_chat_sessions(db_pool)
        return ChatSessionsResponse(sessions=sessions)
    except Exception as e:
        logger.error(f"Error getting sessions: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.delete("/api/chat/session/{session_id}")
async def delete_session(session_id: str):
    try:
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        if not db_pool:
            raise HTTPException(status_code=500, detail="DB not configured")

        async with db_pool.acquire() as conn:
            await conn.execute("DELETE FROM chat_messages WHERE session_id = $1", session_id)
            await conn.execute("DELETE FROM chat_sessions WHERE id = $1", session_id)

        return {"status": "deleted", "session_id": session_id}
    except Exception as e:
        logger.error(f"Error deleting session: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ----------------------
# Stats endpoint
# ----------------------
@app.get("/stats")
async def get_stats():
    try:
        try:
            db_pool = await get_db_pool()
        except HTTPException:
            db_pool = None

        doc_count = "Qdrant not configured"
        if QDRANT_CLIENT:
            try:
                info = QDRANT_CLIENT.get_collection(COLLECTION_NAME)
                doc_count = getattr(info, "points_count", "unknown")
            except Exception as e:
                doc_count = f"Unable to connect to Qdrant: {e}"

        total_sessions = 0
        total_messages = 0
        if db_pool:
            try:
                async with db_pool.acquire() as conn:
                    total_sessions = await conn.fetchval("SELECT COUNT(*) FROM chat_sessions")
                    total_messages = await conn.fetchval("SELECT COUNT(*) FROM chat_messages")
            except Exception as e:
                logger.warning(f"Could not fetch DB stats: {e}. Database tables may not exist.")
                total_sessions = "DB error"
                total_messages = "DB error"

        return {
            "documents_in_vector_db": doc_count,
            "total_chat_sessions": total_sessions,
            "total_messages": total_messages,
            "collection_name": COLLECTION_NAME,
            "qdrant_configured": QDRANT_CLIENT is not None,
            "openai_configured": OPENAI_CLIENT is not None
        }
    except Exception as e:
        logger.error(f"Error getting stats: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ----------------------
# Run
# ----------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))
