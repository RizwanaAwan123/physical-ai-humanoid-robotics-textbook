# Quickstart Guide: RAG Chatbot

**Feature**: 005-rag-chatbot | **Date**: 2025-12-10

## Prerequisites
- Python 3.11+
- Node.js 18+
- OpenAI API key
- Qdrant Cloud account
- Neon Serverless Postgres account

## Backend Setup

```bash
cd backend
python3.11 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env:
# OPENAI_API_KEY=sk-...
# QDRANT_URL=https://...qdrant.io
# QDRANT_API_KEY=...
# NEON_DATABASE_URL=postgresql://...

# Create database schema
python scripts/setup_db.py

# Index textbook content
python scripts/index_content.py --source ../docs/textbook

# Run server
uvicorn src.main:app --reload --port 8000
# Server runs at http://localhost:8000
```

## Frontend Integration

```bash
cd .. # to repo root
npm install

# Test chatbot widget locally
npm run start
# Visit http://localhost:3000, click chat button (bottom-right)
```

## Testing

```bash
# Backend tests
cd backend
pytest tests/ -v --cov=src

# Frontend tests
cd ..
npm test -- ChatWidget
```

## API Usage

```bash
# Health check
curl http://localhost:8000/health

# Submit query
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "query": "What is ROS 2?"
  }'
```

## Troubleshooting

**Import Error**: Check virtual environment is activated
**Qdrant Connection**: Verify `QDRANT_URL` and `QDRANT_API_KEY`
**OpenAI Rate Limit**: Check budget, implement caching
**Slow Responses**: Check vector search latency, optimize chunk size

## Next Steps
- Run `/sp.tasks` to generate implementation task list
- Review `plan.md` for architecture details
- Check `data-model.md` for database schema
