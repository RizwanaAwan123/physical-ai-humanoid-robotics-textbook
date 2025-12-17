# Physical AI & Humanoid Robotics - RAG System

## Overview
This project implements a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics textbook, using Docusaurus for the frontend and Qdrant for vector storage.

## Environment Setup

The system uses the following environment variables:

```bash
# Qdrant Configuration (Cloud)
QDRANT_URL=https://da6bd5f4-e31c-4e9c-a125-f65fe59082c0.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.B-Ffv-Vp32C5DW20bZjuD3IHQl2CEcNR-s00gr3Vtmk
QDRANT_COLLECTION_NAME=physical_ai_textbook

# Document Directory
DOCS_DIR=../docs

# OpenAI Configuration
GEMINI_API_KEY=AIzaSyD2Ux50ivdRE23u99P51Y9QhtGgD6uK-xg

# Neon Postgres Configuration
NEON_DATABASE_URL=postgresql://neondb_owner:npg_3O1gbBQxpUJX@ep-steep-wave-ahhomhvf-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# JWT Secret
JWT_SECRET=mysecretkey123
```

## Ingestion Process

To rebuild the vector store from scratch:

1. Make sure your environment variables are set in `.env`
2. Run the clean ingestion script:
```bash
cd backend
python ingestion/clean_ingest_documents.py
```

This will:
- Delete the existing collection
- Create a new collection
- Read all markdown files from `docs/`
- Chunk content with 500-700 token chunks and 100-token overlap
- Store with clean metadata (chapter, filename, section_title, source_path)
- Maintain proper order for navigation

## API Endpoints

- `POST /api/chat` - Chat endpoint that retrieves relevant textbook content

## File Structure

```
root/
 └── my-website/
     ├── docs/                 # Source markdown files
     ├── backend/
     │   └── ingestion/        # Ingestion scripts
     └── src/                  # Frontend components
```

## Frontend Integration

The Docusaurus site is configured to proxy API requests to the backend server. The chatbot component is integrated on the `/chat` page.

## Troubleshooting

If you encounter issues:
1. Verify environment variables are set correctly
2. Check that Qdrant collection exists and has data
3. Ensure backend server is running on port 8000
4. Verify frontend is running and proxy is configured