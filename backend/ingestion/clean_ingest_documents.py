import hashlib
import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
import frontmatter
import uuid

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CleanDocumentIngestor:
    def __init__(self, qdrant_url: str, qdrant_api_key: str, collection_name: str = "physical_ai_textbook"):
        # Initialize Qdrant client with cloud configuration
        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False  # Use HTTP for better compatibility
        )
        self.collection_name = collection_name

    def delete_existing_collection(self):
        """Delete the existing collection if it exists"""
        try:
            collections = self.qdrant_client.get_collections()
            exists = any(col.name == self.collection_name for col in collections.collections)

            if exists:
                logger.info(f"Deleting existing collection: {self.collection_name}")
                self.qdrant_client.delete_collection(self.collection_name)
                logger.info(f"Collection {self.collection_name} deleted successfully")
        except Exception as e:
            logger.warning(f"Could not delete collection: {e}")

    def initialize_collection(self):
        """Initialize the Qdrant collection with proper configuration"""
        try:
            # Create collection with 1536 dimensions for OpenAI embeddings
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1536,  # Standard size for OpenAI embeddings
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created new collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Collection creation error: {e}")
            raise

    def read_markdown_files(self, docs_dir: str) -> List[Dict[str, Any]]:
        """Read all markdown files from the documentation directory"""
        markdown_files = []
        docs_path = Path(docs_dir)

        for md_file in docs_path.rglob("*.md"):
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()

                # Extract frontmatter metadata if available
                try:
                    post = frontmatter.loads(content)
                    text_content = post.content
                    metadata = post.metadata
                except:
                    text_content = content
                    metadata = {}

                # Extract chapter information from file path
                relative_path = md_file.relative_to(docs_path)
                path_parts = str(relative_path).split(os.sep)

                chapter = path_parts[0] if len(path_parts) > 1 else "preface"
                filename = md_file.name
                section_title = metadata.get("title", self.extract_title_from_content(text_content))

                markdown_files.append({
                    "content": text_content,
                    "metadata": {
                        "chapter": chapter,
                        "filename": filename,
                        "section_title": section_title,
                        "source_path": str(relative_path),
                        "filepath": str(md_file),
                        "title": metadata.get("title", md_file.stem)
                    }
                })

                logger.info(f"Loaded file: {relative_path}")

            except Exception as e:
                logger.warning(f"Error reading file {md_file}: {e}")

        # Sort files to maintain order (preface first, then chapters in order)
        markdown_files.sort(key=lambda x: x["metadata"]["source_path"])

        return markdown_files

    def extract_title_from_content(self, content: str) -> str:
        """Extract the first heading as title if not in frontmatter"""
        lines = content.split('\n')
        for line in lines[:10]:  # Check first 10 lines
            if line.strip().startswith('# '):
                return line.strip('# ').strip()
        return "Untitled"

    def chunk_text(self, text: str, max_tokens: int = 600) -> List[str]:
        """Split text into chunks with overlap (500-700 tokens with 100 token overlap)"""
        # Simple token estimation: 1 token ~ 4 characters
        max_chars = max_tokens * 4
        overlap_chars = 100 * 4

        paragraphs = re.split(r'\n\s*\n', text)
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            if len(current_chunk) + len(paragraph) <= max_chars:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())

                # Add overlap by including part of the current paragraph in the previous chunk
                if len(paragraph) > overlap_chars:
                    # Start a new chunk with the overlap
                    overlap_start = len(paragraph) - overlap_chars
                    current_chunk = paragraph[overlap_start:]
                else:
                    current_chunk = paragraph

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Filter out very short chunks
        return [chunk for chunk in chunks if len(chunk.strip()) > 50]

    def get_embedding(self, text: str) -> List[float]:
        """Generate mock embedding for testing purposes (replace with actual embedding model in production)"""
        try:
            # Create a deterministic embedding based on content hash
            hash_bytes = hashlib.md5(text.encode()).digest()
            return [(hash_bytes[i % len(hash_bytes)] / 255.0) for i in range(1536)]
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            return [0.0] * 1536

    def process_and_ingest(self, docs_dir: str):
        """Process markdown files and ingest into Qdrant"""
        logger.info("Starting clean document ingestion process...")

        # Delete existing collection and create new one
        self.delete_existing_collection()
        self.initialize_collection()

        markdown_files = self.read_markdown_files(docs_dir)
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        points = []
        total_chunks = 0

        for file_data in markdown_files:
            logger.info(f"Processing file: {file_data['metadata']['source_path']}")

            # Create chunks with proper size and overlap
            chunks = self.chunk_text(file_data["content"], max_tokens=600)

            for idx, chunk in enumerate(chunks):
                embedding = self.get_embedding(chunk)

                point_metadata = {
                    **file_data["metadata"],
                    "chunk_index": idx,
                    "total_chunks": len(chunks),
                    "content_length": len(chunk),
                    "content": chunk[:2000],  # Store first 2000 chars of content
                    "full_content_hash": hashlib.sha256(chunk.encode()).hexdigest()
                }

                # Generate a unique ID for the point
                point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_data['metadata']['source_path']}_{idx}"))

                point = models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=point_metadata
                )

                points.append(point)
                total_chunks += 1

                # Batch insert every 100 points
                if len(points) >= 100:
                    self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    logger.info(f"Ingested batch of {len(points)} points")
                    points = []

        # Ingest remaining points
        if points:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Ingested final batch of {len(points)} points")

        logger.info(f"Total chunks ingested: {total_chunks}")

        # Create payload index for faster filtering
        try:
            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source_path",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chapter",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            logger.info("Created payload indexes for 'source_path' and 'chapter'")
        except Exception as e:
            logger.warning(f"Could not create payload index: {e}")

def main():
    from dotenv import load_dotenv
    load_dotenv()

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    docs_dir = os.getenv("DOCS_DIR", "../docs")  # Relative to backend directory
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("Missing QDRANT_URL or QDRANT_API_KEY environment variables")

    ingestor = CleanDocumentIngestor(qdrant_url, qdrant_api_key, collection_name)
    ingestor.process_and_ingest(docs_dir)

if __name__ == "__main__":
    main()