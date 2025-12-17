import hashlib
import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
import frontmatter

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentIngestor:
    def __init__(self, qdrant_url: str, qdrant_api_key: str):
        # Use in-memory Qdrant for local development without requiring a server
        from qdrant_client import QdrantClient
        if qdrant_url == "http://localhost:6333" and not qdrant_api_key:
            self.qdrant_client = QdrantClient(":memory:")
        else:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False
            )
        self.collection_name = "physical_ai_textbook"

    def initialize_collection(self):
        """Initialize the Qdrant collection with proper configuration"""
        try:
            collections = self.qdrant_client.get_collections()
            exists = any(col.name == self.collection_name for col in collections.collections)

            if not exists:
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created new collection: {self.collection_name}")
            else:
                logger.info(f"Collection already exists: {self.collection_name}")

        except Exception as e:
            logger.error(f"Collection error: {e}")
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

                metadata["filename"] = md_file.name
                metadata["filepath"] = str(md_file.relative_to(docs_path))
                metadata["title"] = metadata.get("title", md_file.stem)

                markdown_files.append({
                    "content": text_content,
                    "metadata": metadata,
                    "filepath": str(md_file)
                })

                logger.info(f"Loaded file: {md_file.relative_to(docs_path)}")

            except Exception as e:
                logger.warning(f"Error reading file {md_file}: {e}")

        return markdown_files

    def chunk_text(self, text: str, max_tokens: int = 1000) -> List[str]:
        """Split text into chunks (approximate token count)"""
        paragraphs = re.split(r'\n\s*\n', text)
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            if len(current_chunk) + len(paragraph) <= max_tokens * 4:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = paragraph

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Filter out very short chunks
        return [chunk for chunk in chunks if len(chunk.strip()) > 50]

    def get_embedding(self, text: str) -> List[float]:
        """Generate mock embedding (no external API call)"""
        try:
            hash_bytes = hashlib.md5(text.encode()).digest()
            return [(hash_bytes[i % len(hash_bytes)] / 255.0) for i in range(1536)]
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            return [0.0] * 1536

    def process_and_ingest(self, docs_dir: str):
        """Process markdown files and ingest into Qdrant"""
        logger.info("Starting document ingestion process...")

        self.initialize_collection()
        markdown_files = self.read_markdown_files(docs_dir)
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        points = []
        total_chunks = 0

        for file_data in markdown_files:
            logger.info(f"Processing file: {file_data['metadata']['filepath']}")
            chunks = self.chunk_text(file_data["content"])

            for idx, chunk in enumerate(chunks):
                embedding = self.get_embedding(chunk)

                point_metadata = {
                    **file_data["metadata"],
                    "chunk_index": idx,
                    "total_chunks": len(chunks),
                    "content_length": len(chunk),
                    "content": chunk,  # Add the actual content to the payload
                    "hash": hashlib.sha256(chunk.encode()).hexdigest()
                }

                # Generate a valid UUID for the point ID
                import uuid
                point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_data['metadata']['filepath']}_{idx}"))

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
        self.qdrant_client.create_payload_index(
            collection_name=self.collection_name,
            field_name="filepath",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        logger.info("Created payload index for 'filepath'")


def main():
    from dotenv import load_dotenv
    load_dotenv()

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    docs_dir = os.getenv("DOCS_DIR", "./docs")

    # Allow local setup with empty API key when using local URL
    if not qdrant_url:
        raise ValueError("Missing required environment variables")

    ingestor = DocumentIngestor(qdrant_url, qdrant_api_key)
    ingestor.process_and_ingest(docs_dir)


def load_documents_to_qdrant(qdrant_url: str, qdrant_api_key: str, collection_name="physical_ai_textbook", docs_dir="../../docs"):
    """Function to load documents to an existing Qdrant client instance"""
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    import uuid
    import hashlib
    import logging
    from pathlib import Path
    import frontmatter
    import re

    # Create a new client instance with the same configuration as the main class
    if qdrant_url == "http://localhost:6333" and not qdrant_api_key:
        qdrant_client = QdrantClient(":memory:")
    else:
        qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False
        )

    logger = logging.getLogger(__name__)

    # Initialize collection if it doesn't exist
    try:
        collections = qdrant_client.get_collections()
        exists = any(col.name == collection_name for col in collections.collections)

        if not exists:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=1536,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created new collection: {collection_name}")
        else:
            logger.info(f"Collection already exists: {collection_name}")
    except Exception as e:
        logger.error(f"Collection error: {e}")
        raise

    # Read markdown files
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

            metadata["filename"] = md_file.name
            metadata["filepath"] = str(md_file.relative_to(docs_path))
            metadata["title"] = metadata.get("title", md_file.stem)

            markdown_files.append({
                "content": text_content,
                "metadata": metadata,
                "filepath": str(md_file)
            })

        except Exception as e:
            logger.warning(f"Error reading file {md_file}: {e}")

    logger.info(f"Found {len(markdown_files)} markdown files to process")

    # Chunk text and create embeddings
    points = []
    total_chunks = 0

    for file_data in markdown_files:
        logger.info(f"Processing file: {file_data['metadata']['filepath']}")

        # Split text into chunks
        paragraphs = re.split(r'\n\s*\n', file_data["content"])
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            if len(current_chunk) + len(paragraph) <= 1000 * 4:  # approx 1000 tokens
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = paragraph

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Filter out very short chunks
        chunks = [chunk for chunk in chunks if len(chunk.strip()) > 50]

        for idx, chunk in enumerate(chunks):
            # Generate mock embedding
            try:
                hash_bytes = hashlib.md5(chunk.encode()).digest()
                embedding = [(hash_bytes[i % len(hash_bytes)] / 255.0) for i in range(1536)]
            except Exception as e:
                logger.error(f"Error generating embedding: {e}")
                embedding = [0.0] * 1536

            point_metadata = {
                **file_data["metadata"],
                "chunk_index": idx,
                "total_chunks": len(chunks),
                "content_length": len(chunk),
                "content": chunk,  # Add the actual content to the payload
                "hash": hashlib.sha256(chunk.encode()).hexdigest()
            }

            # Generate a valid UUID for the point ID
            point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_data['metadata']['filepath']}_{idx}"))

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload=point_metadata
            )

            points.append(point)
            total_chunks += 1

            # Batch insert every 100 points
            if len(points) >= 100:
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=points
                )
                logger.info(f"Ingested batch of {len(points)} points")
                points = []

    # Ingest remaining points
    if points:
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Ingested final batch of {len(points)} points")

    logger.info(f"Total chunks ingested: {total_chunks}")

    # Create payload index for faster filtering
    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="filepath",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        logger.info("Created payload index for 'filepath'")
    except Exception as e:
        logger.warning(f"Could not create payload index: {e} (this is normal for in-memory Qdrant)")


if __name__ == "__main__":
    main()