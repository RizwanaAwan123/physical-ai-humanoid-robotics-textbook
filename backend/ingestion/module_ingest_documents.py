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

class ModuleDocumentIngestor:
    def __init__(self, qdrant_url: str, qdrant_api_key: str):
        # Use in-memory Qdrant for local development without requiring a server
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
                # Clear existing collection to start fresh
                self.qdrant_client.delete_collection(self.collection_name)
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Recreated collection: {self.collection_name}")

        except Exception as e:
            logger.error(f"Collection error: {e}")
            raise

    def read_markdown_files(self, docs_dir: str) -> List[Dict[str, Any]]:
        """Read all markdown files from the documentation directory"""
        markdown_files = []
        docs_path = Path(docs_dir)

        for md_file in docs_path.rglob("*.md"):
            # Skip old chapter files, only process new module files
            if 'chapter' in str(md_file).lower():
                logger.info(f"Skipping old chapter file: {md_file}")
                continue

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

                # Determine module based on file path
                filepath_str = str(md_file.relative_to(docs_path))
                if 'module1' in filepath_str:
                    metadata["module"] = "Module 1: The Robotic Nervous System"
                elif 'module2' in filepath_str:
                    metadata["module"] = "Module 2: The Digital Twin"
                elif 'module3' in filepath_str:
                    metadata["module"] = "Module 3: The AI-Robot Brain"
                elif 'module4' in filepath_str:
                    metadata["module"] = "Module 4: Vision-Language-Action"
                elif 'capstone' in filepath_str:
                    metadata["module"] = "Capstone Project: The Autonomous Humanoid"
                else:
                    metadata["module"] = "General"

                markdown_files.append({
                    "content": text_content,
                    "metadata": metadata,
                    "filepath": str(md_file)
                })

                logger.info(f"Loaded file: {md_file.relative_to(docs_path)}")

            except Exception as e:
                logger.warning(f"Error reading file {md_file}: {e}")

        return markdown_files

    def chunk_text_semantically(self, text: str, max_tokens: int = 600) -> List[Dict[str, str]]:
        """Split text into semantically meaningful chunks"""
        # Split by headers first to maintain semantic boundaries
        header_split_pattern = r'(#+\s+.+?\n(?:.|\n)*?)(?=\n#+\s+|$)'
        sections = re.split(header_split_pattern, text)

        chunks = []
        current_section = ""

        for section in sections:
            if not section.strip():
                continue

            # If this is a header section, process it
            if re.match(r'^#+\s+', section.strip()):
                # If we have accumulated content, save it as a chunk
                if current_section.strip():
                    if len(current_section.strip()) > 100:  # Only create chunks with substantial content
                        chunks.append({
                            "content": current_section.strip(),
                            "section_title": self.extract_section_title(current_section)
                        })
                    current_section = ""

                # Add this header section to current
                current_section += section
            else:
                # Regular content, add to current section
                if len(current_section + section) < max_tokens * 4:  # Rough token estimation
                    current_section += section
                else:
                    # Save current chunk and start new one
                    if current_section.strip():
                        if len(current_section.strip()) > 100:
                            chunks.append({
                                "content": current_section.strip(),
                                "section_title": self.extract_section_title(current_section)
                            })
                    current_section = section

        # Add the last section if it exists
        if current_section.strip():
            if len(current_section.strip()) > 100:
                chunks.append({
                    "content": current_section.strip(),
                    "section_title": self.extract_section_title(current_section)
                })

        # If no header-based chunks, fall back to paragraph-based
        if not chunks:
            paragraphs = re.split(r'\n\s*\n', text)
            current_chunk = ""

            for paragraph in paragraphs:
                if len(current_chunk) + len(paragraph) <= max_tokens * 4:
                    current_chunk += "\n\n" + paragraph if current_chunk else paragraph
                else:
                    if current_chunk.strip() and len(current_chunk.strip()) > 100:
                        chunks.append({
                            "content": current_chunk.strip(),
                            "section_title": self.extract_section_title(current_chunk)
                        })
                    current_chunk = paragraph

            if current_chunk.strip() and len(current_chunk.strip()) > 100:
                chunks.append({
                    "content": current_chunk.strip(),
                    "section_title": self.extract_section_title(current_chunk)
                })

        return chunks

    def extract_section_title(self, text: str) -> str:
        """Extract the main title from a text chunk"""
        # Look for the first header in the text
        header_match = re.search(r'^#+\s+(.+)', text, re.MULTILINE)
        if header_match:
            return header_match.group(1).strip()

        # If no header, take the first line or first sentence
        lines = text.strip().split('\n')
        for line in lines:
            line = line.strip()
            if line and not line.startswith('#'):
                # Take first sentence (up to first period)
                first_sentence = line.split('.')[0] + '.'
                return first_sentence[:100]  # Limit length

        return "General Content"

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
        logger.info("Starting module-based document ingestion process...")
        logger.info("This will remove old chapter-based content and add new module-based content")

        self.initialize_collection()
        markdown_files = self.read_markdown_files(docs_dir)
        logger.info(f"Found {len(markdown_files)} module-based markdown files to process")

        points = []
        total_chunks = 0

        for file_data in markdown_files:
            logger.info(f"Processing file: {file_data['metadata']['filepath']}")
            chunks = self.chunk_text_semantically(file_data["content"])

            for idx, chunk_data in enumerate(chunks):
                chunk_content = chunk_data["content"]
                embedding = self.get_embedding(chunk_content)

                point_metadata = {
                    **file_data["metadata"],
                    "chunk_index": idx,
                    "total_chunks": len(chunks),
                    "content_length": len(chunk_content),
                    "content": chunk_content,  # Add the actual content to the payload
                    "hash": hashlib.sha256(chunk_content.encode()).hexdigest(),
                    "section_title": chunk_data["section_title"],
                    "module_name": file_data["metadata"]["module"]
                }

                # Generate a valid UUID for the point ID
                point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_data['metadata']['filepath']}_{idx}_{len(chunk_content)}"))

                point = models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=point_metadata
                )

                points.append(point)
                total_chunks += 1

                # Batch insert every 50 points to avoid memory issues
                if len(points) >= 50:
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

        # Verify the ingestion
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Final collection info: {collection_info}")
            logger.info(f"Total vectors in collection: {collection_info.points_count}")
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")

        # Create payload index for faster filtering
        try:
            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="filepath",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            logger.info("Created payload index for 'filepath'")

            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="module_name",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            logger.info("Created payload index for 'module_name'")
        except Exception as e:
            logger.warning(f"Could not create payload index: {e} (this is normal for in-memory Qdrant)")

def load_module_documents_to_qdrant(qdrant_url: str, qdrant_api_key: str, collection_name="physical_ai_textbook", docs_dir="../../docs"):
    """Function to load module-based documents to an existing Qdrant client instance"""
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

    # Initialize collection (clearing old content)
    try:
        collections = qdrant_client.get_collections()
        exists = any(col.name == collection_name for col in collections.collections)

        if exists:
            qdrant_client.delete_collection(collection_name)
            logger.info(f"Deleted existing collection: {collection_name}")

        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1536,
                distance=models.Distance.COSINE
            )
        )
        logger.info(f"Created new collection: {collection_name}")
    except Exception as e:
        logger.error(f"Collection error: {e}")
        raise

    # Read markdown files (only module-based files, not old chapters)
    markdown_files = []
    docs_path = Path(docs_dir)

    for md_file in docs_path.rglob("*.md"):
        # Skip old chapter files, only process new module files
        if 'chapter' in str(md_file).lower():
            logger.info(f"Skipping old chapter file: {md_file}")
            continue

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

            # Determine module based on file path
            filepath_str = str(md_file.relative_to(docs_path))
            if 'module1' in filepath_str:
                metadata["module"] = "Module 1: The Robotic Nervous System"
            elif 'module2' in filepath_str:
                metadata["module"] = "Module 2: The Digital Twin"
            elif 'module3' in filepath_str:
                metadata["module"] = "Module 3: The AI-Robot Brain"
            elif 'module4' in filepath_str:
                metadata["module"] = "Module 4: Vision-Language-Action"
            elif 'capstone' in filepath_str:
                metadata["module"] = "Capstone Project: The Autonomous Humanoid"
            else:
                metadata["module"] = "General"

            markdown_files.append({
                "content": text_content,
                "metadata": metadata,
                "filepath": str(md_file)
            })

        except Exception as e:
            logger.warning(f"Error reading file {md_file}: {e}")

    logger.info(f"Found {len(markdown_files)} module-based markdown files to process")

    # Process each file with semantic chunking
    points = []
    total_chunks = 0

    for file_data in markdown_files:
        logger.info(f"Processing file: {file_data['metadata']['filepath']}")

        # Split content semantically
        header_split_pattern = r'(#+\s+.+?\n(?:.|\n)*?)(?=\n#+\s+|$)'
        sections = re.split(header_split_pattern, file_data["content"])

        chunks = []
        current_section = ""

        for section in sections:
            if not section.strip():
                continue

            if re.match(r'^#+\s+', section.strip()):
                if current_section.strip():
                    if len(current_section.strip()) > 100:
                        chunks.append({
                            "content": current_section.strip(),
                            "section_title": extract_section_title(current_section)
                        })
                    current_section = ""

                current_section += section
            else:
                current_section += section

        if current_section.strip():
            if len(current_section.strip()) > 100:
                chunks.append({
                    "content": current_section.strip(),
                    "section_title": extract_section_title(current_section)
                })

        # If no header-based chunks, fall back to paragraph-based
        if not chunks:
            paragraphs = re.split(r'\n\s*\n', file_data["content"])
            current_chunk = ""

            for paragraph in paragraphs:
                if len(current_chunk) + len(paragraph) <= 600 * 4:
                    current_chunk += "\n\n" + paragraph if current_chunk else paragraph
                else:
                    if current_chunk.strip() and len(current_chunk.strip()) > 100:
                        chunks.append({
                            "content": current_chunk.strip(),
                            "section_title": extract_section_title(current_chunk)
                        })
                    current_chunk = paragraph

            if current_chunk.strip() and len(current_chunk.strip()) > 100:
                chunks.append({
                    "content": current_chunk.strip(),
                    "section_title": extract_section_title(current_chunk)
                })

        for idx, chunk_data in enumerate(chunks):
            chunk_content = chunk_data["content"]

            # Generate mock embedding
            try:
                hash_bytes = hashlib.md5(chunk_content.encode()).digest()
                embedding = [(hash_bytes[i % len(hash_bytes)] / 255.0) for i in range(1536)]
            except Exception as e:
                logger.error(f"Error generating embedding: {e}")
                embedding = [0.0] * 1536

            point_metadata = {
                **file_data["metadata"],
                "chunk_index": idx,
                "total_chunks": len(chunks),
                "content_length": len(chunk_content),
                "content": chunk_content,  # Add the actual content to the payload
                "hash": hashlib.sha256(chunk_content.encode()).hexdigest(),
                "section_title": chunk_data["section_title"],
                "module_name": file_data["metadata"]["module"]
            }

            # Generate a valid UUID for the point ID
            point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_data['metadata']['filepath']}_{idx}_{len(chunk_content)}"))

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload=point_metadata
            )

            points.append(point)
            total_chunks += 1

            # Batch insert every 50 points
            if len(points) >= 50:
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

    # Create payload indexes
    try:
        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="filepath",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        logger.info("Created payload index for 'filepath'")

        qdrant_client.create_payload_index(
            collection_name=collection_name,
            field_name="module_name",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        logger.info("Created payload index for 'module_name'")
    except Exception as e:
        logger.warning(f"Could not create payload index: {e} (this is normal for in-memory Qdrant)")

def extract_section_title(text: str) -> str:
    """Extract the main title from a text chunk"""
    # Look for the first header in the text
    header_match = re.search(r'^#+\s+(.+)', text, re.MULTILINE)
    if header_match:
        return header_match.group(1).strip()

    # If no header, take the first line or first sentence
    lines = text.strip().split('\n')
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#'):
            # Take first sentence (up to first period)
            first_sentence = line.split('.')[0] + '.'
            return first_sentence[:100]  # Limit length

    return "General Content"

if __name__ == "__main__":
    from dotenv import load_dotenv
    load_dotenv()

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    docs_dir = os.getenv("DOCS_DIR", "./docs")

    if not qdrant_url:
        # Use in-memory for local development
        qdrant_url = "http://localhost:6333"
        qdrant_api_key = None

    ingestor = ModuleDocumentIngestor(qdrant_url, qdrant_api_key)
    ingestor.process_and_ingest(docs_dir)