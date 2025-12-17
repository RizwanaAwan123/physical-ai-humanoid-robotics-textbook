import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = "physical_ai_textbook"

if qdrant_url == "http://localhost:6333" and not qdrant_api_key:
    qdrant_client = QdrantClient(":memory:")
else:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

try:
    collection_info = qdrant_client.get_collection(collection_name)
    print(f"Collection '{collection_name}' exists with {collection_info.points_count} points.")
except Exception as e:
    print(f"Error checking collection: {e}")
