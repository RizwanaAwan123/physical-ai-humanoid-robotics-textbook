import os
from dotenv import load_dotenv

load_dotenv()

neon_url = os.getenv("NEON_DATABASE_URL")
print(f"NEON_DATABASE_URL raw: {repr(neon_url)}")
print(f"NEON_DATABASE_URL: '{neon_url}'")
print(f"Length: {len(neon_url) if neon_url else 0}")
if neon_url:
    print(f"Starts with 'postgresql://' or 'postgres://': {neon_url.startswith(('postgresql://', 'postgres://'))}")

qdrant_url = os.getenv("QDRANT_URL")
print(f"QDRANT_URL: '{qdrant_url}'")

docs_dir = os.getenv("DOCS_DIR")
print(f"DOCS_DIR: '{docs_dir}'")