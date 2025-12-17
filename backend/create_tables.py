import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def create_tables():
    neon_url = os.getenv("NEON_DATABASE_URL")
    if not neon_url:
        print("NEON_DATABASE_URL not configured")
        return

    # Connect to the database
    conn = await asyncpg.connect(neon_url)

    try:
        # Create chat_sessions table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                title VARCHAR(255) DEFAULT 'New Chat',
                created_at TIMESTAMP DEFAULT NOW(),
                updated_at TIMESTAMP DEFAULT NOW()
            )
        """)
        print("Created chat_sessions table")

        # Create chat_messages table
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_messages (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
                role VARCHAR(50) NOT NULL,
                content TEXT NOT NULL,
                context_chunks JSONB,
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)
        print("Created chat_messages table")

        print("Tables created successfully!")

    except Exception as e:
        print(f"Error creating tables: {e}")
    finally:
        await conn.close()

if __name__ == "__main__":
    import asyncio
    asyncio.run(create_tables())