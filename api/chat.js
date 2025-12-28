// Vercel Serverless Function for Chat API with RAG functionality
// Dynamically import QdrantClient to handle cases where it's not available
let QdrantClient;

try {
  ({ QdrantClient } = require('@qdrant/js-client-rest'));
} catch (error) {
  console.log('Qdrant client not available, using mock responses');
}

// Simple embedding function using TF-IDF-like approach
function getSimpleEmbedding(text) {
  const crypto = typeof require !== 'undefined' ? require('crypto') : undefined;
  if (!crypto && typeof window !== 'undefined') {
    // Browser environment - use Web Crypto API
    crypto = {
      createHash: function(str) {
        // Simple fallback for browser environment
        let hash = 0;
        for (let i = 0; i < str.length; i++) {
          const char = str.charCodeAt(i);
          hash = ((hash << 5) - hash) + char;
          hash = hash & hash; // Convert to 32-bit integer
        }
        return {
          update: function() { return this; },
          digest: function() {
            const result = new Uint8Array(16);
            const hashStr = Math.abs(hash).toString();
            for (let i = 0; i < Math.min(hashStr.length, 16); i++) {
              result[i] = parseInt(hashStr[i], 10);
            }
            return result;
          }
        };
      }
    };
  }

  const textLower = text.toLowerCase();
  const embedding = new Array(1536).fill(0.0);

  // Add character-level features
  for (let i = 0; i < Math.min(textLower.length, 500); i++) { // Limit to first 500 chars
    const char = textLower[i];
    const hash = crypto.createHash('md5').update(char + i.toString()).digest();
    const idx = hash[0] % 1536;
    embedding[idx] += 1.0;
  }

  // Add word-level features
  const words = textLower.split(/\s+/);
  for (let i = 0; i < Math.min(words.length, 100); i++) { // Limit to first 100 words
    const word = words[i];
    const hash = crypto.createHash('md5').update(word + i.toString()).digest();
    const idx = hash[0] % 1536;
    embedding[idx] += 2.0;
  }

  // Normalize the embedding
  const norm = Math.sqrt(embedding.reduce((sum, x) => sum + x * x, 0));
  if (norm > 0) {
    for (let i = 0; i < embedding.length; i++) {
      embedding[i] /= norm;
    }
  }

  return embedding;
}

// Cosine similarity function
function cosineSimilarity(vecA, vecB) {
  if (vecA.length !== vecB.length) return 0;

  let dotProduct = 0;
  let normA = 0;
  let normB = 0;

  for (let i = 0; i < vecA.length; i++) {
    dotProduct += vecA[i] * vecB[i];
    normA += vecA[i] * vecA[i];
    normB += vecB[i] * vecB[i];
  }

  return dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
}

// Search documents function (simplified for Vercel)
async function searchDocuments(query, limit = 5) {
  try {
    // Get embedding for the query
    const queryEmbedding = getSimpleEmbedding(query);

    // Check if QdrantClient is available
    if (QdrantClient) {
      const qdrantUrl = process.env.QDRANT_URL;
      const qdrantApiKey = process.env.QDRANT_API_KEY;

      if (qdrantUrl && qdrantApiKey) {
        // Use actual Qdrant client
        const client = new QdrantClient({
          url: qdrantUrl,
          apiKey: qdrantApiKey,
        });

        const results = await client.search("physical_ai_textbook", {
          vector: queryEmbedding,
          limit: limit,
          with_payload: true,
        });

        return results.map(hit => ({
          content: hit.payload.content || '',
          metadata: { ...hit.payload, id: hit.id },
          score: hit.score
        }));
      }
    }

    // Fallback mock responses
    const mockResponses = [
      {
        content: "Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world. This comprehensive curriculum explores how AI can live in the real world and understand physical laws. Students will design, simulate, and deploy humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac.",
        metadata: { filename: 'textbook-intro.md', filepath: 'docs/intro.md' },
        score: 0.95
      },
      {
        content: "The Robotic Nervous System refers to the core architecture of a robot that handles communication, control, and coordination. In the context of ROS 2 (Robot Operating System 2), it includes Nodes, Topics, Services, Actions, and the underlying communication infrastructure that allows different parts of a robot to work together.",
        metadata: { filename: 'robotic-nervous-system.md', filepath: 'docs/module1/robotic-nervous-system.md' },
        score: 0.90
      },
      {
        content: "Humanoid robotics focuses on creating robots with human-like form and capabilities, including bipedal locomotion, dexterous manipulation, and human-like interaction abilities. These robots are designed to operate in human environments and interact with humans effectively.",
        metadata: { filename: 'humanoid-robotics.md', filepath: 'docs/module1/humanoid-robotics.md' },
        score: 0.85
      },
      {
        content: "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
        metadata: { filename: 'ros2-fundamentals.md', filepath: 'docs/module1/ros2-fundamentals.md' },
        score: 0.80
      },
      {
        content: "Qdrant is a vector similarity search engine that enables efficient similarity search for high-dimensional vectors. It's commonly used for semantic search, recommendation systems, and RAG (Retrieval Augmented Generation) applications.",
        metadata: { filename: 'qdrant-integration.md', filepath: 'docs/rag/qdrant-integration.md' },
        score: 0.75
      }
    ];

    // Simple keyword matching to filter relevant results
    const queryLower = query.toLowerCase();
    const filtered = mockResponses.filter(item =>
      item.content.toLowerCase().includes(queryLower) ||
      queryLower.split(' ').some(word => item.content.toLowerCase().includes(word))
    );

    return filtered.length > 0 ? filtered.slice(0, limit) : mockResponses.slice(0, limit);
  } catch (error) {
    console.error('Error searching documents:', error);
    // Fallback to mock responses
    return [
      {
        content: "Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world.",
        metadata: { filename: 'physical_ai_intro.md', filepath: 'docs/chapter1/physical_ai_intro.md' },
        score: 0.95
      }
    ];
  }
}

// Generate response function
async function generateResponse(query, context) {
  if (context && context.trim()) {
    // Extract key information from context
    const sentences = context.split(/[.!?]+/);
    const queryLower = query.toLowerCase();
    const relevantSentences = [];

    for (const sentence of sentences) {
      if (sentence.length > 10) { // Skip very short sentences
        // Score based on keyword matches
        const score = queryLower.split(' ').filter(word =>
          sentence.toLowerCase().includes(word)
        ).length;

        if (score > 0) {
          relevantSentences.push({ sentence: sentence.trim(), score });
        }
      }
    }

    // Sort by relevance and take top sentences
    relevantSentences.sort((a, b) => b.score - a.score);

    if (relevantSentences.length > 0) {
      const topSentences = relevantSentences.slice(0, 2).map(item => item.sentence);
      return topSentences.join('. ') + '.';
    } else {
      // Return first part of context if no specific matches
      return context.substring(0, Math.min(500, context.length));
    }
  } else {
    // If no context is found in the book, respond with the specific message
    return "This information is not available in the book.";
  }
}

module.exports = async (req, res) => {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Credentials', true);
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,OPTIONS,PATCH,DELETE,POST,PUT');
  res.setHeader(
    'Access-Control-Allow-Headers',
    'X-CSRF-Token, X-Requested-With, Accept, Accept-Version, Content-Length, Content-MD5, Content-Type, Date, X-Api-Version'
  );

  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { message, session_id, max_context_chunks = 5 } = req.body;

    // Get context chunks using search
    const contextChunks = await searchDocuments(message, max_context_chunks);

    let responseText;
    if (contextChunks.length === 0) {
      responseText = "This information is not available in the book.";
    } else {
      const contextText = contextChunks.map(chunk => chunk.content).join('\n\n');
      responseText = await generateResponse(message, contextText);
      // Ensure we return the proper response when information is not found
      if (responseText.includes("not available in the book")) {
        responseText = "This information is not available in the book.";
      }
    }

    // Generate a session ID if not provided
    const finalSessionId = session_id || `vercel-session-${Date.now()}`;

    res.status(200).json({
      response: responseText,
      session_id: finalSessionId,
      context_chunks: contextChunks
    });
  } catch (error) {
    console.error('Error in chat API:', error);

    res.status(500).json({
      error: 'Internal server error',
      response: 'This information is not available in the book.',
      session_id: req.body.session_id || `vercel-session-${Date.now()}`,
      context_chunks: []
    });
  }
};