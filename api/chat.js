// Vercel Serverless Function for Chat API
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
    const { message, session_id } = req.body;

    // Enhanced response logic with more comprehensive answers
    const responses = {
      'what is robotic nervous system': 'The Robotic Nervous System refers to the core architecture of a robot that handles communication, control, and coordination. In the context of ROS 2 (Robot Operating System 2), it includes Nodes, Topics, Services, Actions, and the underlying communication infrastructure that allows different parts of a robot to work together. This system is fundamental to how humanoid robots process information and execute tasks.',
      'robotic nervous system': 'The Robotic Nervous System is the foundational architecture of a robot, primarily implemented through ROS 2. It includes Nodes (processes), Topics (publish/subscribe communication), Services (request/response), and Actions (goal-oriented communication). This system enables different robot components to communicate and coordinate effectively.',
      'ros': 'ROS (Robot Operating System) is a flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.',
      'qdrant': 'Qdrant is a vector similarity search engine that enables efficient similarity search for high-dimensional vectors. It\'s commonly used for semantic search, recommendation systems, and RAG (Retrieval Augmented Generation) applications.',
      'rag': 'RAG (Retrieval Augmented Generation) is a technique that combines information retrieval with language model generation. It retrieves relevant documents from a knowledge base and uses them as context for generating more accurate and informed responses.',
      'humanoid robotics': 'Humanoid robotics focuses on creating robots with human-like form and capabilities, including bipedal locomotion, dexterous manipulation, and human-like interaction abilities. These robots are designed to operate in human environments and interact with humans effectively.',
      'physical ai': 'Physical AI integrates artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world. Unlike traditional AI that operates purely in digital domains, Physical AI is embodied and interacts directly with the physical environment.',
      'chat': 'Hello! I\'m your AI assistant for Physical AI and Humanoid Robotics. Ask me about robotic systems, ROS 2, Qdrant, or any topic from the textbook.',
      'hello': 'Hello! I\'m your AI assistant. You can ask me about Physical AI, Humanoid Robotics, ROS 2, or any related topics from the textbook.',
      'help': 'I can answer questions about Physical AI and Humanoid Robotics. Try asking about: robotic nervous system, ROS 2, Qdrant, or any specific topic from the textbook.',
      'module': 'This textbook is organized into 4 core modules: Module 1: The Robotic Nervous System (ROS 2), Module 2: The Digital Twin (Gazebo & Unity), Module 3: The AI-Robot Brain (NVIDIA Isaac), and Module 4: Vision-Language-Action (VLA).',
      'textbook': 'This comprehensive curriculum explores Physical AI - AI that lives in the real world and understands physical laws. Students will design, simulate, and deploy humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac.',
      'ai': 'Artificial Intelligence in robotics enables robots to perceive their environment, make decisions, and execute complex tasks. In the context of Physical AI, it focuses on embodied intelligence that interacts with the physical world.'
    };

    // Simple keyword matching for demo purposes
    const lowerMessage = message ? message.toLowerCase() : '';
    let responseText = 'I\'m your AI assistant for Physical AI and Humanoid Robotics. I can answer questions based on the textbook content. Please ask me about any topic related to robotics, AI, ROS 2, or humanoid systems.';

    // Check for exact matches first
    for (const [key, value] of Object.entries(responses)) {
      if (lowerMessage.includes(key)) {
        responseText = value;
        break;
      }
    }

    // If no exact match, try partial matches
    if (responseText.includes('I\'m your AI assistant for Physical AI')) {
      for (const [key, value] of Object.entries(responses)) {
        if (lowerMessage.split(' ').some(word => word.includes(key.substring(0, 3)))) { // Match first 3 chars of key
          responseText = value;
          break;
        }
      }
    }

    res.status(200).json({
      response: responseText,
      session_id: session_id || 'vercel-session',
      context_chunks: []
    });
  } catch (error) {
    console.error('Error in chat API:', error);
    res.status(500).json({
      error: 'Internal server error',
      response: 'Sorry, I encountered an error. Please try again.'
    });
  }
};