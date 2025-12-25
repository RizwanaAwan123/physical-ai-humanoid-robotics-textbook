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
    const { message } = req.body;

    // Simple response logic for demonstration
    const responses = {
      'what is robotic nervous system': 'The Robotic Nervous System refers to the core architecture of a robot that handles communication, control, and coordination. In the context of ROS 2 (Robot Operating System 2), it includes Nodes, Topics, Services, Actions, and the underlying communication infrastructure that allows different parts of a robot to work together.',
      'robotic nervous system': 'The Robotic Nervous System is the foundational architecture of a robot, primarily implemented through ROS 2. It includes Nodes (processes), Topics (publish/subscribe communication), Services (request/response), and Actions (goal-oriented communication). This system enables different robot components to communicate and coordinate effectively.',
      'chat': 'Hello! I\'m your AI assistant for Physical AI and Humanoid Robotics. Ask me about robotic systems, ROS 2, Qdrant, or any topic from the textbook.',
      'hello': 'Hello! I\'m your AI assistant. You can ask me about Physical AI, Humanoid Robotics, ROS 2, or any related topics from the textbook.',
      'help': 'I can answer questions about Physical AI and Humanoid Robotics. Try asking about: robotic nervous system, ROS 2, Qdrant, or any specific topic from the textbook.'
    };

    // Simple keyword matching for demo purposes
    const lowerMessage = message.toLowerCase();
    let responseText = 'I\'m a demo chatbot. In the full implementation, I would search the textbook content and provide detailed answers based on the relevant chapters. For a complete experience, the backend service with Qdrant vector database needs to be deployed.';

    for (const [key, value] of Object.entries(responses)) {
      if (lowerMessage.includes(key)) {
        responseText = value;
        break;
      }
    }

    // Add a note about the full implementation
    if (!Object.keys(responses).some(key => lowerMessage.includes(key))) {
      responseText += ' For full functionality, please deploy the backend service with the Qdrant vector database to enable semantic search of the textbook content.';
    }

    res.status(200).json({
      response: responseText,
      session_id: 'demo-session',
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