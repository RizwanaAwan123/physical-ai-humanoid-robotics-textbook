// Vercel Serverless Function for FastAPI backend
const { spawn } = require('child_process');
const path = require('path');

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

  // For now, return a simple response to test the connection
  if (req.method === 'POST' && req.url.includes('/api/chat')) {
    const { message } = req.body || {};

    // Simple response logic
    const responses = {
      'what is robotic nervous system': 'The Robotic Nervous System refers to the core architecture of a robot that handles communication, control, and coordination. In the context of ROS 2 (Robot Operating System 2), it includes Nodes, Topics, Services, Actions, and the underlying communication infrastructure that allows different parts of a robot to work together. This system is fundamental to how humanoid robots process information and execute tasks.',
      'robotic nervous system': 'The Robotic Nervous System is the foundational architecture of a robot, primarily implemented through ROS 2. It includes Nodes (processes), Topics (publish/subscribe communication), Services (request/response), and Actions (goal-oriented communication). This system enables different robot components to communicate and coordinate effectively.',
      'chat': 'Hello! I\'m your AI assistant for Physical AI and Humanoid Robotics. Ask me about robotic systems, ROS 2, Qdrant, or any topic from the textbook.',
      'hello': 'Hello! I\'m your AI assistant. You can ask me about Physical AI, Humanoid Robotics, ROS 2, or any related topics from the textbook.',
      'help': 'I can answer questions about Physical AI and Humanoid Robotics. Try asking about: robotic nervous system, ROS 2, Qdrant, or any specific topic from the textbook.'
    };

    const lowerMessage = message ? message.toLowerCase() : '';
    let responseText = 'I\'m your AI assistant for Physical AI and Humanoid Robotics. I can answer questions based on the textbook content. Please ask me about any topic related to robotics, AI, ROS 2, or humanoid systems.';

    for (const [key, value] of Object.entries(responses)) {
      if (lowerMessage.includes(key)) {
        responseText = value;
        break;
      }
    }

    res.status(200).json({
      response: responseText,
      session_id: 'vercel-session',
      context_chunks: []
    });
  } else {
    res.status(404).json({ error: 'Not Found' });
  }
};