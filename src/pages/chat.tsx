import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot';

function ChatPage() {
  return (
    <Layout title="Chat with AI Assistant" description="Ask questions about Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8">
            <div className="text--center padding-horiz--md">
              <h1>📚 Physical AI & Humanoid Robotics</h1>
              <p>Ask questions about the textbook content using the chat assistant.</p>
              <div style={{ textAlign: 'left', marginTop: '20px' }}>
                <h3>How to use the chatbot:</h3>
                <ul>
                  <li>Ask questions about Physical AI concepts</li>
                  <li>Get explanations about humanoid robotics</li>
                  <li>Find specific content from the textbook</li>
                  <li>Learn about kinematics, dynamics, and control systems</li>
                </ul>
              </div>
            </div>
          </div>
          <div className="col col--4">
            <div style={{ position: 'sticky', top: '20px' }}>
              <h3 style={{ textAlign: 'center' }}>🤖 Chat Assistant</h3>
              <Chatbot />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatPage;