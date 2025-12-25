import React, { useState, useEffect, useRef } from 'react';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const Chatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today?',
      role: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      // In production, this will be proxied to the deployed backend
      // In development, this will be proxied to localhost:8000
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          session_id: 'default-session',
        }),
      });

      // Check if the backend service is available
      if (response.status === 404 || response.status === 500 || response.status === 0) {
        throw new Error('Backend service not available. Please ensure the backend is deployed.');
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, I encountered an error. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chatbot-container" style={{
      display: 'flex',
      flexDirection: 'column',
      height: '600px',
      border: '1px solid #ddd',
      borderRadius: '12px',
      overflow: 'hidden',
      backgroundColor: 'white',
      boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)'
    }}>
      <div className="chat-messages" style={{
        flex: 1,
        overflowY: 'auto',
        padding: '20px',
        display: 'flex',
        flexDirection: 'column',
        gap: '15px',
        backgroundColor: '#fafafa'
      }}>
        {messages.map((message) => (
          <div
            key={message.id}
            style={{
              alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
              backgroundColor: message.role === 'user' ? '#4caf50' : '#ffffff',
              color: message.role === 'user' ? 'white' : '#333',
              borderRadius: '18px',
              padding: '12px 18px',
              maxWidth: '85%',
              wordWrap: 'break-word',
              border: message.role === 'assistant' ? '1px solid #eee' : 'none',
              fontWeight: message.role === 'user' ? '500' : '400'
            }}
          >
            <div style={{ fontSize: '15px', lineHeight: '1.5' }}>
              {message.content}
            </div>
            <div style={{
              fontSize: '11px',
              color: message.role === 'user' ? 'rgba(255,255,255,0.8)' : '#999',
              marginTop: '6px',
              textAlign: 'right'
            }}>
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}
        {isLoading && (
          <div
            style={{
              alignSelf: 'flex-start',
              backgroundColor: '#ffffff',
              color: '#333',
              borderRadius: '18px',
              padding: '12px 18px',
              maxWidth: '85%',
              border: '1px solid #eee'
            }}
          >
            <div style={{ fontSize: '15px', color: '#666' }}>Thinking...</div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form
        onSubmit={handleSubmit}
        style={{
          padding: '20px',
          borderTop: '1px solid #eee',
          backgroundColor: 'white'
        }}
      >
        <div style={{ display: 'flex', gap: '10px' }}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about Physical AI & Robotics..."
            style={{
              flex: 1,
              padding: '14px 20px',
              border: '1px solid #ddd',
              borderRadius: '24px',
              fontSize: '15px',
              outline: 'none',
              transition: 'border-color 0.3s'
            }}
            onFocus={(e) => e.target.style.borderColor = '#4caf50'}
            onBlur={(e) => e.target.style.borderColor = '#ddd'}
            disabled={isLoading}
          />
          <button
            type="submit"
            style={{
              padding: '14px 24px',
              backgroundColor: '#4caf50',
              color: 'white',
              border: 'none',
              borderRadius: '24px',
              cursor: isLoading ? 'not-allowed' : 'pointer',
              fontSize: '15px',
              fontWeight: '600',
              transition: 'background-color 0.3s',
              minWidth: '80px'
            }}
           onMouseEnter={(e) => e.currentTarget.style.backgroundColor = '#43a047'}
           onMouseLeave={(e) => e.currentTarget.style.backgroundColor = '#4caf50'}
            disabled={!inputValue.trim() || isLoading}
          >
            Send
          </button>
        </div>
      </form>
    </div>
  );
};

export default Chatbot;