import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();

    const userQuery = inputValue.trim();
    if (!userQuery) {
      setError('Please enter a question');
      return;
    }

    // Add user message
    const userMessage = {
      type: 'user',
      content: userQuery
    };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setError(null);
    setIsLoading(true);

    try {
      // Send query to RAG backend (English only)
      console.log('[ChatWidget] Querying RAG with:', userQuery);
      const response = await fetch('https://umar-30-chatbot-deploy.hf.space/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userQuery,
          top_k: 5,
          model: 'xiaomi/mimo-v2-flash:free'
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.error_message || `HTTP ${response.status}`);
      }

      const data = await response.json();

      // Add bot response (English only)
      const botMessage = {
        type: 'bot',
        content: data.answer,
        sources: data.sources || [],
        metadata: {
          model: data.model_used,
          tokens: data.tokens_used,
        }
      };
      setMessages(prev => [...prev, botMessage]);

      console.log('[ChatWidget] Response displayed');
    } catch (err) {
      console.error('[ChatWidget] Error:', err);
      setError(
        err.message.includes('fetch') || err.message.includes('Failed to fetch')
          ? 'Cannot connect to AI backend. Please try again later.'
          : err.message
      );
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
      handleSubmit(e);
    }
  };

  // UI strings - High-tech AI theme
  const strings = {
    title: 'AI Assistant',
    subtitle: 'Ask questions about Physical AI & Robotics',
    welcome: 'Hello! I\'m your AI guide.',
    welcomeHint: 'Ask me anything about Physical AI, Humanoid Robotics, ROS 2, simulation, or any topic covered in this course.',
    placeholder: 'Ask a question... (Ctrl+Enter to send)',
    sendButton: 'Send',
    thinking: 'Processing',
    sources: 'Sources:',
    error: 'Error:',
    toggleChat: 'Toggle AI Assistant',
    askAI: 'Ask AI Assistant'
  };

  // AI Robot Icon - Futuristic style
  const AIIcon = () => (
    <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      {/* Robot head */}
      <rect x="4" y="6" width="16" height="12" rx="2" />
      {/* Eyes */}
      <circle cx="9" cy="11" r="1.5" fill="currentColor" />
      <circle cx="15" cy="11" r="1.5" fill="currentColor" />
      {/* Antenna */}
      <line x1="12" y1="6" x2="12" y2="3" />
      <circle cx="12" cy="2" r="1" fill="currentColor" />
      {/* Mouth/speaker */}
      <line x1="8" y1="15" x2="16" y2="15" />
    </svg>
  );

  // Brain/Neural Network Icon for header
  const BrainIcon = () => (
    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      {/* Brain outline */}
      <path d="M12 4.5c-1.5-1.5-4-1.5-5 0s-1.5 3.5 0 5c-2 0-3.5 1.5-3.5 3.5s1.5 3.5 3.5 3.5c-1 1.5-0.5 3.5 1.5 4.5s3.5 0 4.5-1.5" />
      <path d="M12 4.5c1.5-1.5 4-1.5 5 0s1.5 3.5 0 5c2 0 3.5 1.5 3.5 3.5s-1.5 3.5-3.5 3.5c1 1.5 0.5 3.5-1.5 4.5s-3.5 0-4.5-1.5" />
      {/* Neural connections */}
      <circle cx="12" cy="12" r="1" fill="currentColor" />
      <line x1="12" y1="8" x2="12" y2="11" />
      <line x1="12" y1="13" x2="12" y2="16" />
      <line x1="9" y1="12" x2="11" y2="12" />
      <line x1="13" y1="12" x2="15" y2="12" />
    </svg>
  );

  const CloseIcon = () => (
    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={strings.toggleChat}
        title={strings.askAI}
      >
        {isOpen ? <CloseIcon /> : <AIIcon />}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <div>
              <h3>
                <span style={{ marginRight: '10px', verticalAlign: 'middle', display: 'inline-flex', color: '#00d4ff' }}>
                  <BrainIcon />
                </span>
                {strings.title}
              </h3>
              <p>{strings.subtitle}</p>
            </div>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>{strings.welcome}</p>
                <p>{strings.welcomeHint}</p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.type === 'user' ? styles.userMessage : styles.botMessage
                }`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>

                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>{strings.sources}</strong>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>
                          <a
                            href={source.url}
                            target="_blank"
                            rel="noopener noreferrer"
                          >
                            {source.page_title || source.url}
                          </a>
                          {source.relevance_score && (
                            <span className={styles.relevance}>
                              {' '}({(source.relevance_score * 100).toFixed(0)}%)
                            </span>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}

                {message.metadata && (
                  <div className={styles.metadata}>
                    <small>
                      Model: {message.metadata.model} | Tokens: {message.metadata.tokens}
                    </small>
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.loader}>
                  <span>{strings.thinking}</span>
                  <span className={styles.dots}>
                    <span>.</span>
                    <span>.</span>
                    <span>.</span>
                  </span>
                </div>
              </div>
            )}

            {error && (
              <div className={styles.error}>
                <strong>{strings.error}</strong> {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.chatInput}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyPress}
              placeholder={strings.placeholder}
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !inputValue.trim()}>
              {strings.sendButton}
            </button>
          </form>
        </div>
      )}
    </>
  );
}
