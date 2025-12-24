import React from 'react';
import { createRoot } from 'react-dom/client';
import ChatWidgetWrapper from './components/ChatWidgetWrapper';

let root = null;

export function onRouteDidUpdate() {
  // Ensure chat widget is always mounted (English-only RAG chatbot)
  const existingWidget = document.getElementById('chat-widget-root');
  if (!existingWidget) {
    const widgetRoot = document.createElement('div');
    widgetRoot.id = 'chat-widget-root';
    document.body.appendChild(widgetRoot);

    // Use React 18 createRoot API
    root = createRoot(widgetRoot);
    root.render(<ChatWidgetWrapper />);
  }
}
