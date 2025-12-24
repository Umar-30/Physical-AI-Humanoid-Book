import React from 'react';
import ChatWidget from './ChatWidget';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function ChatWidgetWrapper() {
  // Only render on client-side (not during SSR)
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return <ChatWidget />;
}
