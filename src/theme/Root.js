import React from 'react';
import { LanguageProvider } from '../contexts/LanguageContext';
import { usePageTranslation } from '../hooks/usePageTranslation';

/**
 * Inner component that uses the translation hook
 * Separated because hooks can't be used in the same component that provides the context
 */
function TranslationWrapper({ children }) {
  // This hook automatically translates page content when language changes
  usePageTranslation();
  return <>{children}</>;
}

export default function Root({children}) {
  return (
    <LanguageProvider>
      <TranslationWrapper>
        {children}
      </TranslationWrapper>
    </LanguageProvider>
  );
}
