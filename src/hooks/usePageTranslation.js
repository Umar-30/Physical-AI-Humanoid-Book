/**
 * usePageTranslation Hook
 *
 * Automatically translates page content when language changes.
 * Integrates LanguageContext with domTranslationManager.
 */

import { useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import { translatePageContent } from '../utils/domTranslationManager';

/**
 * Hook to handle automatic page translation
 * Call this in Root.js to enable translation across all pages
 */
export function usePageTranslation() {
  const { language, setTranslating, setError, clearError } = useLanguage();

  useEffect(() => {
    // Only run in browser
    if (typeof window === 'undefined') return;

    let isMounted = true;

    async function performTranslation() {
      console.log(`[usePageTranslation] Language changed to: ${language}`);

      // Clear any previous errors
      clearError();

      // If switching to English, no API calls needed
      if (language === 'en') {
        translatePageContent('en');
        return;
      }

      // Start translation
      setTranslating(true);

      try {
        // Translate page content
        const result = await translatePageContent(language, (current, total) => {
          console.log(`[usePageTranslation] Progress: ${current}/${total}`);
        });

        if (isMounted) {
          console.log(`[usePageTranslation] Translation complete:`, result);

          if (result.failed > 0) {
            setError(`Translation partially failed: ${result.failed} elements could not be translated`);
          }
        }
      } catch (err) {
        if (isMounted) {
          console.error('[usePageTranslation] Translation failed:', err);
          setError(err.message || 'Translation failed');
        }
      } finally {
        if (isMounted) {
          setTranslating(false);
        }
      }
    }

    // Small delay to ensure DOM is ready
    const timeoutId = setTimeout(performTranslation, 100);

    return () => {
      isMounted = false;
      clearTimeout(timeoutId);
    };
  }, [language, setTranslating, setError, clearError]);
}

export default usePageTranslation;
