/**
 * Translated Content Component
 *
 * Wrapper component that handles content translation when language changes.
 * - Listens to language context changes
 * - Extracts content from the page
 * - Calls translation API
 * - Replaces content with translated version
 * - Shows translation progress
 */

import { useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { useLanguage } from '../contexts/LanguageContext';
import { translateText } from '../services/translationClient';
import {
  extractContent,
  replaceContent,
  validateContent,
} from '../utils/contentExtractor';

/**
 * TranslatedContent Component
 *
 * Manages content translation lifecycle:
 * 1. Detects language changes
 * 2. Extracts page content
 * 3. Translates via API
 * 4. Updates page with translated content
 * 5. Handles caching and errors
 */
export default function TranslatedContent() {
  const { language, isTranslating, setTranslating, setError, clearError } = useLanguage();
  const location = useLocation();
  const originalContentRef = useRef(null);
  const lastTranslatedPathRef = useRef(null);
  const lastTranslatedLanguageRef = useRef('en');

  /**
   * Store original English content before first translation
   */
  const storeOriginalContent = () => {
    if (!originalContentRef.current) {
      const content = extractContent();
      if (content) {
        originalContentRef.current = content;
        console.log('[TranslatedContent] Original content stored');
      }
    }
  };

  /**
   * Restore original English content
   */
  const restoreOriginalContent = () => {
    if (originalContentRef.current) {
      console.log('[TranslatedContent] Restoring original English content');
      replaceContent(originalContentRef.current);
      lastTranslatedLanguageRef.current = 'en';
    }
  };

  /**
   * Translate page content
   */
  const translatePageContent = async (targetLanguage) => {
    // Skip if already translating
    if (isTranslating) {
      console.log('[TranslatedContent] Translation already in progress, skipping');
      return;
    }

    // Skip if target is English (restore original instead)
    if (targetLanguage === 'en') {
      restoreOriginalContent();
      return;
    }

    // Skip if already translated to this language on this page
    if (
      targetLanguage === lastTranslatedLanguageRef.current &&
      location.pathname === lastTranslatedPathRef.current
    ) {
      console.log('[TranslatedContent] Already translated to', targetLanguage);
      return;
    }

    try {
      setTranslating(true);
      clearError();

      // Validate content can be extracted
      const validation = validateContent();
      if (!validation.valid) {
        throw new Error(validation.error);
      }

      console.log('[TranslatedContent] Starting translation to', targetLanguage);

      // Store original content if not already stored
      storeOriginalContent();

      // Extract current content (might be original or previously translated)
      const currentContent = extractContent();
      if (!currentContent) {
        throw new Error('Could not extract page content');
      }

      // Always translate from original English content
      const contentToTranslate = originalContentRef.current || currentContent;

      console.log('[TranslatedContent] Translating', contentToTranslate.length, 'characters');

      // Call translation API
      const response = await translateText(
        contentToTranslate,
        'en',
        targetLanguage,
        true // preserve HTML
      );

      console.log('[TranslatedContent] Translation received', {
        cached: response.cached,
        clientCached: response.clientCached,
        serverCached: response.serverCached,
      });

      // Replace content with translated version
      const success = replaceContent(response.translatedText);
      if (!success) {
        throw new Error('Failed to replace content with translation');
      }

      // Update tracking
      lastTranslatedPathRef.current = location.pathname;
      lastTranslatedLanguageRef.current = targetLanguage;

      console.log('[TranslatedContent] Translation complete and displayed');
    } catch (err) {
      console.error('[TranslatedContent] Translation failed:', err);
      setError(err.message || 'Translation failed');

      // Show user-friendly error notification
      showErrorNotification(err);
    } finally {
      setTranslating(false);
    }
  };

  /**
   * Show error notification to user
   */
  const showErrorNotification = (error) => {
    console.error('[TranslatedContent] Error notification:', error.message);

    // Dispatch custom event for error toast
    window.dispatchEvent(
      new CustomEvent('translation-error', {
        detail: {
          message: error.message,
          code: error.code,
          retryAfter: error.retryAfter,
        },
      })
    );
  };

  /**
   * Handle retry attempts
   */
  useEffect(() => {
    const handleRetry = () => {
      console.log('[TranslatedContent] Retry requested');
      clearError();

      // Retry translation with current language
      if (language !== 'en') {
        setTimeout(() => {
          translatePageContent(language);
        }, 500);
      }
    };

    window.addEventListener('retry-translation', handleRetry);

    return () => {
      window.removeEventListener('retry-translation', handleRetry);
    };
  }, [language]);

  /**
   * Update HTML dir attribute for RTL support
   */
  useEffect(() => {
    const htmlElement = document.documentElement;
    if (language === 'ur') {
      htmlElement.setAttribute('dir', 'rtl');
      htmlElement.setAttribute('lang', 'ur');
      console.log('[TranslatedContent] Set dir="rtl" for Urdu');
    } else {
      htmlElement.setAttribute('dir', 'ltr');
      htmlElement.setAttribute('lang', 'en');
      console.log('[TranslatedContent] Set dir="ltr" for English');
    }
  }, [language]);

  /**
   * Handle language changes
   */
  useEffect(() => {
    // Wait for DOM to be ready
    const timer = setTimeout(() => {
      if (language !== lastTranslatedLanguageRef.current) {
        console.log('[TranslatedContent] Language changed to:', language);
        translatePageContent(language);
      }
    }, 100);

    return () => clearTimeout(timer);
  }, [language]);

  /**
   * Handle page navigation
   */
  useEffect(() => {
    console.log('[TranslatedContent] Page navigation detected:', location.pathname);

    // Reset original content cache on page change
    originalContentRef.current = null;
    lastTranslatedPathRef.current = null;
    lastTranslatedLanguageRef.current = 'en';

    // If we're in Urdu mode, translate the new page
    if (language === 'ur') {
      const timer = setTimeout(() => {
        console.log('[TranslatedContent] Translating new page to Urdu');
        translatePageContent(language);
      }, 300); // Wait for page content to load

      return () => clearTimeout(timer);
    }
  }, [location.pathname]);

  // This component doesn't render anything
  return null;
}
