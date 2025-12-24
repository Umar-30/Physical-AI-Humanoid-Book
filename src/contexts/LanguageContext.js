/**
 * Language Context for managing translation state across the application
 *
 * Provides:
 * - Current language state (en/ur)
 * - Language toggle function
 * - Translation loading state
 * - localStorage persistence
 */

import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';

const LanguageContext = createContext(undefined);

/**
 * Custom hook to access language context
 * @returns {Object} Language context value
 * @throws {Error} If used outside LanguageProvider
 */
export function useLanguage() {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
}

/**
 * Language Provider Component
 *
 * Wraps the application to provide language state management
 * Persists language preference to localStorage
 *
 * @param {Object} props
 * @param {React.ReactNode} props.children - Child components
 */
export function LanguageProvider({ children }) {
  const [language, setLanguageState] = useState('en'); // 'en' or 'ur'
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState(null);

  // Load language preference from localStorage on mount (client-side only)
  useEffect(() => {
    // Check if we're in browser (not SSR)
    if (typeof window === 'undefined') return;

    try {
      const savedLanguage = localStorage.getItem('preferred_language');
      if (savedLanguage === 'en' || savedLanguage === 'ur') {
        setLanguageState(savedLanguage);
      }
    } catch (err) {
      console.error('Failed to load language preference from localStorage:', err);
      // Continue with default 'en'
    }
  }, []);

  // Persist language preference to localStorage whenever it changes (client-side only)
  useEffect(() => {
    // Check if we're in browser (not SSR)
    if (typeof window === 'undefined') return;

    try {
      localStorage.setItem('preferred_language', language);
    } catch (err) {
      console.error('Failed to save language preference to localStorage:', err);
    }
  }, [language]);

  /**
   * Set language and persist to localStorage
   * @param {string} newLanguage - Language code ('en' or 'ur')
   */
  const setLanguage = useCallback((newLanguage) => {
    if (newLanguage !== 'en' && newLanguage !== 'ur') {
      console.error(`Invalid language: ${newLanguage}. Must be 'en' or 'ur'`);
      return;
    }
    setLanguageState(newLanguage);
    setError(null);
  }, []);

  /**
   * Toggle between English and Urdu
   */
  const toggleLanguage = useCallback(() => {
    setLanguage(language === 'en' ? 'ur' : 'en');
  }, [language, setLanguage]);

  /**
   * Set translation loading state
   * @param {boolean} loading - Whether translation is in progress
   */
  const setTranslating = useCallback((loading) => {
    setIsTranslating(loading);
  }, []);

  /**
   * Set error state
   * @param {Error|string|null} err - Error object or message
   */
  const setTranslationError = useCallback((err) => {
    setError(err);
  }, []);

  /**
   * Clear error state
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  const value = {
    language,
    setLanguage,
    toggleLanguage,
    isTranslating,
    setTranslating,
    error,
    setError: setTranslationError,
    clearError,
    // Utility flags
    isUrdu: language === 'ur',
    isEnglish: language === 'en',
  };

  return (
    <LanguageContext.Provider value={value}>
      {children}
    </LanguageContext.Provider>
  );
}

export default LanguageContext;
