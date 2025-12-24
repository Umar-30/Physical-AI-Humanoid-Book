/**
 * Language Toggle Component
 *
 * Button in navbar to switch between English and Urdu.
 * Shows current language and allows toggling.
 * Displays loading spinner when translation in progress.
 */

import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './LanguageToggle.module.css';

/**
 * LanguageToggle Component
 *
 * Provides a button to toggle between English and Urdu languages.
 * Shows loading state during translation.
 *
 * @returns {JSX.Element} Language toggle button
 */
export default function LanguageToggle() {
  const { language, toggleLanguage, isTranslating } = useLanguage();

  const languageDisplay = {
    en: 'EN',
    ur: 'اردو', // Urdu in Urdu script
  };

  const handleClick = () => {
    if (!isTranslating) {
      toggleLanguage();
    }
  };

  return (
    <button
      className={styles.languageToggle}
      onClick={handleClick}
      disabled={isTranslating}
      aria-label={`Switch to ${language === 'en' ? 'Urdu' : 'English'}`}
      title={`Current language: ${language === 'en' ? 'English' : 'Urdu'}. Click to switch.`}
    >
      {/* Loading spinner */}
      {isTranslating && (
        <span className={styles.spinner} aria-hidden="true">
          <svg
            className={styles.spinnerSvg}
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
          >
            <circle
              className={styles.spinnerCircle}
              cx="12"
              cy="12"
              r="10"
              stroke="currentColor"
              strokeWidth="4"
            />
            <path
              className={styles.spinnerPath}
              fill="currentColor"
              d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
            />
          </svg>
        </span>
      )}

      {/* Language badge */}
      <span className={styles.languageBadge}>
        {languageDisplay[language]}
      </span>

      {/* Icon (globe or arrow) */}
      <span className={styles.icon} aria-hidden="true">
        {isTranslating ? '⏳' : '🌐'}
      </span>
    </button>
  );
}
