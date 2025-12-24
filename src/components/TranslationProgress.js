/**
 * Translation Progress Component
 *
 * Shows a loading overlay when translation is in progress.
 * Displays progress message and spinner.
 */

import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './TranslationProgress.module.css';

/**
 * TranslationProgress Component
 *
 * Displays an overlay with loading spinner during translation.
 * Only visible when isTranslating is true.
 */
export default function TranslationProgress() {
  const { isTranslating, language } = useLanguage();

  if (!isTranslating) {
    return null;
  }

  const progressMessage = {
    en: 'Translating to English...',
    ur: 'Translating to Urdu...',
  };

  return (
    <div className={styles.overlay} role="alert" aria-live="polite">
      <div className={styles.container}>
        {/* Spinner */}
        <div className={styles.spinner}>
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
        </div>

        {/* Progress message */}
        <p className={styles.message}>
          {progressMessage[language] || 'Translating...'}
        </p>

        {/* Subtle hint */}
        <p className={styles.hint}>
          This may take a few seconds for the first translation
        </p>
      </div>
    </div>
  );
}
