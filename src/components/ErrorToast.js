/**
 * Error Toast Component
 *
 * Displays temporary error notifications with retry options.
 * Shows at the top of the screen with automatic dismiss.
 */

import React, { useState, useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './ErrorToast.module.css';

/**
 * ErrorToast Component
 *
 * Displays error notifications with:
 * - Error message
 * - Error code/type
 * - Retry button (optional)
 * - Auto-dismiss after timeout
 * - Manual dismiss
 */
export default function ErrorToast() {
  const [errors, setErrors] = useState([]);
  const { isUrdu } = useLanguage();

  useEffect(() => {
    // Listen for translation errors
    const handleTranslationError = (event) => {
      const { message, code, retryAfter } = event.detail;
      addError({
        id: Date.now(),
        message,
        code,
        retryAfter,
        type: 'translation',
      });
    };

    // Listen for network errors
    const handleNetworkError = (event) => {
      const { message } = event.detail;
      addError({
        id: Date.now(),
        message,
        code: 'NETWORK_ERROR',
        type: 'network',
      });
    };

    window.addEventListener('translation-error', handleTranslationError);
    window.addEventListener('network-error', handleNetworkError);

    return () => {
      window.removeEventListener('translation-error', handleTranslationError);
      window.removeEventListener('network-error', handleNetworkError);
    };
  }, []);

  const addError = (error) => {
    setErrors((prev) => [...prev, error]);

    // Auto-dismiss after 5 seconds
    setTimeout(() => {
      dismissError(error.id);
    }, 5000);
  };

  const dismissError = (id) => {
    setErrors((prev) => prev.filter((error) => error.id !== id));
  };

  const handleRetry = (error) => {
    dismissError(error.id);

    // Dispatch retry event
    window.dispatchEvent(
      new CustomEvent('retry-translation', {
        detail: { errorCode: error.code },
      })
    );
  };

  const getErrorIcon = (type) => {
    switch (type) {
      case 'translation':
        return '🌐';
      case 'network':
        return '📡';
      default:
        return '⚠️';
    }
  };

  const getErrorTitle = (code) => {
    const titles = {
      SERVICE_DOWN: isUrdu ? 'سروس دستیاب نہیں' : 'Service Unavailable',
      RATE_LIMIT: isUrdu ? 'بہت زیادہ درخواستیں' : 'Too Many Requests',
      TIMEOUT: isUrdu ? 'ٹائم آؤٹ' : 'Request Timeout',
      NETWORK_ERROR: isUrdu ? 'نیٹ ورک کی خرابی' : 'Network Error',
      INVALID_INPUT: isUrdu ? 'غلط ان پٹ' : 'Invalid Input',
      UNKNOWN_ERROR: isUrdu ? 'نامعلوم خرابی' : 'Unknown Error',
    };
    return titles[code] || (isUrdu ? 'خرابی' : 'Error');
  };

  if (errors.length === 0) {
    return null;
  }

  return (
    <div className={styles.toastContainer}>
      {errors.map((error) => (
        <div
          key={error.id}
          className={`${styles.toast} ${styles[error.type]}`}
          role="alert"
          aria-live="assertive"
        >
          <div className={styles.toastContent}>
            <span className={styles.icon}>{getErrorIcon(error.type)}</span>
            <div className={styles.message}>
              <strong>{getErrorTitle(error.code)}</strong>
              <p>{error.message}</p>
              {error.retryAfter && (
                <small>
                  {isUrdu
                    ? `${error.retryAfter} سیکنڈ بعد دوبارہ کوشش کریں`
                    : `Retry after ${error.retryAfter} seconds`}
                </small>
              )}
            </div>
          </div>

          <div className={styles.actions}>
            {error.retryAfter && (
              <button
                className={styles.retryButton}
                onClick={() => handleRetry(error)}
              >
                {isUrdu ? 'دوبارہ کوشش کریں' : 'Retry'}
              </button>
            )}
            <button
              className={styles.dismissButton}
              onClick={() => dismissError(error.id)}
              aria-label={isUrdu ? 'بند کریں' : 'Dismiss'}
            >
              ✕
            </button>
          </div>
        </div>
      ))}
    </div>
  );
}
