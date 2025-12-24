/**
 * Network Status Indicator
 *
 * Displays connection status and detects offline mode.
 * Shows banner when user goes offline.
 */

import React, { useState, useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './NetworkStatus.module.css';

/**
 * NetworkStatus Component
 *
 * Monitors online/offline status and displays banner when offline.
 * Also provides visual indicator in corner when connection is lost.
 */
export default function NetworkStatus() {
  const [isOnline, setIsOnline] = useState(navigator.onLine);
  const [showBanner, setShowBanner] = useState(false);
  const { isUrdu } = useLanguage();

  useEffect(() => {
    const handleOnline = () => {
      console.log('[NetworkStatus] Connection restored');
      setIsOnline(true);
      setShowBanner(false);

      // Dispatch event for other components
      window.dispatchEvent(new CustomEvent('network-online'));
    };

    const handleOffline = () => {
      console.log('[NetworkStatus] Connection lost');
      setIsOnline(false);
      setShowBanner(true);

      // Dispatch event for other components
      window.dispatchEvent(
        new CustomEvent('network-error', {
          detail: {
            message: isUrdu
              ? 'انٹرنیٹ کنکشن دستیاب نہیں ہے۔ براہ کرم اپنے نیٹ ورک کو چیک کریں۔'
              : 'No internet connection. Please check your network.',
          },
        })
      );
    };

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    // Check initial status
    if (!navigator.onLine) {
      handleOffline();
    }

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [isUrdu]);

  const dismissBanner = () => {
    setShowBanner(false);
  };

  // Only show when offline
  if (isOnline) {
    return null;
  }

  return (
    <>
      {/* Offline Banner */}
      {showBanner && (
        <div className={styles.offlineBanner} role="alert">
          <div className={styles.bannerContent}>
            <span className={styles.icon}>📡</span>
            <div className={styles.message}>
              <strong>
                {isUrdu ? 'آف لائن موڈ' : 'Offline Mode'}
              </strong>
              <p>
                {isUrdu
                  ? 'آپ آف لائن ہیں۔ ترجمہ اور چیٹ کی خصوصیات دستیاب نہیں ہیں۔'
                  : 'You are offline. Translation and chat features are unavailable.'}
              </p>
            </div>
            <button
              className={styles.dismissButton}
              onClick={dismissBanner}
              aria-label={isUrdu ? 'بند کریں' : 'Dismiss'}
            >
              ✕
            </button>
          </div>
        </div>
      )}

      {/* Persistent Offline Indicator */}
      <div
        className={styles.offlineIndicator}
        title={
          isUrdu
            ? 'آف لائن - انٹرنیٹ کنکشن نہیں'
            : 'Offline - No internet connection'
        }
      >
        <span className={styles.indicatorIcon}>📡</span>
        <span className={styles.indicatorText}>
          {isUrdu ? 'آف لائن' : 'Offline'}
        </span>
      </div>
    </>
  );
}
