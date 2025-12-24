/**
 * Performance Monitor Component
 *
 * Tracks and logs performance metrics for translation operations,
 * page loads, and user interactions.
 * Provides insights for optimization and debugging.
 */

import { useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { useLanguage } from '../contexts/LanguageContext';

/**
 * Performance thresholds (milliseconds)
 */
const THRESHOLDS = {
  TRANSLATION_WARN: 2000,
  TRANSLATION_ERROR: 5000,
  PAGE_LOAD_WARN: 3000,
  PAGE_LOAD_ERROR: 8000,
};

/**
 * PerformanceMonitor Component
 *
 * Monitors:
 * - Translation operation duration
 * - Page load performance
 * - Cache hit rates
 * - Memory usage
 * - Network conditions
 */
export default function PerformanceMonitor() {
  const { language, isTranslating } = useLanguage();
  const location = useLocation();
  const translationStartTime = useRef(null);
  const pageLoadStartTime = useRef(null);
  const metricsRef = useRef({
    translationCount: 0,
    cacheHits: 0,
    cacheMisses: 0,
    translationErrors: 0,
    totalTranslationTime: 0,
  });

  // Only run effects in browser, not during SSR
  const isBrowser = typeof window !== 'undefined';

  /**
   * Start tracking translation operation
   */
  useEffect(() => {
    if (!isBrowser) return;

    if (isTranslating && !translationStartTime.current) {
      translationStartTime.current = performance.now();
      console.log('[PerformanceMonitor] Translation started');
    } else if (!isTranslating && translationStartTime.current) {
      const duration = performance.now() - translationStartTime.current;
      translationStartTime.current = null;

      // Record metrics
      metricsRef.current.translationCount += 1;
      metricsRef.current.totalTranslationTime += duration;

      // Log performance
      logTranslationMetrics(duration);

      // Check thresholds
      if (duration > THRESHOLDS.TRANSLATION_ERROR) {
        console.error(`[PerformanceMonitor] Translation took ${duration.toFixed(0)}ms (CRITICAL)`);
      } else if (duration > THRESHOLDS.TRANSLATION_WARN) {
        console.warn(`[PerformanceMonitor] Translation took ${duration.toFixed(0)}ms (SLOW)`);
      } else {
        console.log(`[PerformanceMonitor] Translation completed in ${duration.toFixed(0)}ms`);
      }
    }
  }, [isTranslating]);

  /**
   * Track page navigation performance
   */
  useEffect(() => {
    pageLoadStartTime.current = performance.now();

    // Measure when page is fully loaded
    const measurePageLoad = () => {
      if (pageLoadStartTime.current) {
        const duration = performance.now() - pageLoadStartTime.current;

        console.log(`[PerformanceMonitor] Page load: ${location.pathname} in ${duration.toFixed(0)}ms`);

        if (duration > THRESHOLDS.PAGE_LOAD_ERROR) {
          console.error(`[PerformanceMonitor] Page load CRITICAL: ${duration.toFixed(0)}ms`);
        } else if (duration > THRESHOLDS.PAGE_LOAD_WARN) {
          console.warn(`[PerformanceMonitor] Page load SLOW: ${duration.toFixed(0)}ms`);
        }

        pageLoadStartTime.current = null;
      }
    };

    // Wait for page content to render
    const timer = setTimeout(measurePageLoad, 500);
    return () => clearTimeout(timer);
  }, [location.pathname]);

  /**
   * Monitor cache performance
   */
  useEffect(() => {
    const handleCacheHit = (event) => {
      metricsRef.current.cacheHits += 1;
      console.log('[PerformanceMonitor] Cache hit', {
        hitRate: getCacheHitRate(),
      });
    };

    const handleCacheMiss = (event) => {
      metricsRef.current.cacheMisses += 1;
      console.log('[PerformanceMonitor] Cache miss', {
        hitRate: getCacheHitRate(),
      });
    };

    const handleTranslationError = (event) => {
      metricsRef.current.translationErrors += 1;
      console.error('[PerformanceMonitor] Translation error', {
        errorCount: metricsRef.current.translationErrors,
      });
    };

    window.addEventListener('cache-hit', handleCacheHit);
    window.addEventListener('cache-miss', handleCacheMiss);
    window.addEventListener('translation-error', handleTranslationError);

    return () => {
      window.removeEventListener('cache-hit', handleCacheHit);
      window.removeEventListener('cache-miss', handleCacheMiss);
      window.removeEventListener('translation-error', handleTranslationError);
    };
  }, []);

  /**
   * Periodic metrics reporting
   */
  useEffect(() => {
    const reportInterval = setInterval(() => {
      reportMetrics();
    }, 60000); // Every minute

    return () => clearInterval(reportInterval);
  }, []);

  /**
   * Report metrics on unmount (page close)
   */
  useEffect(() => {
    const handleBeforeUnload = () => {
      reportMetrics();
    };

    window.addEventListener('beforeunload', handleBeforeUnload);
    return () => window.removeEventListener('beforeunload', handleBeforeUnload);
  }, []);

  return null; // No visual rendering
}

/**
 * Log translation performance metrics
 */
function logTranslationMetrics(duration) {
  const metrics = {
    duration: Math.round(duration),
    timestamp: new Date().toISOString(),
    language: document.documentElement.lang,
    pathname: window.location.pathname,
  };

  console.log('[PerformanceMonitor] Translation metrics:', metrics);

  // Send to analytics if available
  if (typeof window.gtag === 'function') {
    window.gtag('event', 'translation_timing', {
      event_category: 'Performance',
      event_label: metrics.language,
      value: Math.round(duration),
    });
  }
}

/**
 * Calculate cache hit rate
 */
function getCacheHitRate() {
  const metrics = window.performanceMonitorMetrics || { cacheHits: 0, cacheMisses: 0 };
  const total = metrics.cacheHits + metrics.cacheMisses;
  if (total === 0) return 0;
  return ((metrics.cacheHits / total) * 100).toFixed(1);
}

/**
 * Report comprehensive metrics
 */
function reportMetrics() {
  const metrics = window.performanceMonitorMetrics;
  if (!metrics) return;

  const avgTranslationTime =
    metrics.translationCount > 0
      ? (metrics.totalTranslationTime / metrics.translationCount).toFixed(0)
      : 0;

  const report = {
    translationCount: metrics.translationCount,
    avgTranslationTime: `${avgTranslationTime}ms`,
    cacheHitRate: `${getCacheHitRate()}%`,
    cacheHits: metrics.cacheHits,
    cacheMisses: metrics.cacheMisses,
    translationErrors: metrics.translationErrors,
    totalTranslationTime: `${(metrics.totalTranslationTime / 1000).toFixed(1)}s`,
  };

  console.log('[PerformanceMonitor] Session metrics:', report);

  // Send to analytics if available
  if (typeof window.gtag === 'function') {
    window.gtag('event', 'session_metrics', {
      event_category: 'Performance',
      ...report,
    });
  }
}

/**
 * Get Web Vitals if available
 */
export function measureWebVitals() {
  if ('PerformanceObserver' in window) {
    // Largest Contentful Paint (LCP)
    const lcpObserver = new PerformanceObserver((list) => {
      const entries = list.getEntries();
      const lastEntry = entries[entries.length - 1];
      console.log('[PerformanceMonitor] LCP:', lastEntry.renderTime || lastEntry.loadTime);
    });
    lcpObserver.observe({ entryTypes: ['largest-contentful-paint'] });

    // First Input Delay (FID)
    const fidObserver = new PerformanceObserver((list) => {
      list.getEntries().forEach((entry) => {
        console.log('[PerformanceMonitor] FID:', entry.processingStart - entry.startTime);
      });
    });
    fidObserver.observe({ entryTypes: ['first-input'] });

    // Cumulative Layout Shift (CLS)
    let clsScore = 0;
    const clsObserver = new PerformanceObserver((list) => {
      list.getEntries().forEach((entry) => {
        if (!entry.hadRecentInput) {
          clsScore += entry.value;
          console.log('[PerformanceMonitor] CLS:', clsScore);
        }
      });
    });
    clsObserver.observe({ entryTypes: ['layout-shift'] });
  }
}

// Initialize Web Vitals measurement
if (typeof window !== 'undefined') {
  measureWebVitals();
}
