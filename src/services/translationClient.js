/**
 * Translation Client Service
 *
 * HTTP client for communicating with the translation API backend.
 * Handles:
 * - Translation requests with timeout
 * - Error mapping from HTTP status to error codes
 * - Cache integration
 * - Request/response validation
 */

import { generateCacheKey, getFromCache, saveToCache } from '../utils/cacheManager';

// Separate ports: RAG on 8000, Translation on 8001
// Browser-safe environment variable access
const API_BASE_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) || 'http://localhost:8000';
const TRANSLATION_API_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_TRANSLATION_URL) || 'http://localhost:8001';
const TRANSLATION_ENDPOINT = `${TRANSLATION_API_URL}/api/translate`;
const REQUEST_TIMEOUT_MS = 10000; // 10 seconds

/**
 * Error codes from backend
 */
export const ErrorCodes = {
  SERVICE_DOWN: 'SERVICE_DOWN',
  RATE_LIMIT: 'RATE_LIMIT',
  INVALID_LANGUAGE: 'INVALID_LANGUAGE',
  TIMEOUT: 'TIMEOUT',
  INVALID_INPUT: 'INVALID_INPUT',
  NETWORK_ERROR: 'NETWORK_ERROR',
  UNKNOWN_ERROR: 'UNKNOWN_ERROR',
};

/**
 * Custom error class for translation errors
 */
export class TranslationError extends Error {
  constructor(message, code, retryAfter = null) {
    super(message);
    this.name = 'TranslationError';
    this.code = code;
    this.retryAfter = retryAfter;
  }
}

/**
 * Map HTTP status codes to error codes
 * @param {number} status - HTTP status code
 * @param {Object} responseBody - Response body with error details
 * @returns {Object} Error object with code and message
 */
function mapHttpErrorToCode(status, responseBody = {}) {
  const errorCode = responseBody.error_code || responseBody.errorCode;
  const errorMessage = responseBody.error || responseBody.message || 'Unknown error';
  const retryAfter = responseBody.retry_after || responseBody.retryAfter;

  // If backend provided error code, use it
  if (errorCode) {
    return {
      code: errorCode,
      message: errorMessage,
      retryAfter,
    };
  }

  // Otherwise, map by HTTP status
  switch (status) {
    case 400:
      return {
        code: ErrorCodes.INVALID_INPUT,
        message: errorMessage || 'Invalid request format or content',
        retryAfter: null,
      };
    case 408:
      return {
        code: ErrorCodes.TIMEOUT,
        message: errorMessage || 'Translation request timed out',
        retryAfter: 5,
      };
    case 429:
      return {
        code: ErrorCodes.RATE_LIMIT,
        message: errorMessage || 'Too many requests, please slow down',
        retryAfter: retryAfter || 60,
      };
    case 503:
      return {
        code: ErrorCodes.SERVICE_DOWN,
        message: errorMessage || 'Translation service temporarily unavailable',
        retryAfter: retryAfter || 60,
      };
    default:
      return {
        code: ErrorCodes.UNKNOWN_ERROR,
        message: errorMessage || `HTTP ${status}: ${errorMessage}`,
        retryAfter: null,
      };
  }
}

/**
 * Fetch with timeout wrapper
 * @param {string} url - URL to fetch
 * @param {Object} options - Fetch options
 * @param {number} timeout - Timeout in milliseconds
 * @returns {Promise<Response>} Fetch response
 */
async function fetchWithTimeout(url, options, timeout) {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(timeoutId);
    return response;
  } catch (err) {
    clearTimeout(timeoutId);

    if (err.name === 'AbortError') {
      throw new TranslationError(
        'Translation request timed out',
        ErrorCodes.TIMEOUT,
        5
      );
    }

    throw err;
  }
}

/**
 * Translate text from English to Urdu
 *
 * @param {string} text - Text to translate
 * @param {string} sourceLanguage - Source language code (default: 'en')
 * @param {string} targetLanguage - Target language code (default: 'ur')
 * @param {boolean} preserveHtml - Whether to preserve HTML tags (default: true)
 * @returns {Promise<Object>} Translation response
 * @throws {TranslationError} If translation fails
 */
export async function translateText(
  text,
  sourceLanguage = 'en',
  targetLanguage = 'ur',
  preserveHtml = true
) {
  // Validate inputs
  if (!text || typeof text !== 'string' || text.trim().length === 0) {
    throw new TranslationError(
      'Text cannot be empty',
      ErrorCodes.INVALID_INPUT
    );
  }

  if (text.length > 50000) {
    throw new TranslationError(
      'Text exceeds maximum length of 50,000 characters',
      ErrorCodes.INVALID_INPUT
    );
  }

  // Generate cache key and check client-side cache first
  const contentHash = generateCacheKey(text);
  const cached = getFromCache(contentHash);

  if (cached) {
    console.log('[TranslationClient] Cache hit (client)');

    // Dispatch cache hit event for performance monitoring
    window.dispatchEvent(new CustomEvent('cache-hit', {
      detail: { type: 'client', contentHash }
    }));

    return {
      translatedText: cached,
      sourceLanguage,
      targetLanguage,
      cached: true,
      clientCached: true,
    };
  }

  // Dispatch cache miss event for performance monitoring
  window.dispatchEvent(new CustomEvent('cache-miss', {
    detail: { type: 'client', contentHash }
  }));

  // Prepare request payload
  const requestBody = {
    text,
    source_language: sourceLanguage,
    target_language: targetLanguage,
    content_hash: contentHash,
    preserve_html: preserveHtml,
  };

  try {
    // Make API request with timeout
    const response = await fetchWithTimeout(
      TRANSLATION_ENDPOINT,
      {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      },
      REQUEST_TIMEOUT_MS
    );

    // Handle non-2xx responses
    if (!response.ok) {
      let errorBody = {};
      try {
        errorBody = await response.json();
      } catch (parseErr) {
        // Response body is not JSON
        console.error('Failed to parse error response:', parseErr);
      }

      const errorInfo = mapHttpErrorToCode(response.status, errorBody);
      throw new TranslationError(
        errorInfo.message,
        errorInfo.code,
        errorInfo.retryAfter
      );
    }

    // Parse successful response
    const data = await response.json();

    // Validate response structure
    if (!data.translated_text) {
      throw new TranslationError(
        'Invalid response format from server',
        ErrorCodes.UNKNOWN_ERROR
      );
    }

    // Cache the result (client-side)
    saveToCache(contentHash, data.translated_text, sourceLanguage, targetLanguage);

    // Dispatch server cache event for performance monitoring
    if (data.cached) {
      window.dispatchEvent(new CustomEvent('cache-hit', {
        detail: { type: 'server', contentHash }
      }));
    }

    console.log('[TranslationClient] Translation successful', {
      cached: data.cached,
      serverCached: data.cached,
      clientCached: false,
    });

    return {
      translatedText: data.translated_text,
      sourceLanguage: data.source_language,
      targetLanguage: data.target_language,
      cached: data.cached,
      serverCached: data.cached,
      clientCached: false,
      modelUsed: data.model_used,
      timestamp: data.timestamp,
    };
  } catch (err) {
    // If it's already a TranslationError, rethrow it
    if (err instanceof TranslationError) {
      throw err;
    }

    // Handle network errors
    if (err.name === 'TypeError' && err.message.includes('fetch')) {
      throw new TranslationError(
        'Network error: Unable to reach translation service. Please check your connection.',
        ErrorCodes.NETWORK_ERROR
      );
    }

    // Handle other unexpected errors
    console.error('[TranslationClient] Unexpected error:', err);
    throw new TranslationError(
      `Unexpected error: ${err.message}`,
      ErrorCodes.UNKNOWN_ERROR
    );
  }
}

/**
 * Get translation service health status
 * @returns {Promise<Object>} Health status
 */
export async function getServiceHealth() {
  try {
    const response = await fetchWithTimeout(
      `${API_BASE_URL}/api/translate/health`,
      { method: 'GET' },
      5000 // 5 second timeout for health check
    );

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    return await response.json();
  } catch (err) {
    console.error('[TranslationClient] Health check failed:', err);
    return {
      status: 'unavailable',
      error: err.message,
    };
  }
}

/**
 * Get cache statistics from server
 * @returns {Promise<Object>} Cache statistics
 */
export async function getCacheStats() {
  try {
    const response = await fetchWithTimeout(
      `${API_BASE_URL}/api/cache/stats`,
      { method: 'GET' },
      5000
    );

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    return await response.json();
  } catch (err) {
    console.error('[TranslationClient] Failed to get cache stats:', err);
    return null;
  }
}

/**
 * Clear server-side translation cache
 * @returns {Promise<boolean>} Success status
 */
export async function clearServerCache() {
  try {
    const response = await fetchWithTimeout(
      `${API_BASE_URL}/api/cache`,
      { method: 'DELETE' },
      5000
    );

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const data = await response.json();
    console.log(`[TranslationClient] Cleared ${data.cleared_entries} server cache entries`);
    return true;
  } catch (err) {
    console.error('[TranslationClient] Failed to clear server cache:', err);
    return false;
  }
}

export default {
  translateText,
  getServiceHealth,
  getCacheStats,
  clearServerCache,
  ErrorCodes,
  TranslationError,
};
