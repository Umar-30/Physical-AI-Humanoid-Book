/**
 * Cache Manager for Translation Results
 *
 * Manages browser localStorage cache for translated content.
 * Functions:
 * - getFromCache(): Retrieve cached translation
 * - saveToCache(): Store translation in cache
 * - evictOldest(): LRU eviction when cache > 8MB
 * - generateCacheKey(): MD5 hash of content
 */

import md5 from 'crypto-js/md5';

const CACHE_PREFIX = 'translation_cache_';
const CACHE_METADATA_KEY = 'translation_cache_metadata';
const MAX_CACHE_SIZE_MB = 8;
const MAX_CACHE_SIZE_BYTES = MAX_CACHE_SIZE_MB * 1024 * 1024;

/**
 * Generate MD5 hash for cache key
 * @param {string} content - Content to hash
 * @returns {string} MD5 hash
 */
export function generateCacheKey(content) {
  return md5(content).toString();
}

/**
 * Get cache metadata (access times, sizes)
 * @returns {Object} Metadata object with access times and sizes
 */
function getCacheMetadata() {
  // Check if we're in browser (not SSR)
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return { entries: {} };
  }

  try {
    const metadata = localStorage.getItem(CACHE_METADATA_KEY);
    return metadata ? JSON.parse(metadata) : { entries: {} };
  } catch (err) {
    console.error('Failed to parse cache metadata:', err);
    return { entries: {} };
  }
}

/**
 * Save cache metadata
 * @param {Object} metadata - Metadata object to save
 */
function saveCacheMetadata(metadata) {
  // Check if we're in browser (not SSR)
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return;
  }

  try {
    localStorage.setItem(CACHE_METADATA_KEY, JSON.stringify(metadata));
  } catch (err) {
    console.error('Failed to save cache metadata:', err);
  }
}

/**
 * Estimate total cache size in bytes
 * @returns {number} Estimated cache size in bytes
 */
function estimateCacheSize() {
  let totalSize = 0;
  const metadata = getCacheMetadata();

  for (const key in metadata.entries) {
    totalSize += metadata.entries[key].size || 0;
  }

  return totalSize;
}

/**
 * Evict least recently used entries until cache is under limit
 */
function evictOldest() {
  const metadata = getCacheMetadata();
  const entries = metadata.entries;

  // Sort entries by access time (oldest first)
  const sortedKeys = Object.keys(entries).sort(
    (a, b) => entries[a].lastAccess - entries[b].lastAccess
  );

  let currentSize = estimateCacheSize();
  let evictedCount = 0;

  // Evict entries until cache is under 75% of max size (to avoid thrashing)
  const targetSize = MAX_CACHE_SIZE_BYTES * 0.75;

  while (currentSize > targetSize && sortedKeys.length > 0) {
    const keyToEvict = sortedKeys.shift();
    const fullKey = `${CACHE_PREFIX}${keyToEvict}`;

    try {
      // Remove from localStorage
      localStorage.removeItem(fullKey);

      // Update size
      currentSize -= entries[keyToEvict].size || 0;

      // Remove from metadata
      delete entries[keyToEvict];
      evictedCount++;
    } catch (err) {
      console.error(`Failed to evict cache entry ${keyToEvict}:`, err);
    }
  }

  // Save updated metadata
  saveCacheMetadata(metadata);

  if (evictedCount > 0) {
    console.log(`Evicted ${evictedCount} cache entries (LRU eviction)`);
  }

  return evictedCount;
}

/**
 * Retrieve cached translation
 * @param {string} contentHash - MD5 hash of content
 * @returns {string|null} Cached translation or null if not found
 */
export function getFromCache(contentHash) {
  // Check if we're in browser (not SSR)
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return null;
  }

  try {
    const key = `${CACHE_PREFIX}${contentHash}`;
    const cached = localStorage.getItem(key);

    if (!cached) {
      return null;
    }

    const entry = JSON.parse(cached);

    // Update access time in metadata
    const metadata = getCacheMetadata();
    if (metadata.entries[contentHash]) {
      metadata.entries[contentHash].lastAccess = Date.now();
      saveCacheMetadata(metadata);
    }

    return entry.translatedText;
  } catch (err) {
    console.error('Failed to retrieve from cache:', err);
    return null;
  }
}

/**
 * Store translation in cache with LRU eviction
 * @param {string} contentHash - MD5 hash of content
 * @param {string} translatedText - Translated text to cache
 * @param {string} sourceLanguage - Source language code
 * @param {string} targetLanguage - Target language code
 */
export function saveToCache(contentHash, translatedText, sourceLanguage, targetLanguage) {
  // Check if we're in browser (not SSR)
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return false;
  }

  try {
    // Calculate entry size
    const entry = {
      translatedText,
      sourceLanguage,
      targetLanguage,
      timestamp: Date.now(),
    };
    const entryStr = JSON.stringify(entry);
    const entrySize = new Blob([entryStr]).size;

    // Check if we need to evict entries
    const currentSize = estimateCacheSize();
    if (currentSize + entrySize > MAX_CACHE_SIZE_BYTES) {
      evictOldest();
    }

    // Store in localStorage
    const key = `${CACHE_PREFIX}${contentHash}`;
    localStorage.setItem(key, entryStr);

    // Update metadata
    const metadata = getCacheMetadata();
    metadata.entries[contentHash] = {
      size: entrySize,
      lastAccess: Date.now(),
    };
    saveCacheMetadata(metadata);

    return true;
  } catch (err) {
    console.error('Failed to save to cache:', err);

    // If localStorage is full, try evicting and retry once
    if (err.name === 'QuotaExceededError') {
      console.warn('localStorage quota exceeded, evicting entries...');
      evictOldest();

      try {
        const key = `${CACHE_PREFIX}${contentHash}`;
        const entry = {
          translatedText,
          sourceLanguage,
          targetLanguage,
          timestamp: Date.now(),
        };
        localStorage.setItem(key, JSON.stringify(entry));
        return true;
      } catch (retryErr) {
        console.error('Failed to save to cache after eviction:', retryErr);
      }
    }

    return false;
  }
}

/**
 * Clear all translation cache entries
 * @returns {number} Number of entries cleared
 */
export function clearCache() {
  try {
    const metadata = getCacheMetadata();
    const keys = Object.keys(metadata.entries);

    keys.forEach((hash) => {
      const key = `${CACHE_PREFIX}${hash}`;
      localStorage.removeItem(key);
    });

    // Clear metadata
    localStorage.removeItem(CACHE_METADATA_KEY);

    console.log(`Cleared ${keys.length} cache entries`);
    return keys.length;
  } catch (err) {
    console.error('Failed to clear cache:', err);
    return 0;
  }
}

/**
 * Get cache statistics
 * @returns {Object} Cache statistics (entries, size, oldest)
 */
export function getCacheStats() {
  try {
    const metadata = getCacheMetadata();
    const entries = metadata.entries;
    const entryCount = Object.keys(entries).length;
    const totalSizeMB = (estimateCacheSize() / (1024 * 1024)).toFixed(2);

    let oldestAge = 0;
    if (entryCount > 0) {
      const oldestTimestamp = Math.min(
        ...Object.values(entries).map((e) => e.lastAccess)
      );
      oldestAge = Math.floor((Date.now() - oldestTimestamp) / (1000 * 60 * 60)); // hours
    }

    return {
      totalEntries: entryCount,
      totalSizeMB: parseFloat(totalSizeMB),
      maxSizeMB: MAX_CACHE_SIZE_MB,
      oldestEntryAgeHours: oldestAge,
    };
  } catch (err) {
    console.error('Failed to get cache stats:', err);
    return {
      totalEntries: 0,
      totalSizeMB: 0,
      maxSizeMB: MAX_CACHE_SIZE_MB,
      oldestEntryAgeHours: 0,
    };
  }
}
