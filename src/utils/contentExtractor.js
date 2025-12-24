/**
 * Content Extractor Utility
 *
 * Extracts translatable content from documentation pages.
 * Functions:
 * - extractContent(): Get HTML from .markdown container
 * - sanitizeHTML(): Prevent XSS before rendering translated content
 */

import DOMPurify from 'dompurify';

/**
 * Extract translatable content from the documentation page
 *
 * Targets the .markdown container which holds Docusaurus page content.
 * Excludes navigation, sidebar, and other UI elements.
 *
 * @returns {string|null} HTML content to translate, or null if not found
 */
export function extractContent() {
  // Target the main content container in Docusaurus
  // Try multiple selectors in order of preference
  const selectors = [
    '.markdown',               // Docusaurus v2/v3 main content
    'article .markdown',       // More specific
    'main .theme-doc-markdown', // Alternative selector
    'article',                 // Fallback to article
    'main',                    // Last resort fallback
  ];

  for (const selector of selectors) {
    const container = document.querySelector(selector);
    if (container) {
      console.log(`[ContentExtractor] Found content with selector: ${selector}`);
      return container.innerHTML;
    }
  }

  console.warn('[ContentExtractor] Could not find content container');
  return null;
}

/**
 * Replace content in the documentation page
 *
 * Updates the .markdown container with translated content.
 * Sanitizes HTML before injection to prevent XSS.
 *
 * @param {string} translatedHTML - Translated HTML content (sanitized)
 * @returns {boolean} Success status
 */
export function replaceContent(translatedHTML) {
  // Sanitize before injection
  const sanitized = sanitizeHTML(translatedHTML);

  // Find the same container used for extraction
  const selectors = [
    '.markdown',
    'article .markdown',
    'main .theme-doc-markdown',
    'article',
    'main',
  ];

  for (const selector of selectors) {
    const container = document.querySelector(selector);
    if (container) {
      console.log(`[ContentExtractor] Replacing content in: ${selector}`);
      container.innerHTML = sanitized;
      return true;
    }
  }

  console.error('[ContentExtractor] Could not find container to replace content');
  return false;
}

/**
 * Sanitize HTML to prevent XSS attacks
 *
 * Uses DOMPurify to clean HTML while preserving safe tags and attributes.
 * Removes scripts, event handlers, and potentially dangerous content.
 *
 * @param {string} html - HTML string to sanitize
 * @returns {string} Sanitized HTML
 */
export function sanitizeHTML(html) {
  if (!html || typeof html !== 'string') {
    return '';
  }

  // Configure DOMPurify with safe defaults
  const config = {
    // Allow common safe tags
    ALLOWED_TAGS: [
      'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
      'p', 'br', 'hr',
      'strong', 'em', 'u', 'i', 'b', 'mark', 'small',
      'a', 'img',
      'ul', 'ol', 'li',
      'table', 'thead', 'tbody', 'tr', 'th', 'td',
      'blockquote', 'pre', 'code',
      'div', 'span',
      'section', 'article', 'aside', 'header', 'footer',
    ],
    // Allow safe attributes
    ALLOWED_ATTR: [
      'href', 'src', 'alt', 'title',
      'class', 'id',
      'width', 'height',
      'target', 'rel',
      'colspan', 'rowspan',
    ],
    // Force external links to open in new tab with noopener
    ADD_ATTR: ['target', 'rel'],
    // Remove data attributes that might contain malicious code
    FORBID_ATTR: ['onerror', 'onload', 'onclick'],
    // Keep relative URLs (for internal links)
    ALLOW_DATA_ATTR: false,
    // Return a string (not DOM node)
    RETURN_DOM: false,
    RETURN_DOM_FRAGMENT: false,
  };

  try {
    const sanitized = DOMPurify.sanitize(html, config);
    console.log('[ContentExtractor] HTML sanitized successfully');
    return sanitized;
  } catch (err) {
    console.error('[ContentExtractor] Sanitization failed:', err);
    // On error, return empty string to avoid injecting potentially unsafe content
    return '';
  }
}

/**
 * Get page text content (for debugging/validation)
 *
 * Extracts plain text without HTML tags.
 *
 * @returns {string} Plain text content
 */
export function getTextContent() {
  const html = extractContent();
  if (!html) {
    return '';
  }

  // Create temporary element to extract text
  const temp = document.createElement('div');
  temp.innerHTML = html;

  // Get text content (strips HTML tags)
  const text = temp.textContent || temp.innerText || '';

  return text.trim();
}

/**
 * Validate content extraction
 *
 * Checks if content can be extracted and meets minimum requirements.
 *
 * @returns {Object} Validation result with status and message
 */
export function validateContent() {
  const html = extractContent();

  if (!html) {
    return {
      valid: false,
      error: 'Content container not found',
    };
  }

  if (html.length === 0) {
    return {
      valid: false,
      error: 'Content container is empty',
    };
  }

  if (html.length > 50000) {
    return {
      valid: false,
      error: 'Content exceeds maximum size (50,000 characters)',
      contentLength: html.length,
    };
  }

  return {
    valid: true,
    contentLength: html.length,
  };
}

export default {
  extractContent,
  replaceContent,
  sanitizeHTML,
  getTextContent,
  validateContent,
};
