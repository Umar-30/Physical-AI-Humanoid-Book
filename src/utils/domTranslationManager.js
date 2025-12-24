/**
 * DOM Translation Manager
 *
 * Handles automatic translation of page content when language changes.
 * Integrates with LanguageContext and translationClient.
 */

import { translateText } from '../services/translationClient';

/**
 * Extract text content from DOM element
 * Only extracts direct text content, not nested children that will be translated separately
 * @param {HTMLElement} element - Element to extract text from
 * @returns {string} Extracted text
 */
function extractText(element) {
  // For elements that typically contain only direct text (not nested content)
  const directTextElements = ['P', 'H1', 'H2', 'H3', 'H4', 'H5', 'H6', 'LI', 'TD', 'TH', 'BLOCKQUOTE', 'FIGCAPTION', 'LABEL', 'A', 'SPAN'];

  if (directTextElements.includes(element.tagName)) {
    // Get all text content including nested elements
    return element.textContent.trim();
  }

  // For other elements, just get direct text nodes
  let text = '';
  for (const node of element.childNodes) {
    if (node.nodeType === Node.TEXT_NODE) {
      text += node.textContent;
    }
  }
  return text.trim();
}

/**
 * Check if element should be translated
 * @param {HTMLElement} element - Element to check
 * @returns {boolean} True if should translate
 */
function shouldTranslateElement(element) {
  // Skip if element or text is empty
  if (!element || !element.textContent || element.textContent.trim().length === 0) {
    return false;
  }

  // Skip code blocks, pre tags, and script tags
  const skipTags = ['CODE', 'PRE', 'SCRIPT', 'STYLE', 'NOSCRIPT', 'SVG', 'PATH'];
  if (skipTags.includes(element.tagName)) {
    return false;
  }

  // Skip if marked as no-translate
  if (element.hasAttribute('translate') && element.getAttribute('translate') === 'no') {
    return false;
  }

  // Skip if inside code block or pre tag
  if (element.closest('pre, code, .token, .prism-code')) {
    return false;
  }

  // Skip if it's a navigation element
  if (element.closest('nav, .navbar, .menu, .pagination')) {
    return false;
  }

  // Skip very short text (likely not meaningful content)
  const text = element.textContent.trim();
  if (text.length < 2) {
    return false;
  }

  return true;
}

/**
 * Find all translatable text nodes in the document
 * @returns {Array<{element: HTMLElement, originalText: string}>} List of translatable elements
 */
function findTranslatableElements() {
  const elements = [];

  // More comprehensive selectors to catch all Docusaurus content
  const selectors = [
    // Main article content
    'article p',
    'article h1',
    'article h2',
    'article h3',
    'article h4',
    'article h5',
    'article h6',
    'article li',
    'article td',
    'article th',
    'article blockquote',
    'article figcaption',
    'article span:not(.token):not(.keyword):not(.punctuation)', // Include spans but exclude code tokens
    'article a',
    'article label',
    'article caption',
    'article dt',
    'article dd',

    // Markdown container (Docusaurus uses this)
    '.markdown p',
    '.markdown h1',
    '.markdown h2',
    '.markdown h3',
    '.markdown h4',
    '.markdown h5',
    '.markdown h6',
    '.markdown li',
    '.markdown td',
    '.markdown th',
    '.markdown blockquote',
    '.markdown a',
    '.markdown span:not(.token):not(.keyword):not(.punctuation)',
    '.markdown label',

    // Theme doc content (Docusaurus specific)
    '.theme-doc-markdown p',
    '.theme-doc-markdown h1',
    '.theme-doc-markdown h2',
    '.theme-doc-markdown h3',
    '.theme-doc-markdown h4',
    '.theme-doc-markdown h5',
    '.theme-doc-markdown h6',
    '.theme-doc-markdown li',
    '.theme-doc-markdown td',
    '.theme-doc-markdown th',
    '.theme-doc-markdown blockquote',

    // MDX content
    '.mdxPageWrapper p',
    '.mdxPageWrapper h1',
    '.mdxPageWrapper h2',
    '.mdxPageWrapper h3',
    '.mdxPageWrapper h4',
    '.mdxPageWrapper h5',
    '.mdxPageWrapper h6',
    '.mdxPageWrapper li',

    // Fallback: any main content area
    'main p',
    'main h1',
    'main h2',
    'main h3',
    'main h4',
    'main h5',
    'main h6',
    'main li',
    'main td',
    'main th',
  ];

  // Use a Set to avoid duplicate elements
  const seenElements = new Set();

  selectors.forEach(selector => {
    try {
      const nodeList = document.querySelectorAll(selector);
      nodeList.forEach(element => {
        // Skip if we've already processed this element
        if (seenElements.has(element)) {
          return;
        }

        if (shouldTranslateElement(element)) {
          const text = extractText(element);
          if (text && text.length > 0) {
            elements.push({
              element,
              originalText: text,
            });
            seenElements.add(element);
          }
        }
      });
    } catch (err) {
      console.warn(`[DOMTranslation] Invalid selector: ${selector}`, err);
    }
  });

  console.log(`[DOMTranslation] Found ${elements.length} unique translatable elements`);
  return elements;
}

/**
 * Translate page content to target language
 * @param {string} targetLanguage - Target language code ('en' or 'ur')
 * @param {Function} onProgress - Progress callback (current, total)
 * @returns {Promise<{success: number, failed: number}>} Translation results
 */
export async function translatePageContent(targetLanguage, onProgress = null) {
  // If switching back to English, restore original content
  if (targetLanguage === 'en') {
    restoreOriginalContent();
    return { success: 0, failed: 0 };
  }

  // Find all translatable elements
  const elements = findTranslatableElements();
  console.log(`[DOMTranslation] Found ${elements.length} translatable elements`);

  if (elements.length === 0) {
    return { success: 0, failed: 0 };
  }

  let successCount = 0;
  let failedCount = 0;

  // Process elements in batches for better performance
  const BATCH_SIZE = 10; // Translate 10 elements concurrently
  const batches = [];

  for (let i = 0; i < elements.length; i += BATCH_SIZE) {
    batches.push(elements.slice(i, i + BATCH_SIZE));
  }

  console.log(`[DOMTranslation] Processing ${batches.length} batches of up to ${BATCH_SIZE} elements each`);

  // Process each batch
  for (let batchIndex = 0; batchIndex < batches.length; batchIndex++) {
    const batch = batches[batchIndex];

    // Report batch progress
    if (onProgress) {
      const currentProgress = batchIndex * BATCH_SIZE;
      onProgress(currentProgress, elements.length);
    }

    // Translate all elements in batch concurrently
    const batchPromises = batch.map(async ({ element, originalText }) => {
      try {
        // Store original text as data attribute
        if (!element.hasAttribute('data-original-text')) {
          element.setAttribute('data-original-text', originalText);
        }

        // Translate the text
        const result = await translateText(originalText, 'en', targetLanguage);

        // Update element with translated text
        element.textContent = result.translatedText;

        return { success: true, cached: result.cached };
      } catch (err) {
        console.error(`[DOMTranslation] Failed to translate element:`, err);
        return { success: false, error: err.message };
      }
    });

    // Wait for batch to complete
    const batchResults = await Promise.allSettled(batchPromises);

    // Count successes and failures
    batchResults.forEach((result) => {
      if (result.status === 'fulfilled' && result.value.success) {
        successCount++;
      } else {
        failedCount++;
      }
    });

    console.log(`[DOMTranslation] Batch ${batchIndex + 1}/${batches.length} complete: ${successCount}/${elements.length} total translated`);
  }

  // Final progress report
  if (onProgress) {
    onProgress(elements.length, elements.length);
  }

  console.log(`[DOMTranslation] Complete: ${successCount} success, ${failedCount} failed`);
  return { success: successCount, failed: failedCount };
}

/**
 * Restore original English content
 */
export function restoreOriginalContent() {
  const translatedElements = document.querySelectorAll('[data-original-text]');

  translatedElements.forEach(element => {
    const originalText = element.getAttribute('data-original-text');
    if (originalText) {
      element.textContent = originalText;
    }
    element.removeAttribute('data-original-text');
  });

  console.log(`[DOMTranslation] Restored ${translatedElements.length} elements to English`);
}

/**
 * Clear all translation data attributes
 */
export function clearTranslationData() {
  const translatedElements = document.querySelectorAll('[data-original-text]');
  translatedElements.forEach(element => {
    element.removeAttribute('data-original-text');
  });
}
