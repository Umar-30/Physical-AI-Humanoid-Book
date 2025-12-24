/**
 * Accessibility Helper Component
 *
 * Enhances accessibility features for the bilingual documentation site.
 * Provides:
 * - Keyboard navigation support
 * - Screen reader announcements
 * - Focus management
 * - ARIA live regions for dynamic content updates
 */

import { useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { useLanguage } from '../contexts/LanguageContext';

/**
 * AccessibilityHelper Component
 *
 * Handles accessibility concerns:
 * 1. Announces language changes to screen readers
 * 2. Announces page navigation to screen readers
 * 3. Manages focus when content updates
 * 4. Ensures proper keyboard navigation
 */
export default function AccessibilityHelper() {
  const { language, isTranslating } = useLanguage();
  const location = useLocation();
  const liveRegionRef = useRef(null);
  const previousLanguage = useRef(language);
  const previousPath = useRef(location.pathname);

  // Only run effects in browser, not during SSR
  const isBrowser = typeof window !== 'undefined';

  /**
   * Create ARIA live region for announcements
   */
  useEffect(() => {
    if (!isBrowser) return;

    if (!liveRegionRef.current) {
      const liveRegion = document.createElement('div');
      liveRegion.setAttribute('role', 'status');
      liveRegion.setAttribute('aria-live', 'polite');
      liveRegion.setAttribute('aria-atomic', 'true');
      liveRegion.className = 'sr-only'; // Screen reader only
      liveRegion.style.position = 'absolute';
      liveRegion.style.left = '-10000px';
      liveRegion.style.width = '1px';
      liveRegion.style.height = '1px';
      liveRegion.style.overflow = 'hidden';
      document.body.appendChild(liveRegion);
      liveRegionRef.current = liveRegion;

      console.log('[AccessibilityHelper] ARIA live region created');
    }

    return () => {
      if (liveRegionRef.current) {
        document.body.removeChild(liveRegionRef.current);
        liveRegionRef.current = null;
      }
    };
  }, []);

  /**
   * Announce language changes to screen readers
   */
  useEffect(() => {
    if (previousLanguage.current !== language && liveRegionRef.current) {
      const languageNames = {
        en: 'English',
        ur: 'Urdu',
      };

      const announcement = `Language changed to ${languageNames[language] || language}`;
      announce(announcement);

      console.log('[AccessibilityHelper] Announced language change:', announcement);
      previousLanguage.current = language;
    }
  }, [language]);

  /**
   * Announce translation status
   */
  useEffect(() => {
    if (liveRegionRef.current) {
      if (isTranslating) {
        announce('Translating page content, please wait');
      } else {
        // Only announce completion if we were previously translating
        if (previousLanguage.current !== language) {
          announce('Translation complete');
        }
      }
    }
  }, [isTranslating]);

  /**
   * Announce page navigation
   */
  useEffect(() => {
    if (previousPath.current !== location.pathname && liveRegionRef.current) {
      // Wait for page title to update
      setTimeout(() => {
        const pageTitle = document.title;
        announce(`Navigated to ${pageTitle}`);
        console.log('[AccessibilityHelper] Announced navigation:', pageTitle);
      }, 300);

      previousPath.current = location.pathname;
    }
  }, [location.pathname]);

  /**
   * Enhance keyboard navigation
   */
  useEffect(() => {
    const handleKeyboardNavigation = (e) => {
      // Skip to main content with Shift+/ (or ?)
      if (e.key === '?' && e.shiftKey) {
        e.preventDefault();
        const mainContent = document.querySelector('main') || document.querySelector('[role="main"]');
        if (mainContent) {
          mainContent.focus();
          mainContent.scrollIntoView({ behavior: 'smooth' });
          announce('Jumped to main content');
        }
      }

      // Toggle language with Alt+L
      if (e.key === 'l' && e.altKey) {
        e.preventDefault();
        const languageToggle = document.querySelector('[aria-label*="Switch to"]');
        if (languageToggle) {
          languageToggle.click();
        }
      }

      // Open chat with Alt+C
      if (e.key === 'c' && e.altKey) {
        e.preventDefault();
        const chatToggle = document.querySelector('[aria-label*="chat"]');
        if (chatToggle) {
          chatToggle.click();
        }
      }
    };

    document.addEventListener('keydown', handleKeyboardNavigation);
    return () => document.removeEventListener('keydown', handleKeyboardNavigation);
  }, []);

  /**
   * Ensure proper focus management on content updates
   */
  useEffect(() => {
    const handleContentUpdate = () => {
      // When content changes, ensure focus is not lost
      const activeElement = document.activeElement;

      // If focus is on body or null, move to main content
      if (!activeElement || activeElement === document.body) {
        const mainContent = document.querySelector('main') || document.querySelector('[role="main"]');
        if (mainContent) {
          mainContent.setAttribute('tabindex', '-1');
          mainContent.focus();
        }
      }
    };

    // Listen for content translation events
    window.addEventListener('content-translated', handleContentUpdate);
    return () => window.removeEventListener('content-translated', handleContentUpdate);
  }, []);

  /**
   * Add keyboard shortcuts help
   */
  useEffect(() => {
    const handleHelpRequest = (e) => {
      // Show keyboard shortcuts with Alt+?
      if (e.key === '?' && e.altKey) {
        e.preventDefault();
        const shortcuts = [
          'Alt + L: Toggle language',
          'Alt + C: Open chat',
          'Shift + /: Jump to main content',
          'Alt + ?: Show this help',
        ];
        announce(`Keyboard shortcuts: ${shortcuts.join('. ')}`);
      }
    };

    document.addEventListener('keydown', handleHelpRequest);
    return () => document.removeEventListener('keydown', handleHelpRequest);
  }, []);

  /**
   * Ensure all images have alt text
   */
  useEffect(() => {
    const checkImages = () => {
      const images = document.querySelectorAll('img:not([alt])');
      images.forEach((img, index) => {
        console.warn(`[AccessibilityHelper] Image missing alt text:`, img.src);
        // Add generic alt text
        img.setAttribute('alt', `Image ${index + 1}`);
      });
    };

    // Check after page load
    const timer = setTimeout(checkImages, 1000);
    return () => clearTimeout(timer);
  }, [location.pathname]);

  /**
   * Ensure proper heading hierarchy
   */
  useEffect(() => {
    const checkHeadings = () => {
      const headings = Array.from(document.querySelectorAll('h1, h2, h3, h4, h5, h6'));
      let previousLevel = 0;

      headings.forEach((heading) => {
        const currentLevel = parseInt(heading.tagName[1]);

        // Check for skipped levels
        if (currentLevel - previousLevel > 1) {
          console.warn(
            `[AccessibilityHelper] Heading hierarchy issue: jumped from h${previousLevel} to h${currentLevel}`,
            heading
          );
        }

        previousLevel = currentLevel;
      });
    };

    const timer = setTimeout(checkHeadings, 1000);
    return () => clearTimeout(timer);
  }, [location.pathname]);

  return null; // No visual rendering
}

/**
 * Announce message to screen readers
 * @param {string} message - Message to announce
 */
function announce(message) {
  const liveRegion = document.querySelector('[role="status"][aria-live="polite"]');
  if (liveRegion) {
    // Clear and set new message
    liveRegion.textContent = '';
    setTimeout(() => {
      liveRegion.textContent = message;
    }, 100);
  }
}

/**
 * Utility: Check if element is focusable
 * @param {HTMLElement} element - Element to check
 * @returns {boolean} Whether element is focusable
 */
export function isFocusable(element) {
  if (!element) return false;

  const focusableTags = ['A', 'BUTTON', 'INPUT', 'SELECT', 'TEXTAREA'];
  const isFocusableTag = focusableTags.includes(element.tagName);
  const hasTabindex = element.hasAttribute('tabindex') && element.getAttribute('tabindex') !== '-1';

  return (isFocusableTag || hasTabindex) && !element.disabled && element.offsetParent !== null;
}

/**
 * Utility: Get all focusable elements in container
 * @param {HTMLElement} container - Container element
 * @returns {HTMLElement[]} Array of focusable elements
 */
export function getFocusableElements(container = document) {
  const selector = 'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';
  return Array.from(container.querySelectorAll(selector));
}

/**
 * Utility: Trap focus within container (for modals)
 * @param {HTMLElement} container - Container to trap focus in
 * @returns {Function} Cleanup function
 */
export function trapFocus(container) {
  const focusableElements = getFocusableElements(container);
  const firstElement = focusableElements[0];
  const lastElement = focusableElements[focusableElements.length - 1];

  const handleKeyDown = (e) => {
    if (e.key !== 'Tab') return;

    if (e.shiftKey) {
      // Shift + Tab
      if (document.activeElement === firstElement) {
        e.preventDefault();
        lastElement.focus();
      }
    } else {
      // Tab
      if (document.activeElement === lastElement) {
        e.preventDefault();
        firstElement.focus();
      }
    }
  };

  container.addEventListener('keydown', handleKeyDown);

  // Focus first element
  if (firstElement) {
    firstElement.focus();
  }

  // Return cleanup function
  return () => {
    container.removeEventListener('keydown', handleKeyDown);
  };
}
