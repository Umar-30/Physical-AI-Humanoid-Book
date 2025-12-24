/**
 * SEO Head Component
 *
 * Manages meta tags for bilingual SEO optimization.
 * Updates document title, description, and language meta tags.
 */

import { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useLanguage } from '../contexts/LanguageContext';

/**
 * SEO metadata for both languages
 */
const seoContent = {
  en: {
    titleSuffix: 'Physical AI & Humanoid Robotics',
    description: 'A comprehensive guide to building intelligent humanoid robots with ROS 2, digital twins, and physical AI.',
    keywords: 'robotics, humanoid robots, ROS 2, physical AI, digital twin, robot operating system, AI robotics',
    author: 'Physical AI & Humanoid Robotics Team',
  },
  ur: {
    titleSuffix: 'فزیکل AI اور ہیومنائیڈ روبوٹکس',
    description: 'ROS 2، ڈیجیٹل ٹوئنز، اور فزیکل AI کے ساتھ ذہین ہیومنائیڈ روبوٹس بنانے کے لیے ایک جامع گائیڈ۔',
    keywords: 'روبوٹکس، ہیومنائیڈ روبوٹس، ROS 2، فزیکل AI، ڈیجیٹل ٹوئن، روبوٹ آپریٹنگ سسٹم، AI روبوٹکس',
    author: 'فزیکل AI اور ہیومنائیڈ روبوٹکس ٹیم',
  },
};

/**
 * SEOHead Component
 *
 * Automatically updates page metadata when language changes.
 * Improves SEO for both English and Urdu content.
 */
export default function SEOHead() {
  const { language } = useLanguage();
  const location = useLocation();

  // Only run effects in browser, not during SSR
  const isBrowser = typeof window !== 'undefined';

  useEffect(() => {
    if (!isBrowser) return;
    const content = seoContent[language] || seoContent.en;

    // Update document title
    const currentTitle = document.title;
    const titleWithoutSuffix = currentTitle.split('|')[0].trim();
    document.title = `${titleWithoutSuffix} | ${content.titleSuffix}`;

    // Update or create meta tags
    updateMetaTag('description', content.description);
    updateMetaTag('keywords', content.keywords);
    updateMetaTag('author', content.author);

    // Language and locale meta tags
    updateMetaTag('language', language);
    updateMetaTag('content-language', language === 'ur' ? 'ur-PK' : 'en-US');

    // Open Graph meta tags
    updateMetaProperty('og:locale', language === 'ur' ? 'ur_PK' : 'en_US');
    updateMetaProperty('og:title', document.title);
    updateMetaProperty('og:description', content.description);
    updateMetaProperty('og:type', 'website');

    // Twitter Card meta tags
    updateMetaName('twitter:card', 'summary_large_image');
    updateMetaName('twitter:title', document.title);
    updateMetaName('twitter:description', content.description);

    // Canonical URL (always English version for canonical)
    updateCanonicalLink(location.pathname);

    // Alternate language links
    updateAlternateLinks(location.pathname);

    console.log(`[SEOHead] Updated meta tags for ${language}`);
  }, [language, location.pathname]);

  // This component doesn't render anything
  return null;
}

/**
 * Update or create a meta tag by name
 */
function updateMetaTag(name, content) {
  let meta = document.querySelector(`meta[name="${name}"]`);
  if (!meta) {
    meta = document.createElement('meta');
    meta.setAttribute('name', name);
    document.head.appendChild(meta);
  }
  meta.setAttribute('content', content);
}

/**
 * Update or create a meta tag by property (Open Graph)
 */
function updateMetaProperty(property, content) {
  let meta = document.querySelector(`meta[property="${property}"]`);
  if (!meta) {
    meta = document.createElement('meta');
    meta.setAttribute('property', property);
    document.head.appendChild(meta);
  }
  meta.setAttribute('content', content);
}

/**
 * Update or create a meta tag by name (Twitter)
 */
function updateMetaName(name, content) {
  let meta = document.querySelector(`meta[name="${name}"]`);
  if (!meta) {
    meta = document.createElement('meta');
    meta.setAttribute('name', name);
    document.head.appendChild(meta);
  }
  meta.setAttribute('content', content);
}

/**
 * Update canonical link
 */
function updateCanonicalLink(pathname) {
  let link = document.querySelector('link[rel="canonical"]');
  if (!link) {
    link = document.createElement('link');
    link.setAttribute('rel', 'canonical');
    document.head.appendChild(link);
  }
  // Canonical always points to English version
  const baseUrl = window.location.origin;
  link.setAttribute('href', `${baseUrl}${pathname}`);
}

/**
 * Update alternate language links for SEO
 */
function updateAlternateLinks(pathname) {
  const baseUrl = window.location.origin;

  // Remove existing alternate links
  document.querySelectorAll('link[rel="alternate"]').forEach((link) => link.remove());

  // Add English alternate
  const enLink = document.createElement('link');
  enLink.setAttribute('rel', 'alternate');
  enLink.setAttribute('hreflang', 'en');
  enLink.setAttribute('href', `${baseUrl}${pathname}`);
  document.head.appendChild(enLink);

  // Add Urdu alternate
  const urLink = document.createElement('link');
  urLink.setAttribute('rel', 'alternate');
  urLink.setAttribute('hreflang', 'ur');
  urLink.setAttribute('href', `${baseUrl}${pathname}?lang=ur`);
  document.head.appendChild(urLink);

  // Add x-default (fallback)
  const defaultLink = document.createElement('link');
  defaultLink.setAttribute('rel', 'alternate');
  defaultLink.setAttribute('hreflang', 'x-default');
  defaultLink.setAttribute('href', `${baseUrl}${pathname}`);
  document.head.appendChild(defaultLink);
}
