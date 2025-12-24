# Testing Guide - Bilingual Translation System

This document provides comprehensive testing procedures for the bilingual (English/Urdu) documentation system.

## Table of Contents

1. [Quick Start Testing](#quick-start-testing)
2. [Component Testing](#component-testing)
3. [Integration Testing](#integration-testing)
4. [Accessibility Testing](#accessibility-testing)
5. [Performance Testing](#performance-testing)
6. [Browser Compatibility](#browser-compatibility)
7. [Common Issues & Troubleshooting](#common-issues--troubleshooting)

---

## Quick Start Testing

### Prerequisites

1. **Start the backend server:**
   ```bash
   cd backend
   python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. **Start the frontend development server:**
   ```bash
   npm start
   ```

3. **Open in browser:**
   ```
   http://localhost:3000
   ```

### Basic Smoke Test (5 minutes)

1. ✅ **Page loads successfully** without errors in console
2. ✅ **Language toggle button** appears in navbar
3. ✅ **Click language toggle** - button shows loading state
4. ✅ **Content translates** to Urdu within 2-5 seconds
5. ✅ **Layout switches** to RTL (text aligns right)
6. ✅ **Toggle back** to English - content restores
7. ✅ **Navigate to another page** - translation persists
8. ✅ **Open chatbot** - interface shows in correct language
9. ✅ **Ask question** in Urdu - receives Urdu response

---

## Component Testing

### 1. Language Toggle Component

**Location:** `src/components/LanguageToggle.js`

**Test Cases:**

| Test ID | Description | Steps | Expected Result |
|---------|-------------|-------|-----------------|
| LT-001 | Initial state | Refresh page | Shows "EN" badge, globe icon |
| LT-002 | Click to translate | Click toggle | Shows loading (⏳), then "اردو" badge |
| LT-003 | Click to restore | Click toggle again | Returns to "EN" badge |
| LT-004 | Disabled during translation | Click toggle, immediately click again | Button disabled, no double-translation |
| LT-005 | Keyboard navigation | Tab to button, press Enter | Same as click |
| LT-006 | Screen reader | Use screen reader | Announces "Switch to Urdu" |

**Manual Test:**
```bash
# 1. Open DevTools Console
# 2. Click language toggle
# 3. Verify console logs:
[LanguageToggle] Toggle clicked, current language: en
[LanguageContext] Language changing from en to ur
[TranslationClient] Cache miss (client)
[TranslationClient] Translation successful
```

---

### 2. Translation Content Component

**Location:** `src/components/TranslatedContent.js`

**Test Cases:**

| Test ID | Description | Steps | Expected Result |
|---------|-------------|-------|-----------------|
| TC-001 | Extract content | Toggle language | Paragraph text extracted, code blocks excluded |
| TC-002 | Replace content | Wait for translation | Translated text replaces original |
| TC-003 | Restore original | Toggle back to EN | Original English text restored |
| TC-004 | Page navigation | Navigate to new page, toggle to UR | New page translates |
| TC-005 | Cache hit | Toggle to UR, back to EN, back to UR | Third toggle instant (cached) |
| TC-006 | Error handling | Disconnect network, toggle | Error toast appears |

**Test Script:**
```javascript
// Run in DevTools console
(async function testTranslation() {
  // Get current content
  const originalContent = document.querySelector('article').innerText;
  console.log('Original:', originalContent.substring(0, 100));

  // Toggle language
  document.querySelector('[aria-label*="Switch"]').click();

  // Wait for translation
  await new Promise(r => setTimeout(r, 3000));

  const translatedContent = document.querySelector('article').innerText;
  console.log('Translated:', translatedContent.substring(0, 100));

  // Verify content changed
  console.assert(originalContent !== translatedContent, 'Content should change');

  // Verify RTL
  const dir = document.documentElement.getAttribute('dir');
  console.assert(dir === 'rtl', 'Dir should be rtl');

  console.log('✅ Translation test passed');
})();
```

---

### 3. Chatbot Component

**Location:** `src/components/ChatWidget.js`

**Test Cases:**

| Test ID | Description | Steps | Expected Result |
|---------|-------------|-------|-----------------|
| CB-001 | Open chatbot | Click chat button | Panel slides in from right |
| CB-002 | English query | Type "What is ROS 2?", send | Receives English answer |
| CB-003 | Switch to Urdu | Toggle language, ask "ROS 2 کیا ہے؟" | Receives Urdu answer |
| CB-004 | Language indicator | Check header | Shows "🌐 EN" or "🌐 اردو" |
| CB-005 | RTL layout | Switch to Urdu | Chat messages align right |
| CB-006 | Sources display | Check answer sources | Shows clickable source links |

**Chatbot Test Queries:**

```
English:
- "What is ROS 2?"
- "How do I install ROS 2?"
- "Explain physical AI"

Urdu:
- "ROS 2 کیا ہے؟"
- "ROS 2 کیسے انسٹال کریں؟"
- "فزیکل AI کی وضاحت کریں"
```

---

### 4. Error Handling

**Location:** `src/components/ErrorToast.js`

**Test Cases:**

| Test ID | Description | Steps | Expected Result |
|---------|-------------|-------|-----------------|
| EH-001 | Network error | Disconnect internet, toggle language | Shows "Network error" toast |
| EH-002 | Timeout error | Simulate slow network | Shows "Timeout" toast after 10s |
| EH-003 | Retry button | Click retry in error toast | Retries translation |
| EH-004 | Auto-dismiss | Wait 5 seconds | Toast auto-dismisses |
| EH-005 | Multiple errors | Trigger multiple errors quickly | Shows stacked toasts |

**Simulate Network Error:**
```javascript
// Run in DevTools console
// Block translation endpoint
const originalFetch = window.fetch;
window.fetch = function(...args) {
  if (args[0].includes('/translate')) {
    return Promise.reject(new Error('Network error'));
  }
  return originalFetch(...args);
};

// Now toggle language to trigger error
```

---

## Integration Testing

### End-to-End User Flows

#### Flow 1: New User First Visit

```
1. User lands on homepage (English)
2. User reads introduction
3. User clicks language toggle
4. Page translates to Urdu (2-3s)
5. User navigates to "Installation" page
6. Page auto-translates to Urdu (cached or 2-3s)
7. User opens chatbot
8. User asks question in Urdu
9. Chatbot responds in Urdu
10. User clicks source link
11. Source page opens in Urdu
```

**Expected Duration:** 30-45 seconds
**Cache Benefit:** Steps 6 reduces from 3s → instant (if cached)

#### Flow 2: Returning User (Cache Hit)

```
1. User returns (language preference saved)
2. Page loads in previously selected language
3. User navigates between pages
4. All translations instant (cache hits)
5. No loading states shown
```

**Expected Duration:** 0s translation time (all cached)

#### Flow 3: Error Recovery

```
1. User toggles language
2. Network fails mid-translation
3. Error toast appears
4. User clicks "Retry"
5. Network restored
6. Translation completes successfully
```

---

## Accessibility Testing

### WCAG 2.1 AA Compliance

#### Keyboard Navigation

| Action | Keyboard Shortcut | Expected Result |
|--------|-------------------|-----------------|
| Jump to content | `Shift + /` | Focus moves to main content |
| Toggle language | `Alt + L` | Language switches |
| Open chat | `Alt + C` | Chat panel opens |
| Show shortcuts | `Alt + ?` | Announces available shortcuts |

**Test Script:**
```bash
# 1. Use keyboard only (no mouse)
# 2. Tab through all interactive elements
# 3. Verify focus indicator visible on all elements
# 4. Press Enter/Space on buttons
# 5. Test all keyboard shortcuts
```

#### Screen Reader Testing

**Tools:** NVDA (Windows), VoiceOver (Mac), TalkBack (Android)

**Test Cases:**

| Test ID | Description | Expected Announcement |
|---------|-------------|----------------------|
| SR-001 | Language toggle | "Switch to Urdu button" |
| SR-002 | Language change | "Language changed to Urdu" |
| SR-003 | Translation progress | "Translating page content, please wait" |
| SR-004 | Translation complete | "Translation complete" |
| SR-005 | Page navigation | "Navigated to [Page Title]" |
| SR-006 | Error | "[Error message]" |

**NVDA Test Commands:**
```
NVDA + Down Arrow = Read next line
NVDA + H = Next heading
NVDA + K = Next link
NVDA + B = Next button
```

#### Color Contrast

All text must meet WCAG AA contrast ratio:
- Normal text: 4.5:1 minimum
- Large text (18pt+): 3:1 minimum

**Test Tool:** Chrome DevTools Lighthouse

```bash
# 1. Open DevTools (F12)
# 2. Go to "Lighthouse" tab
# 3. Select "Accessibility"
# 4. Click "Generate report"
# 5. Score should be 90+ (100 ideal)
```

---

## Performance Testing

### Translation Performance Benchmarks

| Metric | Target | Critical |
|--------|--------|----------|
| First translation | < 3s | < 5s |
| Cached translation | < 100ms | < 500ms |
| Page load (cold) | < 3s | < 8s |
| Page load (cached) | < 1s | < 3s |

### Cache Performance Test

```javascript
// Run in DevTools console
async function testCachePerformance() {
  const iterations = 5;
  const results = [];

  for (let i = 0; i < iterations; i++) {
    const start = performance.now();

    // Toggle to Urdu
    document.querySelector('[aria-label*="Switch"]').click();
    await new Promise(r => setTimeout(r, 100));

    // Wait for translation
    await new Promise(resolve => {
      const check = setInterval(() => {
        if (!document.documentElement.lang !== 'ur') {
          clearInterval(check);
          resolve();
        }
      }, 100);
    });

    const duration = performance.now() - start;
    results.push(duration);

    console.log(`Iteration ${i + 1}: ${duration.toFixed(0)}ms`);

    // Toggle back to English
    document.querySelector('[aria-label*="Switch"]').click();
    await new Promise(r => setTimeout(r, 500));
  }

  const avg = results.reduce((a, b) => a + b) / results.length;
  console.log(`Average: ${avg.toFixed(0)}ms`);
  console.log(`Min: ${Math.min(...results).toFixed(0)}ms`);
  console.log(`Max: ${Math.max(...results).toFixed(0)}ms`);

  // First iteration should be slow (cache miss)
  // Subsequent iterations should be fast (cache hit)
  console.assert(results[0] > 1000, 'First iteration should be slow');
  console.assert(results[4] < 500, 'Last iteration should be fast (cached)');

  console.log('✅ Cache performance test passed');
}

testCachePerformance();
```

### Memory Usage Test

```javascript
// Monitor cache size
function checkCacheSize() {
  let totalSize = 0;
  for (let key in localStorage) {
    if (key.startsWith('translation_cache_')) {
      totalSize += localStorage[key].length * 2; // UTF-16 = 2 bytes per char
    }
  }
  const sizeMB = (totalSize / (1024 * 1024)).toFixed(2);
  console.log(`Cache size: ${sizeMB} MB`);
  return sizeMB;
}

// Should stay under 8MB limit
setInterval(checkCacheSize, 5000);
```

---

## Browser Compatibility

### Supported Browsers

| Browser | Min Version | Status | Notes |
|---------|-------------|--------|-------|
| Chrome | 90+ | ✅ Fully supported | Recommended |
| Firefox | 88+ | ✅ Fully supported | |
| Safari | 14+ | ✅ Fully supported | iOS 14+ |
| Edge | 90+ | ✅ Fully supported | Chromium-based |
| Opera | 76+ | ✅ Fully supported | |
| IE 11 | N/A | ❌ Not supported | Use modern browser |

### Cross-Browser Test Checklist

For each browser:

- [ ] Page loads without errors
- [ ] Language toggle works
- [ ] Translation displays correctly
- [ ] RTL layout renders properly
- [ ] Urdu font displays correctly
- [ ] Code blocks remain LTR
- [ ] Chatbot functions
- [ ] Error toasts appear
- [ ] Keyboard shortcuts work
- [ ] Local storage persists language

---

## Common Issues & Troubleshooting

### Issue 1: Translation Not Working

**Symptoms:**
- Click toggle, nothing happens
- Console shows errors

**Checklist:**
1. ✅ Backend server running on port 8000?
2. ✅ CORS enabled on backend?
3. ✅ `REACT_APP_API_URL` set in `.env`?
4. ✅ Network tab shows 200 response?

**Fix:**
```bash
# Check backend is running
curl http://localhost:8000/api/translate/health

# Should return:
{
  "status": "healthy",
  "model_loaded": true
}
```

### Issue 2: Urdu Font Not Displaying

**Symptoms:**
- Urdu text shows boxes or question marks

**Checklist:**
1. ✅ Internet connection (Google Fonts loading)?
2. ✅ Browser supports web fonts?
3. ✅ Check DevTools Network tab for font errors

**Fix:**
```css
/* Verify in DevTools Elements > Computed */
/* Should see: */
font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", ...
```

### Issue 3: Layout Not Switching to RTL

**Symptoms:**
- Content translates but stays left-aligned

**Checklist:**
1. ✅ `html` element has `dir="rtl"` attribute?
2. ✅ `rtl.css` loaded in DevTools Sources?
3. ✅ Browser cache cleared?

**Fix:**
```javascript
// Force RTL in console
document.documentElement.setAttribute('dir', 'rtl');
document.documentElement.setAttribute('lang', 'ur');
```

### Issue 4: Cache Not Working

**Symptoms:**
- Every translation takes 2-3 seconds
- No "cache hit" logs in console

**Checklist:**
1. ✅ LocalStorage not full (check quota)?
2. ✅ Private browsing mode disabled?
3. ✅ Cache functions exist in cacheManager.js?

**Fix:**
```javascript
// Clear cache and retry
localStorage.clear();
location.reload();
```

### Issue 5: Chat Not Responding

**Symptoms:**
- Chat input doesn't send
- No bot response

**Checklist:**
1. ✅ RAG backend endpoint running?
2. ✅ Check DevTools Network for `/query` request
3. ✅ Console errors related to fetch?

**Fix:**
```bash
# Test RAG endpoint directly
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "top_k": 5, "model": "gpt-4o-mini"}'
```

---

## Automated Testing (Future)

### Unit Tests (Jest + React Testing Library)

```bash
# Install dependencies
npm install --save-dev @testing-library/react @testing-library/jest-dom jest

# Run tests
npm test
```

**Example Test:**
```javascript
// src/components/__tests__/LanguageToggle.test.js
import { render, screen, fireEvent } from '@testing-library/react';
import { LanguageProvider } from '../../contexts/LanguageContext';
import LanguageToggle from '../LanguageToggle';

test('toggles language when clicked', () => {
  render(
    <LanguageProvider>
      <LanguageToggle />
    </LanguageProvider>
  );

  const toggle = screen.getByRole('button');
  expect(toggle).toHaveTextContent('EN');

  fireEvent.click(toggle);
  expect(toggle).toHaveTextContent('اردو');
});
```

### E2E Tests (Playwright)

```bash
# Install Playwright
npm install --save-dev @playwright/test

# Run E2E tests
npx playwright test
```

**Example Test:**
```javascript
// tests/translation.spec.js
import { test, expect } from '@playwright/test';

test('translates page to Urdu', async ({ page }) => {
  await page.goto('http://localhost:3000');

  // Click language toggle
  await page.click('[aria-label*="Switch"]');

  // Wait for translation
  await page.waitForSelector('[dir="rtl"]');

  // Verify RTL layout
  const dir = await page.getAttribute('html', 'dir');
  expect(dir).toBe('rtl');

  // Verify Urdu content
  const content = await page.textContent('article');
  expect(content).toMatch(/[\u0600-\u06FF]/); // Urdu Unicode range
});
```

---

## Performance Monitoring

### Production Monitoring

**Recommended Tools:**
- Google Analytics 4 (GA4) for page views
- Sentry for error tracking
- Lighthouse CI for performance regression

**Key Metrics to Track:**
- Translation success rate
- Average translation duration
- Cache hit rate
- Error rate by type
- User language preference distribution

**Example GA4 Event:**
```javascript
// Already implemented in PerformanceMonitor.js
gtag('event', 'translation_timing', {
  event_category: 'Performance',
  event_label: 'ur',
  value: 2340, // milliseconds
});
```

---

## Summary

This testing guide covers:
- ✅ Manual component testing
- ✅ Integration testing flows
- ✅ Accessibility compliance (WCAG 2.1 AA)
- ✅ Performance benchmarks
- ✅ Browser compatibility
- ✅ Common troubleshooting
- ✅ Future automated testing setup

**Recommended Testing Frequency:**
- Before each deployment: Full smoke test (5 min)
- Weekly: Accessibility audit
- Monthly: Performance benchmark
- On browser updates: Cross-browser compatibility

**Estimated Total Testing Time:** 2-3 hours for comprehensive manual testing

For automated testing setup, refer to the [Future] sections and implement unit/E2E tests as the project matures.
