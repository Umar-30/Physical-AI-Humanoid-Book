# User Guide - Bilingual Documentation System

Welcome to the Physical AI & Humanoid Robotics bilingual documentation! This guide will help you navigate and use the English/Urdu translation features.

## Quick Start

### Switching Languages

1. **Find the language toggle button** in the top navigation bar
2. **Click the button** to switch between English (EN) and Urdu (اردو)
3. **Wait 2-3 seconds** for the first translation (subsequent translations are instant)
4. **Your preference is saved** - the site will remember your language choice

![Language Toggle](docs/images/language-toggle.gif)

### Features

- ✅ **Full Content Translation** - All documentation translates to Urdu
- ✅ **Right-to-Left (RTL) Layout** - Proper Urdu text direction
- ✅ **Beautiful Urdu Typography** - Noto Nastaliq Urdu font
- ✅ **Bilingual Chatbot** - Ask questions in English or Urdu
- ✅ **Instant Caching** - Fast translation after first load
- ✅ **Offline Indicator** - Clear feedback when offline
- ✅ **Keyboard Shortcuts** - Quick navigation

---

## Using the Translation System

### First-Time Translation

When you first switch to Urdu:

1. **Loading indicator appears** - ⏳ icon shows in toggle button
2. **Progress overlay displays** - "Translating page content..."
3. **Content updates** - Page content changes to Urdu
4. **Layout mirrors** - Text aligns right, scrollbars move to left
5. **Font changes** - Beautiful Urdu calligraphy appears

**Duration:** 2-5 seconds depending on page size

### Subsequent Translations

After the first translation:

1. **Instant loading** - Cached translation loads immediately
2. **No waiting** - Near-instant language switching
3. **Smooth transition** - Seamless content update

**Duration:** < 100ms (instant)

### What Gets Translated

✅ **Translated:**
- Headings and titles
- Paragraph text
- List items
- Table content
- Blockquotes
- Button labels
- Navigation items
- Chatbot interface

❌ **Not Translated (Preserved):**
- Code blocks
- Code snippets
- Command examples
- Variable names
- URLs and links
- Numbers

### What Changes in RTL Mode

When you switch to Urdu, the layout transforms:

| Element | English (LTR) | Urdu (RTL) |
|---------|---------------|------------|
| Text alignment | Left | Right |
| Reading direction | Left-to-right | Right-to-left |
| Sidebar position | Left | Right |
| Scrollbar position | Right | Left |
| Navigation flow | →  | ← |
| Table of Contents | Right | Left |

---

## Using the Bilingual Chatbot

### Opening the Chat

**Three ways to open:**

1. **Click the chat icon** at bottom-right corner
2. **Press Alt + C** (keyboard shortcut)
3. **Click "Ask AI"** button in sidebar

### Asking Questions

#### In English:

```
You: What is ROS 2?
Bot: ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...
```

#### In Urdu:

```
آپ: ROS 2 کیا ہے؟
بوٹ: ROS 2 (روبوٹ آپریٹنگ سسٹم 2) روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے...
```

### Chat Features

- **Automatic Language Detection** - Chatbot responds in your current language
- **Source Citations** - See which documentation pages were used
- **Clickable Sources** - Jump directly to relevant pages
- **Conversation History** - Scroll through previous messages
- **Clear Chat** - Reset conversation with "Clear" button

### Sample Questions

**English:**
- "How do I install ROS 2?"
- "What is a digital twin?"
- "Explain physical AI"
- "Show me robot control examples"

**Urdu:**
- "ROS 2 کیسے انسٹال کریں؟"
- "ڈیجیٹل ٹوئن کیا ہے؟"
- "فزیکل AI کی وضاحت کریں"
- "روبوٹ کنٹرول کی مثالیں دکھائیں"

---

## Keyboard Shortcuts

Navigate faster with these shortcuts:

| Shortcut | Action |
|----------|--------|
| `Alt + L` | Toggle language (EN ↔ UR) |
| `Alt + C` | Open/close chatbot |
| `Shift + /` | Jump to main content |
| `Alt + ?` | Show keyboard shortcuts help |
| `Tab` | Navigate between interactive elements |
| `Enter` / `Space` | Activate buttons/links |

**Accessibility Note:** All features work with keyboard only (no mouse needed).

---

## Understanding the Interface

### Language Toggle Button States

| State | Icon | Badge | Meaning |
|-------|------|-------|---------|
| English | 🌐 | EN | Viewing English content |
| Urdu | 🌐 | اردو | Viewing Urdu content |
| Loading | ⏳ | EN/اردو | Translation in progress |
| Error | ⚠️ | EN/اردو | Translation failed |

### Status Indicators

**Translation Progress:**
- **Overlay with message** - "Translating page content..."
- **Progress percentage** (optional) - "Translating... 45%"

**Offline Mode:**
- **Banner at top** - "⚠️ Offline Mode - Translation unavailable"
- **Indicator icon** - 📡 Offline

**Error Messages:**
- **Toast notifications** - Pop-up at bottom-right
- **Retry button** - Click to try translation again
- **Auto-dismiss** - Disappears after 5 seconds

---

## Troubleshooting

### Issue: Translation Not Working

**Symptoms:**
- Click toggle, nothing happens
- Page stays in English

**Solutions:**

1. **Check internet connection**
   - Look for offline indicator
   - Try reloading page

2. **Wait a moment**
   - First translation takes 2-5 seconds
   - Don't click toggle repeatedly

3. **Clear browser cache**
   - Press `Ctrl + Shift + Delete`
   - Select "Cached images and files"
   - Click "Clear data"

4. **Check browser console**
   - Press `F12` to open DevTools
   - Look for error messages
   - Report errors to support

### Issue: Urdu Text Looks Strange

**Symptoms:**
- Boxes (□) instead of Urdu letters
- Jagged or pixelated text

**Solutions:**

1. **Check internet connection**
   - Urdu font loads from Google Fonts
   - Requires internet on first visit

2. **Use supported browser**
   - Chrome 90+
   - Firefox 88+
   - Safari 14+
   - Edge 90+

3. **Update your browser**
   - Use the latest version

### Issue: Layout Not Switching to RTL

**Symptoms:**
- Content translates but stays left-aligned
- Scrollbar on wrong side

**Solutions:**

1. **Wait for full translation**
   - Layout changes after content updates
   - May take 1-2 seconds

2. **Reload page**
   - Press `F5` or `Ctrl + R`

3. **Clear browser cache**

### Issue: Chatbot Not Responding

**Symptoms:**
- Send message, no response
- Loading spinner forever

**Solutions:**

1. **Check backend status**
   - Backend must be running on port 8000
   - Contact system administrator

2. **Check internet connection**

3. **Reload page**
   - Close and reopen chatbot

### Issue: Slow Translation

**Symptoms:**
- Translation takes > 5 seconds
- Page freezes

**Possible Causes:**

1. **First-time load**
   - Model loads on first request (30-60s)
   - Subsequent translations fast

2. **Large page**
   - Pages with lots of text take longer
   - Normal for comprehensive guides

3. **Slow internet**
   - Check connection speed
   - Try on faster network

4. **Server overload**
   - Multiple users translating simultaneously
   - Wait and try again

---

## Best Practices

### For Best Experience

✅ **Do:**
- Use modern browser (Chrome, Firefox, Safari, Edge)
- Have stable internet connection
- Wait for first translation to complete
- Use keyboard shortcuts for faster navigation
- Report issues to help us improve

❌ **Don't:**
- Click language toggle repeatedly
- Use very old browsers (IE11)
- Expect instant translation on first load
- Translate while offline

### Performance Tips

1. **First visit per page** - Allow 2-5s for translation
2. **Return visits** - Instant thanks to caching
3. **Multiple pages** - Switch pages after translation completes
4. **Clear cache if issues** - Refresh can fix problems

### Accessibility Tips

1. **Screen reader users** - Announcements made for language changes
2. **Keyboard users** - All features accessible without mouse
3. **High contrast** - Works with high contrast mode
4. **Zoom** - Text remains readable at 200% zoom

---

## Frequently Asked Questions (FAQ)

### General

**Q: Is the translation automatic?**
A: No, you control when to translate by clicking the language toggle button.

**Q: Will my language choice be remembered?**
A: Yes, your preference is saved in browser storage and persists across visits.

**Q: Can I translate only part of the page?**
A: No, the entire page content translates when you switch languages.

**Q: Does translation work offline?**
A: No, translation requires an internet connection to the backend API.

### Translation Quality

**Q: How accurate is the translation?**
A: We use Facebook's mBART model, state-of-the-art for English-Urdu translation. Technical terms are preserved for accuracy.

**Q: Can I suggest translation improvements?**
A: Yes! Please open an issue on GitHub with the page URL and suggested improvement.

**Q: Why are some words not translated?**
A: Technical terms (ROS 2, API, etc.) are intentionally left in English for clarity and standardization.

### Performance

**Q: Why is the first translation slow?**
A: The translation model needs to load on first use (30-60s). After that, translations are fast.

**Q: How much data does translation use?**
A: First translation of a page: ~50-100KB. Subsequent translations: 0KB (cached).

**Q: Can I preload translations?**
A: Currently no, but you can visit pages in Urdu mode to cache them for later.

### Privacy

**Q: Is my data stored?**
A: Page content is cached locally in your browser for faster reloading. No personal data is collected.

**Q: Can others see what I translate?**
A: No, translations are private to your browser session.

**Q: Does the chatbot store my questions?**
A: Chat history is stored locally in your browser only. Questions are sent to the backend for answers but not permanently stored.

---

## Supported Languages

Currently supported:
- 🇬🇧 **English** (en)
- 🇵🇰 **Urdu** (ur)

**Coming soon:**
- 🇸🇦 Arabic (ar)
- 🇨🇳 Chinese (zh)
- 🇪🇸 Spanish (es)

Want another language? [Request it on GitHub](https://github.com/your-org/roboticAI_book/issues)!

---

## Browser Compatibility

| Browser | Version | Status | Notes |
|---------|---------|--------|-------|
| Chrome | 90+ | ✅ Fully supported | Recommended |
| Firefox | 88+ | ✅ Fully supported | |
| Safari | 14+ | ✅ Fully supported | macOS & iOS |
| Edge | 90+ | ✅ Fully supported | Chromium-based |
| Opera | 76+ | ✅ Fully supported | |
| IE 11 | N/A | ❌ Not supported | Use modern browser |

### Mobile Support

| Platform | Status | Notes |
|----------|--------|-------|
| iOS Safari | ✅ Supported | iOS 14+ |
| Android Chrome | ✅ Supported | Android 8+ |
| Android Firefox | ✅ Supported | Android 8+ |

---

## Getting Help

### Resources

- **User Guide** (this document)
- **Testing Guide** - `TESTING.md`
- **Deployment Guide** - `DEPLOYMENT.md`
- **Technical Documentation** - `docs/`

### Support Channels

- **GitHub Issues**: [Report bugs or request features](https://github.com/your-org/roboticAI_book/issues)
- **Email**: support@yourdomain.com
- **Community Forum**: [Link to forum]

### Reporting Issues

When reporting an issue, please include:

1. **Browser name and version**
2. **Operating system**
3. **Steps to reproduce**
4. **Expected behavior**
5. **Actual behavior**
6. **Screenshots** (if applicable)
7. **Console errors** (press F12, check Console tab)

**Example:**
```
Browser: Chrome 120
OS: Windows 11
Steps: 1. Click language toggle 2. Wait 5 seconds
Expected: Page translates to Urdu
Actual: Error toast appears "Translation failed"
Console: [Error] 500 Internal Server Error
```

---

## Feedback

We value your feedback! Help us improve by:

- ⭐ **Star us on GitHub** if you find this useful
- 📝 **Report bugs** so we can fix them
- 💡 **Suggest features** you'd like to see
- 📚 **Improve documentation** via pull requests
- 🌍 **Request new languages**

---

## Acknowledgments

This bilingual system is made possible by:

- **mBART Model** by Facebook AI (Meta)
- **Transformers** by Hugging Face
- **Docusaurus** by Meta Open Source
- **Noto Nastaliq Urdu Font** by Google Fonts

Special thanks to the open-source community!

---

## Version History

### Version 1.0.0 (Current)
- ✅ English ↔ Urdu translation
- ✅ RTL layout support
- ✅ Bilingual chatbot
- ✅ Client + server caching
- ✅ Error handling with retry
- ✅ Offline detection
- ✅ Keyboard shortcuts
- ✅ Accessibility features
- ✅ Performance monitoring

### Upcoming (Version 1.1.0)
- 🔄 Translation progress percentage
- 🔄 Bulk page translation
- 🔄 Translation export (PDF)
- 🔄 Voice input (Urdu)
- 🔄 Additional languages

---

## Quick Reference Card

**Print this for easy access!**

```
┌─────────────────────────────────────────────────────────────┐
│         Bilingual Documentation - Quick Reference           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Toggle Language:  Click 🌐 button in navbar               │
│                                                             │
│  Keyboard Shortcuts:                                        │
│    Alt + L  →  Toggle language                             │
│    Alt + C  →  Open chat                                   │
│    Shift + / → Jump to content                             │
│                                                             │
│  Status Icons:                                              │
│    🌐 EN    →  English mode                                │
│    🌐 اردو   →  Urdu mode                                  │
│    ⏳      →  Translating...                               │
│    📡      →  Offline                                      │
│                                                             │
│  First Translation:  2-5 seconds                           │
│  Cached Translation: Instant                               │
│                                                             │
│  Need Help?                                                 │
│  📧 support@yourdomain.com                                 │
│  🐙 github.com/your-org/roboticAI_book/issues              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

**Happy Learning! تعلیم مبارک!** 🎓📚

