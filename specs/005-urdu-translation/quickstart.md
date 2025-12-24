# Quickstart Guide: Urdu Translation Agent Integration

**Feature**: 005-urdu-translation
**Audience**: Developers implementing the feature
**Time to Complete**: 10-15 minutes (reading and setup)

## Overview

This quickstart guide provides developers with everything needed to understand and implement the Urdu Translation feature. It consolidates key information from the specification, plan, and research documents.

## What This Feature Does

Adds dynamic Urdu translation to the Physical AI & Humanoid Robotics documentation site, allowing users to:

1. **Toggle Language**: Click "اردو" button in navbar to switch content to Urdu
2. **Read Translated Docs**: View all documentation pages in Urdu with RTL text direction
3. **Use Urdu Chatbot**: Get chatbot responses in Urdu when in Urdu mode
4. **Fast Repeat Visits**: Instant page loads from cache (<200ms)

## Architecture at a Glance

```
┌─────────────────────┐
│   Docusaurus UI     │  Language toggle button
└──────────┬──────────┘
           │ Click "اردو"
           ↓
┌─────────────────────┐
│  LanguageContext    │  React Context (state management)
│  (Frontend)         │
└──────────┬──────────┘
           │ Request translation
           ↓
┌─────────────────────┐
│  localStorage Cache │  Client-side cache (instant load)
│  (Browser)          │
└──────────┬──────────┘
           │ Cache MISS
           ↓
┌─────────────────────┐
│  Translation API    │  POST /api/translate
│  (FastAPI Backend)  │
└──────────┬──────────┘
           │ Check server cache
           ↓
┌─────────────────────┐
│  mBART Model        │  Hugging Face translation model
│  (GPU Inference)    │
└──────────┬──────────┘
           │ Return Urdu text
           ↓
┌─────────────────────┐
│  Render Urdu Page   │  Apply dir="rtl", display content
└─────────────────────┘
```

## Key Technologies

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Frontend** | React (Docusaurus) | UI and state management |
| **Backend** | FastAPI + Python | Translation service API |
| **Translation** | Hugging Face mBART | English → Urdu translation |
| **Cache (Client)** | Browser localStorage | Instant repeat page loads |
| **Cache (Server)** | Python dict (memory) | Deduplication across users |
| **RTL Support** | CSS `dir="rtl"` | Right-to-left text rendering |

## Implementation Phases

### Phase 1: Backend Translation Service (Week 1)

**Goal**: Create `/api/translate` endpoint that translates English to Urdu

**Tasks**:
1. Install Hugging Face transformers: `pip install transformers torch`
2. Load mBART model: `facebook/mbart-large-50-many-to-many-mmt`
3. Create FastAPI endpoint `POST /api/translate`
4. Implement server-side caching (Python dict)
5. Add health check endpoint `GET /api/translate/health`

**Success Criteria**:
- API returns Urdu text for English input
- p95 latency < 2 seconds
- Health check passes

**Key Files**:
- `backend/services/translation_service.py`
- `backend/routers/translation.py`
- `backend/tests/test_translation.py`

---

### Phase 2: Frontend Language Toggle (Week 1)

**Goal**: Add language toggle button to navbar, manage language state

**Tasks**:
1. Create `LanguageContext.js` (React Context for language state)
2. Create `LanguageToggle.js` component (button in navbar)
3. Swizzle Docusaurus navbar: `npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap`
4. Add localStorage persistence for language preference

**Success Criteria**:
- Toggle button visible in navbar
- Clicking toggle updates React Context
- Language preference persists across page reloads

**Key Files**:
- `src/contexts/LanguageContext.js`
- `src/components/LanguageToggle.js`
- `src/theme/Navbar/Content/index.js`

---

### Phase 3: Content Translation (Week 2)

**Goal**: Translate page content dynamically when user selects Urdu

**Tasks**:
1. Swizzle DocItemContent: `npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap`
2. Create `translationClient.js` (fetch wrapper for `/api/translate`)
3. Create `contentExtractor.js` (extract HTML, exclude code blocks)
4. Create `cacheManager.js` (localStorage cache management)
5. Implement RTL support (CSS `dir="rtl"`)

**Success Criteria**:
- Page content translates to Urdu on toggle
- Code blocks remain in English
- RTL text direction applied
- Cache hit loads instantly (<200ms)

**Key Files**:
- `src/theme/DocItem/Content/index.js`
- `src/services/translationClient.js`
- `src/utils/contentExtractor.js`
- `src/utils/cacheManager.js`

---

### Phase 4: Chatbot Integration (Week 2)

**Goal**: Chatbot provides Urdu responses when Urdu mode active

**Tasks**:
1. Extend chatbot API: Add `language` parameter to `/query` endpoint
2. Translate retrieved chunks before passing to LLM
3. Add LLM instruction: "Answer in Urdu language"
4. Update frontend ChatWidget to pass language context

**Success Criteria**:
- Chatbot responds in Urdu when Urdu mode active
- Source page titles translated to Urdu
- Accuracy > 95% (manual validation)

**Key Files**:
- `backend/rag_agent.py`
- `src/components/ChatWidget.js`
- `backend/tests/test_rag_urdu.py`

---

### Phase 5: Navigation Translation (Week 3)

**Goal**: Translate sidebar, breadcrumbs, and UI elements

**Tasks**:
1. Create `ui-strings.json` (translation map for UI labels)
2. Swizzle Sidebar component
3. Swizzle Breadcrumbs component
4. Apply translations based on language context

**Success Criteria**:
- Sidebar items translated
- Breadcrumbs show Urdu page titles
- All navigation functional

**Key Files**:
- `src/translations/ui-strings.json`
- `src/theme/DocSidebar/index.js`
- `src/theme/DocBreadcrumbs/index.js`

---

### Phase 6: Testing & Validation (Week 3)

**Goal**: Ensure all requirements pass tests

**Tasks**:
1. Unit tests (translation service, client cache)
2. Integration tests (frontend-backend flow)
3. E2E tests (user workflows with Playwright)
4. Visual regression tests (layout consistency)
5. Performance tests (latency, cache hit rate)
6. Translation quality review (50 pages, Urdu speaker)

**Success Criteria**:
- All 20 functional requirements pass
- p95 latency < 3 seconds
- Cache hit rate > 80%
- Translation quality > 80%

---

## Critical Decisions Summary

### 1. Translation Service: Hugging Face mBART
- **Why**: Free, self-hosted, 80-85% quality acceptable for technical docs
- **Fallback**: Google Translate API if quality issues arise

### 2. Architecture: Server-Side Translation
- **Why**: GPU inference faster (2s vs 6s), smaller client bundle
- **Benefit**: Works on mobile browsers

### 3. Caching: Two-Tier (Client + Server)
- **Why**: Client cache = instant loads, Server cache = cost efficiency
- **Result**: 80%+ cache hit rate expected

### 4. RTL Handling: Scoped to Content Area
- **Why**: Preserves navbar/UI layout, only article content in RTL
- **Implementation**: `dir="rtl"` on `.markdown` container

### 5. Code Blocks: Excluded from Translation
- **Why**: Preserve syntax integrity, avoid breaking code examples
- **Future**: Phase 2 may add comment translation

### 6. Chatbot: Language Context Parameter
- **Why**: Single translation (better quality than double translation)
- **API**: Add `language: "ur"` param to `/query` endpoint

---

## API Contracts Quick Reference

### Translation API

**Endpoint**: `POST /api/translate`

**Request**:
```json
{
  "text": "<p>Welcome to the guide</p>",
  "source_language": "en",
  "target_language": "ur",
  "preserve_html": true
}
```

**Response**:
```json
{
  "translated_text": "<p>گائیڈ میں خوش آمدید</p>",
  "cached": false,
  "model_used": "facebook/mbart-large-50",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

---

### Chatbot API Extension

**Endpoint**: `POST /query` (existing, extended)

**Request (NEW language param)**:
```json
{
  "query": "ROS 2 کیا ہے؟",
  "top_k": 5,
  "language": "ur"
}
```

**Response**:
```json
{
  "answer": "ROS 2 ایک جدید روبوٹک آپریٹنگ سسٹم ہے...",
  "sources": [
    {"url": "...", "page_title": "ماڈیول 1", "score": 0.92}
  ]
}
```

---

## Code Examples

### React: Using Language Context

```javascript
import { useLanguage } from '@site/src/contexts/LanguageContext';

function MyComponent() {
  const { currentLanguage, setLanguage, isTranslating } = useLanguage();

  return (
    <button onClick={() => setLanguage('ur')} disabled={isTranslating}>
      {currentLanguage === 'en' ? 'اردو' : 'English'}
    </button>
  );
}
```

### Backend: Translation Service

```python
from transformers import MBartForConditionalGeneration, MBart50TokenizerFast

model = MBartForConditionalGeneration.from_pretrained("facebook/mbart-large-50-many-to-many-mmt")
tokenizer = MBart50TokenizerFast.from_pretrained("facebook/mbart-large-50-many-to-many-mmt")

def translate_text(text: str, source_lang: str = "en_XX", target_lang: str = "ur_PK") -> str:
    tokenizer.src_lang = source_lang
    encoded = tokenizer(text, return_tensors="pt")
    generated = model.generate(**encoded, forced_bos_token_id=tokenizer.lang_code_to_id[target_lang])
    translated = tokenizer.batch_decode(generated, skip_special_tokens=True)[0]
    return translated
```

### CSS: RTL Support

```css
/* Apply RTL to content area only */
.markdown[dir="rtl"] {
  direction: rtl;
  text-align: right;
}

/* Keep code blocks LTR */
.markdown[dir="rtl"] pre,
.markdown[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}
```

---

## Common Pitfalls & Solutions

### Pitfall 1: Code Blocks Getting Translated

**Problem**: Translation API translates code syntax, breaking examples

**Solution**:
```python
# Exclude code blocks before translation
def extract_translatable_content(html):
    soup = BeautifulSoup(html, 'html.parser')
    code_blocks = soup.find_all(['code', 'pre'])

    # Replace with placeholders
    for i, block in enumerate(code_blocks):
        placeholder = f"__CODE_{i}__"
        block.replace_with(placeholder)

    # Translate, then restore
    translated = translate(str(soup))
    # ... restore placeholders
```

---

### Pitfall 2: Navbar Layout Breaking with Full RTL

**Problem**: Applying `dir="rtl"` to entire page flips navbar

**Solution**:
```javascript
// Only apply RTL to content area
document.querySelector('.markdown').setAttribute('dir', 'rtl');
// NOT: document.documentElement.setAttribute('dir', 'rtl');
```

---

### Pitfall 3: Cache Growing Too Large

**Problem**: localStorage fills up, quota exceeded error

**Solution**:
```javascript
const MAX_CACHE_SIZE = 8 * 1024 * 1024; // 8MB

function evictOldestEntries(cache) {
  while (getCacheSize(cache) > MAX_CACHE_SIZE) {
    const oldest = findOldestEntry(cache);
    delete cache[oldest];
  }
}
```

---

### Pitfall 4: Translation Flicker Effect

**Problem**: Original content briefly visible before translation

**Solution**: Use component swizzling, translate before render
```javascript
// Swizzled DocItem/Content component
function ContentWrapper(props) {
  const { translatedContent, isTranslating } = useTranslation();

  if (isTranslating) {
    return <LoadingSpinner />; // No flicker
  }

  return <Content>{translatedContent}</Content>;
}
```

---

## Performance Targets

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Translation latency (cold cache) | p95 < 3 seconds | Backend logs |
| Cache hit latency | < 200ms | Frontend performance.now() |
| Cache hit rate | > 80% | Cache stats endpoint |
| Translation quality | > 80% accuracy | Manual review by Urdu speaker |
| Client cache size | < 8MB | localStorage.length |
| Server cache size | < 1GB RAM | Python memory profiler |

---

## Testing Checklist

### Unit Tests
- [ ] Translation service returns Urdu for English input
- [ ] Cache stores and retrieves translations correctly
- [ ] Content extractor excludes code blocks
- [ ] Language context updates state correctly

### Integration Tests
- [ ] Frontend → Backend translation flow works
- [ ] Chatbot accepts language parameter
- [ ] Cache invalidation works on version change

### E2E Tests (Playwright)
- [ ] User clicks toggle, sees Urdu content
- [ ] User navigates to new page, Urdu persists
- [ ] User submits chatbot query, gets Urdu response
- [ ] Translation error shows fallback English content

### Visual Regression Tests
- [ ] Urdu page layout matches English layout
- [ ] RTL text displays correctly
- [ ] Code blocks remain unchanged
- [ ] Navbar not affected by RTL

---

## Debugging Tips

### Problem: Translation API Returns 500 Error

**Check**:
1. Is backend running? `curl http://localhost:8000/api/translate/health`
2. Is model loaded? Check logs: `tail -f backend/logs/app.log`
3. GPU memory? `nvidia-smi` (if using GPU)

**Fix**: Restart backend, ensure model downloads successfully

---

### Problem: Cache Not Working

**Check**:
1. localStorage available? Open Dev Tools → Application → Local Storage
2. Cache key correct? Log `cacheKey` before fetch
3. Cache size limit? Check `getCacheSize()`

**Fix**: Clear cache and retry: `localStorage.clear()`

---

### Problem: RTL Breaking Layout

**Check**:
1. Is `dir="rtl"` only on `.markdown`? Inspect element in Dev Tools
2. CSS specificity correct? Check computed styles

**Fix**: Ensure selector targets content container only, not entire page

---

## Resources

### Documentation
- **Spec**: `specs/005-urdu-translation/spec.md` - Feature requirements
- **Plan**: `specs/005-urdu-translation/plan.md` - Architecture decisions
- **Research**: `specs/005-urdu-translation/research.md` - Technical research
- **Data Model**: `specs/005-urdu-translation/data-model.md` - Entity definitions
- **API Contracts**: `specs/005-urdu-translation/contracts/` - OpenAPI specs

### External References
- [Hugging Face mBART Documentation](https://huggingface.co/facebook/mbart-large-50-many-to-many-mmt)
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [MDN: HTML dir Attribute](https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes/dir)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)

---

## Next Steps

1. **Read Specification**: Review `spec.md` for complete requirements
2. **Review Plan**: Read `plan.md` for architecture decisions and rationale
3. **Start Phase 1**: Implement backend translation service (see plan.md Phase 1)
4. **Run Tests**: Follow `tasks.md` for step-by-step implementation tasks (to be generated)

---

**Questions?** Refer to `plan.md` Section 11 (Open Questions) or contact backend team.

**Ready to implement?** Run `/sp.tasks` to generate detailed implementation tasks with acceptance criteria.
