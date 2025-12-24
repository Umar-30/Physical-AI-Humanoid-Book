# Implementation Plan: Urdu Translation Agent Integration

**Feature**: 005-urdu-translation
**Created**: 2025-12-22
**Status**: Draft

## 1. Scope and Dependencies

### In Scope

- **Language Toggle Component**: Button/dropdown in Docusaurus navbar for language selection
- **Translation Service Backend**: FastAPI endpoint for translating text to Urdu
- **Translation Client**: Frontend service to request translations from backend
- **Content Interceptor**: Mechanism to extract, translate, and replace page content
- **RTL Layout Support**: CSS and DOM changes for right-to-left Urdu text
- **Language State Management**: Browser storage and React context for language preference
- **Cache Layer**: Browser cache for translated content to minimize API calls
- **Chatbot Language Context**: Pass language preference to chatbot for Urdu responses
- **Navigation Translation**: Translate sidebar, breadcrumbs, and menu items
- **Loading States**: Visual indicators during translation processing
- **Error Handling**: Fallback to original content with user-friendly error messages
- **Formatting Preservation**: Maintain HTML structure, styles, and formatting during translation

### Out of Scope

- Translation to languages other than Urdu
- User accounts or permanent preference storage across devices
- Offline translation capability
- Translation of user-generated content (comments, forum posts)
- Voice-to-text or text-to-speech features
- Translation quality feedback or correction mechanisms
- Professional translation review or human validation
- Search functionality in translated content
- Translation of PDF exports or downloadable documentation
- Translation memory or terminology management
- Side-by-side multilanguage comparison view
- Translation of embedded videos or external iframe content
- Automatic language detection from browser settings
- Translation analytics or usage tracking
- Custom translation glossaries or user-defined term overrides

### External Dependencies

| Dependency | Type | Purpose | Ownership | SLA/Availability |
|------------|------|---------|-----------|------------------|
| Translation API (Google/Azure/Hugging Face) | External Service | English to Urdu text translation | Third-party vendor | 99.9% SLA (varies by provider) |
| FastAPI Backend | Internal Service | Translation request proxy and caching | Backend team | Development only (localhost) |
| Docusaurus Framework | Framework | Documentation site infrastructure | Meta Open Source | N/A (installed locally) |
| React | Framework | Component rendering and state management | Meta Open Source | N/A (included in Docusaurus) |
| Browser localStorage | Browser API | Translation cache storage | Browser vendors | N/A (standard API) |

## 2. Key Decisions and Rationale

### Decision 1: Translation Service Provider

**Options Considered**:
- A) Google Cloud Translation API (paid, high quality, 128 languages)
- B) Azure Translator Text API (paid, enterprise-grade, 90+ languages)
- C) Hugging Face Model (mBART, M2M100) - open-source, free, self-hosted
- D) LibreTranslate API (open-source, free tier, self-hosted option)

**Trade-offs**:
- Option A: Best quality for Urdu, $20/1M characters, requires Google Cloud account, simple REST API
- Option B: Enterprise features, $10/1M characters, requires Azure account, supports custom models
- Option C: Free, privacy-friendly (local), slower inference, requires GPU/memory, 80-85% quality
- Option D: Free tier (5000 chars/day), fully open-source, lower quality for Urdu

**Decision**: Option C - Hugging Face Model (mBART or M2M100) with fallback to Option A

**Rationale**:
- Constraint: Must minimize costs for educational project
- Hugging Face models provide acceptable 80-85% quality for technical documentation
- Can be self-hosted on backend server (no API key management)
- Fallback to Google Translate API if quality issues arise
- Success Criteria: "Translation quality of 80-85% accuracy is acceptable"
- Open-source aligns with educational mission (users can reproduce setup)

**Reversibility**: High - Translation service abstracted behind backend API, easy to swap providers

**Measurement**: Translation accuracy reviewed on 50 sample pages, rated by Urdu speaker

**Implementation Note**: Start with `facebook/mbart-large-50-many-to-many-mmt` model

---

### Decision 2: Translation Architecture (Client vs Server)

**Options Considered**:
- A) Client-side translation (load model in browser via TensorFlow.js)
- B) Server-side translation (FastAPI backend with Python transformers)
- C) Hybrid (cache-first client, fallback to server for misses)

**Trade-offs**:
- Option A: No backend needed, offline-capable, but huge model download (>500MB), slow in browser
- Option B: Faster inference with GPU, smaller client bundle, requires backend running, better caching
- Option C: Best of both, complex state management, large initial client bundle

**Decision**: Option B - Server-side translation via FastAPI backend

**Rationale**:
- Constraint: "Minimal latency for translation display" - server GPU inference faster than browser CPU
- Backend already exists (002-rag-agent) - can extend with translation endpoint
- Easier caching strategy (server-side cache + client localStorage)
- Model loading once on server vs. per-client download
- Success Criteria: "Most pages translate within 3 seconds"
- Mobile browsers would struggle with large model inference

**Reversibility**: Medium - Client-side translation would require significant refactoring

**Measurement**: Translation latency p95 < 3 seconds for pages up to 5000 words

---

### Decision 3: Content Extraction and Replacement Strategy

**Options Considered**:
- A) Docusaurus i18n system (official, requires pre-translated markdown files)
- B) DOM manipulation (extract text nodes, translate, replace in-place)
- C) Markdown interception (translate markdown before rendering)
- D) Swizzle Docusaurus components (wrap content components with translation layer)

**Trade-offs**:
- Option A: Official solution, static files, SEO-friendly, but requires pre-translation (not dynamic)
- Option B: Dynamic, works with any content, but fragile (DOM changes), complex selectors
- Option C: Clean separation, translates once, but requires Docusaurus build integration
- Option D: React-friendly, component-based, but requires understanding Docusaurus internals

**Decision**: Option D - Swizzle Docusaurus DocItemContent component with translation wrapper

**Rationale**:
- Requirement FR-002: "Dynamic switching without page reload" - rules out Option A
- Requirement FR-005: "Preserve text formatting" - Option B risks breaking formatting
- Docusaurus provides swizzling mechanism for custom component wrapping
- React Context API can manage language state across all components
- Translation happens at markdown render time, preserving React component structure
- Success Criteria: "100% formatting preservation"

**Reversibility**: High - Wrapper component can be removed, reverting to default Docusaurus

**Measurement**: Visual regression tests comparing original and translated page layouts

---

### Decision 4: Translation Caching Strategy

**Options Considered**:
- A) No caching (translate on every page load)
- B) Client-side only (localStorage cache)
- C) Server-side only (Redis/memory cache)
- D) Two-tier cache (server for deduplication, client for instant load)

**Trade-offs**:
- Option A: Simplest, but slow and expensive (repeated API calls)
- Option B: Fast for repeat visits, but no deduplication across users
- Option C: Reduces API costs, but requires network round-trip every time
- Option D: Best performance, reduces costs, but more complex implementation

**Decision**: Option D - Two-tier caching (server + client)

**Rationale**:
- Success Criteria: "Previously translated pages load instantly (<200ms)"
- Success Criteria: "Translation introduces no more than 100ms overhead in original mode"
- Server cache deduplicates translation requests (same page translated once)
- Client localStorage provides instant subsequent loads
- Cost efficiency: Minimize translation API usage
- Cache key: MD5 hash of original content + language code

**Reversibility**: High - Cache layers independent, can disable either tier

**Measurement**: Cache hit rate >80% for revisited pages, <200ms load time on cache hit

**Cache Structure**:
```javascript
// Client cache (localStorage)
{
  "en-ur-{contentHash}": {
    translated: "<translated HTML>",
    timestamp: 1703260800000,
    version: "1.0"
  }
}

// Server cache (Python dict / Redis)
{
  "en-ur-{contentHash}": {
    "translated": "<translated HTML>",
    "timestamp": "2025-12-22T10:30:00Z",
    "ttl": 86400
  }
}
```

---

### Decision 5: RTL (Right-to-Left) Text Handling

**Options Considered**:
- A) CSS `direction: rtl` on entire page
- B) CSS `direction: rtl` on content only, keep navbar LTR
- C) Language-aware CSS with `[dir="rtl"]` selectors
- D) No RTL (display Urdu in LTR direction)

**Trade-offs**:
- Option A: Simplest, but breaks navbar/UI layout (icons, buttons flip)
- Option B: Preserves UI, but complex CSS for mixed directionality
- Option C: Most flexible, standard approach, requires careful selector management
- Option D: Incorrect rendering, violates Urdu readability standards

**Decision**: Option C - Language-aware CSS with `[dir="rtl"]` attribute on content container

**Rationale**:
- Success Criteria: "Urdu text displays with correct right-to-left direction"
- Standard HTML attribute `dir="rtl"` triggers browser RTL behavior
- Scoped to content area (`.markdown` container) to avoid navbar disruption
- Docusaurus navbar remains LTR for consistency
- Browser native RTL support for text selection, cursor movement

**Reversibility**: High - Single attribute toggle, no structural changes

**Measurement**: Manual review by Urdu speaker, automated tests for `dir` attribute presence

**Implementation**:
```javascript
// Apply dir="rtl" when language is Urdu
if (language === 'ur') {
  document.querySelector('.markdown').setAttribute('dir', 'rtl');
  document.documentElement.setAttribute('lang', 'ur');
} else {
  document.querySelector('.markdown').setAttribute('dir', 'ltr');
  document.documentElement.setAttribute('lang', 'en');
}
```

---

### Decision 6: Chatbot Language Integration

**Options Considered**:
- A) Translate chatbot responses client-side (query in English, translate answer to Urdu)
- B) Send language context to backend, get Urdu responses directly
- C) Separate Urdu chatbot instance with Urdu-embedded documents
- D) No chatbot translation (keep English only)

**Trade-offs**:
- Option A: Simpler backend, but double translation (English → Urdu), consistency issues
- Option B: Single translation, better quality, requires backend support for language param
- Option C: Best quality (RAG on Urdu docs), but doubles embedding storage and complexity
- Option D: Breaks user experience (mixed languages)

**Decision**: Option B - Pass language context to chatbot backend, return Urdu responses

**Rationale**:
- Requirement FR-012: "Chatbot responses provided in Urdu when Urdu mode active"
- Success Criteria: "95%+ chatbot accuracy in Urdu"
- Backend (002-rag-agent) can accept `language` parameter in request
- LLM (Claude, GPT-4) natively supports Urdu output
- Retrieved chunks translated before displaying to user (same translation service)
- Avoids double translation (better quality)

**Reversibility**: Medium - Requires backend changes but cleanly isolated parameter

**Measurement**: Chatbot response accuracy tested with 20 Urdu queries, rated by Urdu speaker

**API Contract Change**:
```json
// QueryRequest
{
  "query": "ROS 2 کیا ہے؟",
  "top_k": 5,
  "language": "ur"  // New parameter
}

// QueryResponse (unchanged structure, translated content)
{
  "answer": "ROS 2 ایک جدید...",
  "sources": [
    {"url": "...", "page_title": "ماڈیول 1", "score": 0.92}
  ]
}
```

---

### Decision 7: Code Block Handling During Translation

**Options Considered**:
- A) Translate everything including code blocks
- B) Skip code blocks entirely (no translation)
- C) Translate code comments only, preserve syntax
- D) Mark code blocks for manual translation

**Trade-offs**:
- Option A: Breaks code syntax, non-functional examples, unacceptable
- Option B: Simplest, safe, but misses helpful comment translation
- Option C: Best UX (translated comments, working code), complex parsing required
- Option D: Manual work, not scalable, defeats "dynamic translation" goal

**Decision**: Option B (Phase 1) - Skip code blocks, Option C (Phase 2) - Translate comments

**Rationale**:
- Requirement FR-006: "Preserve code blocks in original form"
- Requirement FR-007: "Translate code comments to Urdu while keeping code unchanged"
- Phase 1: Simple exclusion (faster delivery, zero risk of code corruption)
- Phase 2: Comment extraction with AST parsing (if demand exists)
- Success Criteria: "Code integrity - no syntax translation"

**Reversibility**: High - Phase 1 is subset of Phase 2

**Measurement**: Automated tests verify code block content unchanged after translation

**Implementation**:
```python
# Phase 1: Exclude code blocks from translation
def extract_translatable_content(html):
    soup = BeautifulSoup(html, 'html.parser')
    code_blocks = soup.find_all(['code', 'pre'])

    # Replace code blocks with placeholders
    placeholders = {}
    for i, block in enumerate(code_blocks):
        placeholder = f"__CODE_BLOCK_{i}__"
        placeholders[placeholder] = str(block)
        block.replace_with(placeholder)

    # Translate text, restore code blocks
    translated_text = translate(str(soup))
    for placeholder, original in placeholders.items():
        translated_text = translated_text.replace(placeholder, original)

    return translated_text
```

---

### Decision 8: Error Handling and Fallback Strategy

**Options Considered**:
- A) Fail silently (show original content, no error message)
- B) Block page load with error modal (force user to dismiss)
- C) Inline banner with error message, show original content
- D) Partial translation (show translated parts, English for failed sections)

**Trade-offs**:
- Option A: Confusing (user selected Urdu, sees English, no explanation)
- Option B: Disruptive (blocks reading), poor UX for temporary failures
- Option C: Informative, non-blocking, allows continued reading
- Option D: Best effort, but inconsistent (mixed languages on page)

**Decision**: Option C - Inline banner error with fallback to original content

**Rationale**:
- Requirement FR-015: "Handle translation API failures with error messages and revert to original"
- Success Criteria: "Translation failures display clear error messages"
- Users can continue reading (documentation access not blocked)
- Error message explains issue and offers retry option
- Timeout after 10 seconds (FR-016) prevents indefinite waits

**Reversibility**: High - Error handling is independent concern

**Measurement**: Simulate API failures (network down, 500 errors, timeouts), verify error display

**Error Messages**:
```javascript
// Network error
"Translation service unavailable. Showing original English content. [Retry]"

// Timeout error
"Translation took too long. Showing original English content. [Retry]"

// API error (429 rate limit)
"Translation service temporarily unavailable (rate limit). Showing original English content. [Retry in 60s]"
```

---

## 3. Interfaces and API Contracts

### Backend Translation API

**Endpoint**: `POST /api/translate`

**Request**:
```json
{
  "text": "Welcome to the Physical AI & Humanoid Robotics guide",
  "source_language": "en",
  "target_language": "ur",
  "content_hash": "a3f5e8c9d2b1...",
  "preserve_html": true
}
```

**Response (Success)**:
```json
{
  "translated_text": "فزیکل AI اور ہیومنائیڈ روبوٹکس گائیڈ میں خوش آمدید",
  "source_language": "en",
  "target_language": "ur",
  "cached": false,
  "model_used": "facebook/mbart-large-50",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

**Response (Error)**:
```json
{
  "error": "Translation service unavailable",
  "error_code": "SERVICE_DOWN",
  "retry_after": 60,
  "timestamp": "2025-12-22T10:30:00Z"
}
```

**Error Codes**:
- `SERVICE_DOWN`: Translation model not loaded or crashed
- `RATE_LIMIT`: Too many requests (if using external API)
- `INVALID_LANGUAGE`: Unsupported source/target language pair
- `TIMEOUT`: Translation took longer than 10 seconds
- `INVALID_INPUT`: Malformed HTML or empty text

---

### Chatbot API Extension

**Endpoint**: `POST /api/query` (existing, extended)

**Request (Extended)**:
```json
{
  "query": "ROS 2 کیا ہے؟",
  "top_k": 5,
  "model": "xiaomi/mimo-v2-flash:free",
  "language": "ur"  // NEW: Optional, defaults to "en"
}
```

**Response (Unchanged Structure)**:
```json
{
  "answer": "ROS 2 ایک جدید روبوٹک آپریٹنگ سسٹم ہے...",
  "sources": [
    {
      "url": "http://localhost:3000/docs/module-1-ros2",
      "page_title": "ماڈیول 1: روبوٹک اعصابی نظام",
      "chunk_id": "chunk_042",
      "relevance_score": 0.92
    }
  ],
  "query": "ROS 2 کیا ہے؟",
  "model": "xiaomi/mimo-v2-flash:free",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

**Backend Behavior**:
- If `language === "ur"`, translate retrieved chunks to Urdu before passing to LLM
- Pass instruction to LLM: "Answer in Urdu language"
- Translate source page titles to Urdu

---

### Frontend Language Context API

**React Context**:
```typescript
interface LanguageContextValue {
  currentLanguage: 'en' | 'ur';
  setLanguage: (lang: 'en' | 'ur') => void;
  isTranslating: boolean;
  translationError: string | null;
  clearError: () => void;
}

const LanguageContext = React.createContext<LanguageContextValue>(null);
```

**Usage**:
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

---

### Browser Storage Schema

**localStorage Key**: `docusaurus-language-preference`

**Value**:
```json
{
  "language": "ur",
  "timestamp": 1703260800000
}
```

**localStorage Key (Cache)**: `translation-cache-v1`

**Value**:
```json
{
  "en-ur-a3f5e8c9d2b1": {
    "translated": "<div>فزیکل AI...</div>",
    "timestamp": 1703260800000,
    "version": "1.0"
  },
  "en-ur-f2d8a4c1b5e3": {
    "translated": "<div>ماڈیول 1...</div>",
    "timestamp": 1703260799000,
    "version": "1.0"
  }
}
```

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance

**Latency Targets**:
- Translation API response: p95 < 2 seconds for 5000 words
- Client-side content replacement: < 100ms
- Cache hit retrieval: < 50ms (localStorage read)
- Language toggle action: < 200ms to show loading state
- Page navigation in cached Urdu mode: < 300ms total

**Throughput**:
- Backend can handle 10 concurrent translation requests
- Client can process 20 DOM updates per second during replacement

**Resource Limits**:
- Translation model memory usage: < 4GB RAM on backend
- Client localStorage cache size: < 10MB total
- Maximum translatable content per page: 50,000 characters

### Reliability

**SLOs (Service Level Objectives)**:
- Translation service uptime: 95% (development environment)
- Translation success rate: 90% (account for rate limits, timeouts)
- Cache hit rate: > 80% for repeat page visits

**Error Budget**:
- Acceptable failure rate: 10% of translation requests can fail (fallback to original)
- Maximum consecutive failures before disabling auto-translate: 3

**Degradation Strategy**:
- If translation fails 3 times consecutively, show banner: "Translation temporarily disabled"
- Allow manual retry per page
- Fallback to original English content always available

### Security

**Authentication/Authorization**:
- No authentication required for translation (public documentation)
- Backend translation endpoint rate-limited: 100 requests/minute per IP

**Data Handling**:
- No PII (personally identifiable information) in translation requests
- Translation cache stored locally (no cloud storage)
- Cache cleared on browser storage clear (no persistent tracking)

**Secrets Management**:
- If using external API (Google/Azure), API key stored in backend `.env` file
- API key never exposed to client
- Backend proxies all translation requests

**Auditing**:
- Backend logs translation requests (timestamp, language, success/failure)
- No content logging (privacy-friendly)
- Error logs include error codes, not full content

### Cost

**Unit Economics**:
- Hugging Face model (self-hosted): $0 per translation (GPU compute only)
- Google Translate API (fallback): $20 per 1M characters
- Estimated usage: 100 pages × 5000 words × 5 chars/word = 2.5M chars
- Estimated cost with Google (worst case): $50 for full site translation
- Cache reduces cost: 80% cache hit rate → $10 effective cost

**Budget**:
- Development phase: $0 (use Hugging Face)
- Production (if deployed): < $20/month with aggressive caching

---

## 5. Data Management and Migration

### Source of Truth

**Original Content**: Markdown files in `/docs` directory (English)
**Translated Content**: Dynamically generated, cached in:
- Backend: In-memory cache (Python dict) or Redis
- Frontend: Browser localStorage

**No database storage** - translation is ephemeral and regenerated on demand

### Schema Evolution

**Translation Cache Schema v1.0**:
```json
{
  "translated": "<HTML string>",
  "timestamp": "unix timestamp",
  "version": "1.0"
}
```

**Future v2.0 (if needed)**:
```json
{
  "translated": "<HTML string>",
  "timestamp": "unix timestamp",
  "version": "2.0",
  "model": "facebook/mbart-large-50",
  "quality_score": 0.85
}
```

**Migration Strategy**:
- Version field in cache allows detecting old cache entries
- On version mismatch, invalidate cache entry and retranslate
- No data migration needed (cache is disposable)

### Data Retention

**Backend Cache**:
- TTL (Time To Live): 24 hours
- Eviction policy: LRU (Least Recently Used) when memory > 1GB
- Manual flush: Admin endpoint `DELETE /api/cache` for development

**Frontend Cache**:
- No automatic expiration (browser manages localStorage)
- User can clear via browser settings
- Cache invalidated on Docusaurus version update (version key in cache)

---

## 6. Operational Readiness

### Observability

**Logs**:
```python
# Backend logging
logger.info("Translation request", extra={
    "language_pair": "en-ur",
    "content_length": 5234,
    "cached": False,
    "duration_ms": 1834
})

logger.error("Translation failed", extra={
    "error_code": "TIMEOUT",
    "language_pair": "en-ur",
    "content_hash": "a3f5e8c9"
})
```

**Metrics**:
- `translation_requests_total` (counter): Total translation requests
- `translation_duration_seconds` (histogram): Translation latency
- `translation_cache_hits_total` (counter): Cache hit count
- `translation_errors_total` (counter by error_code): Error count

**Traces**:
- Not applicable for development environment
- Production (if deployed): OpenTelemetry traces for end-to-end translation flow

### Alerting

**Development Environment**: Manual monitoring (console logs)

**Production Alerts** (if deployed):
- Translation service down (3 consecutive failures) → Slack notification
- High error rate (> 20% over 5 minutes) → Email alert
- Cache hit rate drops below 60% → Warning log

**On-call**: Not applicable (educational project)

### Runbooks

**Runbook 1: Translation Service Not Responding**

1. Check backend server status: `curl http://localhost:8000/health`
2. Check translation model loaded: `curl http://localhost:8000/api/translate/health`
3. Restart backend: `cd backend && uvicorn main:app --reload`
4. If model loading fails, check GPU memory: `nvidia-smi`
5. Fallback: Switch to Google Translate API in `.env`

**Runbook 2: High Error Rate**

1. Check backend logs: `tail -f backend/logs/translation.log`
2. Identify error codes (TIMEOUT, SERVICE_DOWN, RATE_LIMIT)
3. If TIMEOUT: Reduce batch size or increase timeout threshold
4. If RATE_LIMIT: Wait for rate limit reset or upgrade API plan
5. If SERVICE_DOWN: Restart translation service

**Runbook 3: Cache Not Working**

1. Check localStorage available: Browser Dev Tools → Application → Local Storage
2. Check cache version matches: `localStorage.getItem('translation-cache-v1')`
3. Clear cache and retry: `localStorage.clear()`
4. Check backend cache: `curl http://localhost:8000/api/cache/stats`

### Deployment and Rollback

**Development Deployment**:
1. Start backend: `cd backend && uvicorn main:app --reload`
2. Start frontend: `npm run start`
3. Access: `http://localhost:3000`

**Production Deployment** (if needed):
1. Build Docusaurus: `npm run build`
2. Deploy backend to cloud (AWS/Azure/GCP)
3. Update frontend config with production backend URL
4. Deploy frontend to GitHub Pages / Netlify / Vercel

**Rollback Strategy**:
- Feature flag: `enableTranslation: false` in `docusaurus.config.js`
- Disable translation endpoint on backend: Comment out route
- Remove swizzled component: Delete `src/theme/DocItem/Content/index.js`
- Full rollback: `git revert` and redeploy

### Feature Flags

**Flag**: `enableUrduTranslation`

**Location**: `docusaurus.config.js`

```javascript
customFields: {
  enableUrduTranslation: true,  // Set to false to disable
}
```

**Usage**:
```javascript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function LanguageToggle() {
  const { siteConfig } = useDocusaurusContext();
  if (!siteConfig.customFields.enableUrduTranslation) {
    return null;  // Hide toggle if feature disabled
  }
  return <button>Toggle Language</button>;
}
```

---

## 7. Risk Analysis and Mitigation

### Risk 1: Translation Quality Issues

**Impact**: High - Poor translations confuse users, reduce trust
**Probability**: Medium - Automated translation has inherent limitations
**Blast Radius**: All Urdu users affected

**Mitigation**:
- Use highest-quality model available (mBART or Google Translate)
- Display disclaimer: "Auto-translated content. Report issues [here]."
- Implement feedback mechanism (future enhancement)
- Test on 50 sample pages, review by Urdu speaker

**Kill Switch**: Feature flag `enableUrduTranslation: false`

---

### Risk 2: Translation API Rate Limits / Costs

**Impact**: Medium - Translation unavailable, cost overruns
**Probability**: Low - Caching reduces API calls by 80%
**Blast Radius**: All users attempting to switch to Urdu

**Mitigation**:
- Aggressive two-tier caching (server + client)
- Use free Hugging Face model as primary
- Google Translate API only as fallback
- Rate limiting on backend (100 req/min)
- Monitor API usage with alerts

**Kill Switch**: Disable translation endpoint, show maintenance message

---

### Risk 3: RTL Layout Breaking UI

**Impact**: Medium - UI unusable, navigation broken
**Probability**: Low - Scoped RTL to content area only
**Blast Radius**: Urdu users only

**Mitigation**:
- Scope `dir="rtl"` to `.markdown` content container only
- Keep navbar, footer, sidebar in LTR mode
- Visual regression tests comparing English and Urdu layouts
- Manual testing in Chrome, Firefox, Safari

**Guardrails**: CSS specificity ensures navbar not affected

**Kill Switch**: Remove `dir="rtl"` attribute application

---

### Risk 4: Browser Cache Storage Limits

**Impact**: Low - Cache evicted, slower performance
**Probability**: Medium - localStorage has 5-10MB limit
**Blast Radius**: Users with many cached pages

**Mitigation**:
- Limit cache to 100 most recent pages
- Evict oldest entries when approaching 8MB
- Compress cached HTML (gzip before storing)
- Monitor cache size: `JSON.stringify(cache).length`

**Guardrails**: Cache size check before each write

---

### Risk 5: Translation Breaking Code Examples

**Impact**: High - Code examples non-functional, tutorials fail
**Probability**: Low - Code blocks excluded from translation
**Blast Radius**: All users following code tutorials in Urdu

**Mitigation**:
- Exclude all `<code>` and `<pre>` tags from translation
- Automated tests verify code block integrity after translation
- Visual diff between English and Urdu code blocks

**Guardrails**: Whitelist-based content extraction (only translate allowed tags)

**Kill Switch**: Revert to English content for code-heavy pages

---

## 8. Evaluation and Validation

### Definition of Done

**Feature Complete When**:
- ✅ Language toggle button visible in navbar
- ✅ Clicking toggle translates page content to Urdu
- ✅ Translated content displays with RTL text direction
- ✅ Code blocks remain untranslated (English)
- ✅ Chatbot responds in Urdu when Urdu mode active
- ✅ Navigation (sidebar, breadcrumbs) translated to Urdu
- ✅ Translation failures show error message, fallback to English
- ✅ Translated pages load instantly from cache on revisit
- ✅ All 20 functional requirements tested and passing
- ✅ Visual regression tests pass (no layout breaks)

**Tests**:
- Unit tests for translation service (backend)
- Integration tests for frontend-backend translation flow
- E2E tests for user workflows (toggle, navigate, chatbot)
- Visual regression tests (English vs Urdu layout)
- Performance tests (translation latency, cache hit rate)
- Error scenario tests (API down, timeout, rate limit)

**Security Scans**:
- OWASP dependency check (no XSS via translated content)
- Rate limiting validated (cannot DOS translation endpoint)
- API key not exposed in frontend code

### Output Validation

**Translation Content**:
- No HTML corruption (tags closed properly)
- No code block translation (code remains English)
- No URL translation (links remain valid)
- RTL characters render correctly (no mojibake)

**Performance**:
- p95 latency < 3 seconds for translation
- Cache hit latency < 200ms
- No memory leaks (localStorage bounded, backend cache evicts)

**Error Handling**:
- All error codes tested (SERVICE_DOWN, TIMEOUT, RATE_LIMIT)
- Error messages user-friendly (no stack traces)
- Fallback to English content works

---

## 9. Constitution Check

### Technical Accuracy (NON-NEGOTIABLE)

**Requirement**: All technical content must be accurate and verifiable

**Status**: ⚠️ CONDITIONAL PASS

**Analysis**:
- Translation introduces accuracy risk (automated translation can misinterpret technical terms)
- Urdu technical terminology may not exist for some concepts (e.g., "ROS 2 node")

**Mitigation**:
- Display disclaimer: "Content auto-translated. Refer to English version for technical accuracy."
- Keep technical terms in English with Urdu explanations (e.g., "ROS 2 node (نوڈ)")
- Sample validation: 50 pages reviewed by Urdu-speaking engineer
- Preserve code blocks in English (100% technical accuracy for code)

**Gate**: Must validate translation quality on 50 sample pages before release

---

### Clarity for Learners

**Requirement**: Content must be accessible to beginners

**Status**: ✅ PASS

**Analysis**:
- Urdu translation improves accessibility for Urdu-speaking learners
- RTL rendering improves readability for native Urdu readers
- Preserves original formatting (headings, lists, emphasis)

**Enhancement**: Translation makes content MORE accessible to target audience

---

### Consistency with Standards

**Requirement**: Follow Docusaurus standards

**Status**: ✅ PASS

**Analysis**:
- Uses Docusaurus swizzling mechanism (official customization approach)
- Preserves Docusaurus markdown structure
- Navbar customization follows Docusaurus theme guidelines
- No breaking changes to Docusaurus core functionality

**Compliance**: Full adherence to Docusaurus best practices

---

### Practicality with Actionable Instructions

**Requirement**: Step-by-step instructions that readers can execute

**Status**: ⚠️ CONDITIONAL PASS

**Analysis**:
- Urdu-translated instructions may have terminology mismatches with English UI tools
- CLI commands remain in English (correct)
- File paths remain in English (correct)

**Mitigation**:
- Keep all commands, file paths, and code in English
- Translate only instructional text (e.g., "Run the following command:")
- Display English version link for technical procedures

**Gate**: Review 10 tutorial pages to ensure actionable in Urdu

---

### Reproducibility (NON-NEGOTIABLE)

**Requirement**: Instructions must work as written

**Status**: ⚠️ CONDITIONAL PASS

**Analysis**:
- Commands remain in English (reproducible)
- Code blocks remain in English (reproducible)
- Configuration files remain in English (reproducible)
- Risk: Translated procedural text may introduce ambiguity

**Mitigation**:
- Exclude all code, commands, configs from translation
- Test 20 tutorial workflows in Urdu mode
- Provide "View in English" fallback for complex procedures

**Gate**: All tutorial workflows must succeed following Urdu instructions

---

### Verification Before Inclusion

**Requirement**: All content validated before final inclusion

**Status**: ✅ PASS (with process)

**Validation Process**:
1. Automated tests verify code blocks untranslated
2. Visual regression tests catch layout breaks
3. 50-page sample review by Urdu speaker
4. 20 tutorial workflows tested in Urdu mode
5. Error scenarios tested (API failures, timeouts)

**Sign-off Required**: Urdu-speaking engineer approval

---

## 10. Implementation Phases

### Phase 0: Research and Prototyping (Completed Above)

**Outputs**:
- ✅ Decision on translation service (Hugging Face mBART)
- ✅ Architecture decision (server-side translation)
- ✅ Caching strategy (two-tier: server + client)
- ✅ RTL handling approach (scoped `dir="rtl"`)
- ✅ Code block exclusion strategy
- ✅ Chatbot integration approach

---

### Phase 1: Backend Translation Service

**Tasks**:
1. Set up Hugging Face mBART model on backend
2. Create FastAPI endpoint `POST /api/translate`
3. Implement request/response contracts (JSON schemas)
4. Add server-side caching (Python dict with TTL)
5. Implement error handling (timeouts, model failures)
6. Add health check endpoint `GET /api/translate/health`
7. Write unit tests for translation service
8. Load test with 100 concurrent requests

**Outputs**:
- `backend/services/translation_service.py`
- `backend/routers/translation.py`
- `backend/tests/test_translation.py`
- API documentation (OpenAPI schema)

**Success Criteria**:
- Translation API returns Urdu text for English input
- p95 latency < 2 seconds for 5000 words
- Cache hit rate > 80% for repeat requests
- Error handling returns proper JSON error responses

---

### Phase 2: Frontend Language Context and State

**Tasks**:
1. Create React Context for language state (`LanguageContext`)
2. Add language toggle button to Docusaurus navbar
3. Implement localStorage persistence for language preference
4. Create translation client service (fetch wrapper)
5. Add loading and error states to context
6. Write unit tests for context and client service

**Outputs**:
- `src/contexts/LanguageContext.js`
- `src/services/translationClient.js`
- `src/theme/Navbar/Content/index.js` (swizzled)
- `src/components/LanguageToggle.js`

**Success Criteria**:
- Language toggle button visible in navbar
- Clicking toggle updates context state
- Language preference persists across page reloads
- Loading state shows spinner during translation

---

### Phase 3: Content Translation and Rendering

**Tasks**:
1. Swizzle Docusaurus `DocItemContent` component
2. Create content extraction logic (exclude code blocks)
3. Integrate translation client with content wrapper
4. Implement client-side cache (localStorage)
5. Add RTL `dir="rtl"` attribute for Urdu mode
6. Add error boundary for translation failures
7. Write E2E tests for content translation

**Outputs**:
- `src/theme/DocItem/Content/index.js` (swizzled with translation wrapper)
- `src/utils/contentExtractor.js`
- `src/utils/cacheManager.js`
- `tests/e2e/translation.spec.js`

**Success Criteria**:
- Page content translates to Urdu on toggle
- Code blocks remain in English
- RTL text direction applied correctly
- Translation errors show fallback English content
- Cache hit loads instantly (<200ms)

---

### Phase 4: Chatbot Integration

**Tasks**:
1. Extend chatbot API to accept `language` parameter
2. Translate retrieved chunks before passing to LLM
3. Add LLM instruction: "Answer in Urdu language"
4. Update frontend chatbot client to pass language context
5. Translate chatbot UI labels (placeholder, send button)
6. Write integration tests for Urdu chatbot queries

**Outputs**:
- `backend/rag_agent.py` (updated with language support)
- `src/components/ChatWidget.js` (updated with language context)
- `backend/tests/test_rag_urdu.py`

**Success Criteria**:
- Chatbot responds in Urdu when Urdu mode active
- Retrieved sources have Urdu page titles
- Chatbot answer accuracy > 95% (validated by Urdu speaker)

---

### Phase 5: Navigation and UI Translation

**Tasks**:
1. Extract navigation labels (sidebar, breadcrumbs, footer)
2. Create translation map for UI strings
3. Apply translations to Docusaurus theme components
4. Swizzle Sidebar component with language context
5. Test all navigation flows in Urdu mode

**Outputs**:
- `src/translations/ui-strings.json`
- `src/theme/DocSidebar/index.js` (swizzled)
- `src/theme/DocBreadcrumbs/index.js` (swizzled)

**Success Criteria**:
- Sidebar items translated to Urdu
- Breadcrumbs show Urdu page titles
- Footer links translated
- All navigation functional in both languages

---

### Phase 6: Testing and Validation

**Tasks**:
1. Run visual regression tests (English vs Urdu layouts)
2. Performance testing (translation latency, cache hit rate)
3. Error scenario testing (API down, timeouts)
4. 50-page translation quality review (Urdu speaker)
5. 20 tutorial workflow tests in Urdu mode
6. Browser compatibility tests (Chrome, Firefox, Safari)
7. Mobile responsiveness tests

**Outputs**:
- Test reports (unit, integration, E2E)
- Performance benchmarks
- Translation quality scorecard
- Bug fixes and refinements

**Success Criteria**:
- All 20 functional requirements pass tests
- Visual regression tests pass (no layout breaks)
- p95 translation latency < 3 seconds
- Cache hit rate > 80%
- Translation quality > 80% accuracy (Urdu speaker rating)

---

## 11. Open Questions and Follow-ups

### Open Questions

1. **Translation Model Selection**: Should we use mBART or M2M100? (Need to benchmark both)
2. **Cache Invalidation**: When documentation updates, how do we invalidate stale translations?
3. **Mobile UX**: Should mobile users see a language selector in hamburger menu instead of navbar?
4. **Search Integration**: Should Docusaurus search index Urdu content? (Currently out of scope)

### Follow-up Tasks (Post-Release)

1. **Translation Quality Feedback**: Add "Report translation issue" button
2. **Multiple Languages**: Extend to Arabic, Hindi (if demand exists)
3. **Code Comment Translation**: Parse code comments and translate (Phase 2 of Decision 7)
4. **Search in Urdu**: Index translated content for Docusaurus search
5. **Translation Memory**: Build custom glossary for consistent technical term translation
6. **Analytics**: Track Urdu usage (page views, chatbot queries, toggle events)

---

## 12. Appendix

### Translation Service Comparison

| Service | Cost (per 1M chars) | Quality (Urdu) | Latency | Self-hosted | API Key Required |
|---------|---------------------|----------------|---------|-------------|------------------|
| Google Translate API | $20 | 95% | 500ms | No | Yes |
| Azure Translator | $10 | 93% | 600ms | No | Yes |
| Hugging Face mBART | $0 | 85% | 2000ms | Yes | No |
| LibreTranslate | $0 (5k/day) | 75% | 1500ms | Yes | Optional |

**Recommendation**: Start with Hugging Face mBART (free, self-hosted), fallback to Google Translate if quality issues arise.

---

### RTL CSS Reference

```css
/* Apply RTL to content area only */
.markdown[dir="rtl"] {
  direction: rtl;
  text-align: right;
}

/* Keep code blocks LTR even in RTL mode */
.markdown[dir="rtl"] pre,
.markdown[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}

/* Keep tables LTR for data consistency */
.markdown[dir="rtl"] table {
  direction: ltr;
}

/* Flip list bullets/numbers to right side */
.markdown[dir="rtl"] ul,
.markdown[dir="rtl"] ol {
  padding-right: 2em;
  padding-left: 0;
}
```

---

### Sample Translation Flow

```
User clicks "اردو" button
  ↓
Frontend: Update LanguageContext (en → ur)
  ↓
Frontend: Extract page content (exclude code blocks)
  ↓
Frontend: Check localStorage cache
  ↓
Cache HIT → Render cached Urdu content (skip backend)
  ↓
Cache MISS → Send POST /api/translate request
  ↓
Backend: Check server cache
  ↓
Backend Cache HIT → Return cached translation
  ↓
Backend Cache MISS → Load mBART model
  ↓
Backend: Translate text (English → Urdu)
  ↓
Backend: Store in server cache (TTL: 24h)
  ↓
Backend: Return translated JSON
  ↓
Frontend: Store in localStorage cache
  ↓
Frontend: Replace page content with Urdu text
  ↓
Frontend: Apply dir="rtl" to .markdown container
  ↓
Frontend: Update chatbot context (language: ur)
  ↓
Page rendered in Urdu with RTL layout
```

---

**End of Implementation Plan**
