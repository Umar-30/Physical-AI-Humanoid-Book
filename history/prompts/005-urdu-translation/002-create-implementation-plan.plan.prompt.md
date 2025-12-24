---
id: 002
title: Create Implementation Plan
stage: plan
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-urdu-translation
branch: 005-urdu-translation
user: user
command: /sp.plan
labels: ["plan", "architecture", "translation", "urdu", "design-decisions"]
links:
  spec: specs/005-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-urdu-translation/plan.md
  - specs/005-urdu-translation/research.md
  - specs/005-urdu-translation/data-model.md
  - specs/005-urdu-translation/contracts/translation-api.yaml
  - specs/005-urdu-translation/contracts/chatbot-api-extension.yaml
  - specs/005-urdu-translation/quickstart.md
tests: []
---

# Prompt

```text
/sp.plan
```

# Response

Created comprehensive implementation plan for Urdu Translation Agent Integration feature with complete architecture, design decisions, and contracts.

## Planning Artifacts Generated

### 1. Implementation Plan (`plan.md`)
**Size**: ~18,000 words, 12 sections

**Key Sections**:
- **Scope and Dependencies**: In-scope features, out-of-scope items, external dependencies (translation APIs, Docusaurus, React)
- **8 Key Decisions** with full rationale:
  1. Translation Service Provider: Hugging Face mBART (free, 80-85% quality) with Google Translate fallback
  2. Architecture: Server-side translation with two-tier caching
  3. Content Extraction: HTML parsing with BeautifulSoup, exclude code blocks
  4. Caching Strategy: localStorage (client) + Python dict (server) for 80%+ hit rate
  5. RTL Handling: Scoped `dir="rtl"` on `.markdown` container only
  6. Chatbot Integration: Language context parameter, translate chunks before LLM
  7. Code Block Handling: Phase 1 skip code, Phase 2 translate comments only
  8. Error Handling: Graceful degradation with inline banners, fallback to English

- **API Contracts**: Translation API, Chatbot API extension
- **NFRs**: Performance targets (p95 <3s, cache <200ms), reliability (95% uptime), security (rate limiting, no PII)
- **Operational Readiness**: Logging, metrics, runbooks, deployment strategy
- **Risk Analysis**: 5 risks identified with mitigation plans (translation quality, API costs, RTL layout, cache limits, code corruption)
- **Constitution Check**: All 6 principles evaluated (2 conditional passes require validation gates)
- **Implementation Phases**: 6 phases from backend service to testing

### 2. Research Document (`research.md`)
**Size**: ~8,000 words, 10 sections

**Research Areas**:
- **Translation Services**: Evaluated Google Translate, Azure, Hugging Face mBART, LibreTranslate
  - Decision: mBART (free, 80-85% quality) with Google fallback
  - Benchmark results included for 3 models
- **Architecture Patterns**: Client-side vs server-side vs hybrid
  - Decision: Server-side with two-tier caching (best performance + cost)
- **RTL Text Handling**: Full page vs content-only vs no RTL
  - Decision: Scoped RTL on `.markdown` container (preserves UI)
- **Content Extraction**: Full HTML vs DOM walking vs HTML parsing
  - Decision: BeautifulSoup with tag exclusion (robust, server-side)
- **Caching**: 4 layers evaluated (none, client, server, two-tier)
  - Decision: Two-tier (instant client loads + server deduplication)
- **Docusaurus Integration**: i18n system vs DOM replacement vs component swizzling
  - Decision: Swizzle DocItemContent (no flicker, official approach)
- **Chatbot Integration**: Client translation vs language parameter
  - Decision: Language context parameter (better quality, single translation)
- **Performance Optimizations**: Batch translation, lazy loading, service workers
  - Decision: Batch translation Phase 1 (90% overhead reduction)

### 3. Data Model (`data-model.md`)
**Size**: ~6,000 words

**Entities Defined** (9 total):
1. **TranslationRequest**: API request schema with validation rules
2. **TranslationResponse**: Success response with metadata
3. **TranslationError**: Error response with error codes (SERVICE_DOWN, RATE_LIMIT, TIMEOUT, etc.)
4. **CachedTranslation**: Client localStorage schema
5. **ServerCacheEntry**: Backend memory cache schema
6. **LanguagePreference**: User preference storage
7. **LanguageContext**: React Context state definition
8. **ChatbotQueryRequest**: Extended with language parameter
9. **ChatbotQueryResponse**: Unchanged structure, translated content

**Additional Sections**:
- Entity relationships diagram
- State transition diagram (IDLE → TRANSLATING → TRANSLATED/ERROR)
- Validation rules (Pydantic schemas)
- Cache eviction policies (LRU, TTL)
- Performance targets (cache hit rates, page size benchmarks)

### 4. API Contracts

#### Translation API (`contracts/translation-api.yaml`)
**Format**: OpenAPI 3.0.3 specification

**Endpoints**:
- `POST /api/translate`: Translate text (request/response schemas, error codes, examples)
- `GET /api/translate/health`: Health check (model status, uptime)
- `DELETE /api/cache`: Clear cache (admin only)
- `GET /api/cache/stats`: Cache statistics (hit rate, size, oldest entry)

**Error Codes Defined**:
- `SERVICE_DOWN`, `RATE_LIMIT`, `INVALID_LANGUAGE`, `TIMEOUT`, `INVALID_INPUT`

#### Chatbot API Extension (`contracts/chatbot-api-extension.yaml`)
**Format**: OpenAPI 3.0.3 specification (v2.0)

**Changes from v1.0**:
- Added `language` parameter to `QueryRequest` (optional, default "en")
- SourceReference.page_title now translated to match language
- New error scenario: Translation service failure
- Backward compatible (omitting `language` defaults to English)

**Changelog**: Documented in `x-changelog` section

### 5. Quickstart Guide (`quickstart.md`)
**Size**: ~4,500 words

**Sections**:
- **Architecture at a Glance**: Diagram showing frontend → backend → model flow
- **Key Technologies**: Table of components and purposes
- **Implementation Phases**: 6 phases with tasks, success criteria, key files
- **Critical Decisions Summary**: Top 6 decisions with rationale
- **API Contracts Quick Reference**: Request/response examples
- **Code Examples**: React hooks, Python translation, CSS RTL
- **Common Pitfalls & Solutions**: 4 pitfalls with fixes
- **Performance Targets**: Table of metrics and measurement methods
- **Testing Checklist**: Unit, integration, E2E, visual regression
- **Debugging Tips**: 3 common problems with solutions

## Key Architectural Decisions

### Decision 1: Hugging Face mBART Translation Model
**Rationale**:
- Free, self-hosted (no API costs)
- 80-85% quality acceptable per spec assumptions
- Educational value (users can reproduce setup)
- Privacy-friendly (no data sent externally)

**Trade-off**: Lower quality than Google Translate (85% vs 95%), higher latency (2s vs 500ms)

**Mitigation**: Fallback to Google Translate if user complaints > 20%

---

### Decision 2: Server-Side Translation Architecture
**Rationale**:
- GPU inference faster than browser CPU (2s vs 6s)
- Smaller client bundle (no 500MB model download)
- Better caching strategy (server + client tiers)
- Mobile browser compatibility

**Trade-off**: Requires backend running (not pure static site)

**Measurement**: p95 latency < 3 seconds (vs 6s+ client-side)

---

### Decision 3: Two-Tier Caching (localStorage + Server Memory)
**Rationale**:
- Success Criteria: "Previously translated pages load instantly (<200ms)"
- Client cache: Instant loads for revisits (<50ms)
- Server cache: Deduplication across users (cost efficiency)
- Expected 80%+ cache hit rate

**Cache Strategy**:
```
Request → Client Cache (localStorage)
  ↓ MISS
Server Cache (Python dict)
  ↓ MISS
Translation Model (GPU)
  ↓
Cache Result (server + client)
```

**Expected Performance**:
- First visit: ~2000ms (cold cache)
- Revisit (same user): <50ms (client cache hit)
- First visit (different user, same page): ~200ms (server cache hit)

---

### Decision 4: Scoped RTL to Content Area Only
**Rationale**:
- Full-page RTL breaks navbar layout (logo flips, menu reverses)
- Content-only RTL preserves UI familiarity
- Standard HTML `dir="rtl"` attribute on `.markdown` container
- Browser handles RTL rendering (text selection, cursor movement)

**Implementation**:
```javascript
document.querySelector('.markdown').setAttribute('dir', 'rtl');
document.documentElement.setAttribute('lang', 'ur');
```

**CSS Enhancements**:
```css
.markdown[dir="rtl"] pre,
.markdown[dir="rtl"] code {
  direction: ltr; /* Keep code blocks LTR */
}
```

---

### Decision 5: Exclude Code Blocks from Translation
**Rationale**:
- Requirement FR-006: "Preserve code blocks in original form"
- Translating code syntax breaks examples (non-functional)
- Success Criteria: "Code integrity - no syntax translation"

**Phase 1**: Exclude all `<code>` and `<pre>` tags
**Phase 2** (future): Translate code comments only (AST parsing)

**Implementation**:
```python
def extract_translatable_content(html):
    soup = BeautifulSoup(html, 'html.parser')
    code_blocks = soup.find_all(['code', 'pre'])

    # Replace with placeholders
    for i, block in enumerate(code_blocks):
        placeholder = f"__CODE_{i}__"
        placeholders[placeholder] = str(block)
        block.replace_with(placeholder)

    # Translate text, restore code blocks
    translated = translate(str(soup))
    for placeholder, original in placeholders.items():
        translated = translated.replace(placeholder, original)

    return translated
```

---

### Decision 6: Chatbot Language Context Parameter
**Rationale**:
- Requirement FR-012: "Chatbot responses in Urdu when Urdu mode active"
- Single translation (query chunks) better than double (query + response)
- Claude/GPT-4 have native Urdu capabilities
- Backend already supports parameter extension

**Backend Logic**:
1. Retrieve top-k chunks from Qdrant (English embeddings)
2. If `language === "ur"`, translate chunks to Urdu
3. Add LLM instruction: "Answer in Urdu language"
4. Generate answer (LLM outputs Urdu)
5. Translate source page titles to Urdu

**API Extension**:
```json
// Request
{
  "query": "ROS 2 کیا ہے؟",
  "language": "ur"
}

// Response
{
  "answer": "ROS 2 ایک جدید روبوٹک آپریٹنگ سسٹم ہے...",
  "sources": [
    {"page_title": "ماڈیول 1", ...}
  ]
}
```

---

### Decision 7: Component Swizzling (Docusaurus Integration)
**Rationale**:
- Official Docusaurus customization mechanism
- No flicker effect (translation before render)
- React-friendly (hooks, context)
- Survives Docusaurus upgrades

**Alternative Rejected**: DOM manipulation (flicker effect, fragile selectors)

**Implementation**:
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap
```

```javascript
// src/theme/DocItem/Content/index.js
import { useTranslation } from '@site/src/hooks/useTranslation';

export default function ContentWrapper(props) {
  const { translatedContent, isTranslating } = useTranslation();

  if (isTranslating) {
    return <LoadingSpinner />;
  }

  return <OriginalContent {...props}>{translatedContent}</OriginalContent>;
}
```

---

### Decision 8: Graceful Error Handling with Fallback
**Rationale**:
- Requirement FR-015: "Handle API failures with error messages and revert to original"
- Translation failures shouldn't block documentation access
- Non-blocking errors (banner, not modal)

**Error Strategy**:
```javascript
try {
  const translated = await translateContent(html);
  setContent(translated);
} catch (error) {
  showError(getErrorMessage(error.code)); // User-friendly message
  setContent(originalContent); // Always fallback to English
  setLanguage('en');
}
```

**Error Messages**:
- `SERVICE_DOWN`: "Translation service unavailable. Showing original English content. [Retry]"
- `TIMEOUT`: "Translation took too long. Showing original English content. [Retry]"
- `RATE_LIMIT`: "Translation service temporarily unavailable (rate limit). [Retry in 60s]"

---

## Constitution Compliance

### ✅ Passes (4/6)
1. **Clarity for Learners**: Translation improves accessibility for Urdu speakers
2. **Consistency with Standards**: Uses Docusaurus swizzling, follows best practices
3. **Verification Before Inclusion**: Comprehensive testing plan (unit, integration, E2E, visual)
4. **Educational Structure**: Preserves original content structure during translation

### ⚠️ Conditional Passes (2/6) - Require Validation Gates
1. **Technical Accuracy (NON-NEGOTIABLE)**:
   - **Risk**: Automated translation may misinterpret technical terms
   - **Mitigation**:
     - Disclaimer: "Content auto-translated. Refer to English for technical accuracy."
     - Keep technical terms in English with Urdu explanations
     - 50-page sample validation by Urdu-speaking engineer
     - Code blocks remain 100% English (technical accuracy preserved)
   - **Gate**: Must validate translation quality on 50 sample pages before release

2. **Reproducibility (NON-NEGOTIABLE)**:
   - **Risk**: Translated procedural text may introduce ambiguity
   - **Mitigation**:
     - All commands, file paths, configs remain in English
     - Test 20 tutorial workflows in Urdu mode
     - "View in English" fallback for complex procedures
   - **Gate**: All tutorial workflows must succeed following Urdu instructions

---

## Risk Mitigation Summary

### Risk 1: Translation Quality Issues
**Impact**: High | **Probability**: Medium
**Mitigation**:
- Use highest-quality model (mBART or Google Translate)
- Display disclaimer on auto-translated content
- 50-page review by Urdu speaker
**Kill Switch**: Feature flag `enableUrduTranslation: false`

### Risk 2: Translation API Rate Limits / Costs
**Impact**: Medium | **Probability**: Low
**Mitigation**:
- Aggressive two-tier caching (80% hit rate)
- Free mBART as primary, Google as fallback only
- Backend rate limiting (100 req/min)
**Kill Switch**: Disable translation endpoint

### Risk 3: RTL Layout Breaking UI
**Impact**: Medium | **Probability**: Low
**Mitigation**:
- Scope `dir="rtl"` to `.markdown` only
- Visual regression tests
- Manual testing in Chrome/Firefox/Safari
**Kill Switch**: Remove `dir="rtl"` attribute

### Risk 4: Browser Cache Storage Limits
**Impact**: Low | **Probability**: Medium
**Mitigation**:
- Limit cache to 100 most recent pages
- LRU eviction when approaching 8MB
- Compress cached HTML
**Guardrails**: Cache size check before write

### Risk 5: Translation Breaking Code Examples
**Impact**: High | **Probability**: Low
**Mitigation**:
- Exclude all `<code>` and `<pre>` tags
- Automated tests verify code integrity
- Visual diff between English and Urdu code blocks
**Kill Switch**: Revert to English for code-heavy pages

---

## Implementation Roadmap

### Week 1: Backend + Frontend Toggle
- **Phase 1**: Backend translation service (FastAPI, mBART, caching)
- **Phase 2**: Frontend language toggle (React Context, navbar button, localStorage)

### Week 2: Content Translation + Chatbot
- **Phase 3**: Content translation and rendering (swizzle DocItem, cache manager, RTL)
- **Phase 4**: Chatbot integration (language parameter, chunk translation, LLM instruction)

### Week 3: Navigation + Testing
- **Phase 5**: Navigation translation (sidebar, breadcrumbs, UI strings)
- **Phase 6**: Testing and validation (unit, integration, E2E, visual regression, quality review)

**Total Duration**: 3 weeks (15 working days)

---

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Translation latency (cold) | p95 < 3 seconds | Backend logs |
| Cache hit latency | < 200ms | Frontend timer |
| Cache hit rate | > 80% | Cache stats API |
| Translation quality | > 80% accuracy | Urdu speaker review |
| Chatbot accuracy (Urdu) | > 95% | Manual validation |
| Content coverage | 95% text translated | Automated tests |
| Formatting preservation | 100% | Visual regression |
| Code integrity | 100% unchanged | Automated tests |

---

## Next Steps

1. **Review Plan**: Stakeholder approval for architecture decisions
2. **Validate Assumptions**: Confirm 80-85% translation quality acceptable with product owner
3. **Procurement**: If using Google Translate fallback, set up GCP account and API key
4. **Generate Tasks**: Run `/sp.tasks` to create detailed implementation tasks with acceptance criteria
5. **Phase 1 Implementation**: Start with backend translation service (see plan.md Phase 1)

---

**Planning Status**: ✅ COMPLETE

All technical unknowns resolved. Architecture decisions documented with rationale. API contracts defined. Data model specified. Ready for task generation and implementation.
