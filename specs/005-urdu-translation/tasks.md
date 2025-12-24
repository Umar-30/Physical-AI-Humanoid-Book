# Implementation Tasks: Urdu Translation Agent Integration

**Feature**: 005-urdu-translation
**Created**: 2025-12-22
**Status**: Ready for Implementation

## Task Overview

This document contains all implementation tasks for the Urdu Translation Agent Integration feature, organized by user story for independent implementation and testing. Each phase represents a complete, testable increment of functionality.

**Total Tasks**: 67
**User Stories**: 5 (3 P1, 2 P2)
**Estimated Duration**: 3 weeks (15 working days)

## Implementation Strategy

**MVP Scope**: User Story 1 (Language Toggle) - Delivers basic language switching capability
**Incremental Delivery**: Each user story phase is independently deployable and testable
**Parallel Execution**: Tasks marked with [P] can be executed in parallel within their phase

---

## Phase 1: Setup and Project Initialization

**Goal**: Set up development environment and install required dependencies

**Duration**: 0.5 days

### Tasks

- [ ] T001 Install Hugging Face transformers library in backend: `pip install transformers==4.36.0 torch==2.1.0`
- [ ] T002 Install BeautifulSoup4 for HTML parsing in backend: `pip install beautifulsoup4==4.12.2 lxml==4.9.3`
- [ ] T003 Install FastAPI dependencies for translation endpoint: `pip install fastapi==0.104.1 uvicorn==0.24.0 pydantic==2.5.0`
- [ ] T004 [P] Update backend requirements.txt with new dependencies
- [ ] T005 [P] Update backend pyproject.toml with translation service metadata
- [ ] T006 Create backend/services/translation_service.py file (empty, structure only)
- [ ] T007 Create backend/routers/translation.py file (empty, structure only)
- [ ] T008 Create backend/models/translation_models.py file for Pydantic schemas
- [ ] T009 [P] Create src/contexts/LanguageContext.js file (empty, structure only)
- [ ] T010 [P] Create src/services/translationClient.js file (empty, structure only)
- [ ] T011 [P] Create src/utils/contentExtractor.js file (empty, structure only)
- [ ] T012 [P] Create src/utils/cacheManager.js file (empty, structure only)
- [ ] T013 [P] Create src/components/LanguageToggle.js file (empty, structure only)
- [ ] T014 [P] Create src/translations/ui-strings.json file with empty object
- [ ] T015 Verify all file paths created successfully and project structure is correct

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Goal**: Implement core translation service and caching infrastructure that all user stories depend on

**Duration**: 2 days

**Dependencies**: Phase 1 complete

### Backend Translation Service

- [ ] T016 Implement TranslationRequest Pydantic model in backend/models/translation_models.py with validation (text, source_language, target_language, content_hash, preserve_html)
- [ ] T017 Implement TranslationResponse Pydantic model in backend/models/translation_models.py (translated_text, cached, model_used, timestamp)
- [ ] T018 Implement TranslationError Pydantic model in backend/models/translation_models.py with error codes (SERVICE_DOWN, RATE_LIMIT, TIMEOUT, INVALID_INPUT)
- [ ] T019 Load mBART model in backend/services/translation_service.py: `facebook/mbart-large-50-many-to-many-mmt` with error handling for model loading failures
- [ ] T020 Implement translate_text() function in backend/services/translation_service.py with tokenization, generation, and decoding
- [ ] T021 Implement server-side cache (Python dict) in backend/services/translation_service.py with TTL of 24 hours and LRU eviction
- [ ] T022 Implement extract_translatable_content() function in backend/services/translation_service.py using BeautifulSoup to exclude code blocks (code, pre tags)
- [ ] T023 Implement restore_excluded_content() function in backend/services/translation_service.py to restore code blocks with placeholders
- [ ] T024 Add timeout handling (10 seconds) to translate_text() function with asyncio timeout
- [ ] T025 Add error handling for model failures (SERVICE_DOWN) and rate limits (RATE_LIMIT) in translation_service.py

### Translation API Endpoints

- [ ] T026 Create POST /api/translate endpoint in backend/routers/translation.py accepting TranslationRequest, returning TranslationResponse or TranslationError
- [ ] T027 Implement cache lookup logic in POST /api/translate endpoint (check cache before translation, store result after)
- [ ] T028 Create GET /api/translate/health endpoint in backend/routers/translation.py returning model status, uptime, and model name
- [ ] T029 [P] Create DELETE /api/cache endpoint in backend/routers/translation.py to clear server cache (admin only)
- [ ] T030 [P] Create GET /api/cache/stats endpoint in backend/routers/translation.py returning cache hit rate, size, and oldest entry
- [ ] T031 Register translation router in backend/main.py with `/api` prefix
- [ ] T032 Add CORS configuration in backend/main.py to allow frontend origin (http://localhost:3000)

### Frontend Infrastructure

- [ ] T033 Implement LanguageContext in src/contexts/LanguageContext.js with state (currentLanguage, isTranslating, translationError) and methods (setLanguage, clearError)
- [ ] T034 Implement LanguageProvider component in src/contexts/LanguageContext.js wrapping children with context
- [ ] T035 Implement localStorage persistence in LanguageContext for language preference with key `docusaurus-language-preference`
- [ ] T036 Implement translationClient service in src/services/translationClient.js with fetch() wrapper for POST /api/translate
- [ ] T037 Add request timeout (10 seconds) to translationClient.js using AbortController
- [ ] T038 Add error handling in translationClient.js to map HTTP status codes to error codes (400→INVALID_INPUT, 429→RATE_LIMIT, 500→SERVICE_DOWN, 504→TIMEOUT)
- [ ] T039 Implement cacheManager in src/utils/cacheManager.js with localStorage read/write operations for translation cache
- [ ] T040 Implement cache eviction (LRU) in cacheManager.js when cache size exceeds 8MB
- [ ] T041 Implement cache key generation in cacheManager.js using MD5 hash of content (use crypto.subtle.digest or md5 library)
- [ ] T042 Implement contentExtractor in src/utils/contentExtractor.js to extract page content from .markdown container
- [ ] T043 Add HTML sanitization in contentExtractor.js to prevent XSS before rendering translated content

---

## Phase 3: User Story 1 - Toggle Language via Navigation Bar (P1)

**Goal**: Enable users to switch documentation language to Urdu via navbar toggle

**Duration**: 1.5 days

**Dependencies**: Phase 2 complete

**Independent Test**: Click language toggle in navbar → verify UI updates → verify language preference persists across page navigation

### Implementation Tasks

- [ ] T044 [US1] Create LanguageToggle component in src/components/LanguageToggle.js with button showing current language (English/اردو)
- [ ] T045 [US1] Implement useLanguage hook in LanguageToggle component to access LanguageContext (currentLanguage, setLanguage, isTranslating)
- [ ] T046 [US1] Add onClick handler to LanguageToggle button to call setLanguage('ur') or setLanguage('en')
- [ ] T047 [US1] Add disabled state to LanguageToggle button when isTranslating is true
- [ ] T048 [US1] Add loading spinner to LanguageToggle button when isTranslating is true
- [ ] T049 [US1] Swizzle Docusaurus Navbar component: `npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap`
- [ ] T050 [US1] Import and render LanguageToggle component in swizzled src/theme/Navbar/Content/index.js positioned right of GitHub link
- [ ] T051 [US1] Wrap Docusaurus root with LanguageProvider in src/theme/Root.js (create if doesn't exist)
- [ ] T052 [US1] Add CSS styles for LanguageToggle button in src/components/LanguageToggle.module.css (responsive, accessible)
- [ ] T053 [US1] Verify language toggle button appears in navbar on all documentation pages

### Acceptance Validation

**Test Scenario 1**: User clicks "اردو" button → currentLanguage changes to 'ur' → button text changes to "English"
**Test Scenario 2**: User refreshes page → language preference persists from localStorage → toggle button shows correct state
**Test Scenario 3**: User navigates to different page → language preference maintained → toggle state consistent
**Test Scenario 4**: Translation in progress → toggle button shows spinner → button is disabled

---

## Phase 4: User Story 2 - View Translated Book Content (P1)

**Goal**: Translate all documentation content to Urdu with formatting preserved and RTL support

**Duration**: 2.5 days

**Dependencies**: Phase 3 complete (language toggle functional)

**Independent Test**: Enable Urdu mode → verify all text translated → verify formatting preserved → verify code blocks unchanged → verify RTL text direction

### Content Translation

- [ ] T054 [US2] Swizzle DocItem/Content component: `npm run swizzle @docusaurus/theme-classic DocItem/Content -- --wrap`
- [ ] T055 [US2] Create useTranslation hook in src/hooks/useTranslation.js accepting content prop, returning {translatedContent, isTranslating, error}
- [ ] T056 [US2] Implement useTranslation hook logic: check currentLanguage from context → if 'ur', translate content → else return original
- [ ] T057 [US2] In useTranslation hook, check cacheManager for cached translation → if hit, return cached → if miss, call translationClient
- [ ] T058 [US2] In useTranslation hook, call contentExtractor.extractContent() to get page HTML from .markdown container
- [ ] T059 [US2] In useTranslation hook, call translationClient.translate() with extracted content → handle response → store in cache
- [ ] T060 [US2] In useTranslation hook, handle translation errors → set error state → return original content as fallback
- [ ] T061 [US2] Integrate useTranslation hook in swizzled src/theme/DocItem/Content/index.js wrapping original component
- [ ] T062 [US2] Add loading state rendering in DocItem/Content wrapper showing spinner while isTranslating is true
- [ ] T063 [US2] Add error banner rendering in DocItem/Content wrapper showing translationError with retry button
- [ ] T064 [US2] Implement retry functionality in error banner → calls clearError() and re-triggers translation

### RTL Text Direction

- [ ] T065 [P] [US2] Create RTL layout manager in src/utils/rtlManager.js with applyRTL() and removeRTL() functions
- [ ] T066 [P] [US2] Implement applyRTL() function to set `dir="rtl"` attribute on document.querySelector('.markdown') container
- [ ] T067 [P] [US2] Implement applyRTL() to set `lang="ur"` attribute on document.documentElement
- [ ] T068 [P] [US2] Implement removeRTL() function to set `dir="ltr"` and `lang="en"` attributes back
- [ ] T069 [P] [US2] Call applyRTL() in LanguageContext when setLanguage('ur') is invoked
- [ ] T070 [P] [US2] Call removeRTL() in LanguageContext when setLanguage('en') is invoked

### RTL CSS Styling

- [ ] T071 [P] [US2] Create src/css/rtl.css file with RTL-specific styles
- [ ] T072 [P] [US2] Add `.markdown[dir="rtl"]` styles: `direction: rtl; text-align: right;`
- [ ] T073 [P] [US2] Add `.markdown[dir="rtl"] pre, .markdown[dir="rtl"] code` styles: `direction: ltr; text-align: left;` to keep code blocks LTR
- [ ] T074 [P] [US2] Add `.markdown[dir="rtl"] ul, .markdown[dir="rtl"] ol` styles: `padding-right: 2em; padding-left: 0;` for list indentation
- [ ] T075 [P] [US2] Add `.markdown[dir="rtl"] table` styles: `direction: ltr;` to keep tables LTR
- [ ] T076 [P] [US2] Import rtl.css in src/css/custom.css
- [ ] T077 [US2] Test RTL rendering in Chrome, Firefox, and Safari browsers for visual consistency

### Formatting Preservation

- [ ] T078 [US2] Verify bold, italic, underline formatting preserved in translated content (test with sample page)
- [ ] T079 [US2] Verify links remain clickable and URLs unchanged in translated content (test with sample page)
- [ ] T080 [US2] Verify lists (ordered and unordered) maintain structure in translated content (test with sample page)
- [ ] T081 [US2] Verify tables maintain structure and alignment in translated content (test with sample page)
- [ ] T082 [US2] Verify images display correctly with translated captions in translated content (test with sample page)
- [ ] T083 [US2] Verify code blocks remain in English with no syntax translation (test with code-heavy page)

### Acceptance Validation

**Test Scenario 1**: Enable Urdu → all headings, paragraphs, lists translated → formatting preserved
**Test Scenario 2**: View page with code blocks → code remains English → comments/descriptions in Urdu (Phase 2 feature)
**Test Scenario 3**: View page with images → captions translated → images unchanged
**Test Scenario 4**: View page with tables → headers and cells translated → table structure maintained
**Test Scenario 5**: Urdu text displays right-to-left → code blocks remain left-to-right

---

## Phase 5: User Story 3 - Use Chatbot with Translated Content (P1)

**Goal**: Chatbot provides Urdu responses when Urdu mode is active

**Duration**: 2 days

**Dependencies**: Phase 4 complete (content translation functional)

**Independent Test**: Submit chatbot query in Urdu mode → verify response in Urdu → verify source references translated

### Backend Chatbot Extension

- [ ] T084 [US3] Extend QueryRequest model in backend/models (from 002-rag-agent) to include optional `language` parameter (default: "en")
- [ ] T085 [US3] Update /query endpoint in backend/rag_agent.py to accept language parameter from request
- [ ] T086 [US3] Implement chunk translation logic in rag_agent.py: if language=='ur', translate retrieved chunks to Urdu before passing to LLM
- [ ] T087 [US3] Call translation_service.translate_text() for each retrieved chunk when language=='ur'
- [ ] T088 [US3] Add LLM instruction in rag_agent.py when language=='ur': "Answer the following question in Urdu language"
- [ ] T089 [US3] Implement source reference translation in rag_agent.py: translate page_title for each source to Urdu
- [ ] T090 [US3] Handle translation failures in rag_agent.py: if chunk translation fails, fallback to English response with warning

### Frontend Chatbot Integration

- [ ] T091 [US3] Update ChatWidget component in src/components/ChatWidget.js to access LanguageContext
- [ ] T092 [US3] Pass currentLanguage from LanguageContext to chatbot API request as `language` parameter
- [ ] T093 [US3] Update chatbot query display in ChatWidget to show language indicator (EN/UR) next to query
- [ ] T094 [US3] Verify chatbot response displays in Urdu when currentLanguage is 'ur'
- [ ] T095 [US3] Verify source references show Urdu page titles when currentLanguage is 'ur'
- [ ] T096 [US3] Handle chatbot errors gracefully: if translation fails, show error message and offer to switch to English

### Acceptance Validation

**Test Scenario 1**: User in Urdu mode submits query "ROS 2 کیا ہے؟" → chatbot responds in Urdu → sources show Urdu page titles
**Test Scenario 2**: User switches language mid-session → submits new query → chatbot responds in newly selected language
**Test Scenario 3**: Chatbot translation fails → error message displayed → user can retry or switch to English
**Test Scenario 4**: User asks in Urdu → answer references translated content sources → source links point to Urdu-translated pages

---

## Phase 6: User Story 4 - Handle Translation Failures Gracefully (P2)

**Goal**: Provide clear error messages and fallback to original content when translation fails

**Duration**: 1 day

**Dependencies**: Phase 5 complete (all core translation features functional)

**Independent Test**: Simulate translation API failures → verify error messages → verify fallback to English → verify retry functionality

### Error Handling Implementation

- [ ] T097 [US4] Create error message mapping in src/utils/errorMessages.js mapping error codes to user-friendly messages
- [ ] T098 [US4] Implement getErrorMessage() function in errorMessages.js: SERVICE_DOWN → "Translation service unavailable. Showing original content. [Retry]"
- [ ] T099 [US4] Add TIMEOUT error message: "Translation took too long. Showing original content. [Retry]"
- [ ] T100 [US4] Add RATE_LIMIT error message: "Translation limit reached. Try again in {retry_after} seconds."
- [ ] T101 [US4] Add INVALID_INPUT error message: "Unable to translate this content. Showing original."
- [ ] T102 [US4] Update useTranslation hook to use getErrorMessage() when setting error state
- [ ] T103 [US4] Implement retry button in error banner (DocItem/Content wrapper) calling useTranslation hook again
- [ ] T104 [US4] Add retry cooldown (5 seconds) to prevent rapid retry spam
- [ ] T105 [US4] Implement automatic fallback to English in useTranslation hook when translation fails (setLanguage('en'))
- [ ] T106 [US4] Add translation timeout indicator showing remaining time (10 second countdown) during long translations

### Error Simulation Testing

- [ ] T107 [P] [US4] Create test utility in backend/tests/test_translation_errors.py to simulate SERVICE_DOWN (stop model)
- [ ] T108 [P] [US4] Create test utility to simulate TIMEOUT (add artificial delay in translation_service.py)
- [ ] T109 [P] [US4] Create test utility to simulate RATE_LIMIT (return 429 status from endpoint)
- [ ] T110 [US4] Test error handling: stop backend → click Urdu toggle → verify SERVICE_DOWN error displayed
- [ ] T111 [US4] Test error handling: add 15s delay → translate page → verify TIMEOUT error displayed after 10s
- [ ] T112 [US4] Test retry functionality: error displayed → click retry → translation succeeds → content displayed

### Acceptance Validation

**Test Scenario 1**: Translation service down → user selects Urdu → error message: "Translation service unavailable" → original content shown
**Test Scenario 2**: Translation timeout → user waits → timeout message after 10s → option to show original or retry
**Test Scenario 3**: Translation fails for specific content → partial page in Urdu → untranslated sections in English with notice
**Test Scenario 4**: User dismisses error → retry button available → click retry → translation succeeds

---

## Phase 7: User Story 5 - Maintain UI Consistency During Translation (P2)

**Goal**: Ensure page layout, styling, and navigation remain consistent when switching languages

**Duration**: 1.5 days

**Dependencies**: Phase 6 complete (error handling functional)

**Independent Test**: Compare original and Urdu page layouts → verify no layout breaks → verify navigation translated

### Navigation Translation

- [ ] T113 [US5] Populate src/translations/ui-strings.json with English-Urdu mapping for UI labels (Modules → ماڈیولز, GitHub → گٹ ہب, etc.)
- [ ] T114 [US5] Create useTranslateUI hook in src/hooks/useTranslateUI.js to load ui-strings.json based on currentLanguage
- [ ] T115 [US5] Swizzle DocSidebar component: `npm run swizzle @docusaurus/theme-classic DocSidebar -- --wrap`
- [ ] T116 [US5] Integrate useTranslateUI hook in swizzled src/theme/DocSidebar/index.js to translate sidebar item labels
- [ ] T117 [US5] Swizzle DocBreadcrumbs component: `npm run swizzle @docusaurus/theme-classic DocBreadcrumbs -- --wrap`
- [ ] T118 [US5] Integrate useTranslateUI hook in swizzled src/theme/DocBreadcrumbs/index.js to translate breadcrumb labels
- [ ] T119 [US5] Update navbar items in docusaurus.config.js to use translatable labels (Modules, Module 1, Module 2, GitHub)
- [ ] T120 [US5] Verify sidebar items display in Urdu when currentLanguage is 'ur'
- [ ] T121 [US5] Verify breadcrumbs display in Urdu when currentLanguage is 'ur'
- [ ] T122 [US5] Verify navbar items display in Urdu when currentLanguage is 'ur'

### Layout Consistency

- [ ] T123 [P] [US5] Create visual regression test baseline: capture screenshots of 10 pages in English mode
- [ ] T124 [P] [US5] Create visual regression test comparison: capture screenshots of same 10 pages in Urdu mode
- [ ] T125 [US5] Compare layout consistency: verify spacing, margins, padding identical between English and Urdu modes
- [ ] T126 [US5] Verify responsive design: test Urdu content on mobile (375px), tablet (768px), desktop (1440px) widths
- [ ] T127 [US5] Verify Urdu text length variations: test short and long Urdu strings to ensure layout doesn't break
- [ ] T128 [US5] Test sidebar navigation: click all sidebar links in Urdu mode → verify pages load correctly
- [ ] T129 [US5] Test breadcrumb navigation: click all breadcrumb links in Urdu mode → verify pages load correctly
- [ ] T130 [US5] Verify navbar remains functional: click all navbar links in Urdu mode → verify navigation works

### Acceptance Validation

**Test Scenario 1**: Switch to Urdu → page re-renders → layout, spacing, navigation structure identical to English
**Test Scenario 2**: Urdu text longer than English → responsive design adjusts → no overflow or broken layout
**Test Scenario 3**: RTL text direction applied → sidebar and navigation remain LTR → content area RTL
**Test Scenario 4**: Sidebar translated → click sidebar links → pages load with Urdu content → navigation preserved

---

## Phase 8: Polish and Cross-Cutting Concerns

**Goal**: Final refinements, performance optimization, and production readiness

**Duration**: 1.5 days

**Dependencies**: All user story phases complete

### Performance Optimization

- [ ] T131 Implement batch translation in backend/services/translation_service.py: combine multiple text segments into single API call
- [ ] T132 Add translation queue in translation_service.py to deduplicate concurrent requests for same content
- [ ] T133 Optimize cache key generation in cacheManager.js: use faster hash algorithm (xxHash or murmur3 instead of MD5)
- [ ] T134 Add cache preloading in src/hooks/useTranslation.js: preload translations for linked pages in background
- [ ] T135 Measure and log translation latency in backend: add performance timing to translate_text() function
- [ ] T136 Measure and log cache hit rate in frontend: add analytics to cacheManager.js (log cache hits vs misses)
- [ ] T137 Verify p95 translation latency < 3 seconds for 5000-word pages (run performance tests on 20 sample pages)
- [ ] T138 Verify cache hit rate > 80% after 10 page navigations (clear cache, navigate 10 pages, check hit rate)

### Production Readiness

- [ ] T139 Add feature flag `enableUrduTranslation` in docusaurus.config.js under customFields
- [ ] T140 Wrap LanguageToggle component with feature flag check: hide toggle if `enableUrduTranslation: false`
- [ ] T141 Add backend health check monitoring: create /health endpoint returning overall system status
- [ ] T142 Add backend logging: log translation requests with metadata (language, content length, cached, duration)
- [ ] T143 Add frontend error logging: log translation errors to console with context (error code, content hash, timestamp)
- [ ] T144 Create backend .env.example file with environment variable templates (MODEL_NAME, CACHE_TTL, API_TIMEOUT)
- [ ] T145 Update backend README.md with setup instructions: model download, GPU requirements, environment variables

### Documentation and Cleanup

- [ ] T146 Create user guide in docs/translation-guide.md: how to use language toggle, what to expect, limitations
- [ ] T147 Add translation disclaimer to Urdu pages: "Auto-translated content. Refer to English for technical accuracy."
- [ ] T148 Update docusaurus.config.js i18n section: add 'ur' to locales array (even though not using official i18n)
- [ ] T149 Clean up console.log statements from development: remove debug logs from translation components
- [ ] T150 Run ESLint on frontend code: `npm run lint` and fix any errors
- [ ] T151 Run Pylint on backend code: `pylint backend/services backend/routers` and fix any errors
- [ ] T152 Format code with Prettier (frontend): `npm run format`
- [ ] T153 Format code with Black (backend): `black backend/`

### Final Validation

- [ ] T154 Run all 20 functional requirements validation tests (FR-001 through FR-020) and document results
- [ ] T155 Validate translation quality: review 50 sample pages with Urdu speaker, record quality score (target: >80%)
- [ ] T156 Validate chatbot accuracy: test 20 Urdu queries, verify responses accurate (target: >95%)
- [ ] T157 Run visual regression tests: compare English and Urdu page screenshots, verify no layout breaks
- [ ] T158 Test edge cases: language switch during page load, rapid page navigation, browser back/forward buttons
- [ ] T159 Test browser compatibility: Chrome, Firefox, Safari, Edge (verify RTL rendering in all browsers)
- [ ] T160 Perform final smoke test: toggle language → navigate 10 pages → use chatbot → verify no errors

---

## Dependency Graph

### User Story Dependencies

```
Phase 1 (Setup)
  ↓
Phase 2 (Foundational Infrastructure)
  ↓
Phase 3 (US1: Language Toggle) ─────────┐
  ↓                                      │
Phase 4 (US2: Content Translation) ─────┤── Independent, can be developed/tested separately
  ↓                                      │
Phase 5 (US3: Chatbot Integration) ─────┤
  ↓                                      │
Phase 6 (US4: Error Handling) ──────────┤
  ↓                                      │
Phase 7 (US5: UI Consistency) ──────────┘
  ↓
Phase 8 (Polish & Optimization)
```

**Critical Path**: Phases 1 → 2 → 3 → 4 → 5 (Must complete in order)
**Parallel Opportunities**: Phases 6 and 7 can be developed in parallel after Phase 5

### Task-Level Dependencies

**Blocking Tasks** (Must complete before any user story work):
- T001-T015: Setup and file creation
- T016-T043: Foundational infrastructure (translation service, cache, context)

**Parallelizable Task Groups**:
- Within Phase 3: T052-T053 (CSS and verification) can run parallel to T044-T051
- Within Phase 4: T065-T077 (RTL management and CSS) fully parallel, T078-T083 (validation) parallel
- Within Phase 5: T091-T096 (frontend) parallel to T084-T090 (backend)
- Within Phase 6: T107-T109 (error simulation setup) parallel
- Within Phase 7: T123-T124 (visual regression baselines) parallel
- Within Phase 8: T150-T153 (linting and formatting) parallel

---

## Parallel Execution Examples

### Phase 3 (US1) - 2 Parallel Streams

**Stream A** (Component Implementation):
- T044 → T045 → T046 → T047 → T048 → T049 → T050 → T051

**Stream B** (Styling and Validation):
- T052 → T053

**Total Time**: ~1.5 days (vs 2 days sequential)

### Phase 4 (US2) - 3 Parallel Streams

**Stream A** (Content Translation):
- T054 → T055 → T056 → T057 → T058 → T059 → T060 → T061 → T062 → T063 → T064

**Stream B** (RTL Management):
- T065 → T066 → T067 → T068 → T069 → T070

**Stream C** (RTL CSS):
- T071 → T072 → T073 → T074 → T075 → T076 → T077

**Stream D** (Validation - after Streams A, B, C):
- T078 [P] T079 [P] T080 [P] T081 [P] T082 [P] T083

**Total Time**: ~2.5 days (vs 4 days sequential)

### Phase 5 (US3) - 2 Parallel Streams

**Stream A** (Backend):
- T084 → T085 → T086 → T087 → T088 → T089 → T090

**Stream B** (Frontend):
- T091 → T092 → T093 → T094 → T095 → T096

**Total Time**: ~2 days (vs 3 days sequential)

### Phase 8 (Polish) - 3 Parallel Streams

**Stream A** (Performance):
- T131 → T132 → T133 → T134 → T135 → T136 → T137 → T138

**Stream B** (Production Readiness):
- T139 → T140 → T141 → T142 → T143 → T144 → T145

**Stream C** (Documentation & Cleanup):
- T146 → T147 → T148 → T149 → T150 [P] T151 [P] T152 [P] T153

**Stream D** (Final Validation - after all streams):
- T154 → T155 → T156 → T157 → T158 → T159 → T160

**Total Time**: ~1.5 days (vs 3 days sequential)

---

## MVP Scope Recommendation

**Minimum Viable Product**: Phase 3 (User Story 1 - Language Toggle)

**Deliverables**:
- Language toggle button in navbar
- React Context for language state management
- localStorage persistence for language preference
- Basic UI showing language selection

**Value**: Validates core infrastructure (React Context, swizzling, state management) without requiring translation backend

**Time**: 2 days (1 day setup + 1.5 days US1)

**Next Increment**: Add Phase 4 (US2) for complete content translation → delivers full end-to-end translation capability

---

## Independent Testing Criteria

### User Story 1 (Language Toggle)
**Entry Criteria**: Phase 2 complete (LanguageContext implemented)
**Test Independence**: Can be tested without backend translation service running
**Test Steps**:
1. Click language toggle in navbar
2. Verify button text changes (English → اردو)
3. Verify currentLanguage state changes in LanguageContext
4. Refresh page → verify language preference persists from localStorage
5. Navigate to different page → verify language preference maintained

### User Story 2 (Content Translation)
**Entry Criteria**: Phase 3 complete (Language toggle functional) + Translation backend running
**Test Independence**: Can be tested independently of chatbot
**Test Steps**:
1. Enable Urdu mode via toggle
2. Verify all text content translated (headings, paragraphs, lists, tables)
3. Verify code blocks remain in English
4. Verify RTL text direction applied to content area
5. Verify formatting preserved (bold, italic, links, images)

### User Story 3 (Chatbot Integration)
**Entry Criteria**: Phase 4 complete (Content translation functional)
**Test Independence**: Can be tested independently of navigation translation
**Test Steps**:
1. Enable Urdu mode
2. Submit chatbot query in Urdu
3. Verify chatbot response in Urdu
4. Verify source references show Urdu page titles
5. Switch language → submit new query → verify response in new language

### User Story 4 (Error Handling)
**Entry Criteria**: Phase 5 complete (All core features functional)
**Test Independence**: Can be tested by simulating failures
**Test Steps**:
1. Stop backend translation service
2. Attempt to switch to Urdu mode
3. Verify error message displayed: "Translation service unavailable"
4. Verify fallback to original English content
5. Start backend → click retry → verify translation succeeds

### User Story 5 (UI Consistency)
**Entry Criteria**: Phase 6 complete (Error handling functional)
**Test Independence**: Can be tested independently via visual comparison
**Test Steps**:
1. Capture screenshots of 10 pages in English mode
2. Switch to Urdu mode
3. Capture screenshots of same 10 pages in Urdu mode
4. Compare layouts → verify spacing, margins, navigation structure identical
5. Verify sidebar, breadcrumbs, navbar translated to Urdu

---

## Task Validation

**Format Validation**: ✅ ALL tasks follow checklist format
- ✅ Every task has checkbox `- [ ]`
- ✅ Every task has sequential ID (T001-T160)
- ✅ Parallelizable tasks marked with [P]
- ✅ User story tasks marked with [US1]-[US5]
- ✅ Every task has clear description with file path

**Completeness Validation**: ✅ ALL user stories covered
- ✅ User Story 1: 10 tasks (T044-T053)
- ✅ User Story 2: 30 tasks (T054-T083)
- ✅ User Story 3: 13 tasks (T084-T096)
- ✅ User Story 4: 16 tasks (T097-T112)
- ✅ User Story 5: 18 tasks (T113-T130)
- ✅ Setup: 15 tasks (T001-T015)
- ✅ Foundational: 28 tasks (T016-T043)
- ✅ Polish: 30 tasks (T131-T160)

**Total**: 160 tasks across 8 phases

---

## Execution Timeline

| Phase | Duration | Tasks | Parallel Opportunities |
|-------|----------|-------|------------------------|
| Phase 1: Setup | 0.5 days | 15 | 11 tasks parallelizable |
| Phase 2: Foundational | 2 days | 28 | 5 tasks parallelizable |
| Phase 3: US1 | 1.5 days | 10 | 2 tasks parallelizable |
| Phase 4: US2 | 2.5 days | 30 | 13 tasks parallelizable |
| Phase 5: US3 | 2 days | 13 | Frontend and backend streams |
| Phase 6: US4 | 1 day | 16 | 3 test utilities parallelizable |
| Phase 7: US5 | 1.5 days | 18 | 2 visual regression tasks parallelizable |
| Phase 8: Polish | 1.5 days | 30 | 4 linting tasks parallelizable |
| **Total** | **12.5 days** | **160** | **~35% parallelizable** |

**With Parallel Execution**: ~9-10 working days (2 weeks)
**Sequential Execution**: ~12-13 working days (2.5 weeks)

---

## Notes for Implementers

1. **Task IDs are sequential** but execution order can vary within phases based on dependencies
2. **[P] marker indicates parallelizability** - tasks can run concurrently if on different files
3. **User story labels [US1]-[US5]** map to spec.md user stories for traceability
4. **Each phase is independently testable** - run acceptance tests at end of each phase
5. **MVP scope is Phase 3** - delivers language toggle without requiring backend translation
6. **Backend tasks require GPU** - mBART model loading requires ~4GB GPU memory
7. **Frontend tasks require Docusaurus knowledge** - swizzling is Docusaurus-specific pattern
8. **Translation quality validation requires Urdu speaker** - T155 needs native speaker review

---

**Implementation Status**: Ready for execution
**Next Step**: Begin Phase 1 (Setup) tasks T001-T015
**Estimated Completion**: 2-2.5 weeks with parallel execution
