# Implementation Plan: Frontend-Backend Integration

**Feature**: 003-frontend-backend-integration
**Created**: 2025-12-22
**Status**: Draft

## 1. Scope and Dependencies

### In Scope

- **Query Input Component**: Text input field for user queries with validation
- **Submit Button**: Trigger query submission with keyboard support (Enter key)
- **Loading State UI**: Visual indicator (spinner/text) during API requests
- **Answer Display Component**: Render backend response with preserved formatting
- **Source References Component**: Display clickable source links with titles and relevance scores
- **Error Display Component**: User-friendly error messages for network and backend errors
- **HTTP Client**: Fetch API integration for POST requests to localhost:8000/query
- **JSON Serialization**: Construct QueryRequest and parse QueryResponse
- **CORS Handling**: Ensure requests include appropriate headers for cross-origin
- **Client-Side Validation**: Prevent empty/whitespace-only query submission
- **State Management**: Manage loading, error, and response states

### Out of Scope

- User authentication or session management
- Query history or conversation persistence
- Advanced UI features (markdown rendering, syntax highlighting, code blocks)
- Mobile-responsive design optimization
- Accessibility features (ARIA labels, screen reader support, keyboard navigation)
- Internationalization or multi-language UI
- Dark mode or theme customization
- Query suggestions or autocomplete
- Editing or re-submitting previous queries from history
- Sharing or exporting query results
- Analytics or usage tracking
- Production build optimization or deployment
- Custom parameter controls (top_k, model selection UI)
- Multi-turn conversation interface
- Real-time streaming of responses
- Backend modifications or API changes
- Error recovery retry logic with exponential backoff

### External Dependencies

| Dependency | Type | Purpose | Ownership | SLA/Availability |
|------------|------|---------|-----------|------------------|
| RAG Agent Backend (002-rag-agent) | Internal Service | Query processing and answer generation | Backend team | Development only (localhost) |
| Browser Fetch API | Browser API | HTTP client for API requests | Browser vendors | N/A (standard API) |
| JSON.stringify/parse | JavaScript API | Request/response serialization | JavaScript runtime | N/A (built-in) |

## 2. Key Decisions and Rationale

### Decision 1: Vanilla HTML/JavaScript vs Framework

**Options Considered**:
- A) Vanilla HTML + JavaScript (no framework)
- B) React (component-based, state management)
- C) Vue.js (progressive framework, simpler than React)

**Trade-offs**:
- Option A: Zero dependencies, fastest to set up, easy to debug, but manual DOM manipulation
- Option B: Component reusability, rich ecosystem, but requires build tooling (Webpack/Vite), steeper learning curve
- Option C: Easier than React, good documentation, but still requires build setup

**Decision**: Option A - Vanilla HTML + JavaScript

**Rationale**:
- Constraint: "No backend logic changes" implies minimal complexity requirement
- Local development only - no need for production build optimization
- Simple use case (single page, basic interactions) doesn't justify framework overhead
- Zero setup time - create single HTML file and open in browser
- Easier to debug for developers unfamiliar with frameworks
- No build step required - instant development workflow

**Reversibility**: High - business logic (API calls, state management) can be extracted to React/Vue later

**Measurement**: Development completed in single HTML file < 300 lines, no build errors

---

### Decision 2: Fetch API vs XMLHttpRequest vs Axios

**Options Considered**:
- A) Fetch API (modern, promise-based, native)
- B) XMLHttpRequest (legacy, callback-based)
- C) Axios (third-party library, feature-rich)

**Trade-offs**:
- Option A: Modern syntax, promise-based, native (no dependencies), but less IE11 support
- Option B: Universal browser support, but callback hell, verbose syntax
- Option C) automatic JSON parsing, better error handling, but external dependency

**Decision**: Option A - Fetch API

**Rationale**:
- Constraint: Browser compatibility targets modern browsers only (no IE11)
- Native API - no external dependencies to manage
- Promise-based syntax aligns with async/await patterns
- Sufficient for simple POST requests with JSON payloads
- Error handling adequate for local development (network errors, HTTP status)

**Reversibility**: High - HTTP client is isolated in single function, easy to swap

**Measurement**: API requests complete successfully with proper error handling

---

### Decision 3: State Management Pattern

**Options Considered**:
- A) Global JavaScript variables with direct DOM manipulation
- B) Simple state object with render function
- C) State machine with explicit state transitions

**Trade-offs**:
- Option A: Simplest to implement, but scattered state logic, hard to debug
- Option B: Centralized state, predictable updates, clear separation of state/rendering
- Option C: Most robust, prevents invalid states, but over-engineered for simple UI

**Decision**: Option B - Simple state object with render function

**Rationale**:
- Balances simplicity with maintainability
- Single source of truth for UI state (loading, error, response)
- Clear separation: state updates → render → DOM changes
- Easy to debug (inspect state object in console)
- Sufficient for managing 4 states: idle, loading, success, error

**Reversibility**: High - can upgrade to state machine if complexity grows

**Measurement**: No inconsistent UI states (e.g., loading spinner with error message)

**State Structure**:
```javascript
const state = {
  status: 'idle' | 'loading' | 'success' | 'error',
  query: string,
  response: { answer, sources, ... } | null,
  error: string | null
}
```

---

### Decision 4: Error Handling Strategy

**Options Considered**:
- A) Generic error messages ("Something went wrong")
- B) Specific messages per error type (network, backend, validation)
- C) Full error details with stack traces

**Trade-offs**:
- Option A: Simplest, but unhelpful for users debugging issues
- Option B: User-friendly, actionable feedback, moderate implementation effort
- Option C) Most informative, but exposes technical details to non-technical users

**Decision**: Option B - Specific error messages per type

**Rationale**:
- Success Criteria: "Error Clarity: Connection errors and backend errors display clear, user-friendly messages"
- Different error types require different user actions:
  - Network error → "Cannot connect to backend. Is the server running on localhost:8000?"
  - Backend error → Display backend error message (already user-friendly from backend)
  - Validation error → "Query cannot be empty"
- Helps users self-diagnose common issues (forgot to start backend)

**Reversibility**: High - error messages are strings, easy to modify

**Measurement**: Error messages tested for network failure, backend 500, backend 422, empty query

**Error Types**:
```javascript
// Network error (fetch failed)
"Cannot connect to backend at http://localhost:8000. Please ensure the backend server is running."

// HTTP error (backend returned 4xx/5xx)
"Backend error: {error message from backend}"

// Empty query
"Please enter a question before submitting."
```

---

### Decision 5: Source Reference Display Format

**Options Considered**:
- A) Plain list of URLs
- B) URLs with page titles
- C) URLs with page titles and relevance scores

**Trade-offs**:
- Option A: Simplest, but unhelpful (URLs not descriptive)
- Option B: More useful, shows context of source, moderate implementation
- Option C: Most informative, aligns with spec requirement, minimal extra effort

**Decision**: Option C - URLs with page titles and relevance scores

**Rationale**:
- Functional Requirement FR-007: "Frontend MUST display source references including URL, page title, and relevance score"
- Success Criteria: "Source Attribution: Source references are displayed as clickable links with titles and relevance scores"
- Relevance scores help users understand which sources are most relevant
- Page titles provide context (e.g., "ROS 2 Overview" more useful than URL)

**Reversibility**: High - display format is CSS/HTML, easy to modify

**Measurement**: All three fields (URL, title, score) displayed for each source

**Display Format**:
```html
<div class="source">
  <a href="{url}" target="_blank">{page_title}</a>
  <span class="relevance-score">Relevance: {score}</span>
</div>
```

---

## 3. Interfaces and API Contracts

### Frontend → Backend API Contract

**Endpoint**: `POST http://localhost:8000/query`

**Request Schema** (QueryRequest):
```json
{
  "query": "string (required, 1-2000 chars)",
  "top_k": "integer (optional, default: 5, range: 1-50)",
  "model": "string (optional, default: 'xiaomi/mimo-v2-flash:free')"
}
```

**Frontend Behavior**:
- Send only `query` field (use backend defaults for `top_k` and `model`)
- Validate query is non-empty before sending
- Set `Content-Type: application/json` header

**Response Schema** (QueryResponse - Success):
```json
{
  "query": "string (echoed back)",
  "answer": "string (generated answer)",
  "sources": [
    {
      "url": "string",
      "page_title": "string",
      "relevance_score": "number (0-1, 4 decimal places)",
      "chunk_index": "integer | null"
    }
  ],
  "model_used": "string",
  "tokens_used": "integer",
  "retrieval_count": "integer"
}
```

**Frontend Behavior**:
- Extract `answer` and `sources` for display
- Display `model_used` and `tokens_used` as metadata (optional)
- Handle empty `sources` array gracefully

**Response Schema** (ErrorResponse - Failure):
```json
{
  "error_type": "string",
  "error_message": "string",
  "query": "string (echoed back)",
  "status_code": "integer"
}
```

**HTTP Status Codes**:
- `200 OK` - Successful query processing
- `422 Unprocessable Entity` - Validation error (invalid query format)
- `503 Service Unavailable` - Retrieval service failure (Qdrant/Cohere unavailable)
- `502 Bad Gateway` - Generation service failure (OpenRouter API error)
- `429 Too Many Requests` - Rate limit exceeded (OpenRouter)
- `500 Internal Server Error` - Unexpected backend error

**Frontend Error Handling**:
- Network error (fetch rejected) → "Cannot connect to backend..."
- HTTP 422 → Display `error_message` from backend
- HTTP 500/502/503 → Display `error_message` from backend
- HTTP 429 → "Rate limit exceeded. Please try again later."
- Other HTTP errors → "Unexpected error: HTTP {status}"

**Versioning Strategy**: N/A (local development, no versioning needed)

**Idempotency**: Not required (each query is independent, no side effects)

**Timeouts**: 60 seconds (browser default fetch timeout acceptable for local development)

**Retries**: None (constraint: "Error recovery retry logic with exponential backoff" is out of scope)

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance

**Targets**:
- **UI Responsiveness**: Query submission triggers loading state within 100ms (user feedback)
- **Response Rendering**: Answer displayed within 2 seconds of receiving backend response (Success Criteria)
- **Empty Query Validation**: Validation feedback displayed instantly (< 50ms)

**Resource Caps**:
- **Memory**: < 10 MB for single-page application (baseline browser tab usage)
- **Network**: Single POST request per query submission (no polling, no multiple requests)

**Degradation Strategy**:
- Slow backend (> 10 seconds) → Loading indicator remains visible, no timeout error (allow backend to complete)
- Large responses (> 10 KB) → Render normally (browser can handle large text blocks)

**Measurement**:
- Chrome DevTools Performance tab: Time from button click to loading state < 100ms
- Manual testing: Response display after backend returns < 2 seconds

---

### Reliability

**SLOs** (Service Level Objectives):
- **UI Availability**: 100% (static HTML, no server-side dependencies)
- **Backend Availability**: Not controlled by frontend (dependency on 002-rag-agent)
- **Error Handling Coverage**: 100% of error types (network, HTTP 4xx/5xx) display user-friendly messages

**Error Budgets**: N/A (local development, no uptime requirements)

**Degradation Strategy**:
- Backend unavailable → Error message with actionable guidance ("Is the server running?")
- Network timeout → Browser default timeout (60s), no custom handling

---

### Security

**Authentication/Authorization**: None (constraint: "No authentication" per spec)

**Data Handling**:
- No sensitive data stored (queries and responses are ephemeral, not persisted)
- No cookies or local storage used
- No third-party analytics or tracking

**Secrets**: None (all communication to localhost, no API keys in frontend)

**Auditing**: None (no logging or monitoring)

**Input Validation**:
- Client-side: Prevent empty queries (whitespace trimming)
- Server-side: Backend validates all inputs (frontend validation is UX enhancement only)

**XSS Prevention**:
- Use `textContent` instead of `innerHTML` for user-generated content (query, answer)
- Exception: Source URLs use `href` attribute (safe for URLs from trusted backend)

---

### Cost

**Unit Economics**: N/A (local development, no hosting costs)

---

## 5. Data Management and Migration

### Source of Truth

- **Query State**: Frontend application state (ephemeral, not persisted)
- **Responses**: Backend API (frontend is read-only consumer)

### Schema Evolution

N/A (no data persistence)

### Migration and Rollback

N/A (no data migration)

### Data Retention

- **Queries and Responses**: Not persisted (cleared on page refresh)
- **Error Messages**: Cleared on new query submission

---

## 6. Operational Readiness

### Observability

**Logs**:
- Console.log statements for debugging (development only):
  - Query submission: `console.log('Submitting query:', query)`
  - API response: `console.log('Received response:', response)`
  - Errors: `console.error('Error:', error)`

**Metrics**: None (no metrics collection)

**Traces**: None (no distributed tracing)

---

### Alerting

None (local development)

---

### Runbooks

**Common Tasks**:

1. **Starting the Frontend**:
   - Open `frontend.html` in modern browser (Chrome, Firefox, Safari, Edge)
   - Ensure backend is running on `http://localhost:8000`

2. **Testing Query Submission**:
   - Enter query: "What is ROS 2?"
   - Click "Submit" or press Enter
   - Verify loading indicator appears
   - Verify answer and sources display after response

3. **Testing Error Handling**:
   - Stop backend server
   - Submit query
   - Verify connection error message displays
   - Restart backend
   - Submit query again
   - Verify error clears and response displays

4. **Verifying CORS**:
   - Open browser DevTools Console
   - Check for CORS errors (should be none if backend configured correctly)
   - If CORS error appears: Backend CORS configuration needs updating

---

### Deployment and Rollback

**Deployment**: N/A (constraint: "No deployment or auth")

**Rollback**: N/A (single HTML file, no versioning needed)

---

### Feature Flags

None (all features enabled by default)

---

## 7. Risk Analysis and Mitigation

### Top Risks

**Risk 1: Backend Not Running**

- **Probability**: High (common during development)
- **Impact**: High (frontend non-functional without backend)
- **Blast Radius**: Local development only
- **Mitigation**:
  - Clear error message: "Cannot connect to backend at http://localhost:8000. Please ensure the backend server is running."
  - Instructions in README: "Start backend before opening frontend"
- **Kill Switch**: N/A (user can close browser tab)
- **Guardrails**: Error message prevents user confusion

---

**Risk 2: CORS Not Configured on Backend**

- **Probability**: Low (backend CORS already configured per spec dependency)
- **Impact**: High (all requests blocked by browser)
- **Blast Radius**: Local development only
- **Mitigation**:
  - Assumption documented: "Backend CORS configuration allows requests from frontend origin"
  - Validation step: Test frontend-backend communication before marking feature complete
  - Fallback: Update backend CORS if needed (violates "no backend changes" constraint - escalate to user)
- **Kill Switch**: N/A
- **Guardrails**: Browser DevTools console shows CORS errors (developer can diagnose)

---

**Risk 3: Large Responses Cause UI Performance Issues**

- **Probability**: Low (typical answers < 2 KB based on backend testing)
- **Impact**: Medium (slow rendering, but functional)
- **Blast Radius**: Single page only
- **Mitigation**:
  - Browser handles large text blocks natively (up to 100 KB acceptable)
  - If performance issue arises: Add CSS max-height + scrollbar (simple fix)
- **Kill Switch**: N/A
- **Guardrails**: Success Criteria includes "Formatting Preservation" test with long answers

---

## 8. Evaluation and Validation

### Definition of Done

- [ ] Frontend sends POST request to `http://localhost:8000/query` with valid JSON payload
- [ ] Query validation prevents empty/whitespace-only submission with error message
- [ ] Loading indicator displays while request is in progress
- [ ] Answer text renders in UI with preserved formatting (line breaks)
- [ ] Source references display with clickable URLs, page titles, and relevance scores
- [ ] Source URLs open in new tab when clicked
- [ ] Network errors display user-friendly message with backend URL
- [ ] Backend errors (4xx/5xx) display error message from backend response
- [ ] Error messages clear when new query is submitted
- [ ] Multiple sequential queries work without page refresh
- [ ] No CORS errors in browser console
- [ ] All Success Criteria from spec validated (10 criteria)

### Output Validation

**Format Validation**:
- HTML validates as HTML5 (no syntax errors)
- JavaScript lints without errors (ESLint or browser console)

**Requirements Validation**:
- All 12 Functional Requirements (FR-001 to FR-012) implemented and tested
- All 4 User Stories validated with acceptance scenarios
- All 7 Edge Cases tested (long queries, slow responses, malformed JSON, etc.)

**Safety Validation**:
- XSS prevention: User input uses `textContent`, not `innerHTML`
- No JavaScript errors in browser console during normal operation
- No memory leaks (page can remain open for 1+ hour without performance degradation)

---

## 9. Architecture Decision Record (ADR)

**ADR-001**: Use Vanilla HTML/JavaScript Instead of React/Vue
**Context**: Need simple web interface for querying RAG backend locally
**Decision**: Implement using vanilla HTML + JavaScript without frameworks
**Rationale**: Constraints favor simplicity (no deployment, local dev only). Single-page use case doesn't justify framework overhead. Zero build step enables instant development.
**Status**: Accepted
**Consequences**: Faster development, easier debugging, but manual DOM manipulation. If complexity grows, may need to migrate to framework.

---

**ADR-002**: Use Fetch API for HTTP Requests
**Context**: Need HTTP client to communicate with localhost:8000 backend
**Decision**: Use native Fetch API with async/await
**Rationale**: Modern browsers support Fetch. Promise-based syntax cleaner than XMLHttpRequest. No external dependencies (Axios) needed for simple POST requests.
**Status**: Accepted
**Consequences**: No IE11 support (acceptable per constraints). Simple error handling sufficient for local dev.

---

**ADR-003**: Display Relevance Scores with Source References
**Context**: Backend provides relevance scores, unclear if frontend should display them
**Decision**: Display relevance score alongside URL and title for each source
**Rationale**: FR-007 requires "relevance score" display. Helps users understand which sources are most relevant. Minimal implementation effort.
**Status**: Accepted
**Consequences**: Slightly more UI complexity, but improves transparency and aligns with spec.

---

## 10. Technical Context

### Technology Stack

- **Frontend**: HTML5, CSS3, JavaScript (ES6+)
- **HTTP Client**: Fetch API (native browser API)
- **JSON Handling**: JSON.stringify, JSON.parse (native JavaScript)
- **Browser Target**: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+ (modern browsers with Fetch API support)

### File Structure

```
specs/003-frontend-backend-integration/
├── plan.md (this file)
├── spec.md
├── checklists/
│   └── requirements.md
└── frontend.html (implementation - to be created)
```

### Integration Points

**Backend Dependency**:
- Service: RAG Agent API (002-rag-agent)
- Endpoint: POST http://localhost:8000/query
- Contract: QueryRequest/QueryResponse schemas (documented in Section 3)

**Browser APIs**:
- Fetch API: HTTP requests
- DOM API: UI rendering and event handling
- Console API: Debugging logs

---

## Constitution Check

### Alignment with Core Principles

**I. Technical Accuracy** ✅
- All API contracts match backend implementation (002-rag-agent spec)
- Fetch API syntax verified against MDN documentation
- Error handling covers all backend HTTP status codes

**II. Clarity for Learners** ✅
- Plan avoids implementation details where possible
- Rationales explain "why" for each decision
- Trade-offs documented for architectural choices

**III. Consistency with Standards** ✅
- Follows REST API conventions (POST /query)
- JSON request/response format standard
- HTTP status codes follow RFC 7231 semantics

**IV. Practicality with Actionable Instructions** ✅
- Runbooks provide step-by-step testing procedures
- Error messages include actionable guidance
- Clear Definition of Done checklist

**V. Reproducibility** ✅
- All dependencies are standard browser APIs (universally available)
- Backend dependency clearly documented (002-rag-agent)
- No environment-specific configuration required

**VI. Verification Before Inclusion** ✅
- Plan includes testing procedures for all features
- Edge cases identified and require testing
- Success Criteria provide measurable validation

**VII. Educational Structure** ✅
- Plan progresses logically (decisions → interfaces → NFRs → validation)
- Rationales support learning (explain trade-offs)

### Gates and Violations

**Gate 1: No Backend Changes** ✅
Status: PASS
Validation: All decisions assume backend API is stable (no modifications)

**Gate 2: Local Development Only** ✅
Status: PASS
Validation: No deployment, no production optimizations, no auth

**Gate 3: Use Backend Defaults** ✅
Status: PASS
Validation: Frontend sends only `query` field (no top_k, model parameters)

**Gate 4: CORS Dependency** ✅
Status: PASS
Validation: Assumption documented, risk mitigation included for CORS failures

---

## Next Steps

1. ✅ Planning Complete (this file)
2. ⏭️ Create `tasks.md` from this plan (`/sp.tasks`)
3. ⏭️ Implement `frontend.html` following task list
4. ⏭️ Test all Success Criteria and Edge Cases
5. ⏭️ Validate Constitution Compliance
6. ⏭️ Mark feature complete
