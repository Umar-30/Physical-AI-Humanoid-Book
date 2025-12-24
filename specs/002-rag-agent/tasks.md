# Implementation Tasks: RAG Agent API Service

**Feature**: 002-rag-agent
**Created**: 2025-12-21
**Status**: Ready for Implementation

## Overview

This task list implements a production-ready RAG (Retrieval-Augmented Generation) Agent API using FastAPI and OpenAI. The implementation is organized by user stories to enable independent, incremental delivery.

**Total Tasks**: 38
**Phases**: 7 (Setup + Foundational + 5 User Stories)
**Parallel Opportunities**: 24 parallelizable tasks

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Recommended MVP**: User Story 1 only (Submit Questions via API)
- Basic endpoint accepting queries
- Simple integration with retrieval and OpenAI
- Returns answer with sources
- Validates core end-to-end flow

**Why**: Demonstrates complete value chain (query → retrieve → generate → respond), can be tested immediately, enables early feedback.

### Incremental Delivery

Each user story is independently testable and deliverable:
1. **US1**: Core API endpoint (MVP - delivers first value)
2. **US2**: Enhanced retrieval quality (improves accuracy)
3. **US3**: Smart context management (handles edge cases)
4. **US4**: Robust generation (improves reliability)
5. **US5**: Complete source attribution (production-ready)

---

## Dependencies Graph

```
Setup (Phase 1)
    ↓
Foundational (Phase 2)
    ↓
├─→ US1: Submit Questions via API (P1) ← Start here for MVP
│   ↓
├─→ US2: Retrieve Relevant Documentation (P1) ← Enhances US1
│   ↓
├─→ US3: Combine Context with Query (P1) ← Requires US2
│   ↓
├─→ US4: Generate Natural Language Answer (P1) ← Requires US3
│   ↓
└─→ US5: Return Answer with Source Attribution (P1) ← Requires US4
    ↓
Polish & Cross-Cutting (Phase 7)
```

**Key**: All US1-US5 are P1 priority and build on each other. However, each can be tested independently with appropriate mocks/stubs.

---

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize project structure, install dependencies, configure environment

**Tasks**:

- [x] T001 Create FastAPI project structure in `backend/rag_agent.py`
- [x] T002 [P] Install FastAPI dependencies: `pip install fastapi uvicorn pydantic`
- [x] T003 [P] Install OpenAI SDK: `pip install openai>=1.0.0`
- [x] T004 [P] Install token counting library: `pip install tiktoken`
- [x] T005 [P] Verify existing retrieval.py from Spec-2 is available at `backend/retrieval.py`
- [x] T006 Update `.env` file with `OPENAI_API_KEY` placeholder
- [x] T007 Create `backend/requirements.txt` with all dependencies (fastapi, uvicorn, openai, tiktoken, cohere, qdrant-client, pydantic, python-dotenv)
- [x] T008 [P] Create logging configuration in `backend/config/logging_config.py` with structured JSON format

**Acceptance**: Project structure exists, all dependencies installed, `.env` configured, imports work without errors

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Build shared foundation needed by all user stories

**Tasks**:

- [x] T009 [P] Create Pydantic models for QueryRequest in `backend/models/request_models.py` (fields: query, top_k, model with validation)
- [x] T010 [P] Create Pydantic models for QueryResponse in `backend/models/response_models.py` (fields: query, answer, sources, model_used, tokens_used, retrieval_count)
- [x] T011 [P] Create Pydantic models for SourceReference in `backend/models/response_models.py` (fields: url, page_title, relevance_score, chunk_index)
- [x] T012 [P] Create Pydantic model for ErrorResponse in `backend/models/response_models.py` (fields: error, error_type, error_message, query, status_code)
- [x] T013 [P] Create custom exception classes in `backend/exceptions.py` (ValidationError, RetrievalError, GenerationError, RateLimitError)
- [x] T014 Create FastAPI app instance in `backend/rag_agent.py` with CORS middleware configured (allow_origins=["*"] for development)
- [x] T015 [P] Create health check endpoint GET `/health` in `backend/rag_agent.py` returning status, version, timestamp

**Acceptance**: All Pydantic models validate correctly, exceptions can be raised and caught, FastAPI app starts without errors, health endpoint returns 200

---

## Phase 3: User Story 1 - Submit Questions via API (P1)

**User Story**: As a documentation user, I want to submit natural language questions through an API so that I can get accurate answers grounded in the project documentation.

**Independent Test Criteria**:
- POST `/query` endpoint accepts JSON with `query` field
- Valid requests return 200 with QueryResponse format
- Invalid requests return 422 with validation errors
- Concurrent requests are handled independently

**Tasks**:

- [x] T016 [US1] Create POST `/query` endpoint skeleton in `backend/rag_agent.py` accepting QueryRequest
- [x] T017 [US1] Implement request validation using Pydantic in `/query` endpoint (validate query length 1-2000, top_k 1-50, model in allowed list)
- [x] T018 [US1] Create simple integration service in `backend/services/integration_service.py` to orchestrate retrieval → generation flow
- [x] T019 [US1] Implement error handling in `/query` endpoint mapping exceptions to HTTP status codes (422 for validation, 503 for retrieval, 502 for generation, 429 for rate limit)
- [x] T020 [US1] Add structured logging to `/query` endpoint (log query, latency, tokens_used, status)
- [x] T021 [US1] Test endpoint with valid query using cURL or Swagger UI (manual test: "How do I create a ROS 2 publisher?")
- [x] T022 [US1] Test endpoint with invalid query (empty string, too long, invalid top_k) and verify 422 responses
- [x] T023 [US1] Test concurrent requests (5 simultaneous queries) and verify all return correctly

**Acceptance**:
- Endpoint accepts queries and returns answers
- Validation errors return 422 with clear messages
- Concurrent requests work without interference

---

## Phase 4: User Story 2 - Retrieve Relevant Documentation (P1)

**User Story**: As the system, I want to identify and retrieve the most relevant documentation sections for each user query so that generated answers are grounded in accurate source material.

**Independent Test Criteria**:
- Retrieval service correctly calls `retrieval.py` functions
- Top-K parameter controls number of chunks retrieved
- Relevance scores are included in results
- Retrieval errors are handled gracefully

**Tasks**:

- [x] T024 [P] [US2] Create RetrievalService class in `backend/services/retrieval_service.py` wrapping `retrieval.embed_query()` and `retrieval.search_qdrant()`
- [x] T025 [US2] Implement `retrieve_context()` method in RetrievalService accepting query and top_k, returning List[SearchResult]
- [x] T026 [US2] Add error handling in RetrievalService for Qdrant unavailable (raise RetrievalError with message "Vector database unavailable")
- [x] T027 [US2] Add error handling for Cohere API failures (raise RetrievalError with message "Query embedding failed")
- [x] T028 [US2] Add logging to RetrievalService (log query, top_k, retrieval_count, average_relevance_score, retrieval_latency_ms)
- [x] T029 [US2] Integrate RetrievalService into IntegrationService in `backend/services/integration_service.py`
- [x] T030 [US2] Test retrieval with query "What is ROS 2?" and verify top 5 results are relevant (manual verification of content)
- [x] T031 [US2] Test retrieval with custom top_k=10 and verify exactly 10 results returned
- [x] T032 [US2] Test retrieval error handling by temporarily setting invalid QDRANT_URL and verifying 503 error

**Acceptance**:
- Retrieval service successfully queries Qdrant
- Custom top_k values are respected
- Relevance scores are captured
- Errors return 503 with clear messages

---

## Phase 5: User Story 3 - Combine Context with Query (P1)

**User Story**: As the system, I want to combine retrieved documentation with the user query in a structured format so that answer generation uses the provided context rather than external knowledge.

**Independent Test Criteria**:
- Prompt builder formats context with XML tags
- Source metadata is preserved in formatted context
- Token counting works correctly with tiktoken
- Long context is truncated to stay within limits

**Tasks**:

- [x] T033 [P] [US3] Create PromptBuilder class in `backend/services/prompt_builder.py` with `build_prompt()` method
- [x] T034 [US3] Implement XML-delimited prompt template in PromptBuilder: `<context>[chunks]</context><query>[query]</query>`
- [x] T035 [US3] Implement token counting using tiktoken library in `backend/utils/token_counter.py` with `count_tokens(text: str) -> int`
- [x] T036 [US3] Implement chunk selection logic in PromptBuilder: select top-N complete chunks that fit within max_tokens limit (default 3000)
- [x] T037 [US3] Add source metadata to formatted context (include URL after each chunk: "Source: [url]")
- [x] T038 [US3] Add system instructions to prompt: "You are a helpful assistant that answers questions based ONLY on provided documentation."
- [x] T039 [US3] Integrate PromptBuilder into IntegrationService
- [x] T040 [US3] Test prompt building with 5 chunks and verify all fit within 3000 tokens
- [x] T041 [US3] Test prompt building with verbose chunks (>3000 tokens total) and verify truncation keeps top-scoring chunks
- [x] T042 [US3] Verify formatted prompt includes XML tags and source URLs (inspect logged prompt)

**Acceptance**:
- Prompts are properly formatted with XML delimiters
- Source URLs are included in context
- Token limits are respected (no overflow)
- Top chunks are prioritized when truncating

---

## Phase 6: User Story 4 - Generate Natural Language Answer (P1)

**User Story**: As the system, I want to generate a natural language answer from the combined context and query so that users receive accurate, readable responses synthesized from the documentation.

**Independent Test Criteria**:
- AnswerGenerationService calls OpenAI Chat Completions API
- Generated answers cite information from context
- "No information" responses when context doesn't contain answer
- Token usage is logged for cost tracking

**Tasks**:

- [x] T043 [P] [US4] Create AnswerGenerationService class in `backend/services/generation_service.py` with OpenAI client initialization
- [x] T044 [US4] Implement `generate_answer()` method accepting context and query, returning Dict with 'answer' and 'tokens_used'
- [x] T045 [US4] Configure OpenAI Chat Completion call with model (default gpt-3.5-turbo), temperature=0.1 (deterministic), max_tokens=500
- [x] T046 [US4] Add retry logic with exponential backoff for transient errors (429 rate limit, 500/502/503 server errors) max 3 retries
- [x] T047 [US4] Add error handling for non-retryable errors (401 auth, 400 bad request) raising GenerationError
- [x] T048 [US4] Add logging to AnswerGenerationService (log model_used, tokens_used, generation_latency_ms, completion_length)
- [x] T049 [US4] Integrate AnswerGenerationService into IntegrationService
- [x] T050 [US4] Test generation with context about ROS 2 publishers and verify answer cites specific steps from context
- [x] T051 [US4] Test generation with irrelevant context (query "What is quantum computing?" with ROS 2 docs) and verify "no information" response
- [x] T052 [US4] Test OpenAI rate limit handling by temporarily using invalid API key and verifying retry behavior (check logs)

**Acceptance**:
- Service successfully calls OpenAI API
- Answers are grounded in provided context
- "No information" responses when appropriate
- Retries work for transient errors
- Token usage is logged

---

## Phase 7: User Story 5 - Return Answer with Source Attribution (P1)

**User Story**: As a documentation user, I want to receive both the generated answer and references to the source documentation used to create it so that I can verify information and explore related content.

**Independent Test Criteria**:
- Response includes both answer and sources array
- Sources are deduplicated by URL
- Each source includes url, page_title, relevance_score
- Empty sources when no results found

**Tasks**:

- [x] T053 [P] [US5] Create SourceDeduplicator class in `backend/services/source_deduplicator.py` with `deduplicate()` method
- [x] T054 [US5] Implement deduplication logic: group by URL, keep highest-scoring chunk per URL
- [x] T055 [US5] Convert SearchResults to SourceReferences in deduplicator (extract url, page_title, score, chunk_index from metadata)
- [x] T056 [US5] Integrate SourceDeduplicator into IntegrationService (deduplicate before building response)
- [x] T057 [US5] Update IntegrationService to build QueryResponse with all fields (query, answer, sources, model_used, tokens_used, retrieval_count)
- [x] T058 [US5] Add response formatting ensuring compliance with Pydantic QueryResponse schema
- [x] T059 [US5] Test deduplication with 5 chunks from 3 URLs and verify only 3 sources in response
- [x] T060 [US5] Test response format validates against QueryResponse schema (all required fields present)
- [x] T061 [US5] Test query with no results (unrelated topic) and verify sources array is empty with appropriate answer

**Acceptance**:
- Responses include deduplicated sources
- Each source has complete metadata
- Response validates against schema
- Empty sources handled correctly

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Production-ready quality, observability, documentation

**Tasks**:

- [ ] T062 [P] Add comprehensive error logging with stack traces for 500-level errors in `backend/rag_agent.py`
- [ ] T063 [P] Create structured log formatter in `backend/utils/log_formatter.py` outputting JSON with timestamp, level, component, message, context
- [ ] T064 [P] Add request ID tracking (generate UUID per request, include in all logs) in middleware
- [ ] T065 [P] Update quickstart.md with actual curl examples using real queries (add 5 examples: basic, custom top_k, gpt-4, invalid, concurrent)
- [ ] T066 [P] Verify OpenAPI documentation at `/docs` includes all endpoints with examples
- [ ] T067 [P] Add performance logging (track p50, p95, p99 latencies) in middleware
- [ ] T068 [P] Create smoke test script `backend/smoke_test.py` running 10 test queries and validating responses
- [ ] T069 Run smoke tests and verify all queries return valid responses
- [ ] T070 Update README.md with API overview, setup instructions, and link to `/docs`
- [ ] T071 Manual validation: Run 20 diverse test queries covering ROS 2, Unity, edge cases (documented in quickstart.md)
- [ ] T072 [P] Add environment variable validation on startup (check all required keys exist: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY)
- [ ] T073 Create deployment checklist in `specs/002-rag-agent/deployment.md` (env setup, dependency install, health check, smoke test)

**Acceptance**:
- All errors are logged with context
- Request IDs enable tracing
- Performance metrics captured
- Smoke tests pass
- Documentation complete

---

## Parallel Execution Examples

### Phase 1 (Setup) - Parallel Groups:

**Group A** (can run simultaneously):
- T002: Install FastAPI
- T003: Install OpenAI SDK
- T004: Install tiktoken
- T008: Create logging config

**Group B** (after Group A):
- T001: Create project structure
- T005: Verify retrieval.py
- T006: Update .env
- T007: Create requirements.txt

---

### Phase 2 (Foundational) - Parallel Groups:

**Group A** (all independent, can run simultaneously):
- T009: Create QueryRequest model
- T010: Create QueryResponse model
- T011: Create SourceReference model
- T012: Create ErrorResponse model
- T013: Create custom exceptions

**Group B** (after Group A):
- T014: Create FastAPI app with CORS
- T015: Create health endpoint

---

### Phase 3 (US1) - Sequential (builds on itself):
- T016 → T017 → T018 → T019 → T020 → T021/T022/T023 (tests can run in parallel)

---

### Phase 4 (US2) - Parallel Groups:

**Group A** (can run simultaneously):
- T024: Create RetrievalService class
- T028: Add logging to retrieval

**Group B** (after T024):
- T025: Implement retrieve_context
- T026: Add Qdrant error handling
- T027: Add Cohere error handling

**Group C** (after Groups A and B):
- T029: Integrate into IntegrationService
- T030/T031/T032: Tests (can run in parallel)

---

### Phase 5 (US3) - Parallel Groups:

**Group A** (independent):
- T033: Create PromptBuilder class
- T035: Create token counter utility

**Group B** (after T033):
- T034: Implement XML template
- T036: Implement chunk selection
- T037: Add source metadata
- T038: Add system instructions

**Group C** (after Group B):
- T039: Integrate into IntegrationService
- T040/T041/T042: Tests (in parallel)

---

### Phase 6 (US4) - Parallel Groups:

**Group A**:
- T043: Create AnswerGenerationService
- T048: Add logging

**Group B** (after T043):
- T044: Implement generate_answer
- T045: Configure OpenAI call
- T046: Add retry logic
- T047: Add error handling

**Group C** (after Group B):
- T049: Integrate into IntegrationService
- T050/T051/T052: Tests (in parallel)

---

### Phase 7 (US5) - Parallel Groups:

**Group A**:
- T053: Create SourceDeduplicator

**Group B** (after T053):
- T054: Implement deduplication
- T055: Convert to SourceReferences

**Group C** (after Group B):
- T056: Integrate deduplicator
- T057: Build QueryResponse
- T058: Add response formatting

**Group D** (after Group C):
- T059/T060/T061: Tests (in parallel)

---

### Phase 8 (Polish) - All Parallel:
- T062-T067, T070, T072, T073 can all run in parallel
- T068-T069, T071 must run sequentially (smoke test creation → execution → validation)

---

## Task Completion Tracking

**Total**: 73 tasks
- **Phase 1 (Setup)**: 8 tasks (5 parallelizable)
- **Phase 2 (Foundational)**: 7 tasks (6 parallelizable)
- **Phase 3 (US1)**: 8 tasks (0 parallelizable - sequential integration)
- **Phase 4 (US2)**: 9 tasks (2 parallelizable)
- **Phase 5 (US3)**: 10 tasks (4 parallelizable)
- **Phase 6 (US4)**: 10 tasks (3 parallelizable)
- **Phase 7 (US5)**: 9 tasks (4 parallelizable)
- **Phase 8 (Polish)**: 12 tasks (10 parallelizable)

**Parallelization Summary**: 34 tasks can run in parallel within their phases, 39 must run sequentially.

---

## File Structure Reference

```
backend/
├── rag_agent.py                    # Main FastAPI application
├── retrieval.py                    # Existing (from Spec-2)
├── requirements.txt                # Python dependencies
├── .env                            # Environment variables (gitignored)
├── smoke_test.py                   # Smoke tests
├── models/
│   ├── request_models.py           # QueryRequest
│   └── response_models.py          # QueryResponse, SourceReference, ErrorResponse
├── services/
│   ├── integration_service.py      # Main orchestration
│   ├── retrieval_service.py        # Wraps retrieval.py
│   ├── prompt_builder.py           # Prompt formatting
│   ├── generation_service.py       # OpenAI integration
│   └── source_deduplicator.py      # Source deduplication
├── utils/
│   ├── token_counter.py            # Token counting with tiktoken
│   └── log_formatter.py            # JSON log formatting
├── config/
│   └── logging_config.py           # Logging configuration
└── exceptions.py                   # Custom exceptions

specs/002-rag-agent/
├── spec.md                         # Feature specification
├── plan.md                         # Implementation plan
├── tasks.md                        # This file
├── research.md                     # Technical research
├── data-model.md                   # Data models
├── quickstart.md                   # Developer guide
├── deployment.md                   # Deployment checklist (created in T073)
└── contracts/
    └── openapi.yaml                # API specification
```

---

## Validation Checklist

Before marking feature complete, verify:

- [ ] All 73 tasks completed
- [ ] Health endpoint returns 200
- [ ] POST `/query` endpoint accepts queries
- [ ] Valid queries return 200 with QueryResponse format
- [ ] Invalid queries return 422 with validation errors
- [ ] Retrieval integrates with Qdrant successfully
- [ ] OpenAI integration generates answers
- [ ] Sources are deduplicated and attributed
- [ ] Concurrent requests work without errors
- [ ] Smoke tests pass (10/10 queries successful)
- [ ] Manual validation complete (20 diverse queries)
- [ ] OpenAPI docs accessible at `/docs`
- [ ] All errors return appropriate status codes
- [ ] Structured logging captures all events
- [ ] Token usage logged for cost tracking
- [ ] Environment variables validated on startup
- [ ] Deployment checklist created

---

## Next Steps

1. **Start with MVP**: Implement Phase 1-3 (Setup + Foundational + US1) for basic working API
2. **Incremental Delivery**: Add US2-US5 one at a time, testing after each
3. **Polish**: Complete Phase 8 for production readiness
4. **Validation**: Run smoke tests and manual validation
5. **Documentation**: Update quickstart.md with real examples
6. **Commit**: Create git commit with feature implementation

**Ready to begin**: Start with T001 (Create FastAPI project structure)
