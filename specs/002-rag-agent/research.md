# Research: RAG Agent API Service

**Feature**: 002-rag-agent
**Created**: 2025-12-21
**Purpose**: Document technical research and decisions for RAG Agent implementation

## Research Questions

### Q1: FastAPI vs Flask vs Django for RAG API Service

**Context**: Need to select web framework for the API service

**Research Findings**:

- **FastAPI**:
  - Modern async/await support (better for I/O-bound operations like API calls)
  - Automatic OpenAPI documentation generation
  - Built-in request validation with Pydantic
  - Type hints throughout (better code quality)
  - Fast performance (comparable to Node.js/Go)
  - Growing ecosystem, excellent for ML/AI APIs

- **Flask**:
  - Mature, stable, well-documented
  - Simple for basic APIs
  - Synchronous by default (can add async with extensions)
  - Manual validation needed
  - Larger ecosystem, but older patterns

- **Django**:
  - Full-featured framework (ORM, admin, auth)
  - Overkill for API-only service
  - Heavier, more opinionated
  - Better for traditional web apps

**Decision**: FastAPI

**Rationale**:
- Async support critical for handling concurrent OpenAI API calls
- Automatic request/response validation reduces error handling code
- Built-in OpenAPI docs aid development and testing
- Modern Python patterns (type hints, async/await)
- Lightweight and focused on APIs
- Aligns with user specification

**Alternatives Considered**: Flask with async extensions (more boilerplate), Django REST Framework (too heavy)

---

### Q2: OpenAI SDK Integration Pattern

**Context**: How to structure OpenAI API calls for maintainability and error handling

**Research Findings**:

- **OpenAI SDK v1.0+**:
  - New client architecture with better typing
  - Improved error handling with specific exception types
  - Streaming support (future enhancement)
  - Retry logic built-in
  - Better async support

- **Integration Patterns**:
  - **Pattern A**: Direct calls in route handlers (simple but harder to test)
  - **Pattern B**: Service layer abstraction (better separation, easier to mock/test)
  - **Pattern C**: Dependency injection (most flexible, but adds complexity)

**Decision**: Service layer abstraction (Pattern B)

**Rationale**:
- Separates API logic from business logic
- Easier to unit test (can mock OpenAI calls)
- Centralized error handling for API failures
- Can swap providers if needed (Anthropic, local models)
- Clean architecture without over-engineering

**Implementation Approach**:
```python
# services/generation.py
class AnswerGenerationService:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        self.client = OpenAI(api_key=api_key)
        self.model = model

    async def generate_answer(self, context: str, query: str) -> str:
        # Centralized OpenAI call logic
        ...
```

**Alternatives Considered**: Direct route calls (harder to test), DI framework (overkill)

---

### Q3: Prompt Engineering Strategy

**Context**: How to structure prompts for grounded answer generation

**Research Findings**:

- **System Message vs User Message**:
  - System message: Sets behavior/persona, higher priority in model processing
  - User message: Contains actual query and context
  - Best practice: System for instructions, user for content

- **Context Injection Patterns**:
  - **Pattern A**: Context in system message
  - **Pattern B**: Context in user message with clear delimiters
  - **Pattern C**: Separate context and query as different user messages

**Decision**: Pattern B - Context in user message with clear XML-style delimiters

**Rationale**:
- Clear separation prevents context/query confusion
- XML tags (`<context>`, `<query>`) are well-understood by LLMs
- User message has more token budget than system message
- Easier to debug and inspect full prompts
- Follows OpenAI best practices for RAG

**Example Prompt Structure**:
```
System: "You are a helpful assistant that answers questions based ONLY on provided documentation. If the answer is not in the documentation, say so clearly."

User:
<context>
[Retrieved chunk 1]
Source: [URL 1]

[Retrieved chunk 2]
Source: [URL 2]
</context>

<query>
[User's question]
</query>

Provide an accurate answer based solely on the context above. Cite sources when possible.
```

**Alternatives Considered**: System message context (token limits), multi-turn conversation (more complex)

---

### Q4: Token Management Strategy

**Context**: How to handle context length limits (4k for gpt-3.5-turbo, 8k for gpt-4)

**Research Findings**:

- **GPT-3.5-turbo**: 4k tokens (~3k words), fast, cheap
- **GPT-4**: 8k-128k tokens depending on variant, slower, expensive

- **Token Management Approaches**:
  - **Approach A**: Truncate retrieved chunks to fit limit
  - **Approach B**: Select top-N chunks that fit within limit
  - **Approach C**: Summarize chunks before injection (adds latency)
  - **Approach D**: Use larger context model (gpt-4-turbo-preview)

**Decision**: Approach B - Select top-N chunks that fit within limit

**Rationale**:
- Preserves complete chunk content (no truncation mid-sentence)
- Uses relevance ranking (top chunks are most relevant)
- Predictable token usage for cost estimation
- Can upgrade to gpt-4 for longer context if needed
- Simple to implement with tiktoken library

**Implementation**:
```python
def select_chunks_within_limit(chunks: List[str], max_tokens: int = 3000) -> List[str]:
    """Select chunks that fit within token limit"""
    selected = []
    total_tokens = 0
    for chunk in chunks:
        chunk_tokens = count_tokens(chunk)
        if total_tokens + chunk_tokens <= max_tokens:
            selected.append(chunk)
            total_tokens += chunk_tokens
        else:
            break
    return selected
```

**Alternatives Considered**: Truncation (loses context), summarization (too slow), gpt-4 (unnecessary cost)

---

### Q5: Error Handling and Resilience

**Context**: How to handle failures gracefully (Qdrant down, OpenAI rate limits, etc.)

**Research Findings**:

- **Error Types**:
  - Network errors (timeout, connection refused)
  - API errors (rate limit, authentication, invalid request)
  - Business logic errors (empty results, low relevance)

- **Handling Strategies**:
  - **Strategy A**: Fail fast with HTTP error codes
  - **Strategy B**: Retry with exponential backoff
  - **Strategy C**: Fallback to cached/default responses
  - **Strategy D**: Circuit breaker pattern

**Decision**: Strategy A (fail fast) + Strategy B (retry for transient errors)

**Rationale**:
- Clear error messages help debugging
- Retry makes sense for rate limits and transient network issues
- No caching in v1 (out of scope)
- Circuit breaker adds complexity without clear benefit for low traffic

**Error Response Format**:
```json
{
  "error": true,
  "error_type": "RetrievalError",
  "error_message": "Vector database unavailable",
  "query": "original query",
  "sources": [],
  "answer": null
}
```

**Retry Policy**:
- Retry on: 429 (rate limit), 500/502/503 (server errors), network timeout
- Don't retry on: 400 (bad request), 401 (auth), 404 (not found)
- Max retries: 3
- Backoff: exponential with jitter (1s, 2s, 4s)

**Alternatives Considered**: No retries (poor UX), aggressive retries (wastes resources), caching (out of scope)

---

### Q6: CORS Configuration

**Context**: API needs to be accessible from web frontends

**Research Findings**:

- **CORS in FastAPI**:
  - Built-in `CORSMiddleware` available
  - Can configure allowed origins, methods, headers
  - Development vs Production configurations differ

- **Security Considerations**:
  - Development: Allow all origins (`*`) for flexibility
  - Production: Restrict to specific domains
  - Consider credentials, headers, methods

**Decision**: Permissive CORS for development, configurable for production

**Implementation**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Development only
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Rationale**:
- Development needs flexibility (localhost, different ports)
- Production config can be tightened via environment variables
- FastAPI middleware is well-tested and performant
- Simple to configure and maintain

**Alternatives Considered**: Proxy setup (more complex), no CORS (breaks web clients)

---

### Q7: Logging and Observability

**Context**: Need to monitor queries, responses, and performance

**Research Findings**:

- **Logging Libraries**:
  - Standard library `logging`: Simple, built-in, structured possible
  - `structlog`: Better structured logging, JSON output
  - `loguru`: Easier API, auto-formatting
  - Third-party services: DataDog, Sentry (overkill for v1)

**Decision**: Standard library `logging` with structured format

**Rationale**:
- No external dependencies
- Sufficient for development and testing
- JSON format possible with custom formatter
- Easy to upgrade to structlog later if needed
- Aligns with existing retrieval.py logging

**Log Structure**:
```python
{
  "timestamp": "2025-12-21T10:30:00Z",
  "level": "INFO",
  "component": "rag_agent",
  "query": "How do I create a ROS 2 publisher?",
  "top_k": 5,
  "retrieved_count": 5,
  "model": "gpt-3.5-turbo",
  "tokens_used": 450,
  "latency_ms": 1250
}
```

**Alternatives Considered**: Structlog (extra dependency), no logging (poor observability), external service (premature)

---

### Q8: Source Deduplication Strategy

**Context**: Multiple chunks may come from same documentation page

**Research Findings**:

- **Deduplication Approaches**:
  - **Approach A**: Deduplicate by URL (simple, loses chunk details)
  - **Approach B**: Deduplicate by URL but keep highest-scoring chunk
  - **Approach C**: Keep all chunks but group by URL in response
  - **Approach D**: No deduplication (show all chunks)

**Decision**: Approach B - Deduplicate by URL, keep highest-scoring chunk

**Rationale**:
- Prevents redundant source attribution in response
  - Preserves most relevant chunk from each page
- Simple to implement (dict keyed by URL)
- User sees diverse sources rather than repetitive ones
- Aligns with FR-008 in spec

**Implementation**:
```python
def deduplicate_sources(results: List[SearchResult]) -> List[SearchResult]:
    """Keep only highest-scoring chunk per URL"""
    seen_urls = {}
    for result in results:
        url = result.metadata['url']
        if url not in seen_urls or result.score > seen_urls[url].score:
            seen_urls[url] = result
    return list(seen_urls.values())
```

**Alternatives Considered**: No dedup (redundant sources), group by URL (more complex response)

---

## Research Summary

All technical decisions have been researched and documented. Key technology choices:

1. **Framework**: FastAPI (async, validation, docs)
2. **LLM Integration**: OpenAI SDK v1.0+ with service layer
3. **Prompt Strategy**: XML-delimited context in user message
4. **Token Management**: Select top-N chunks within limit
5. **Error Handling**: Fail fast + retry for transient errors
6. **CORS**: Permissive for development
7. **Logging**: Standard library with structured format
8. **Deduplication**: Keep highest-scoring chunk per URL

No outstanding NEEDS CLARIFICATION items. Ready to proceed to Phase 1 (Design & Contracts).
