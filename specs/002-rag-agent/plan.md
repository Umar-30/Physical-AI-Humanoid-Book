# Implementation Plan: RAG Agent API Service

**Feature**: 002-rag-agent
**Created**: 2025-12-21
**Status**: Draft

## 1. Scope and Dependencies

### In Scope

- **FastAPI REST API**: HTTP POST endpoint for query submission with JSON request/response
- **Request Validation**: Pydantic models for query validation (1-2000 chars, top_k 1-50, valid model names)
- **Retrieval Integration**: Reuse existing retrieval.py from Spec-2 for Qdrant queries
- **Prompt Engineering**: Structured prompt template with XML-delimited context and query
- **OpenAI Integration**: Chat Completion API calls with gpt-3.5-turbo/gpt-4 support
- **Token Management**: Context truncation using tiktoken to stay within model limits
- **Source Deduplication**: Consolidate multiple chunks from same URL, keep highest-scoring
- **Error Handling**: Graceful handling of validation errors, retrieval failures, OpenAI API errors
- **CORS Support**: Cross-origin middleware for web client integration
- **Structured Logging**: JSON-formatted logs for queries, retrievals, generations, and errors
- **Health Endpoint**: Basic service status check

### Out of Scope

- User authentication and authorization (API is open)
- Rate limiting or quota management
- Response caching or memoization
- Streaming responses (standard request/response only)
- Multi-turn conversations or chat history
- Frontend UI (API only)
- Model fine-tuning or custom training
- Non-English language support
- Advanced techniques (chain-of-thought, few-shot prompting)
- Deployment configuration (Docker, Kubernetes)
- Monitoring dashboards (Grafana, DataDog)
- A/B testing infrastructure

### External Dependencies

| Dependency | Type | Purpose | Ownership | SLA/Availability |
|------------|------|---------|-----------|------------------|
| OpenAI API (Chat Completions) | External Service | Answer generation | OpenAI | 99.9% (per OpenAI SLA) |
| Qdrant Cloud | External Service | Vector search | Qdrant | 99.9% (configured in Spec-1) |
| Cohere API (embed-english-v3.0) | External Service | Query embedding | Cohere | 99.9% (used in Spec-2) |
| retrieval.py module | Internal | Retrieval functions | Spec-2 (001-retrieval-pipeline-testing) | N/A |
| Python 3.9+ | Runtime | Execution environment | System | N/A |

## 2. Key Decisions and Rationale

### Decision 1: FastAPI Framework

**Options Considered**:
- A) FastAPI (async, built-in validation, auto docs)
- B) Flask (mature, synchronous, manual validation)
- C) Django REST Framework (full-featured, heavy)

**Trade-offs**:
- Option A: Modern async support, automatic OpenAPI docs, Pydantic validation, but newer ecosystem
- Option B: Mature and stable, but synchronous (poor for I/O-bound tasks), manual validation needed
- Option C: Comprehensive features (auth, ORM, admin), but overkill for API-only service

**Decision**: Option A - FastAPI

**Rationale**:
- Async/await critical for concurrent OpenAI API calls (I/O-bound operations)
- Automatic request validation with Pydantic reduces boilerplate
- Built-in OpenAPI documentation aids development and testing
- Modern Python patterns (type hints) improve code quality
- Aligns with user specification and industry best practices for ML APIs

**Reversibility**: Medium - switching frameworks requires rewrite, but business logic can be preserved

**Measurement**: Development velocity (faster endpoint creation), automatic API docs available at `/docs`

---

### Decision 2: Service Layer Architecture

**Options Considered**:
- A) Direct OpenAI calls in route handlers
- B) Service layer abstraction (AnswerGenerationService class)
- C) Dependency injection framework (e.g., python-dependency-injector)

**Trade-offs**:
- Option A: Simple and direct, but hard to test (can't mock OpenAI), tightly coupled
- Option B: Clean separation, easier to test, can swap providers, balanced complexity
- Option C: Most flexible and testable, but adds framework dependency and learning curve

**Decision**: Option B - Service layer abstraction

**Rationale**:
- Separates API logic from business logic (single responsibility principle)
- Easy to unit test with mocked OpenAI client
- Centralized error handling for API failures
- Can swap LLM providers (Anthropic, local models) if needed
- Avoids over-engineering while maintaining clean architecture

**Reversibility**: High - service layer can be inlined or extracted to DI framework

**Measurement**: Test coverage >80% for service layer, < 200 lines per service class

---

### Decision 3: Prompt Structure with XML Delimiters

**Options Considered**:
- A) Context in system message, query in user message
- B) Both context and query in user message with XML tags
- C) Multi-turn conversation (context as previous message)

**Trade-offs**:
- Option A: Follows role separation, but system message has token limits, harder to debug
- Option B: Clear separation with XML, user message has more token budget, easier to inspect
- Option C: More flexible, but complex state management, harder to reason about

**Decision**: Option B - XML-delimited context in user message

**Rationale**:
- XML tags (`<context>`, `<query>`) are well-understood by OpenAI models
- User message has larger token budget than system message
- Clear visual separation aids debugging and prompt inspection
- Aligns with OpenAI best practices for RAG applications
- Simple to implement and maintain

**Reversibility**: High - prompt template is configurable, easy to A/B test

**Measurement**: Answer quality validated through manual review of 20 test queries

**Example Prompt**:
```
System: "You are a helpful assistant that answers questions based ONLY on provided documentation."

User:
<context>
[Chunk 1]
Source: [URL]

[Chunk 2]
Source: [URL]
</context>

<query>
How do I create a ROS 2 publisher?
</query>

Answer based solely on the context above.
```

---

### Decision 4: Token Management Strategy

**Options Considered**:
- A) Truncate chunks mid-content to fit limit
- B) Select top-N complete chunks that fit within limit
- C) Summarize chunks before injection (adds LLM call)
- D) Always use gpt-4-turbo (128k context)

**Trade-offs**:
- Option A: Maximizes content, but breaks sentences, confuses model
- Option B: Preserves complete chunks, uses relevance ranking, predictable
- Option C: Maximizes information density, but adds latency and cost
- Option D: No limits needed, but 10x cost, slower responses

**Decision**: Option B - Select top-N complete chunks within limit

**Rationale**:
- Preserves semantic integrity (no mid-sentence cuts)
- Uses existing relevance ranking (top chunks are most relevant)
- Predictable token usage for cost estimation
- Can upgrade to gpt-4 for specific queries if needed
- Simple implementation with tiktoken library

**Reversibility**: High - can switch to summarization or larger model later

**Measurement**: 95% of queries fit within 3000-token context, <5% require truncation

**Implementation**:
```python
def select_chunks_within_limit(chunks: List[str], max_tokens: int = 3000) -> List[str]:
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

---

### Decision 5: Source Deduplication by URL

**Options Considered**:
- A) No deduplication (show all chunks)
- B) Deduplicate by URL, keep highest-scoring chunk
- C) Group chunks by URL in response (show all but grouped)

**Trade-offs**:
- Option A: Most transparent, but redundant sources confuse users
- Option B: Clean source list, shows diverse pages, but loses chunk details
- Option C: Shows all chunks, but complex response structure

**Decision**: Option B - Deduplicate by URL, keep highest-scoring

**Rationale**:
- Prevents redundant source attribution (same URL multiple times)
- Preserves most relevant chunk from each page
- Simple implementation (dict keyed by URL)
- Aligns with FR-008 requirement
- Users see diverse sources rather than repetitive ones

**Reversibility**: High - can add "chunk_count" field to show how many chunks per URL

**Measurement**: Average 3-5 unique sources per response (vs 5-10 before dedup)

---

### Decision 6: Error Handling Strategy

**Options Considered**:
- A) Fail fast with HTTP error codes, no retries
- B) Retry with exponential backoff for transient errors
- C) Circuit breaker pattern for dependency failures
- D) Fallback to cached responses

**Trade-offs**:
- Option A: Simple and predictable, but poor UX for transient errors
- Option B: Better UX, handles rate limits well, but adds complexity
- Option C: Prevents cascading failures, but overkill for low traffic
- Option D: Best UX, but caching is out of scope for v1

**Decision**: Option A (fail fast) + Option B (retry for transient errors only)

**Rationale**:
- Clear error messages aid debugging and user understanding
- Retry makes sense for 429 (rate limit) and 5xx (server errors)
- No retries for 400-level errors (client's fault)
- Circuit breaker is premature for expected low traffic
- Aligns with FR-009 requirement

**Retry Policy**:
- Retry on: 429 (rate limit), 500/502/503 (server errors), network timeout
- Don't retry on: 400 (bad request), 401 (auth), 422 (validation)
- Max retries: 3
- Backoff: exponential with jitter (1s, 2s, 4s)

**Reversibility**: High - retry logic is isolated in service layer

**Measurement**: <1% of requests fail after retries, 95% recover on first retry

---

### Decision 7: CORS Configuration

**Options Considered**:
- A) No CORS (API only accessible from same origin)
- B) Permissive CORS for development (`allow_origins=["*"]`)
- C) Strict CORS with whitelisted domains

**Trade-offs**:
- Option A: Most secure, but breaks web clients
- Option B: Flexible for development, but not production-ready
- Option C: Production-ready, but requires configuration management

**Decision**: Option B for v1 (development), configurable for future production

**Rationale**:
- Development needs flexibility (localhost, different ports)
- Production config can be tightened via environment variables
- FastAPI middleware is well-tested and performant
- Aligns with out-of-scope deployment configuration

**Reversibility**: High - middleware configuration is one-line change

**Measurement**: Web clients can successfully call API from any origin in dev

---

## 3. Interfaces and API Contracts

### 3.1 REST API Endpoint

**POST /query**

Submit a natural language question and receive a grounded answer with sources.

**Request**:
```json
{
  "query": "How do I create a ROS 2 publisher in Python?",
  "top_k": 5,
  "model": "gpt-3.5-turbo"
}
```

**Response (200 OK)**:
```json
{
  "query": "How do I create a ROS 2 publisher in Python?",
  "answer": "To create a ROS 2 publisher...",
  "sources": [
    {
      "url": "https://docs.ros.org/...",
      "page_title": "Writing a simple publisher",
      "relevance_score": 0.8642,
      "chunk_index": 2
    }
  ],
  "model_used": "gpt-3.5-turbo",
  "tokens_used": 456,
  "retrieval_count": 5
}
```

**Error Response (422 Validation Error)**:
```json
{
  "error": true,
  "error_type": "ValidationError",
  "error_message": "Query cannot be empty or whitespace",
  "query": "",
  "status_code": 422
}
```

**Full OpenAPI Specification**: See `contracts/openapi.yaml`

---

### 3.2 Internal Service Contracts

#### AnswerGenerationService

```python
class AnswerGenerationService:
    """Service for generating answers using OpenAI Chat Completions"""

    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        """
        Initialize OpenAI client.

        Args:
            api_key: OpenAI API key from environment
            model: Default model to use
        """

    async def generate_answer(
        self,
        context: str,
        query: str,
        model: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate answer from context and query.

        Args:
            context: Retrieved documentation chunks (formatted)
            query: User's question
            model: Optional override for default model

        Returns:
            Dict with 'answer' (str) and 'tokens_used' (int)

        Raises:
            GenerationError: If OpenAI API call fails
            RateLimitError: If rate limit exceeded
        """
```

#### PromptBuilder

```python
class PromptBuilder:
    """Builds prompts from retrieved results and query"""

    @staticmethod
    def build_prompt(
        results: List[SearchResult],
        query: str,
        max_tokens: int = 3000
    ) -> str:
        """
        Build structured prompt with context and query.

        Args:
            results: Retrieved search results from Qdrant
            query: User's question
            max_tokens: Maximum tokens for context (default 3000)

        Returns:
            Formatted prompt string ready for LLM

        Example:
            <context>
            [Chunk 1]
            Source: [URL 1]
            ...
            </context>

            <query>
            [User question]
            </query>
        """
```

#### SourceDeduplicator

```python
class SourceDeduplicator:
    """Deduplicates search results by URL"""

    @staticmethod
    def deduplicate(results: List[SearchResult]) -> List[SourceReference]:
        """
        Deduplicate results by URL, keeping highest-scoring chunk.

        Args:
            results: List of search results from retrieval

        Returns:
            List of unique source references

        Logic:
            - Group results by metadata['url']
            - For each URL, keep result with highest score
            - Convert to SourceReference format
        """
```

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### 4.1 Performance

**Latency Targets**:
- **p50**: < 2 seconds (median response time)
- **p95**: < 5 seconds (95th percentile)
- **p99**: < 10 seconds (99th percentile, includes retries)

**Throughput**:
- **Sustained**: 10 concurrent requests without degradation
- **Burst**: 20 concurrent requests with graceful handling

**Resource Caps**:
- **Memory**: < 512 MB per worker process
- **CPU**: < 1 core per worker (I/O-bound, not CPU-bound)

**Measurement**:
```python
# Log latency for each request
logger.info({
    "query": query,
    "latency_ms": end_time - start_time,
    "retrieval_ms": retrieval_time,
    "generation_ms": generation_time,
    "total_ms": total_time
})
```

---

### 4.2 Reliability

**SLOs (Service Level Objectives)**:
- **Availability**: 99% uptime during development (allows for maintenance)
- **Success Rate**: 95% of requests return 200 OK (excludes validation errors)
- **Error Budget**: 5% of requests can fail (due to external API issues)

**Error Budget**:
- **Monthly**: 7.2 hours of downtime allowed
- **Daily**: ~15 minutes of downtime allowed

**Degradation Strategy**:
- If Qdrant unavailable: Return 503 error (cannot proceed without retrieval)
- If OpenAI unavailable: Retry 3 times, then return 502 error
- If Cohere unavailable: Return 503 error (cannot embed query)

**Measurement**:
- Track 200 vs 5xx response ratio in logs
- Alert if error rate >10% over 5-minute window (future)

---

### 4.3 Security

**Authentication/Authorization**:
- **v1**: None (API is open for development)
- **Future**: API key authentication via header

**Data Handling**:
- **PII**: No personal data stored (queries are not persisted)
- **Secrets**: API keys stored in `.env`, never logged or exposed
- **Input Validation**: All requests validated with Pydantic (prevents injection)

**Secrets Management**:
- Environment variables for all API keys
- `.env` file added to `.gitignore`
- No hardcoded credentials in source code

**Auditing**:
- All queries logged with timestamp (no PII)
- Token usage logged for cost tracking
- Error types logged for debugging

**Measurement**:
- No API keys in logs (automated scan)
- All requests pass Pydantic validation

---

### 4.4 Cost

**Token Costs** (OpenAI pricing as of Dec 2024):
- **GPT-3.5-turbo**: $0.0005 per 1k tokens (input), $0.0015 per 1k tokens (output)
- **GPT-4**: $0.03 per 1k tokens (input), $0.06 per 1k tokens (output)

**Estimated Cost Per Query**:
- **Context**: ~3000 tokens (retrieved chunks)
- **Query**: ~50 tokens (user question)
- **Completion**: ~400 tokens (generated answer)
- **Total**: ~3450 tokens per query

**Cost Breakdown**:
- **GPT-3.5-turbo**: ~$0.001-0.003 per query
- **GPT-4**: ~$0.10-0.20 per query

**Budget**:
- **Development**: $100/month (33,000-100,000 queries with gpt-3.5-turbo)
- **Testing**: $50/month (automated test suite)

**Measurement**:
```python
# Log token usage for cost tracking
logger.info({
    "model": model_used,
    "tokens_used": tokens_used,
    "estimated_cost_usd": calculate_cost(model, tokens_used)
})
```

---

## 5. Data Management and Migration

### 5.1 Source of Truth

**Configuration**:
- **Source**: `.env` file (environment variables)
- **Schema**: Key-value pairs (OPENAI_API_KEY, QDRANT_URL, etc.)
- **Migration**: None (stateless API)

**Retrieved Documentation**:
- **Source**: Qdrant vector database (managed by Spec-1)
- **Schema**: Defined in Spec-1 (004-website-embedding)
- **Migration**: Not applicable (read-only access)

**Request/Response**:
- **Source**: In-memory (ephemeral)
- **Persistence**: None (v1 does not store queries or responses)

---

### 5.2 Schema Evolution

**API Schema Changes**:
- **Versioning**: Not implemented in v1 (breaking changes require client updates)
- **Future**: Use `/v1/query`, `/v2/query` for versioned endpoints

**Pydantic Model Changes**:
- **Backward-compatible**: Add optional fields (won't break existing clients)
- **Breaking changes**: Require API version bump

---

### 5.3 Data Retention

**Logs**:
- **Retention**: 7 days (development)
- **Rotation**: Daily log files (rag_agent.log.YYYY-MM-DD)
- **Cleanup**: Manual deletion (automated in production)

**Metrics**:
- **Retention**: In-memory only (v1)
- **Future**: Export to Prometheus/DataDog with 30-day retention

---

## 6. Operational Readiness

### 6.1 Observability

**Logs**:
- **Format**: Structured JSON logs
- **Fields**: timestamp, level, component, query, latency_ms, tokens_used, error (if any)
- **Example**:
  ```json
  {
    "timestamp": "2025-12-21T10:30:00Z",
    "level": "INFO",
    "component": "rag_agent",
    "query": "How do I create a publisher?",
    "retrieval_count": 5,
    "model": "gpt-3.5-turbo",
    "tokens_used": 456,
    "latency_ms": 1250
  }
  ```

**Metrics**:
- **Request count**: Total requests (200, 422, 5xx)
- **Latency**: p50, p95, p99 response times
- **Token usage**: Total tokens consumed per hour/day
- **Error rate**: Percentage of 5xx errors

**Traces**:
- **v1**: Not implemented (logs sufficient)
- **Future**: OpenTelemetry for distributed tracing

---

### 6.2 Alerting

**v1**: No automated alerting (manual log review)

**Future Alerts** (once monitoring added):
- **Critical**: Error rate >20% for 5 minutes → Page on-call
- **Warning**: Error rate >10% for 10 minutes → Email notification
- **Info**: Latency p95 >7s for 15 minutes → Slack notification

---

### 6.3 Runbooks

**Common Tasks**:

1. **Restart API Server**:
   ```bash
   # Stop server
   pkill -f "uvicorn rag_agent:app"

   # Start server
   cd backend
   uvicorn rag_agent:app --reload --host 0.0.0.0 --port 8000
   ```

2. **Check Service Health**:
   ```bash
   curl http://localhost:8000/health
   # Expected: {"status": "healthy", "version": "1.0.0"}
   ```

3. **View Logs**:
   ```bash
   tail -f backend/rag_agent.log
   # Or for JSON parsing:
   tail -f backend/rag_agent.log | jq .
   ```

4. **Test Query**:
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"query": "How do I create a ROS 2 publisher?"}'
   ```

---

### 6.4 Deployment and Rollback

**v1 Deployment** (Development):
```bash
# 1. Install dependencies
cd backend
pip install -r requirements.txt

# 2. Configure environment
cp .env.example .env
# Edit .env with API keys

# 3. Start server
uvicorn rag_agent:app --reload --host 0.0.0.0 --port 8000
```

**Rollback**:
```bash
# Git rollback
git checkout previous-commit-hash
pip install -r requirements.txt
uvicorn rag_agent:app --reload
```

**Future** (Production):
- Blue-green deployment with Docker containers
- Health checks before traffic cutover
- Automated rollback on health check failure

---

### 6.5 Feature Flags

**v1**: Not implemented

**Future**:
- `ENABLE_GPT4`: Boolean to enable/disable GPT-4 model option
- `MAX_CONCURRENT_REQUESTS`: Integer to limit throughput
- `ENABLE_CACHING`: Boolean to enable response caching

---

## 7. Risk Analysis and Mitigation

### Risk 1: OpenAI API Rate Limits

**Probability**: Medium (depends on traffic volume)
**Impact**: High (queries fail, poor user experience)
**Blast Radius**: All users affected simultaneously

**Mitigation**:
- Retry with exponential backoff (handles temporary limits)
- Return clear error message: "Rate limit exceeded, try again in 60s"
- Monitor token usage to stay within tier limits

**Kill Switch**: Environment variable `DISABLE_GENERATION=true` to disable OpenAI calls (return mock responses)

---

### Risk 2: Qdrant Database Unavailable

**Probability**: Low (Qdrant has 99.9% SLA)
**Impact**: Critical (cannot retrieve context, all queries fail)
**Blast Radius**: All users, complete service outage

**Mitigation**:
- Health check endpoint to detect Qdrant availability
- Clear error message: "Database unavailable, try again later"
- No retry (retrieval is synchronous, retry at API level)

**Kill Switch**: None (service cannot function without retrieval)

---

### Risk 3: Token Limit Exceeded (Context Too Long)

**Probability**: Medium (verbose documentation chunks)
**Impact**: Low (single query fails, others unaffected)
**Blast Radius**: Individual query only

**Mitigation**:
- Pre-calculate tokens before sending to OpenAI
- Select top-N chunks that fit within limit (3000 tokens)
- Fallback to fewer chunks if needed

**Guardrails**: Hard limit on context tokens (3000 for gpt-3.5-turbo, 7000 for gpt-4)

---

## 8. Constitution Check

### 8.1 Pre-Design Gate

**Technical Accuracy** (NON-NEGOTIABLE):
- ✅ All API contracts validated with OpenAPI spec
- ✅ Data models defined with Pydantic for runtime validation
- ✅ Error handling covers all failure modes (validation, retrieval, generation)
- ✅ Token management prevents context overflow

**Reproducibility** (NON-NEGOTIABLE):
- ✅ All dependencies listed in requirements.txt
- ✅ Environment variables documented in quickstart.md
- ✅ API can be started with documented commands
- ✅ Test queries provided in quickstart.md

**Clarity for Learners**:
- ✅ Quickstart guide provides step-by-step setup
- ✅ API documentation auto-generated by FastAPI
- ✅ Examples provided for cURL, Python, JavaScript
- ✅ Troubleshooting section covers common errors

**Consistency with Standards**:
- ✅ REST API follows standard HTTP methods and status codes
- ✅ OpenAPI 3.0 specification for contracts
- ✅ Pydantic for data validation (industry standard)
- ✅ Structured logging with JSON format

**Practicality**:
- ✅ Reuses existing retrieval.py module (DRY principle)
- ✅ All configuration via environment variables (.env)
- ✅ Minimal dependencies (no unnecessary frameworks)
- ✅ Clear error messages for debugging

**Verification Before Inclusion**:
- ✅ OpenAPI spec validates correctly
- ✅ Pydantic models have examples
- ✅ All edge cases documented (empty query, no results, errors)
- ✅ Performance targets defined and measurable

**GATE**: ✅ PASSED - Proceed to implementation (tasks.md)

---

### 8.2 Post-Design Review

**Design Principles**:
- ✅ Smallest viable change: Reuses retrieval.py, no refactoring
- ✅ Reversible decisions: Service layer, prompt template, token management
- ✅ Measurable outcomes: Latency targets, error rates, token usage

**Risk Mitigation**:
- ✅ Top 3 risks identified with blast radius and mitigation
- ✅ Kill switches defined (DISABLE_GENERATION flag)
- ✅ Retry logic for transient failures

**Operational Readiness**:
- ✅ Logging strategy defined (structured JSON)
- ✅ Runbooks for common tasks (restart, health check, view logs)
- ✅ Deployment steps documented in quickstart.md

**GATE**: ✅ PASSED - Ready for task breakdown

---

## 9. Evaluation and Validation

### 9.1 Definition of Done

A query is successfully processed when:
- ✅ Request passes Pydantic validation
- ✅ Retrieval from Qdrant succeeds (or returns empty results)
- ✅ Context is built within token limit
- ✅ OpenAI generates answer (or returns error)
- ✅ Response matches QueryResponse schema
- ✅ Sources are deduplicated by URL
- ✅ Latency is logged for monitoring

### 9.2 Testing Strategy

**Unit Tests**:
- Pydantic model validation (valid/invalid inputs)
- PromptBuilder (context formatting, token counting)
- SourceDeduplicator (URL grouping, score ranking)
- AnswerGenerationService (mocked OpenAI calls)

**Integration Tests**:
- End-to-end query flow (retrieval → generation → response)
- Error handling (Qdrant down, OpenAI rate limit)
- Token limit edge cases

**Manual Validation** (20 Test Queries):
- ROS 2 basics (publisher, subscriber, nodes)
- Unity integration (Digital Twin module)
- Edge cases (no results, conflicting info, long query)

### 9.3 Output Validation

**Format**:
- ✅ All responses are valid JSON
- ✅ Responses match OpenAPI schema
- ✅ Pydantic models enforce types and constraints

**Requirements**:
- ✅ FR-001 to FR-012 mapped to implementation components
- ✅ All edge cases handled (empty query, no results, errors)

**Safety**:
- ✅ No API keys in logs or responses
- ✅ Input validation prevents injection attacks
- ✅ Rate limiting handled gracefully (429 errors)

---

## 10. Architecture Decision Records (ADR)

Significant architectural decisions from this plan:

1. **ADR-001**: FastAPI Framework Selection
   - **Decision**: Use FastAPI for REST API implementation
   - **Context**: Need modern async framework with auto-validation and docs
   - **Consequences**: Faster development, better performance, auto OpenAPI docs

2. **ADR-002**: Service Layer Architecture Pattern
   - **Decision**: Separate business logic into service classes
   - **Context**: Need testable, maintainable code with mocked dependencies
   - **Consequences**: Easier testing, can swap LLM providers, cleaner code

3. **ADR-003**: XML-Delimited Prompt Structure
   - **Decision**: Use `<context>` and `<query>` tags in user message
   - **Context**: Need clear separation of retrieved docs and user question
   - **Consequences**: Better prompt clarity, easier debugging, aligns with best practices

4. **ADR-004**: Token Management via Chunk Selection
   - **Decision**: Select top-N complete chunks that fit within token limit
   - **Context**: gpt-3.5-turbo has 4k token limit
   - **Consequences**: Preserves semantic integrity, predictable costs, may lose lower-ranked context

**Recommendation**: Create ADRs for decisions 1-4 using `/sp.adr` command after this plan is approved.

---

## 11. Next Steps

1. **Review and Approve Plan**: Validate all design decisions and constraints
2. **Generate ADRs**: Document key architectural decisions
3. **Create tasks.md**: Break down implementation into testable tasks
4. **Implement Core Service**: Build FastAPI app, service layer, prompt builder
5. **Integration Testing**: Test with real Qdrant and OpenAI APIs
6. **Manual Validation**: Run 20 test queries for quality assurance
7. **Documentation**: Update quickstart.md with actual usage examples

---

**Plan Status**: Ready for task generation (`/sp.tasks`)
