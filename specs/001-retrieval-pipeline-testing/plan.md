# Implementation Plan: Retrieval Pipeline Testing for RAG Ingestion

**Feature**: 001-retrieval-pipeline-testing
**Created**: 2025-12-20
**Status**: Draft

## 1. Scope and Dependencies

### In Scope

- **Query Embedding Generation**: Generate vector embeddings for user queries using Cohere API
- **Vector Search Implementation**: Query Qdrant collection to retrieve top-K most similar chunks
- **Metadata Validation**: Verify all metadata fields (url, page_title, chunk_index, total_chunks) are returned correctly
- **Text Integrity Verification**: Confirm retrieved text matches original embedded content
- **JSON Response Formatting**: Structure results as clean, parseable JSON with consistent schema
- **Automated Test Suite**: Create comprehensive tests covering diverse query types and edge cases
- **Error Handling**: Graceful handling of empty queries, API failures, and no-match scenarios
- **Performance Logging**: Track query-to-response latency for performance validation

### Out of Scope

- Query rewriting, expansion, or spelling correction
- Result caching or optimization beyond basic implementation
- Web API or user interface (CLI/script only)
- Multi-language support (English only)
- Hybrid search (vector + keyword)
- Production monitoring or alerting
- Authentication/authorization

### External Dependencies

| Dependency | Type | Purpose | Ownership |
|------------|------|---------|-----------|
| Cohere API (embed-english-v3.0) | External Service | Generate query embeddings | Cohere |
| Qdrant Cloud | External Service | Vector storage and search | Qdrant |
| 004-website-embedding | Upstream Feature | Provides embedded vectors | Internal |
| Python 3.9+ | Runtime | Execution environment | System |
| cohere, qdrant-client libraries | Package | API clients | PyPI |

## 2. Key Decisions and Rationale

### Decision 1: Use Same Embedding Model for Queries and Documents

**Options Considered**:
- A) Use same model (embed-english-v3.0) for both query and document embedding
- B) Use different model optimized for queries (e.g., embed-english-light-v3.0)
- C) Use multiple models and ensemble results

**Trade-offs**:
- Option A: Ensures vector space compatibility, simpler pipeline, but may not be optimal for short queries
- Option B: Potentially better query performance, but vectors live in different spaces (incompatible)
- Option C: Best accuracy, but complex, slow, and expensive

**Decision**: Option A - Use embed-english-v3.0 for both

**Rationale**:
- Vector compatibility is critical - query and document vectors must live in same space for cosine similarity to be meaningful
- Cohere embed-english-v3.0 handles both documents and queries well with input_type parameter
- Simplicity reduces potential errors and maintenance burden
- Performance is acceptable (<2s target easily achievable)

**Reversibility**: Low - changing embedding model requires re-embedding all documents

**Measurement**: Query relevance validated by similarity scores >0.7 for known relevant content

---

### Decision 2: Return Top-K Results Without Post-Processing

**Options Considered**:
- A) Return raw top-K results from Qdrant directly
- B) Apply similarity threshold filter (e.g., only return results >0.7)
- C) Re-rank results using LLM or additional model

**Trade-offs**:
- Option A: Simple, fast, deterministic, but may include low-relevance results
- Option B: Higher quality results, but may return fewer than K results
- Option C: Best relevance, but adds latency and cost

**Decision**: Option A - Return raw top-K results

**Rationale**:
- Testing phase should validate retrieval quality, not mask issues with filtering
- Caller can apply their own threshold based on use case
- Deterministic behavior aids debugging and validation
- Performance is optimal without additional processing

**Reversibility**: High - filtering can be added later as separate function

**Measurement**: Test suite validates that relevant results appear in top-5 positions

---

### Decision 3: JSON Response Schema with Flat Structure

**Options Considered**:
- A) Flat structure: `{query, top_k, results: [{score, text, metadata}]}`
- B) Nested structure: `{query_info: {}, search_config: {}, results: {items: [], metadata: {}}}`
- C) Paginated structure: `{page, total_pages, results: []}`

**Trade-offs**:
- Option A: Simple to parse, minimal nesting, easy to validate
- Option B: More organized for complex responses, but harder to parse
- Option C: Supports large result sets, but adds complexity for testing

**Decision**: Option A - Flat structure

**Rationale**:
- Testing use case requires simplicity over scalability
- top_k is capped at 50, no pagination needed
- Easier to validate in automated tests
- Consistent with common API patterns

**Reversibility**: Medium - schema changes may break existing consumers

**Measurement**: JSON validates against schema in 100% of tests

---

### Decision 4: Command-Line Script vs Python Module

**Options Considered**:
- A) Single command-line script (test_retrieval.py)
- B) Reusable Python module with CLI wrapper
- C) Jupyter notebook for interactive testing

**Trade-offs**:
- Option A: Quick to implement, easy to run, but less reusable
- Option B: Better architecture, reusable for future RAG features, but more complex
- Option C: Interactive and visual, but harder to automate

**Decision**: Option B - Reusable Python module with CLI

**Rationale**:
- Module can be imported by future RAG chatbot implementation
- Separates concerns: retrieval logic vs CLI interface
- Easier to unit test individual functions
- Professional architecture for production evolution

**Reversibility**: High - can always create standalone script from module

**Measurement**: Module functions are independently testable and importable

## 3. Interfaces and API Contracts

### 3.1 Query Embedding Function

```python
def embed_query(query_text: str) -> List[float]:
    """
    Generate embedding vector for query text.

    Args:
        query_text: Natural language query string

    Returns:
        1024-dimensional embedding vector

    Raises:
        ValueError: If query_text is empty or None
        CohereAPIError: If API call fails
    """
```

**Contract**:
- Input: Non-empty string (1-2000 characters)
- Output: List of 1024 floats
- Idempotent: Same query always produces same embedding
- Timeout: 10 seconds max
- Errors: Raises specific exception types with descriptive messages

---

### 3.2 Vector Search Function

```python
def search_qdrant(
    query_embedding: List[float],
    top_k: int = 5,
    collection_name: str = "website_embeddings"
) -> List[SearchResult]:
    """
    Search Qdrant for similar chunks.

    Args:
        query_embedding: 1024-dim vector from embed_query()
        top_k: Number of results to return (default 5, max 50)
        collection_name: Qdrant collection to search

    Returns:
        List of SearchResult objects ranked by similarity

    Raises:
        ValueError: If top_k < 1 or > 50
        QdrantAPIError: If search fails
    """
```

**Contract**:
- Input: Valid 1024-dim vector, top_k in [1, 50]
- Output: List of exactly top_k results (or fewer if collection smaller)
- Ordering: Results sorted by descending similarity score
- Timeout: 5 seconds max
- Errors: Raises specific exceptions, never returns None

---

### 3.3 JSON Response Formatter

```python
def format_results(
    query: str,
    results: List[SearchResult],
    top_k: int
) -> dict:
    """
    Format search results as JSON-serializable dict.

    Args:
        query: Original query text
        results: List of search results
        top_k: Requested number of results

    Returns:
        Dict with schema: {
            "query": str,
            "top_k": int,
            "result_count": int,
            "results": [
                {
                    "rank": int,
                    "score": float,
                    "text": str,
                    "metadata": {
                        "url": str,
                        "page_title": str,
                        "chunk_index": int,
                        "total_chunks": int,
                        "source": str
                    }
                }
            ]
        }
    """
```

**Contract**:
- Input: Valid query string and results list
- Output: JSON-serializable dict conforming to schema
- Deterministic: Same inputs always produce identical output
- Validation: All required fields present, types correct

---

### 3.4 CLI Interface

```bash
# Basic query
python test_retrieval.py --query "What is ROS 2?"

# Custom top_k
python test_retrieval.py --query "Unity integration" --top-k 10

# Batch testing
python test_retrieval.py --test-suite test_queries.json

# Output to file
python test_retrieval.py --query "DDS explained" --output results.json
```

**Contract**:
- Exit code 0: Success
- Exit code 1: Invalid arguments
- Exit code 2: API error (Cohere or Qdrant)
- Exit code 3: No results found
- Output: Always valid JSON to stdout (or file if --output specified)

## 4. Non-Functional Requirements and Budgets

### 4.1 Performance

| Metric | Target | Budget | Measurement |
|--------|--------|--------|-------------|
| Query Embedding Latency | p95 < 500ms | p99 < 1s | Log all Cohere API calls |
| Vector Search Latency | p95 < 300ms | p99 < 500ms | Log all Qdrant queries |
| End-to-End Latency | p95 < 2s | p99 < 3s | Log total query-to-response time |
| Throughput | 10 queries/min | N/A (testing tool) | Not a priority for testing |

**Performance Strategy**:
- Use async I/O if batch testing implemented
- No caching in initial version (testing should be stateless)
- Log slow queries for investigation

---

### 4.2 Reliability

| Requirement | Target | Strategy |
|-------------|--------|----------|
| Success Rate | 95% for valid queries | Retry logic with exponential backoff (3 attempts) |
| Error Recovery | Graceful degradation | Return error JSON instead of crashing |
| API Availability | Dependent on external services | Validate connectivity before batch tests |

**SLO**: 95% of valid queries complete successfully within 2 seconds

**Error Budget**: 5% of queries may fail due to transient API issues

**Degradation Strategy**:
- If Cohere unavailable: Return error immediately, don't retry embedding
- If Qdrant unavailable: Retry up to 3 times with backoff
- If both unavailable: Fail fast with clear error message

---

### 4.3 Security

| Concern | Mitigation |
|---------|------------|
| API Key Exposure | Load from .env file, never log keys |
| Query Injection | Treat queries as opaque strings, no SQL/NoSQL injection risk |
| Data Privacy | No PII in test queries or logs |
| Audit Trail | Log all queries (text only, not results) for debugging |

**Secrets Management**:
- COHERE_API_KEY and QDRANT_API_KEY in .env file
- .env file in .gitignore
- Validate keys exist on startup

---

### 4.4 Cost

| Resource | Unit Cost | Expected Usage | Monthly Budget |
|----------|-----------|----------------|----------------|
| Cohere Embeddings | $0.0001/1K tokens | ~500 test queries × 20 tokens | $0.01 |
| Qdrant Search | Free tier | Unlimited reads | $0.00 |
| **Total** | | | **$0.01/month** |

**Cost Optimization**:
- Batch queries if testing at scale (future)
- No cost concern for testing phase

## 5. Data Management and Migration

### 5.1 Source of Truth

- **Vector Data**: Qdrant `website_embeddings` collection (created by 004-website-embedding)
- **Test Queries**: `test_queries.json` file in repository
- **Expected Results**: `expected_results.json` for validation

### 5.2 Schema Evolution

**Current Schema** (v1):
```json
{
  "query": "string",
  "top_k": "integer",
  "result_count": "integer",
  "results": [
    {
      "rank": "integer",
      "score": "float",
      "text": "string",
      "metadata": {
        "url": "string",
        "page_title": "string",
        "chunk_index": "integer",
        "total_chunks": "integer",
        "source": "string"
      }
    }
  ]
}
```

**Versioning Strategy**:
- Add `schema_version: "1.0"` field to response
- Future changes add new fields, never remove existing fields
- Consumers ignore unknown fields (forward compatibility)

### 5.3 Data Retention

- Test query results: Not stored permanently (ephemeral)
- Logs: Keep for 30 days locally for debugging
- No migration needed (testing tool, not data store)

## 6. Operational Readiness

### 6.1 Observability

**Logging**:
- Level: INFO for normal operations, DEBUG for troubleshooting
- Format: `[timestamp] [level] [component] message`
- Components: EMBED, SEARCH, FORMAT, CLI
- Log file: `retrieval_test.log` (rotated daily)

**Metrics** (logged, not monitored):
- Query count
- Average/p95/p99 latency per component
- Success/failure rate
- Top-K distribution

**Tracing**: Not needed for testing tool

---

### 6.2 Deployment and Rollback

**Deployment**:
1. Ensure 004-website-embedding is complete (Qdrant has data)
2. Verify .env file has valid API keys
3. Install dependencies: `uv add` (already done)
4. Run validation: `python test_retrieval.py --test-suite test_queries.json`

**Rollback**: N/A (testing tool, not deployed service)

**Feature Flags**: Not needed

---

### 6.3 Runbooks

**Common Task: Run Single Query**
```bash
cd backend
python test_retrieval.py --query "What is DDS in ROS 2?"
```

**Common Task: Run Full Test Suite**
```bash
cd backend
python test_retrieval.py --test-suite test_queries.json --verbose
```

**Troubleshooting: API Connection Failure**
1. Check .env file exists and has keys
2. Test Cohere: `curl -H "Authorization: Bearer $COHERE_API_KEY" https://api.cohere.ai/v1/embed`
3. Test Qdrant: Check collection in Qdrant dashboard
4. Verify network connectivity

**Troubleshooting: Low Similarity Scores**
1. Check if query embedding model matches document embedding model
2. Verify Qdrant collection uses cosine distance
3. Test with known good queries from documentation
4. Check if collection has sufficient data (run check_qdrant.py)

## 7. Risk Analysis and Mitigation

### Risk 1: Embedding Model Mismatch

**Description**: Query embeddings generated with different model than documents
**Probability**: Low (explicit validation in code)
**Impact**: High (all results irrelevant)
**Blast Radius**: Entire testing phase invalid
**Mitigation**:
- Hardcode model name as constant: `MODEL = "embed-english-v3.0"`
- Validate on startup by checking Qdrant collection config
- Document model requirement prominently in README
**Kill Switch**: N/A (testing tool)
**Guardrails**: Fail startup if model mismatch detected

---

### Risk 2: Qdrant Collection Empty or Missing

**Description**: 004-website-embedding not completed or collection deleted
**Probability**: Medium (dependency on upstream feature)
**Impact**: High (no results possible)
**Blast Radius**: All tests fail
**Mitigation**:
- Check collection exists and has >0 points on startup
- Provide clear error message directing to 004-website-embedding
- Document prerequisite in README
**Kill Switch**: Fail fast with error message

---

### Risk 3: API Rate Limits

**Description**: Cohere or Qdrant rate limits exceeded during batch testing
**Probability**: Low (testing load is light)
**Impact**: Medium (some queries fail)
**Blast Radius**: Failed test queries
**Mitigation**:
- Implement retry with exponential backoff
- Add configurable delay between batch queries (--delay flag)
- Monitor rate limit headers if available
**Guardrails**: Max 10 queries/minute default for batch mode

## 8. Evaluation and Validation

### 8.1 Definition of Done

- [ ] Retrieval module (`retrieval.py`) implemented with all functions
- [ ] CLI script (`test_retrieval.py`) accepts arguments and outputs JSON
- [ ] Test suite (`test_queries.json`) with 15+ diverse queries created
- [ ] Expected results (`expected_results.json`) documented
- [ ] Automated tests validate:
  - [ ] Query embedding generates 1024-dim vector
  - [ ] Search returns exactly top_k results (or fewer if collection smaller)
  - [ ] Similarity scores are floats in [0.0, 1.0]
  - [ ] All metadata fields present and valid
  - [ ] JSON validates against schema
  - [ ] Retrieved text matches original (sample validation)
- [ ] Manual testing confirms:
  - [ ] Relevant queries return appropriate chunks in top 5
  - [ ] End-to-end latency <2s for p95
  - [ ] Error handling works (empty query, API failure simulation)
- [ ] Documentation complete:
  - [ ] README with usage examples
  - [ ] Test results summary document

### 8.2 Output Validation

**Query Validation**:
- Non-empty string
- Length 1-2000 characters
- No injection attack patterns (not a security risk, but logged)

**Embedding Validation**:
- Output is list of exactly 1024 floats
- Values are finite (not NaN or Inf)

**Search Results Validation**:
- Result count ≤ top_k
- Each result has required fields: rank, score, text, metadata
- Scores in descending order
- Scores in range [0.0, 1.0]
- Metadata fields: url (valid URL), page_title (non-empty), chunk_index (≥0), total_chunks (>chunk_index)

**JSON Validation**:
- Valid JSON (parseable by json.loads())
- Conforms to schema
- No missing required fields
- Types correct for all fields

### 8.3 Test Queries

**Technical Queries** (exact terminology):
1. "What is DDS in ROS 2?"
2. "How do QoS profiles work?"
3. "Explain URDF syntax"

**Conceptual Queries** (understanding):
4. "How does ROS 2 differ from ROS 1?"
5. "What is a digital twin?"
6. "Why use Unity for robot simulation?"

**How-To Queries** (procedural):
7. "How to create a ROS 2 publisher in Python?"
8. "How to configure Gazebo physics?"
9. "How to integrate Unity with ROS 2?"

**Multi-Topic Queries**:
10. "Robot visualization and simulation"
11. "ROS 2 communication patterns"

**Edge Cases**:
12. "" (empty string - should error)
13. "quantum entanglement" (irrelevant - low scores)
14. "ros2" (very short - should still work)
15. "a" * 3000 (very long - should truncate or error)

## 9. Architectural Decision Records (ADRs)

### ADR Candidates

The following architectural decisions meet significance criteria and should be documented:

1. **Use Same Embedding Model for Queries and Documents**
   - **Impact**: Long-term (affects all future retrieval)
   - **Alternatives**: Multiple models considered
   - **Scope**: Cross-cutting (affects both embedding and retrieval)
   - **Recommendation**: Document as ADR-001

2. **JSON Response Schema Design**
   - **Impact**: Medium (affects API consumers)
   - **Alternatives**: Multiple schema structures considered
   - **Scope**: Affects downstream RAG chatbot integration
   - **Recommendation**: Document as ADR-002

📋 **Architectural decisions detected**:
1. Use same embedding model (embed-english-v3.0) for queries and documents - ensures vector space compatibility
2. Flat JSON response schema for simplicity and ease of validation

Document reasoning and tradeoffs? Run `/sp.adr "embedding-model-consistency"` and `/sp.adr "json-response-schema"`

## 10. Implementation Phases

### Phase 1: Core Retrieval Module (2-3 hours)

**Tasks**:
- Create `backend/retrieval.py` module
- Implement `embed_query()` function with Cohere API
- Implement `search_qdrant()` function
- Implement `format_results()` function
- Add error handling and logging
- Write unit tests for each function

**Acceptance**: Module functions work independently, pass unit tests

---

### Phase 2: CLI Interface (1-2 hours)

**Tasks**:
- Create `backend/test_retrieval.py` CLI script
- Parse command-line arguments (--query, --top-k, --output)
- Integrate with retrieval module
- Add JSON output formatting
- Test with sample queries

**Acceptance**: CLI runs single queries and outputs valid JSON

---

### Phase 3: Test Suite (2-3 hours)

**Tasks**:
- Create `backend/test_queries.json` with 15+ test queries
- Implement `--test-suite` mode in CLI
- Run tests and document results
- Create `backend/expected_results.json` baseline
- Validate retrieval accuracy

**Acceptance**: All test queries run successfully, results documented

---

### Phase 4: Documentation and Validation (1 hour)

**Tasks**:
- Update backend/README.md with retrieval testing section
- Document test results
- Create troubleshooting guide
- Final validation against success criteria

**Acceptance**: Documentation complete, all DoD items checked

---

**Total Estimated Effort**: 6-9 hours

**Critical Path**: Phase 1 (core module) blocks all other phases

**Parallelization**: Phase 2 and Phase 3 can partially overlap (CLI development while designing test queries)
