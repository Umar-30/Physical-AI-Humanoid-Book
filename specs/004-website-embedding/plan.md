# Implementation Plan: Website Embedding Pipeline

**Branch**: `004-website-embedding` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/004-website-embedding/spec.md`

## Summary

Create a robust pipeline that converts the deployed Docusaurus book website into searchable vector embeddings for RAG (Retrieval-Augmented Generation) usage. The pipeline will discover all pages via sitemap parsing, extract clean text content, chunk it appropriately, generate embeddings using Cohere's API, and store them in Qdrant vector database with comprehensive metadata. The system supports both sitemap index and regular sitemap formats, handles errors gracefully, and can resume from interruptions.

## 1. Scope and Dependencies

### In Scope
- Sitemap discovery and parsing (both index and regular formats)
- Text extraction from HTML pages with cleanup
- Text chunking for optimal embedding
- Cohere embedding generation with batching
- Qdrant vector storage with metadata
- Error handling and retry logic
- Progress tracking and logging
- Environment-based configuration

### Out of Scope (Explicitly Excluded)
- Authentication for private pages (only public pages)
- Real-time incremental updates (batch processing only)
- Custom web crawling beyond sitemap
- Embedding model fine-tuning
- RAG query interface (separate feature)
- Multi-language support (English only)
- Image or multimedia content embedding

### External Dependencies

| Dependency | Version | Ownership | Purpose | Risk |
|------------|---------|-----------|---------|------|
| Cohere API | v3.0 | External SaaS | Embedding generation | Rate limits, API availability |
| Qdrant | Latest | Self-hosted/Cloud | Vector storage | Storage capacity, network |
| Requests | 2.31+ | PyPI | HTTP requests | Network reliability |
| BeautifulSoup4 | 4.12+ | PyPI | HTML parsing | None (stable) |
| Python | 3.10+ | System | Runtime | None |
| Target Website | Live | Self-hosted (GitHub Pages) | Content source | Deployment status |

## 2. Key Decisions and Rationale

### Decision 1: Sitemap-Based Discovery vs. Web Crawling

**Options Considered**:
1. **Sitemap-based discovery** (SELECTED)
   - Parse sitemap.xml for URL list
   - Supports both sitemap index and regular formats
   - Recursive processing for nested sitemaps

2. Web crawling with link following
   - Start from homepage and follow all links
   - Requires deduplication and depth limits
   - More complex, higher risk of missing pages or infinite loops

**Trade-offs**:
- Sitemap: Fast, reliable, gets all intended pages, respects site structure
- Crawling: Flexible, can find orphaned pages, but complex and error-prone

**Rationale**: Docusaurus automatically generates comprehensive sitemaps. Sitemap-based discovery is faster, more reliable, and aligns with web standards. The sitemap contains exactly the pages we want to index.

**Principles**: Smallest viable change, measurable (count URLs found), reversible (can add crawling later)

### Decision 2: Cohere for Embeddings vs. OpenAI/Local Models

**Options Considered**:
1. **Cohere embed-english-v3.0** (SELECTED)
   - 1024-dimension embeddings
   - Optimized for semantic search
   - Good rate limits and pricing

2. OpenAI text-embedding-ada-002
   - 1536-dimension embeddings
   - Higher cost per token
   - Widely used but more expensive

3. Local models (sentence-transformers)
   - Free, no API limits
   - Requires GPU for speed
   - Lower quality embeddings

**Trade-offs**:
- Cohere: Good balance of quality, cost, and ease of use
- OpenAI: Higher quality but more expensive
- Local: Free but requires infrastructure and lower quality

**Rationale**: Cohere provides production-quality embeddings at reasonable cost with good developer experience. No local infrastructure needed.

**Principles**: Measurable (embedding quality tests), reversible (can swap embedding provider)

### Decision 3: Chunk Size Strategy

**Options Considered**:
1. **Fixed word count (~512 words)** (SELECTED)
   - Simple, predictable
   - Works well with Cohere's token limits
   - Good balance for semantic coherence

2. Semantic chunking (paragraph/section boundaries)
   - More semantically coherent
   - Variable sizes, complex implementation
   - May exceed token limits

3. Fixed token count
   - Precise API alignment
   - Requires tokenizer, more complex
   - Over-engineering for current need

**Trade-offs**:
- Fixed words: Simple, fast, predictable size
- Semantic: Better quality but complex
- Fixed tokens: Most precise but over-engineered

**Rationale**: Fixed word count provides good semantic coherence while remaining simple. Docusaurus content is well-structured, so word-based chunking works effectively.

**Principles**: Smallest viable change, measurable (chunks generated), reversible (can refine chunking later)

## 3. Interfaces and API Contracts

### Public API: Main Pipeline Entry Point

```python
def main() -> None:
    """
    Execute the complete embedding pipeline.

    Environment Variables Required:
    - COHERE_API_KEY: Cohere API key
    - QDRANT_URL: Qdrant instance URL (default: localhost:6333)
    - QDRANT_API_KEY: Qdrant API key (for cloud instances)

    Raises:
        ValueError: If COHERE_API_KEY is not set
        requests.exceptions.RequestException: For network errors
        Exception: For other critical failures

    Side Effects:
        - Creates/recreates Qdrant collection 'rag_embedding'
        - Prints progress logs to stdout
    """
```

### Internal API: URL Discovery

```python
def get_all_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """
    Handle both sitemap index and regular sitemap.

    Args:
        sitemap_url: URL to sitemap.xml or sitemap_index.xml

    Returns:
        List[str]: All page URLs found (recursively for indexes)

    Errors:
        - Returns empty list on parse errors (logged)
        - Continues on network errors for individual sitemaps

    Idempotency: Yes (same input → same output)
    """
```

### Internal API: Text Processing

```python
def chunk_text(text: str, chunk_size: int = 512) -> List[str]:
    """
    Split text into word-based chunks.

    Args:
        text: Input text to chunk
        chunk_size: Target words per chunk (default: 512)

    Returns:
        List[str]: Text chunks, each ~chunk_size words

    Guarantees:
        - No chunk exceeds chunk_size + max_word_length
        - Chunks maintain word boundaries (no mid-word splits)
        - Empty input → empty list

    Idempotency: Yes
    """
```

### External API: Cohere Embedding

```python
# Cohere SDK call
cohere_client.embed(
    texts: List[str],         # Max 96 texts per batch
    model: "embed-english-v3.0",
    input_type: "search_document"
) -> EmbeddingsResponse

# Response
{
    "embeddings": List[List[float]],  # 1024-dim vectors
    "meta": {...}
}

# Errors
- 429: Rate limit exceeded → backoff and retry
- 400: Invalid input → log and skip
- 500: Server error → retry up to 3 times
```

### External API: Qdrant Storage

```python
qdrant_client.upsert(
    collection_name: str,
    points: List[PointStruct]
)

# PointStruct schema
{
    "id": int,                    # hash(url + chunk_index)
    "vector": List[float],        # 1024-dim from Cohere
    "payload": {
        "text": str,              # Original chunk text
        "url": str,               # Source page URL
        "chunk_index": int        # Position in page
    }
}

# Errors
- Network timeout → retry with backoff
- Collection not found → create collection first
- Invalid vector dimension → fail fast
```

### Error Taxonomy

| Status Code | Meaning | Action | Retry |
|-------------|---------|--------- |-------|
| 200 | Success | Continue | N/A |
| 400 | Invalid request | Log, skip item | No |
| 401/403 | Auth error | Fail fast, check API keys | No |
| 429 | Rate limit | Exponential backoff | Yes (3x) |
| 500 | Server error | Log, retry | Yes (3x) |
| Network timeout | Connection issue | Log, retry | Yes (3x) |

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance

- **Latency**:
  - p95 sitemap parsing: <5 seconds
  - p95 single page extraction: <2 seconds
  - p95 embedding batch (10 chunks): <3 seconds
  - p95 Qdrant upsert batch: <1 second

- **Throughput**:
  - Target: 50-100 pages/hour (depends on page size)
  - Embedding: 10 chunks per batch (Cohere best practice)
  - Total pipeline: ~2 hours for full site (estimated 100-150 pages)

- **Resource Caps**:
  - Memory: <500MB RAM (streaming processing)
  - Network: <100MB total transfer (compressed HTML + API)
  - Cohere API: <1M tokens/month (free tier: 100 API calls/month)
  - Qdrant: <1GB storage (estimated 100K chunks × 1024 dims × 4 bytes)

### Reliability

- **SLOs**:
  - 95% page extraction success rate
  - 99% embedding generation success (with retries)
  - 99.9% Qdrant storage success (local/cloud)

- **Error Budget**: 5% of pages can fail without halting pipeline

- **Degradation Strategy**:
  - On Cohere rate limit: Exponential backoff (1s, 2s, 4s, fail)
  - On page load failure: Log and skip, continue with other pages
  - On Qdrant unavailable: Fail fast (critical dependency)

### Security

- **Authentication**:
  - API keys via environment variables only
  - Never commit keys to git (`.env` in `.gitignore`)

- **Authorization**:
  - Cohere: API key-based
  - Qdrant: API key for cloud, no auth for local

- **Data Handling**:
  - Only public website data (no PII)
  - No sensitive content filtering needed
  - API keys stored in memory only during runtime

- **Secrets Management**:
  - Use `.env` file for local development
  - Use environment variables in production
  - Document required keys in README

- **Auditing**:
  - Log all API calls with timestamps
  - Log page processing with URLs
  - No sensitive data in logs

### Cost

- **Unit Economics**:
  - Cohere: ~$0.0001 per 1K tokens (search documents)
  - Estimated: 100 pages × 5K words/page × 1.3 tokens/word = 650K tokens
  - Total embedding cost: ~$0.065 per full run
  - Qdrant cloud: ~$25/month (1GB tier) OR self-hosted (free)

- **Budget**: <$1/month for embeddings (monthly refresh), $0 if self-hosted Qdrant

## 5. Data Management and Migration

### Source of Truth

- **Website Content**: GitHub Pages (deployed Docusaurus site)
- **Embeddings**: Qdrant collection `rag_embedding`
- **Configuration**: `.env` file (local), environment variables (production)

### Schema Evolution

**Current Schema (v1)**:
```python
{
    "id": hash(url + chunk_index),
    "vector": List[float],  # 1024-dim
    "payload": {
        "text": str,
        "url": str,
        "chunk_index": int
    }
}
```

**Future Extensions** (not in scope):
- Add `page_title`, `section_heading` to payload
- Add `timestamp` for freshness tracking
- Add `version` field for schema versioning

**Versioning Strategy**:
- Use collection name suffixes for breaking changes (`rag_embedding_v2`)
- Add new payload fields (backwards compatible)
- Migration script for schema updates (future)

### Migration and Rollback

**Initial Load**:
1. Create collection with `recreate_collection()` (drops existing)
2. Process all pages and upsert embeddings
3. Validate: Query sample chunks to verify retrieval

**Updates** (future):
1. Create new collection (`rag_embedding_tmp`)
2. Run pipeline to populate new collection
3. Validate new collection
4. Rename collections (atomic swap)
5. Delete old collection

**Rollback**:
- Keep previous collection for 24 hours
- If issues found, revert collection rename
- Re-run pipeline if needed (idempotent)

### Data Retention

- **Embeddings**: Indefinite (overwritten on refresh)
- **Logs**: 30 days (stdout/file logs)
- **Checkpoints**: Not implemented (future: keep last successful state)

## 6. Operational Readiness

### Observability

**Logs**:
```python
# Log format
[TIMESTAMP] [LEVEL] [COMPONENT] Message
[2025-12-20 10:30:45] [INFO] [SITEMAP] Found sitemap index with 3 sitemaps
[2025-12-20 10:30:48] [INFO] [SITEMAP] Found regular sitemap with 47 URLs
[2025-12-20 10:31:02] [INFO] [EXTRACT] Processing https://example.com/page1
[2025-12-20 10:31:05] [INFO] [CHUNK] Text split into 12 chunks
[2025-12-20 10:31:08] [INFO] [EMBED] Generating embeddings for batch 1...
[2025-12-20 10:31:10] [INFO] [STORE] Processed 10/12 chunks
[2025-12-20 10:31:45] [ERROR] [EXTRACT] Error extracting text from https://example.com/page2: 404
```

**Metrics** (future):
- Total pages processed
- Total chunks created
- Embedding API calls made
- Success/failure rates
- Processing time per page

**Traces** (future):
- Full pipeline execution trace
- Per-page processing timeline
- API call latencies

### Alerting

**Critical Alerts** (future):
- Cohere API key invalid/expired
- Qdrant unavailable
- >20% page extraction failures

**Warning Alerts** (future):
- Slow page loads (>5s p95)
- High Cohere error rate (>5%)
- Low embedding quality scores

**On-call**: Not applicable (batch process, not user-facing service)

### Runbooks

**Runbook 1: Run Full Pipeline**
```bash
# Prerequisites
cd backend
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt

# Set environment variables
export COHERE_API_KEY="your-key-here"
export QDRANT_URL="localhost:6333"  # or cloud URL
export QDRANT_API_KEY="your-key"    # if using cloud

# Run pipeline
python main.py

# Verify
# - Check logs for "Embedding pipeline completed successfully!"
# - Query Qdrant to verify chunks stored
```

**Runbook 2: Troubleshoot Cohere Rate Limits**
```bash
# Symptom: 429 errors from Cohere API
# Solution 1: Wait for rate limit reset (usually 1 minute)
# Solution 2: Reduce batch size in code (change batch_size from 10 to 5)
# Solution 3: Add delays between batches (time.sleep(2) between batches)
```

**Runbook 3: Reset Qdrant Collection**
```python
# If collection corrupted or schema needs reset
from qdrant_client import QdrantClient
client = QdrantClient(host="localhost", port=6333)
client.delete_collection("rag_embedding")
# Re-run main.py to recreate and populate
```

### Deployment and Rollback

**Deployment Strategy**:
1. **Local Development**: Run `python main.py` directly
2. **CI/CD** (future): GitHub Actions workflow
   - Trigger: Manual or on website content update
   - Steps: Install deps → Run pipeline → Verify Qdrant
3. **Production** (future): Scheduled cron job (daily/weekly refresh)

**Rollback Strategy**:
- If embeddings corrupted: Delete collection and re-run pipeline
- If code bug introduced: Revert git commit, re-run with previous version
- Estimated rollback time: <1 hour (reprocessing time)

**Feature Flags** (future):
- `ENABLE_SITEMAP_INDEX`: Toggle sitemap index support
- `BATCH_SIZE`: Configurable embedding batch size
- `CHUNK_SIZE`: Configurable text chunk size

### Backwards Compatibility

**Version 1.0** (current):
- Python 3.10+
- Cohere embed-english-v3.0
- Qdrant 1.x

**Future Compatibility**:
- If Cohere model updated: Add `model` env variable, default to v3.0
- If Qdrant schema changes: Version collection names (`rag_embedding_v2`)
- If Python 3.11+: Maintain Python 3.10 compatibility for 6 months

## 7. Risk Analysis and Mitigation

### Risk 1: Cohere API Rate Limits

**Impact**: Pipeline stalls or fails mid-execution
**Probability**: Medium (especially on free tier)
**Blast Radius**: Partial data (some pages embedded, others not)

**Mitigation**:
- Implement exponential backoff on 429 errors
- Batch processing with delays between batches
- Monitor API usage and upgrade tier if needed
- Future: Checkpoint/resume capability

**Kill Switch**: N/A (manual process, can stop/restart anytime)

### Risk 2: Website Structure Changes

**Impact**: Sitemap location or format changes, breaking discovery
**Probability**: Low (Docusaurus generates standard sitemaps)
**Blast Radius**: Complete failure (no URLs discovered)

**Mitigation**:
- Check multiple sitemap locations (`/sitemap.xml`, `/sitemap_index.xml`)
- Fallback to base URL if no sitemap found
- Monitor sitemap fetch success rate
- Future: Add manual URL list fallback

**Guardrail**: Log clear error if no sitemap found, don't fail silently

### Risk 3: Qdrant Storage Capacity

**Impact**: Unable to store all embeddings
**Probability**: Low (estimated <1GB for full site)
**Blast Radius**: Partial data (early pages stored, later pages fail)

**Mitigation**:
- Monitor Qdrant disk usage before runs
- Estimate storage needs (pages × avg_chunks × vector_size)
- Use cloud Qdrant with auto-scaling if self-hosted insufficient
- Future: Implement storage limit checks before processing

**Guardrail**: Fail fast on Qdrant storage errors, don't continue processing

## 8. Evaluation and Validation

### Definition of Done

**Code Complete**:
- [x] Sitemap parsing (index + regular) implemented
- [x] HTML text extraction with cleanup
- [x] Text chunking function
- [x] Cohere embedding integration
- [x] Qdrant storage with metadata
- [ ] Error handling and retry logic (partial)
- [ ] Environment variable configuration
- [ ] Logging with structured format

**Testing**:
- [ ] Manual test with live website sitemap
- [ ] Verify all pages discovered
- [ ] Verify embeddings generated correctly
- [ ] Verify Qdrant retrieval works
- [ ] Test error scenarios (bad URL, rate limit)

**Documentation**:
- [ ] README with setup instructions
- [ ] Environment variable documentation
- [ ] Example .env file
- [ ] Runbook for common operations

**Deployment**:
- [ ] requirements.txt with pinned versions
- [ ] Successful run on target environment
- [ ] Verification queries return relevant results

### Output Validation

**Format Validation**:
```python
# Verify embedding dimensions
assert len(embedding) == 1024, "Cohere embeddings must be 1024-dim"

# Verify payload structure
assert "text" in payload and "url" in payload and "chunk_index" in payload

# Verify chunk size bounds
assert chunk_size >= 256 and chunk_size <= 1024, "Chunks must be 256-1024 words"
```

**Requirements Validation**:
- FR-001: Verify >95% of sitemap URLs processed
- FR-004: Verify all chunks have embeddings
- FR-005: Verify all stored points have complete metadata
- SC-001: Measure processing time (<2 hours target)
- SC-002: Sample check metadata completeness
- SC-003: Run test queries and measure precision

**Safety Validation**:
- No API keys in logs
- No PII in stored data
- No malicious code execution (HTML parsed safely with BeautifulSoup)

## 9. Architectural Decision Record (ADR)

**ADR-001: Sitemap-Based URL Discovery**
**Status**: Accepted
**Context**: Need reliable way to discover all pages on Docusaurus site
**Decision**: Use sitemap.xml parsing (both index and regular formats)
**Consequences**: Simple, fast, relies on Docusaurus sitemap generation, may miss pages not in sitemap
**Alternatives Rejected**: Web crawling (too complex), manual URL list (not maintainable)

**ADR-002: Cohere for Embedding Generation**
**Status**: Accepted
**Context**: Need high-quality embeddings for semantic search
**Decision**: Use Cohere embed-english-v3.0 API
**Consequences**: External dependency, API costs, excellent quality, simple integration
**Alternatives Rejected**: OpenAI (more expensive), local models (infrastructure + quality concerns)

**ADR-003: Fixed Word-Count Chunking**
**Status**: Accepted
**Context**: Need to split long pages into embeddable chunks
**Decision**: Use fixed 512-word chunking with word boundaries
**Consequences**: Simple, predictable, may split semantic units, good enough for well-structured content
**Alternatives Rejected**: Semantic chunking (too complex), fixed token count (over-engineering)

**ADR-004: Qdrant for Vector Storage**
**Status**: Accepted
**Context**: Need scalable vector database for embeddings
**Decision**: Use Qdrant with local or cloud deployment
**Consequences**: Flexible deployment, good performance, requires separate service
**Alternatives Rejected**: Pinecone (more expensive), ChromaDB (less mature), PostgreSQL pgvector (less optimized)

## 10. Implementation Phases

### Phase 0: Research (COMPLETED)
- [x] Understand Docusaurus sitemap structure
- [x] Review Cohere API documentation
- [x] Review Qdrant client library
- [x] Identify best practices for text chunking

### Phase 1: Core Pipeline (IN PROGRESS)
- [x] Implement sitemap parsing (parse_sitemap)
- [x] Implement sitemap index handling (get_all_urls_from_sitemap)
- [x] Implement URL discovery (get_all_urls)
- [x] Implement text extraction (extract_text_from_url)
- [x] Implement text chunking (chunk_text)
- [x] Implement embedding generation (embed)
- [x] Implement Qdrant storage (save_chunk_to_qdrant)
- [x] Implement main pipeline orchestration (main)

### Phase 2: Hardening (NEXT)
- [ ] Add retry logic for API calls
- [ ] Add rate limit handling with backoff
- [ ] Improve error messages and logging
- [ ] Add progress indicators (% complete)
- [ ] Create requirements.txt
- [ ] Add environment variable validation
- [ ] Create .env.example file

### Phase 3: Testing & Validation (PENDING)
- [ ] Test with live website
- [ ] Verify embedding quality with sample queries
- [ ] Test error scenarios
- [ ] Document any edge cases found
- [ ] Create verification script

### Phase 4: Documentation (PENDING)
- [ ] Create comprehensive README
- [ ] Document environment setup
- [ ] Add usage examples
- [ ] Create troubleshooting guide
- [ ] Document architecture decisions

## 11. Tasks Overview

Detailed task breakdown will be generated using the `/sp.tasks` command and stored in `tasks.md`. High-level task categories:

1. **Pipeline Hardening** (5-7 tasks)
   - Retry logic, error handling, logging improvements

2. **Configuration & Environment** (3-4 tasks)
   - requirements.txt, .env setup, validation

3. **Testing & Validation** (4-5 tasks)
   - Live testing, quality checks, edge case handling

4. **Documentation** (3-4 tasks)
   - README, setup guide, troubleshooting

**Total Estimated Tasks**: 15-20 tasks

---

**Plan Status**: Ready for task generation (`/sp.tasks`)
**Next Command**: `/sp.tasks` to generate detailed, testable tasks from this plan
**Review Date**: 2025-12-20
**Last Updated**: 2025-12-20
