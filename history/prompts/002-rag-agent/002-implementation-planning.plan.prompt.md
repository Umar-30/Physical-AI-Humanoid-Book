---
id: 002
title: Implementation Planning RAG Agent
stage: plan
date: 2025-12-21
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-rag-agent
branch: 002-rag-agent
user: user
command: /sp.plan
labels: ["plan", "rag", "api", "fastapi", "openai", "architecture"]
links:
  spec: specs/002-rag-agent/spec.md
  plan: specs/002-rag-agent/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-agent/plan.md
  - specs/002-rag-agent/research.md
  - specs/002-rag-agent/data-model.md
  - specs/002-rag-agent/contracts/openapi.yaml
  - specs/002-rag-agent/quickstart.md
tests: []
---

# Prompt

```text
/sp.plan
```

# Response

Created comprehensive implementation plan for RAG Agent API Service:

**Feature**: 002-rag-agent
**Branch**: 002-rag-agent
**Plan File**: specs/002-rag-agent/plan.md

## Planning Workflow Completed

### Phase 0: Research & Discovery

Created `research.md` documenting 8 key technical decisions:

1. **FastAPI vs Flask vs Django**: Selected FastAPI for async support, auto-validation, and built-in OpenAPI docs
2. **OpenAI SDK Integration Pattern**: Service layer abstraction for testability and clean separation
3. **Prompt Engineering Strategy**: XML-delimited context in user message for clarity
4. **Token Management**: Select top-N complete chunks within limit (preserves semantic integrity)
5. **Error Handling**: Fail fast + retry for transient errors (rate limits, server errors)
6. **CORS Configuration**: Permissive for development, configurable for production
7. **Logging Strategy**: Standard library with structured JSON format
8. **Source Deduplication**: Keep highest-scoring chunk per URL to avoid redundant sources

**Result**: All NEEDS CLARIFICATION items resolved, ready for design phase.

---

### Phase 1: Design & Contracts

#### Data Model (`data-model.md`)

Defined 4 core entities with complete Pydantic models:

1. **QueryRequest**: User input with validation (query 1-2000 chars, top_k 1-50, valid model names)
2. **SourceReference**: Documentation source with URL, title, relevance score, chunk index
3. **QueryResponse**: Complete response with answer, sources, model_used, tokens_used, retrieval_count
4. **ErrorResponse**: Standardized error format with error_type, message, status_code

**Validation Rules**: All fields have type constraints, ranges, and business logic validators
**State Transitions**: Documented query lifecycle from Created → Validated → Retrieving → Generating → Returned

---

#### API Contracts (`contracts/openapi.yaml`)

Generated OpenAPI 3.0 specification with:

- **Endpoints**: POST /query (main endpoint), GET /health (health check)
- **Request/Response Examples**: 7 examples covering success, validation errors, service failures
- **Status Codes**: 200 (success), 422 (validation), 429 (rate limit), 502 (generation failure), 503 (retrieval failure)
- **Schemas**: Complete component schemas for all request/response types

**Auto-Documentation**: FastAPI will serve this at http://localhost:8000/docs

---

#### Developer Guide (`quickstart.md`)

Created comprehensive getting-started guide with:

- **Prerequisites**: Python 3.9+, API keys (OpenAI, Qdrant, Cohere)
- **Installation**: Step-by-step dependency installation and environment setup
- **Usage Examples**: cURL, Python (requests), JavaScript (fetch API)
- **Troubleshooting**: Common issues (auth errors, database unavailable, token limits, CORS)
- **Testing Checklist**: Manual test scenarios for validation

---

### Implementation Plan (`plan.md`)

Generated 11-section comprehensive plan:

#### 1. Scope and Dependencies
- **In Scope**: 11 items (FastAPI REST API, validation, retrieval integration, prompt engineering, OpenAI integration, token management, source deduplication, error handling, CORS, logging, health endpoint)
- **Out of Scope**: 12 items (auth, rate limiting, caching, streaming, multi-turn, UI, model fine-tuning, i18n, advanced techniques, deployment, monitoring, A/B testing)
- **External Dependencies**: 5 tracked (OpenAI API, Qdrant, Cohere, retrieval.py module, Python 3.9+)

#### 2. Key Decisions (7 Total)

All documented with:
- Options considered (A/B/C)
- Trade-offs analysis
- Rationale for selection
- Reversibility assessment
- Measurement criteria

**Decision Highlights**:
- FastAPI for async and auto-docs
- Service layer for testability
- XML-delimited prompts for clarity
- Chunk selection for token management
- Retry logic for resilience

#### 3. Interfaces and API Contracts

- REST API endpoint specification
- Internal service contracts (AnswerGenerationService, PromptBuilder, SourceDeduplicator)
- Request/response examples
- Error response formats

#### 4. Non-Functional Requirements

**Performance**:
- Latency targets: p50 <2s, p95 <5s, p99 <10s
- Throughput: 10 concurrent requests sustained, 20 burst
- Resource caps: <512MB memory, <1 core CPU

**Reliability**:
- SLOs: 99% uptime, 95% success rate
- Error budget: 5% request failures allowed
- Degradation strategy for dependency failures

**Security**:
- No auth in v1 (development only)
- API keys in .env, never logged
- Input validation with Pydantic

**Cost**:
- Token costs: $0.001-0.003 per query (gpt-3.5-turbo), $0.10-0.20 (gpt-4)
- Budget: $100/month dev, $50/month testing
- Monitoring: Log tokens_used for cost tracking

#### 5. Data Management

- Configuration source of truth: .env file
- No data persistence (stateless API)
- Logs: 7-day retention, daily rotation

#### 6. Operational Readiness

**Observability**:
- Structured JSON logs with timestamp, level, component, query, latency, tokens
- Metrics: request count, latency percentiles, token usage, error rate

**Runbooks**:
- Restart server
- Check health
- View logs
- Test queries

**Deployment**:
- v1: Manual uvicorn start (development)
- Future: Docker with blue-green deployment

#### 7. Risk Analysis

**Top 3 Risks**:
1. OpenAI rate limits (Medium prob, High impact) → Retry + clear error message
2. Qdrant unavailable (Low prob, Critical impact) → Health check + error message
3. Token limit exceeded (Medium prob, Low impact) → Pre-calculate tokens + chunk selection

**Mitigation**: Kill switches, retries, guardrails

#### 8. Constitution Check

**Pre-Design Gate**: ✅ PASSED
- Technical accuracy: API contracts validated, data models defined
- Reproducibility: Dependencies listed, env vars documented, setup commands provided
- Clarity: Quickstart guide, API auto-docs, examples for multiple languages
- Consistency: REST standards, OpenAPI 3.0, Pydantic validation
- Practicality: Reuses retrieval.py, minimal dependencies, clear errors
- Verification: OpenAPI validates, Pydantic examples, edge cases documented

**Post-Design Gate**: ✅ PASSED
- Smallest viable change: No refactoring, reuses existing modules
- Reversible decisions: Service layer, prompts, token management
- Measurable outcomes: Latency, error rates, token usage all tracked
- Risk mitigation: Top 3 risks with blast radius and mitigation
- Operational readiness: Logging, runbooks, deployment docs

#### 9. Evaluation and Validation

**Definition of Done**: 7 criteria (validation, retrieval, context building, generation, schema compliance, deduplication, latency logging)

**Testing Strategy**:
- Unit tests: Pydantic, PromptBuilder, SourceDeduplicator, AnswerGenerationService
- Integration tests: End-to-end flow, error handling, token limits
- Manual validation: 20 test queries covering ROS 2, Unity, edge cases

**Output Validation**: Format (valid JSON), requirements (FR-001 to FR-012), safety (no secrets, input validation)

#### 10. Architecture Decision Records (ADR)

Identified 4 significant decisions requiring ADR documentation:
- ADR-001: FastAPI Framework Selection
- ADR-002: Service Layer Architecture Pattern
- ADR-003: XML-Delimited Prompt Structure
- ADR-004: Token Management via Chunk Selection

**Recommendation**: Run `/sp.adr` to create formal ADRs

#### 11. Next Steps

1. Review and approve plan
2. Generate ADRs for key decisions
3. Create tasks.md with testable implementation tasks
4. Implement core service
5. Integration testing
6. Manual validation (20 queries)
7. Update documentation

---

## Artifacts Generated

1. **research.md**: 8 technical decisions researched and documented
2. **data-model.md**: 4 entities with Pydantic models, validation rules, relationships
3. **contracts/openapi.yaml**: Complete OpenAPI 3.0 specification with examples
4. **quickstart.md**: Developer guide with setup, usage, troubleshooting
5. **plan.md**: 11-section comprehensive implementation plan

---

## Quality Gates

**Constitution Compliance**: ✅ ALL CHECKS PASSED
- Technical Accuracy (NON-NEGOTIABLE): ✅
- Reproducibility (NON-NEGOTIABLE): ✅
- Clarity for Learners: ✅
- Consistency with Standards: ✅
- Practicality: ✅
- Verification Before Inclusion: ✅

**Plan Completeness**:
- ✅ Scope clearly defined (in/out)
- ✅ Dependencies tracked with ownership
- ✅ Decisions documented with rationale
- ✅ NFRs specified (performance, reliability, security, cost)
- ✅ Risks identified with mitigation
- ✅ Operational readiness addressed

---

## Technology Stack Summary

**Framework**: FastAPI (async, auto-validation, OpenAPI docs)
**LLM Integration**: OpenAI SDK v1.0+ (gpt-3.5-turbo default, gpt-4 optional)
**Retrieval**: Reuse retrieval.py from Spec-2 (Qdrant + Cohere)
**Validation**: Pydantic models with type enforcement
**Server**: Uvicorn ASGI server
**Logging**: Standard library with structured JSON

**Dependencies** (new for this feature):
- fastapi
- uvicorn
- openai (>=1.0.0)
- tiktoken (token counting)

**Reused Dependencies** (from Spec-2):
- cohere (query embedding)
- qdrant-client (vector search)
- python-dotenv (env vars)

---

## Next Command

Run `/sp.tasks` to generate implementation task breakdown.

# Outcome

**Status**: SUCCESS
**Artifact**: Complete implementation plan with research, data model, contracts, and quickstart guide
**Files Created**: 5 (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md)
**Constitution Gates**: ALL PASSED
**Ready For**: `/sp.tasks` command to create task list
