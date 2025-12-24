# Feature Specification: Retrieval Pipeline Testing for RAG Ingestion

**Feature Branch**: `001-retrieval-pipeline-testing`
**Created**: 2025-12-20
**Status**: Draft
**Input**: User description: "Retrieval + pipeline testing for RAG ingestion. Goal: verify that stored vectors in Qdrant can be retrieved accurately. Success criteria: Query Qdrant and receive correct top_k matches, retrieved chunks match original text, metadata (url, chunk_id) returns correctly, end-to-end test: input query -> Qdrant response -> clean JSON output"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query Qdrant and Retrieve Top-K Matches (Priority: P1)

As a RAG system developer, I want to query Qdrant with a text query and receive the top-K most relevant chunks so that I can verify the retrieval pipeline returns accurate results.

**Why this priority**: This is the core functionality that validates whether the stored embeddings can be searched effectively - without accurate retrieval, the RAG system cannot function.

**Independent Test**: Can be fully tested by submitting various queries and verifying that the returned chunks are ranked correctly by relevance.

**Acceptance Scenarios**:

1. **Given** a text query about ROS 2 topics, **When** the query is embedded and searched in Qdrant, **Then** the top 5 results contain relevant chunks about ROS 2 pub/sub architecture
2. **Given** a query about Unity integration, **When** the retrieval pipeline processes it, **Then** chunks from the Digital Twin module are ranked higher than unrelated content
3. **Given** a query using technical terminology from the documentation, **When** semantic search is performed, **Then** exact terminology matches appear in top 3 results
4. **Given** a query with multiple relevant topics, **When** top-K is set to 10, **Then** exactly 10 results are returned ranked by cosine similarity score

---

### User Story 2 - Validate Retrieved Content Matches Original Text (Priority: P1)

As a RAG system developer, I want to verify that retrieved chunks contain the exact text that was originally embedded so that I can ensure data integrity throughout the pipeline.

**Why this priority**: Data integrity is critical - if retrieved text doesn't match what was stored, the RAG system will provide incorrect information to users.

**Independent Test**: Can be fully tested by comparing retrieved chunk text against the original source documents.

**Acceptance Scenarios**:

1. **Given** a specific chunk ID from the embedding process, **When** that chunk is retrieved from Qdrant, **Then** the text field exactly matches the original chunked text
2. **Given** a retrieved result for "DDS explained", **When** the chunk text is compared to the source file, **Then** no text corruption or truncation has occurred
3. **Given** chunks with special characters or code snippets, **When** they are retrieved, **Then** formatting and special characters are preserved
4. **Given** multiple chunks from the same page, **When** retrieved sequentially, **Then** they reconstruct the original page content in correct order

---

### User Story 3 - Verify Metadata Returns Correctly (Priority: P2)

As a RAG system developer, I want to verify that all metadata fields (URL, chunk_id, page_title, chunk_index) are returned correctly so that I can trace answers back to their source.

**Why this priority**: Source attribution requires accurate metadata - users need to verify information by referencing the original documentation.

**Independent Test**: Can be fully tested by retrieving chunks and validating all metadata fields against expected values.

**Acceptance Scenarios**:

1. **Given** a retrieved chunk from a ROS 2 documentation page, **When** metadata is inspected, **Then** it contains valid URL, page_title, chunk_index, and total_chunks fields
2. **Given** chunks from different modules, **When** retrieved, **Then** URLs correctly point to distinct documentation sections
3. **Given** a chunk from position 3 of 7 in a document, **When** retrieved, **Then** chunk_index=2 and total_chunks=7 (0-indexed)
4. **Given** any retrieved result, **When** metadata URL is accessed, **Then** it corresponds to a valid documentation page

---

### User Story 4 - End-to-End Test with Clean JSON Output (Priority: P1)

As a RAG system developer, I want to perform end-to-end tests from query input to JSON response so that I can validate the complete retrieval pipeline integration.

**Why this priority**: End-to-end validation ensures all components (embedding, search, formatting) work together correctly before production deployment.

**Independent Test**: Can be fully tested by running complete queries and validating the JSON response structure and content.

**Acceptance Scenarios**:

1. **Given** a natural language query, **When** processed through the complete pipeline (embed query → search Qdrant → format response), **Then** a valid JSON response is returned with results array
2. **Given** a retrieval response, **When** JSON is parsed, **Then** it contains: query text, top_k count, results array with score/text/metadata for each result
3. **Given** multiple test queries covering different topics, **When** executed sequentially, **Then** all return valid JSON without errors
4. **Given** an edge case query (empty string, very long query, special characters), **When** processed, **Then** appropriate error JSON is returned with error message

---

### Edge Cases

- What happens when a query returns zero matches (no relevant content in database)?
- How does the system handle queries that are too short (1-2 words)?
- What if the Cohere API is unavailable during query embedding?
- How does the system behave with malformed queries (special characters, SQL injection attempts)?
- What happens when top_k exceeds the total number of stored vectors?
- How does retrieval perform with queries in different languages or with typos?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept natural language text queries and generate embeddings using the same Cohere model used for document embedding
- **FR-002**: System MUST query Qdrant using the query embedding and return the top-K most similar chunks ranked by cosine similarity
- **FR-003**: System MUST allow configurable top_k parameter with default value of 5 and maximum of 50
- **FR-004**: System MUST return complete metadata for each retrieved chunk including url, page_title, chunk_index, total_chunks, and source
- **FR-005**: System MUST preserve exact text content when retrieving chunks from Qdrant (no truncation or corruption)
- **FR-006**: System MUST format all retrieval results as valid JSON with consistent structure
- **FR-007**: System MUST include similarity score (0.0 to 1.0) for each returned result
- **FR-008**: System MUST handle edge cases gracefully (empty queries, no matches, API failures) and return appropriate error messages in JSON format
- **FR-009**: System MUST validate that retrieved chunks can be traced back to original source files via metadata
- **FR-010**: System MUST support batch testing with multiple predefined queries to validate consistency

### Key Entities

- **Query Embedding**: A 1024-dimensional vector representation of the user's query text, generated using Cohere embed-english-v3.0 model
- **Retrieval Result**: A single search result containing the chunk text, similarity score, and complete metadata
- **Similarity Score**: A float value between 0.0 and 1.0 representing cosine similarity between query and chunk embeddings
- **JSON Response**: Structured output containing query details, result count, and array of retrieval results with scores and metadata
- **Test Case**: A predefined query with expected results used to validate retrieval accuracy

## Success Criteria *(mandatory)*

The retrieval pipeline testing feature is successful when:

1. **Accuracy**: Queries about specific topics (ROS 2, Digital Twin, URDF) return relevant chunks in top 5 results with similarity scores above 0.7
2. **Data Integrity**: 100% of retrieved chunks match their original source text byte-for-byte with no corruption
3. **Metadata Completeness**: All retrieved results include valid url, page_title, chunk_index, and total_chunks fields
4. **Response Format**: All queries return valid, parseable JSON responses with consistent structure
5. **Performance**: Query embedding and retrieval complete in under 2 seconds for 95% of queries
6. **Edge Case Handling**: System handles empty queries, no-match scenarios, and API failures without crashes, returning appropriate error JSON
7. **Test Coverage**: Automated test suite covers at least 15 diverse queries spanning both modules and validates expected results
8. **Relevance Ranking**: For multi-topic queries, results are correctly ranked with most relevant chunks appearing first
9. **End-to-End Validation**: Complete pipeline from user query to JSON response works without manual intervention
10. **Traceability**: Every retrieved chunk's metadata URL points to the correct source documentation page

## Assumptions *(optional)*

- The website embedding pipeline (Spec-1) has completed successfully and Qdrant contains embedded documentation
- The same Cohere API key and model (embed-english-v3.0) will be used for query embedding as was used for document embedding
- The Qdrant collection name is `website_embeddings` and uses cosine distance metric
- Network connectivity to both Cohere API and Qdrant cloud instance is stable during testing
- Test queries will be in English matching the language of the documentation
- The existing `.env` file contains valid COHERE_API_KEY, QDRANT_URL, and QDRANT_API_KEY values

## Dependencies *(optional)*

- **Upstream Dependency**: 004-website-embedding feature must be completed with vectors stored in Qdrant
- **External Services**:
  - Cohere API (embed-english-v3.0 model) for query embedding
  - Qdrant cloud instance for vector storage and retrieval
- **Python Libraries**: cohere, qdrant-client, python-dotenv (already installed)

## Out of Scope *(optional)*

- Query rewriting or expansion (e.g., handling synonyms, spelling corrections)
- Caching of query results for performance optimization
- User interface or web API for retrieval (this is backend testing only)
- Multi-language query support (only English queries)
- Hybrid search combining vector and keyword search
- Re-ranking results using LLM or additional models
- Monitoring and alerting for production retrieval performance
- Authentication or authorization for retrieval access

## Constraints *(optional)*

- Must use the same embedding model (Cohere embed-english-v3.0) for query embedding as used in document embedding to ensure vector compatibility
- Retrieval results must be deterministic for the same query (same top-K results in same order)
- JSON output must be valid and parseable by standard JSON libraries
- Test queries must be representative of actual user questions about ROS 2 and Digital Twin topics
- Similarity score threshold of 0.7 is recommended minimum for considering a result relevant
