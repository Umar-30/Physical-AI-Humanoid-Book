# Data Model: RAG Agent API Service

**Feature**: 002-rag-agent
**Created**: 2025-12-21
**Purpose**: Define data structures, validation rules, and entity relationships

## Entity Definitions

### 1. QueryRequest

**Purpose**: User input for question answering

**Fields**:

| Field | Type | Required | Default | Validation | Description |
|-------|------|----------|---------|------------|-------------|
| `query` | string | Yes | - | 1-2000 chars, non-empty | User's natural language question |
| `top_k` | integer | No | 5 | 1-50 | Number of documentation chunks to retrieve |
| `model` | string | No | "gpt-3.5-turbo" | Valid OpenAI model name | LLM model to use for generation |

**Validation Rules**:
- `query` must not be empty or whitespace-only
- `query` length must be between 1 and 2000 characters
- `top_k` must be between 1 and 50 (inclusive)
- `model` must be one of: `gpt-3.5-turbo`, `gpt-4`, `gpt-4-turbo-preview`

**Example**:
```json
{
  "query": "How do I create a ROS 2 publisher in Python?",
  "top_k": 5,
  "model": "gpt-3.5-turbo"
}
```

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator

class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    top_k: int = Field(5, ge=1, le=50, description="Number of chunks to retrieve")
    model: str = Field("gpt-3.5-turbo", description="OpenAI model to use")

    @validator('query')
    def query_not_empty(cls, v):
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace")
        return v.strip()

    @validator('model')
    def valid_model(cls, v):
        allowed = ["gpt-3.5-turbo", "gpt-4", "gpt-4-turbo-preview"]
        if v not in allowed:
            raise ValueError(f"Model must be one of {allowed}")
        return v
```

---

### 2. SourceReference

**Purpose**: Reference to source documentation used in answer generation

**Fields**:

| Field | Type | Required | Default | Validation | Description |
|-------|------|----------|---------|------------|-------------|
| `url` | string | Yes | - | Valid URL | Documentation page URL |
| `page_title` | string | Yes | - | Non-empty | Human-readable page title |
| `relevance_score` | float | Yes | - | 0.0-1.0 | Similarity score from vector search |
| `chunk_index` | integer | No | null | >= 0 | Position of chunk within page |

**Validation Rules**:
- `url` must be valid HTTP/HTTPS URL
- `page_title` must not be empty
- `relevance_score` must be between 0.0 and 1.0 (inclusive)
- `chunk_index` must be non-negative if provided

**Example**:
```json
{
  "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher.html",
  "page_title": "Writing a simple publisher (Python)",
  "relevance_score": 0.8642,
  "chunk_index": 2
}
```

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, HttpUrl
from typing import Optional

class SourceReference(BaseModel):
    url: HttpUrl = Field(..., description="Source documentation URL")
    page_title: str = Field(..., min_length=1, description="Page title")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score")
    chunk_index: Optional[int] = Field(None, ge=0, description="Chunk position in page")
```

---

### 3. QueryResponse

**Purpose**: Complete response to user query with answer and sources

**Fields**:

| Field | Type | Required | Default | Validation | Description |
|-------|------|----------|---------|------------|-------------|
| `query` | string | Yes | - | - | Original user query (echoed back) |
| `answer` | string | Yes | - | Non-empty | Generated natural language answer |
| `sources` | array[SourceReference] | Yes | - | - | List of source references (deduplicated) |
| `model_used` | string | Yes | - | - | Actual model used for generation |
| `tokens_used` | integer | Yes | - | >= 0 | Total tokens consumed (prompt + completion) |
| `retrieval_count` | integer | Yes | - | >= 0 | Number of chunks retrieved before deduplication |

**Validation Rules**:
- `answer` must not be empty
- `sources` may be empty array if no relevant results found
- `tokens_used` must be non-negative
- `retrieval_count` must be non-negative

**Example**:
```json
{
  "query": "How do I create a ROS 2 publisher in Python?",
  "answer": "To create a ROS 2 publisher in Python, you need to:\n\n1. Import the rclpy library\n2. Create a node class that inherits from Node\n3. In the constructor, call create_publisher() with your message type and topic name\n4. Use the publish() method to send messages\n\nHere's a basic example:\n```python\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass MinimalPublisher(Node):\n    def __init__(self):\n        super().__init__('minimal_publisher')\n        self.publisher_ = self.create_publisher(String, 'topic', 10)\n```\n\nFor complete working code, see the sources below.",
  "sources": [
    {
      "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher.html",
      "page_title": "Writing a simple publisher (Python)",
      "relevance_score": 0.8642,
      "chunk_index": 2
    },
    {
      "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace.html",
      "page_title": "Creating a workspace",
      "relevance_score": 0.7234,
      "chunk_index": 5
    }
  ],
  "model_used": "gpt-3.5-turbo",
  "tokens_used": 456,
  "retrieval_count": 5
}
```

**Pydantic Model**:
```python
from pydantic import BaseModel, Field
from typing import List

class QueryResponse(BaseModel):
    query: str = Field(..., description="Original user query")
    answer: str = Field(..., min_length=1, description="Generated answer")
    sources: List[SourceReference] = Field(..., description="Source references")
    model_used: str = Field(..., description="Model used for generation")
    tokens_used: int = Field(..., ge=0, description="Total tokens consumed")
    retrieval_count: int = Field(..., ge=0, description="Number of chunks retrieved")
```

---

### 4. ErrorResponse

**Purpose**: Standardized error response for failures

**Fields**:

| Field | Type | Required | Default | Validation | Description |
|-------|------|----------|---------|------------|-------------|
| `error` | boolean | Yes | true | Must be true | Error flag |
| `error_type` | string | Yes | - | Non-empty | Error classification (e.g., "ValidationError", "RetrievalError") |
| `error_message` | string | Yes | - | Non-empty | Human-readable error description |
| `query` | string | No | null | - | Original query if available |
| `status_code` | integer | Yes | - | 400-599 | HTTP status code |

**Validation Rules**:
- `error` must always be `true` for error responses
- `error_type` must not be empty
- `error_message` must provide actionable information
- `status_code` must be valid HTTP error code (400-599)

**Example**:
```json
{
  "error": true,
  "error_type": "RetrievalError",
  "error_message": "Vector database is currently unavailable. Please try again later.",
  "query": "How do I create a ROS 2 publisher?",
  "status_code": 503
}
```

**Pydantic Model**:
```python
from pydantic import BaseModel, Field
from typing import Optional

class ErrorResponse(BaseModel):
    error: bool = Field(True, description="Error flag")
    error_type: str = Field(..., min_length=1, description="Error classification")
    error_message: str = Field(..., min_length=1, description="Error description")
    query: Optional[str] = Field(None, description="Original query if available")
    status_code: int = Field(..., ge=400, le=599, description="HTTP status code")
```

---

## Entity Relationships

```
QueryRequest
    │
    ├─> (processed by) RetrievalService
    │       │
    │       └─> SearchResult[] (from Qdrant)
    │
    ├─> (processed by) AnswerGenerationService
    │       │
    │       └─> GeneratedAnswer (from OpenAI)
    │
    └─> QueryResponse
            │
            ├─> answer: string
            ├─> sources: SourceReference[]
            ├─> model_used: string
            └─> metadata: (tokens, retrieval_count)
```

**Data Flow**:
1. Client sends `QueryRequest` to API
2. API validates request using Pydantic
3. Retrieval service searches Qdrant → `SearchResult[]`
4. Source deduplication → `SourceReference[]`
5. Context builder prepares prompt from results
6. Answer generation service calls OpenAI → `GeneratedAnswer`
7. Response builder creates `QueryResponse`
8. API returns JSON to client

**Error Flow**:
1. Validation error → `ErrorResponse` with 422 status
2. Retrieval error → `ErrorResponse` with 503 status
3. Generation error → `ErrorResponse` with 502 status
4. Rate limit → `ErrorResponse` with 429 status

---

## State Transitions

### QueryRequest Lifecycle

```
[Created]
    ↓
[Validated] → (invalid) → [Rejected: 422 ErrorResponse]
    ↓
[Retrieving Context] → (retrieval fails) → [Failed: 503 ErrorResponse]
    ↓
[Context Retrieved]
    ↓
[Generating Answer] → (generation fails) → [Failed: 502 ErrorResponse]
    ↓
[Answer Generated]
    ↓
[Response Built]
    ↓
[Returned: 200 QueryResponse]
```

**States**:
- **Created**: Initial request received
- **Validated**: Pydantic validation passed
- **Rejected**: Validation failed (400-level error)
- **Retrieving Context**: Calling Qdrant for chunks
- **Context Retrieved**: Got search results from Qdrant
- **Generating Answer**: Calling OpenAI for completion
- **Answer Generated**: Got LLM response
- **Response Built**: Final response constructed
- **Returned**: Sent to client
- **Failed**: Error occurred (500-level error)

---

## Validation Summary

All entities have:
- ✅ Complete field definitions
- ✅ Type specifications
- ✅ Validation rules
- ✅ Pydantic models for runtime validation
- ✅ Example JSON payloads
- ✅ Relationship definitions

No ambiguous or undefined fields remain. Ready for API contract generation.
