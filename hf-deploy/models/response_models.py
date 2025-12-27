"""
Response Models for RAG Agent API

Pydantic models for API responses.

Models:
    SourceReference: Reference to a documentation source
    QueryResponse: Successful query response with answer and sources
    ErrorResponse: Error response with details
"""

from pydantic import BaseModel, Field, HttpUrl
from typing import List, Optional


class SourceReference(BaseModel):
    """
    Reference to a documentation source used in answer generation.

    Attributes:
        url: URL of the source documentation page
        page_title: Human-readable title of the page
        relevance_score: Similarity score from vector search (0.0-1.0)
        chunk_index: Position of chunk within the page (optional)

    Example:
        {
            "url": "https://docs.ros.org/...",
            "page_title": "Writing a simple publisher",
            "relevance_score": 0.8642,
            "chunk_index": 2
        }
    """

    url: str = Field(
        ...,
        description="URL of the source documentation page"
    )

    page_title: str = Field(
        ...,
        min_length=1,
        description="Human-readable title of the documentation page"
    )

    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score from vector search (0.0-1.0, higher is more relevant)"
    )

    chunk_index: Optional[int] = Field(
        default=None,
        ge=0,
        description="Position of the chunk within the page (0-indexed)"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher.html",
                    "page_title": "Writing a simple publisher (Python)",
                    "relevance_score": 0.8642,
                    "chunk_index": 2
                }
            ]
        }
    }


class QueryResponse(BaseModel):
    """
    Successful response to a query request.

    Attributes:
        query: Original user query (echoed back)
        answer: Generated natural language answer
        sources: List of source references (deduplicated by URL)
        model_used: Actual OpenAI model used for generation
        tokens_used: Total tokens consumed (prompt + completion)
        retrieval_count: Number of chunks retrieved before deduplication

    Example:
        {
            "query": "How do I create a ROS 2 publisher?",
            "answer": "To create a ROS 2 publisher...",
            "sources": [...],
            "model_used": "gpt-3.5-turbo",
            "tokens_used": 456,
            "retrieval_count": 5
        }
    """

    query: str = Field(
        ...,
        description="Original user query (echoed back)"
    )

    answer: str = Field(
        ...,
        min_length=1,
        description="Generated natural language answer"
    )

    sources: List[SourceReference] = Field(
        ...,
        description="List of source references (deduplicated by URL)"
    )

    model_used: str = Field(
        ...,
        description="Actual OpenAI model used for generation"
    )

    tokens_used: int = Field(
        ...,
        ge=0,
        description="Total tokens consumed (prompt + completion)"
    )

    retrieval_count: int = Field(
        ...,
        ge=0,
        description="Number of chunks retrieved before deduplication"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "query": "How do I create a ROS 2 publisher in Python?",
                    "answer": "To create a ROS 2 publisher in Python, you need to import rclpy, create a node class, and use create_publisher()...",
                    "sources": [
                        {
                            "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher.html",
                            "page_title": "Writing a simple publisher (Python)",
                            "relevance_score": 0.8642,
                            "chunk_index": 2
                        }
                    ],
                    "model_used": "gpt-3.5-turbo",
                    "tokens_used": 456,
                    "retrieval_count": 5
                }
            ]
        }
    }


class ErrorResponse(BaseModel):
    """
    Error response for failed requests.

    Attributes:
        error: Error flag (always True for error responses)
        error_type: Error classification (ValidationError, RetrievalError, etc.)
        error_message: Human-readable error description
        query: Original query if available
        status_code: HTTP status code

    Example:
        {
            "error": true,
            "error_type": "ValidationError",
            "error_message": "Query cannot be empty",
            "query": "",
            "status_code": 422
        }
    """

    error: bool = Field(
        default=True,
        description="Error flag (always True for error responses)"
    )

    error_type: str = Field(
        ...,
        min_length=1,
        description="Error classification"
    )

    error_message: str = Field(
        ...,
        min_length=1,
        description="Human-readable error description"
    )

    query: Optional[str] = Field(
        default=None,
        description="Original query if available"
    )

    status_code: int = Field(
        ...,
        ge=400,
        le=599,
        description="HTTP status code"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": True,
                    "error_type": "ValidationError",
                    "error_message": "Query cannot be empty or whitespace only",
                    "query": "",
                    "status_code": 422
                },
                {
                    "error": True,
                    "error_type": "RetrievalError",
                    "error_message": "Vector database is currently unavailable",
                    "query": "How do I create a publisher?",
                    "status_code": 503
                }
            ]
        }
    }
