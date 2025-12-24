"""
Request Models for RAG Agent API

Pydantic models for validating incoming API requests.

Models:
    QueryRequest: User query with optional parameters
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional


class QueryRequest(BaseModel):
    """
    Request model for POST /query endpoint.

    Attributes:
        query: User's natural language question (1-2000 characters)
        top_k: Number of documentation chunks to retrieve (1-50, default 5)
        model: OpenAI model to use (default 'gpt-3.5-turbo')

    Example:
        {
            "query": "How do I create a ROS 2 publisher in Python?",
            "top_k": 5,
            "model": "gpt-3.5-turbo"
        }
    """

    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's natural language question",
        examples=["How do I create a ROS 2 publisher in Python?"]
    )

    top_k: int = Field(
        default=5,
        ge=1,
        le=50,
        description="Number of documentation chunks to retrieve"
    )

    model: str = Field(
        default="xiaomi/mimo-v2-flash:free",
        description="LLM model to use for answer generation (OpenAI or OpenRouter)"
    )

    @field_validator('query')
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """
        Validate that query is not empty or whitespace only.

        Args:
            v: Query string to validate

        Returns:
            Stripped query string

        Raises:
            ValueError: If query is empty or whitespace only
        """
        if not v or not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()

    @field_validator('model')
    @classmethod
    def valid_model(cls, v: str) -> str:
        """
        Validate that model is in the allowed list.

        Args:
            v: Model name to validate

        Returns:
            Model name if valid

        Raises:
            ValueError: If model is not in allowed list
        """
        allowed_models = [
            "gpt-3.5-turbo",
            "gpt-4",
            "gpt-4-turbo-preview",
            "xiaomi/mimo-v2-flash:free"
        ]
        if v not in allowed_models:
            raise ValueError(f"Model must be one of {allowed_models}")
        return v

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "query": "How do I create a ROS 2 publisher in Python?",
                    "top_k": 5,
                    "model": "xiaomi/mimo-v2-flash:free"
                },
                {
                    "query": "What is a URDF file?",
                    "top_k": 10,
                    "model": "gpt-4"
                }
            ]
        }
    }
