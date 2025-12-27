"""
Custom Exceptions for RAG Agent API

Exception classes for different error types in the RAG pipeline.

Exceptions:
    RAGException: Base exception for all RAG Agent errors
    ValidationError: Request validation failures
    RetrievalError: Document retrieval failures
    GenerationError: Answer generation failures
    RateLimitError: API rate limit exceeded
"""


class RAGException(Exception):
    """
    Base exception for all RAG Agent errors.

    Attributes:
        message: Error message
        status_code: HTTP status code
        error_type: Error classification
    """

    def __init__(self, message: str, status_code: int = 500, error_type: str = "UnknownError"):
        """
        Initialize RAG exception.

        Args:
            message: Error message
            status_code: HTTP status code (default 500)
            error_type: Error classification (default "UnknownError")
        """
        self.message = message
        self.status_code = status_code
        self.error_type = error_type
        super().__init__(self.message)


class ValidationError(RAGException):
    """
    Exception for request validation failures.

    Raised when:
        - Query is empty or invalid
        - top_k is out of range
        - Model name is invalid

    Status Code: 422 Unprocessable Entity
    """

    def __init__(self, message: str):
        """
        Initialize validation error.

        Args:
            message: Validation error message
        """
        super().__init__(message, status_code=422, error_type="ValidationError")


class RetrievalError(RAGException):
    """
    Exception for document retrieval failures.

    Raised when:
        - Qdrant database is unavailable
        - Cohere API fails to embed query
        - Network errors during retrieval

    Status Code: 503 Service Unavailable
    """

    def __init__(self, message: str):
        """
        Initialize retrieval error.

        Args:
            message: Retrieval error message
        """
        super().__init__(message, status_code=503, error_type="RetrievalError")


class GenerationError(RAGException):
    """
    Exception for answer generation failures.

    Raised when:
        - OpenAI API is unavailable
        - OpenAI returns error response
        - Network errors during generation
        - Authentication failures

    Status Code: 502 Bad Gateway
    """

    def __init__(self, message: str):
        """
        Initialize generation error.

        Args:
            message: Generation error message
        """
        super().__init__(message, status_code=502, error_type="GenerationError")


class RateLimitError(RAGException):
    """
    Exception for API rate limit exceeded.

    Raised when:
        - OpenAI rate limit exceeded
        - Cohere rate limit exceeded
        - Qdrant rate limit exceeded

    Status Code: 429 Too Many Requests
    """

    def __init__(self, message: str):
        """
        Initialize rate limit error.

        Args:
            message: Rate limit error message
        """
        super().__init__(message, status_code=429, error_type="RateLimitError")
