"""
Retrieval Service for RAG Agent

Provides a clean service layer abstraction over the retrieval pipeline.
Wraps retrieval.py functions with enhanced error handling and logging.

This service:
1. Embeds user queries using Cohere
2. Searches Qdrant for relevant documentation chunks
3. Handles errors gracefully
4. Provides detailed logging and metrics

Usage:
    from services.retrieval_service import RetrievalService

    retrieval_service = RetrievalService()
    results = retrieval_service.retrieve_context("How do I create a publisher?", top_k=5)
"""

import logging
from typing import List
from datetime import datetime

from retrieval import embed_query, search_qdrant, SearchResult
from exceptions import RetrievalError

logger = logging.getLogger('retrieval_service')


class RetrievalService:
    """
    Service for retrieving relevant documentation chunks.

    This service wraps the lower-level retrieval.py functions and provides:
    - Enhanced error handling with custom exceptions
    - Detailed logging with metrics (latency, relevance scores)
    - Clean abstraction for integration layer

    Methods:
        retrieve_context: Retrieve relevant chunks for a query
    """

    def __init__(self):
        """Initialize retrieval service."""
        logger.info("RetrievalService initialized")

    def retrieve_context(self, query: str, top_k: int = 5) -> List[SearchResult]:
        """
        Retrieve relevant documentation chunks for a query.

        This method:
        1. Embeds the query using Cohere (via retrieval.embed_query)
        2. Searches Qdrant for similar chunks (via retrieval.search_qdrant)
        3. Calculates and logs relevance metrics
        4. Handles errors with specific error messages

        Args:
            query: User's natural language question
            top_k: Number of chunks to retrieve (default 5)

        Returns:
            List of SearchResult objects with text, metadata, and relevance scores

        Raises:
            RetrievalError: If embedding fails (Cohere issue)
            RetrievalError: If vector search fails (Qdrant issue)

        Example:
            >>> service = RetrievalService()
            >>> results = service.retrieve_context("What is ROS 2?", top_k=5)
            >>> print(f"Retrieved {len(results)} chunks")
            >>> print(f"Top score: {results[0].score}")
        """
        start_time = datetime.utcnow()

        logger.info(
            f"Starting retrieval",
            extra={
                "query": query[:50] + "..." if len(query) > 50 else query,
                "top_k": top_k
            }
        )

        try:
            # Step 1: Embed query using Cohere
            logger.debug(f"Embedding query with Cohere")
            query_embedding = embed_query(query)

        except ValueError as e:
            # Handle validation errors from retrieval.py
            error_message = f"Query embedding failed: {str(e)}"
            logger.error(error_message, exc_info=True)
            raise RetrievalError(error_message)

        except Exception as e:
            # Handle Cohere API failures
            error_message = f"Query embedding failed: {str(e)}"
            logger.error(
                error_message,
                extra={"query": query, "error_type": type(e).__name__},
                exc_info=True
            )
            raise RetrievalError(error_message)

        try:
            # Step 2: Search Qdrant for similar chunks
            logger.debug(f"Searching Qdrant with top_k={top_k}")
            results = search_qdrant(query_embedding, top_k=top_k)

        except ValueError as e:
            # Handle validation errors from retrieval.py
            error_message = f"Vector database search failed: {str(e)}"
            logger.error(error_message, exc_info=True)
            raise RetrievalError(error_message)

        except Exception as e:
            # Handle Qdrant connection/query failures
            error_message = f"Vector database unavailable: {str(e)}"
            logger.error(
                error_message,
                extra={"query": query, "top_k": top_k, "error_type": type(e).__name__},
                exc_info=True
            )
            raise RetrievalError("Vector database unavailable")

        # Calculate metrics
        latency_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)
        retrieval_count = len(results)

        # Calculate average relevance score
        average_relevance_score = 0.0
        if retrieval_count > 0:
            average_relevance_score = sum(r.score for r in results) / retrieval_count

        # Log success with metrics
        logger.info(
            f"Retrieval completed successfully",
            extra={
                "query": query[:50] + "..." if len(query) > 50 else query,
                "top_k": top_k,
                "retrieval_count": retrieval_count,
                "average_relevance_score": round(average_relevance_score, 4),
                "retrieval_latency_ms": latency_ms
            }
        )

        return results
