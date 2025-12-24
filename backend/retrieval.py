"""
Retrieval Pipeline Module

Provides functions for querying embedded documentation in Qdrant.

Functions:
    embed_query: Generate query embedding using Cohere
    search_qdrant: Search for similar chunks in Qdrant
    format_results: Format search results as JSON
    format_error: Format errors as JSON response
    setup_logging: Configure structured logging

Example:
    >>> from retrieval import embed_query, search_qdrant
    >>> embedding = embed_query("What is ROS 2?")
    >>> results = search_qdrant(embedding, top_k=5)
"""

import os
import logging
from dataclasses import dataclass
from typing import List, Dict, Any
from dotenv import load_dotenv
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from pathlib import Path

# Load environment variables - explicit path for uvicorn compatibility
_env_file = Path(__file__).parent / '.env'
load_dotenv(dotenv_path=_env_file, override=True)

# Constants
MODEL = "embed-english-v3.0"
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "website_embeddings")
DEFAULT_TOP_K = 5
MAX_TOP_K = 50

# Logging setup
def setup_logging():
    """Configure structured logging"""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

logger = logging.getLogger('retrieval')


@dataclass
class SearchResult:
    """Single search result from Qdrant"""
    rank: int
    score: float
    text: str
    metadata: Dict[str, Any]


def embed_query(query_text: str) -> List[float]:
    """
    Generate embedding vector for query text using Cohere.

    Args:
        query_text: Natural language query string

    Returns:
        1024-dimensional embedding vector

    Raises:
        ValueError: If query_text is empty or None
        CohereAPIError: If API call fails
    """
    # Validation
    if not query_text or not query_text.strip():
        raise ValueError("Query text cannot be empty")

    if len(query_text) > 2000:
        raise ValueError("Query text too long (max 2000 characters)")

    # Initialize Cohere client
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        raise ValueError("COHERE_API_KEY not found in environment")

    client = CohereClient(api_key=api_key)

    # Log query embedding
    logger.info(f"Embedding query: {query_text[:50]}...")

    # Generate embedding
    response = client.embed(
        texts=[query_text],
        model=MODEL,
        input_type='search_query'  # Important: different from 'search_document'
    )

    return response.embeddings[0]


def search_qdrant(
    query_embedding: List[float],
    top_k: int = DEFAULT_TOP_K,
    collection_name: str = COLLECTION_NAME
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
        ValueError: If top_k < 1 or > 50, or embedding dimension wrong
        QdrantException: If search fails
    """
    # Validation
    if not isinstance(query_embedding, list) or len(query_embedding) != 1024:
        raise ValueError("query_embedding must be list of 1024 floats")

    if not 1 <= top_k <= MAX_TOP_K:
        raise ValueError(f"top_k must be between 1 and {MAX_TOP_K}")

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL required")

    # For local Qdrant instances, API key is not required
    # Only require API key if using a remote Qdrant instance
    if qdrant_url != "localhost:6333" and qdrant_url.startswith("http") and not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY required for remote Qdrant instances")

    # Initialize Qdrant client based on URL type
    if qdrant_url == "localhost:6333":
        # For local instance, use host and port
        client = QdrantClient(host="localhost", port=6333)
    elif qdrant_api_key:
        # For remote instance with API key
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        # For remote instance without API key
        client = QdrantClient(url=qdrant_url)

    # Verify collection exists
    try:
        collection_info = client.get_collection(collection_name)
        if collection_info.points_count == 0:
            raise ValueError(f"Collection '{collection_name}' is empty")
    except Exception as e:
        raise ValueError(f"Collection '{collection_name}' not found or inaccessible: {e}")

    # Log search
    logger.info(f"Searching Qdrant for top {top_k} results...")

    # Perform search
    search_results = client.query_points(
        collection_name=collection_name,
        query=query_embedding,
        limit=top_k,
        with_payload=True,
        with_vectors=False
    )

    # Convert to SearchResult objects
    results = []
    for rank, hit in enumerate(search_results.points, start=1):
        result = SearchResult(
            rank=rank,
            score=hit.score,
            text=hit.payload.get('text', ''),
            metadata={
                'url': hit.payload.get('url', ''),
                'page_title': hit.payload.get('page_title', ''),
                'chunk_index': hit.payload.get('chunk_index', -1),
                'total_chunks': hit.payload.get('total_chunks', -1),
                'source': hit.payload.get('source', ''),
                'file_path': hit.payload.get('file_path', '')
            }
        )
        results.append(result)

    logger.info(f"Retrieved {len(results)} results")
    return results


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
        Dict conforming to response schema
    """
    logger.info("Formatting results as JSON")

    return {
        "query": query,
        "top_k": top_k,
        "result_count": len(results),
        "results": [
            {
                "rank": r.rank,
                "score": round(r.score, 4),  # Round to 4 decimal places
                "text": r.text,
                "metadata": {
                    "url": r.metadata['url'],
                    "page_title": r.metadata['page_title'],
                    "chunk_index": r.metadata['chunk_index'],
                    "total_chunks": r.metadata['total_chunks'],
                    "source": r.metadata['source']
                }
            }
            for r in results
        ]
    }


def format_error(error: Exception, query: str = "") -> dict:
    """
    Format error as JSON response.

    Args:
        error: Exception that occurred
        query: Original query text (if available)

    Returns:
        Error dict conforming to response schema
    """
    logger.error(f"Error occurred: {type(error).__name__}: {str(error)}")

    return {
        "error": True,
        "error_type": type(error).__name__,
        "error_message": str(error),
        "query": query,
        "result_count": 0,
        "results": []
    }
