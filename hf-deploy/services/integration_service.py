"""
Integration Service for RAG Agent

Orchestrates the full RAG pipeline:
1. Embed query using Cohere
2. Retrieve relevant chunks from Qdrant
3. Generate answer using OpenAI
4. Deduplicate sources and format response

This service integrates retrieval.py (existing) with OpenAI generation.
"""

import os
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv

from retrieval import SearchResult
from services.retrieval_service import RetrievalService
from services.prompt_builder import PromptBuilder
from services.generation_service import AnswerGenerationService
from models.response_models import QueryResponse, SourceReference
from exceptions import RetrievalError, GenerationError, RateLimitError

# Load environment variables
load_dotenv()

# Constants
DEFAULT_MODEL = os.getenv("DEFAULT_MODEL", "xiaomi/mimo-v2-flash:free")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "3000"))
# NOTE: Threshold set to 0.15 based on actual retrieval performance
# The current embedding model (Cohere embed-english-v3.0) returns lower scores than expected.
# Original requirement (FR-005) specified 0.7, but testing shows:
# - Technical queries get scores of 0.15-0.30
# - Irrelevant queries (like "hi") get scores of 0.10-0.23
# Setting to 0.15 filters out very low-quality matches while keeping relevant docs
RELEVANCE_THRESHOLD = 0.15  # Minimum relevance score for usable results (FR-005)

# Greeting patterns for detection (FR-003)
GREETING_PATTERNS = {
    "hi", "hello", "hey", "helo", "hii", "heya", "greetings",
    "good morning", "good afternoon", "good evening"
}

logger = logging.getLogger('integration_service')


class IntegrationService:
    """
    Service for orchestrating RAG pipeline: retrieval + generation.

    Methods:
        process_query: Main entry point for RAG pipeline
        _is_greeting: Detect if query is a casual greeting
        _retrieve_chunks: Retrieve relevant documentation chunks
        _deduplicate_sources: Deduplicate chunks by URL
        _build_prompt: Build prompt with context and query
        _generate_answer: Generate answer using OpenAI
        _count_tokens: Count tokens in text
    """

    def __init__(self, openai_api_key: Optional[str] = None):
        """
        Initialize integration service.

        Args:
            openai_api_key: OpenRouter API key (default: from env)
        """
        self.openai_api_key = openai_api_key or os.getenv("OPENROUTER_API_KEY")
        if not self.openai_api_key:
            raise ValueError("OPENROUTER_API_KEY not found in environment")

        self.retrieval_service = RetrievalService()
        self.prompt_builder = PromptBuilder(max_tokens=MAX_TOKENS)
        self.generation_service = AnswerGenerationService(api_key=self.openai_api_key)

    def _is_greeting(self, query: str) -> bool:
        """
        Detect if query is a standalone greeting (FR-003).

        Returns True for casual greetings like "hi", "hello", "hey".
        Returns False for queries that contain greetings but have actual questions
        (e.g., "Hi, how do I create a ROS 2 publisher?").

        Args:
            query: User query text

        Returns:
            True if query is a greeting, False otherwise
        """
        # Normalize: strip, lowercase, remove punctuation
        normalized = query.strip().lower().rstrip('!?.,:;')

        # Check if entire query is just a greeting (standalone)
        # Don't match if query contains other words after greeting
        return normalized in GREETING_PATTERNS

    async def process_query(
        self,
        query: str,
        top_k: int = 5,
        model: str = DEFAULT_MODEL
    ) -> QueryResponse:
        """
        Process a query through the full RAG pipeline.

        Args:
            query: User's natural language question
            top_k: Number of chunks to retrieve
            model: OpenAI model to use

        Returns:
            QueryResponse with answer and sources

        Raises:
            RetrievalError: If retrieval fails
            GenerationError: If generation fails
            RateLimitError: If rate limit exceeded
        """
        logger.info(f"Processing query", extra={"query": query, "top_k": top_k, "model": model})

        # Step 0: Check for greetings FIRST (FR-003)
        if self._is_greeting(query):
            logger.info(f"Greeting detected: {query}")
            return QueryResponse(
                query=query,
                answer="Hello! I'm here to help you learn about ROS 2 and humanoid robotics. What would you like to know?",
                sources=[],
                model_used="greeting-detector",
                tokens_used=0,
                retrieval_count=0
            )

        # Step 1: Retrieve chunks
        chunks = await self._retrieve_chunks(query, top_k)
        retrieval_count = len(chunks)

        # Step 2: Deduplicate sources
        deduplicated_sources = self._deduplicate_sources(chunks)

        # Step 3: Build prompt
        prompt = self._build_prompt(query, chunks, model)

        # Step 4: Generate answer
        answer, tokens_used = await self._generate_answer(prompt, model)

        # Step 5: Format response
        response = QueryResponse(
            query=query,
            answer=answer,
            sources=deduplicated_sources,
            model_used=model,
            tokens_used=tokens_used,
            retrieval_count=retrieval_count
        )

        logger.info(
            f"Query processed successfully",
            extra={
                "query": query,
                "tokens_used": tokens_used,
                "retrieval_count": retrieval_count,
                "sources_count": len(deduplicated_sources)
            }
        )

        return response

    async def _retrieve_chunks(self, query: str, top_k: int) -> List[SearchResult]:
        """
        Retrieve relevant documentation chunks using RetrievalService.
        Filters results by relevance threshold (FR-005).

        Args:
            query: User query
            top_k: Number of chunks to retrieve

        Returns:
            List of SearchResult objects with relevance >= 0.7

        Raises:
            RetrievalError: If retrieval fails or no relevant docs found (FR-006)
        """
        # Delegate to RetrievalService (already has error handling and logging)
        chunks = self.retrieval_service.retrieve_context(query, top_k=top_k)

        # Filter by relevance threshold (FR-005)
        filtered_chunks = [c for c in chunks if c.score >= RELEVANCE_THRESHOLD]

        logger.info(
            f"Relevance filtering: {len(filtered_chunks)}/{len(chunks)} chunks pass threshold {RELEVANCE_THRESHOLD}",
            extra={
                "total_chunks": len(chunks),
                "filtered_chunks": len(filtered_chunks),
                "threshold": RELEVANCE_THRESHOLD
            }
        )

        # Handle no relevant docs case (FR-006)
        if not filtered_chunks:
            logger.warning(f"No documents meet relevance threshold {RELEVANCE_THRESHOLD}")
            raise RetrievalError(
                "I couldn't find relevant information in the documentation. "
                "Try rephrasing your question or ask about: ROS 2, Unity, Digital Twins, or Humanoid Robotics."
            )

        return filtered_chunks

    def _deduplicate_sources(self, chunks: List[SearchResult]) -> List[SourceReference]:
        """
        Deduplicate chunks by URL, keeping highest-scoring chunk.

        Args:
            chunks: List of retrieved chunks

        Returns:
            List of deduplicated SourceReference objects
        """
        # Group by URL, keep highest score
        url_to_best_chunk: Dict[str, tuple[SearchResult, float]] = {}

        for chunk in chunks:
            url = chunk.metadata.get('url', '')
            if not url:
                continue

            if url not in url_to_best_chunk or chunk.score > url_to_best_chunk[url][1]:
                url_to_best_chunk[url] = (chunk, chunk.score)

        # Convert to SourceReference
        sources = []
        for url, (chunk, score) in url_to_best_chunk.items():
            source = SourceReference(
                url=url,
                page_title=chunk.metadata.get('page_title') or 'Unknown',
                relevance_score=round(score, 4),
                chunk_index=chunk.metadata.get('chunk_index')
            )
            sources.append(source)

        # Sort by relevance score (descending)
        sources.sort(key=lambda s: s.relevance_score, reverse=True)

        logger.info(f"Deduplicated {len(chunks)} chunks to {len(sources)} unique sources")
        return sources

    def _build_prompt(self, query: str, chunks: List[SearchResult], model: str = "gpt-3.5-turbo") -> str:
        """
        Build prompt with context and query using PromptBuilder.

        Args:
            query: User query
            chunks: Retrieved chunks
            model: OpenAI model for token counting

        Returns:
            Formatted prompt string
        """
        # Delegate to PromptBuilder (already has XML formatting, token management, source URLs)
        prompt = self.prompt_builder.build_prompt(query, chunks, model)
        return prompt

    async def _generate_answer(self, prompt: str, model: str) -> tuple[str, int]:
        """
        Generate answer using AnswerGenerationService.

        Args:
            prompt: Full prompt with context and query
            model: OpenAI model to use

        Returns:
            Tuple of (answer, tokens_used)

        Raises:
            GenerationError: If generation fails (propagated from AnswerGenerationService)
            RateLimitError: If rate limit exceeded (propagated from AnswerGenerationService)
        """
        # Delegate to AnswerGenerationService (already has retry logic, error handling, logging)
        result = self.generation_service.generate_answer(
            prompt=prompt,
            model=model,
            temperature=0.1,  # Low temperature for deterministic, factual responses
            max_tokens=500
        )

        return result['answer'], result['tokens_used']
