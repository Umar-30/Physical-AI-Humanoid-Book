"""
Answer Generation Service for RAG Agent

Handles interaction with OpenAI Chat Completions API to generate natural language
answers from documentation context. Includes retry logic, error handling, and
comprehensive logging.

This service:
1. Calls OpenAI Chat Completions API
2. Implements retry logic with exponential backoff
3. Handles authentication and rate limit errors
4. Logs generation metrics (tokens, latency, completion length)
5. Returns structured results with answer and token usage

Usage:
    from services.generation_service import AnswerGenerationService

    service = AnswerGenerationService(api_key="your-key")
    result = service.generate_answer(prompt="<context>...</context>...", model="gpt-3.5-turbo")
    print(result['answer'], result['tokens_used'])
"""

import os
import logging
import time
from typing import Dict, Optional
from datetime import datetime
from openai import OpenAI

from exceptions import GenerationError, RateLimitError

logger = logging.getLogger('generation_service')


class AnswerGenerationService:
    """
    Service for generating natural language answers using OpenAI.

    This service manages all OpenAI API interactions with:
    - Retry logic for transient errors (429, 500, 502, 503)
    - Error handling for non-retryable errors (401, 400)
    - Comprehensive logging (model, tokens, latency)
    - Deterministic generation (low temperature)

    Attributes:
        openai_client: OpenAI client instance
        max_retries: Maximum retry attempts (default 3)

    Methods:
        generate_answer: Generate answer from prompt
    """

    def __init__(self, api_key: Optional[str] = None, max_retries: int = 3):
        """
        Initialize answer generation service.

        Args:
            api_key: OpenRouter API key (default: from env)
            max_retries: Maximum retry attempts for transient errors (default 3)

        Raises:
            ValueError: If API key not provided and not in environment
        """
        self.api_key = api_key or os.getenv("OPENROUTER_API_KEY")
        if not self.api_key:
            raise ValueError("OPENROUTER_API_KEY not found in environment")

        self.openai_client = OpenAI(
            api_key=self.api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        self.max_retries = max_retries

        logger.info(
            f"AnswerGenerationService initialized with OpenRouter",
            extra={"max_retries": max_retries, "base_url": "https://openrouter.ai/api/v1"}
        )

    def generate_answer(
        self,
        prompt: str,
        model: str = "xiaomi/mimo-v2-flash:free",
        temperature: float = 0.1,
        max_tokens: int = 500
    ) -> Dict[str, any]:
        """
        Generate natural language answer from prompt using OpenAI.

        Calls OpenAI Chat Completions API with retry logic for transient errors.
        Uses low temperature (0.1) for deterministic, factual responses.

        Args:
            prompt: Full prompt with context and query
            model: OpenAI model to use (default: gpt-3.5-turbo)
            temperature: Sampling temperature (default: 0.1 for deterministic)
            max_tokens: Maximum tokens in completion (default: 500)

        Returns:
            Dict with:
                - answer: Generated text response
                - tokens_used: Total tokens consumed (prompt + completion)
                - model_used: Actual model used
                - completion_length: Character length of answer

        Raises:
            GenerationError: If generation fails (non-retryable or max retries exceeded)
            RateLimitError: If rate limit persists after retries

        Example:
            >>> service = AnswerGenerationService()
            >>> result = service.generate_answer(
            ...     prompt="<context>ROS 2 docs...</context><query>What is ROS 2?</query>...",
            ...     model="gpt-3.5-turbo"
            ... )
            >>> print(result['answer'])
            >>> print(f"Used {result['tokens_used']} tokens")
        """
        start_time = datetime.utcnow()

        logger.info(
            f"Starting answer generation",
            extra={
                "model": model,
                "temperature": temperature,
                "max_tokens": max_tokens,
                "prompt_length": len(prompt)
            }
        )

        # Retry loop for transient errors
        last_error = None
        for attempt in range(self.max_retries):
            try:
                # Call OpenAI Chat Completions API
                response = self.openai_client.chat.completions.create(
                    model=model,
                    messages=[
                        {
                            "role": "system",
                            "content": "You are a helpful assistant that answers questions based on provided documentation. Always cite the documentation and be precise."
                        },
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ],
                    temperature=temperature,
                    max_tokens=max_tokens
                )

                # Extract answer and metrics
                answer = response.choices[0].message.content
                tokens_used = response.usage.total_tokens
                completion_length = len(answer)

                # Calculate latency
                latency_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)

                # Log success
                logger.info(
                    f"Answer generated successfully",
                    extra={
                        "model": model,
                        "tokens_used": tokens_used,
                        "generation_latency_ms": latency_ms,
                        "completion_length": completion_length,
                        "attempt": attempt + 1
                    }
                )

                return {
                    "answer": answer,
                    "tokens_used": tokens_used,
                    "model_used": model,
                    "completion_length": completion_length
                }

            except Exception as e:
                last_error = e
                error_message = str(e)
                error_type = type(e).__name__

                # Check for rate limit errors (429) - retryable
                if "rate_limit" in error_message.lower() or "429" in error_message:
                    if attempt < self.max_retries - 1:
                        # Calculate exponential backoff
                        wait_time = 2 ** attempt  # 1s, 2s, 4s
                        logger.warning(
                            f"Rate limit hit, retrying in {wait_time}s (attempt {attempt + 1}/{self.max_retries})",
                            extra={
                                "error_type": error_type,
                                "attempt": attempt + 1,
                                "wait_time": wait_time
                            }
                        )
                        time.sleep(wait_time)
                        continue
                    else:
                        # Max retries exceeded for rate limit
                        logger.error(
                            f"Rate limit persists after {self.max_retries} retries",
                            extra={"error_message": error_message}
                        )
                        raise RateLimitError("OpenAI rate limit exceeded. Please try again later.")

                # Check for server errors (500, 502, 503) - retryable
                if any(code in error_message for code in ["500", "502", "503"]):
                    if attempt < self.max_retries - 1:
                        wait_time = 2 ** attempt
                        logger.warning(
                            f"Server error, retrying in {wait_time}s (attempt {attempt + 1}/{self.max_retries})",
                            extra={
                                "error_type": error_type,
                                "error_message": error_message,
                                "attempt": attempt + 1,
                                "wait_time": wait_time
                            }
                        )
                        time.sleep(wait_time)
                        continue

                # Non-retryable errors (401 auth, 400 bad request, etc.)
                logger.error(
                    f"Generation failed with non-retryable error: {error_message}",
                    extra={
                        "error_type": error_type,
                        "model": model,
                        "attempt": attempt + 1
                    },
                    exc_info=True
                )
                raise GenerationError(f"Failed to generate answer: {error_message}")

        # If we exhausted all retries
        logger.error(
            f"Generation failed after {self.max_retries} retries",
            extra={"last_error": str(last_error)}
        )
        raise GenerationError(f"Failed to generate answer after {self.max_retries} retries: {str(last_error)}")
