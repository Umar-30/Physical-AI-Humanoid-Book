"""
Prompt Builder Service for RAG Agent

Combines retrieved documentation chunks with user queries into well-formatted
prompts for OpenAI. Handles token management, XML formatting, and source attribution.

This service:
1. Formats context with XML delimiters
2. Manages token budgets to stay within API limits
3. Selects top-scoring chunks that fit
4. Includes source metadata for attribution
5. Adds system instructions for grounded answers

Usage:
    from services.prompt_builder import PromptBuilder

    builder = PromptBuilder(max_tokens=3000)
    prompt = builder.build_prompt(query="What is ROS 2?", chunks=results)
"""

import logging
from typing import List
from retrieval import SearchResult
from utils.token_counter import count_tokens

logger = logging.getLogger('prompt_builder')


class PromptBuilder:
    """
    Service for building prompts with context and query.

    This service creates well-formatted prompts that:
    - Use XML tags (<context>, <query>) for clear structure
    - Stay within token limits
    - Include source URLs for attribution
    - Provide system instructions for grounded answers

    Attributes:
        max_tokens: Maximum tokens for entire prompt (default 3000)
        reserved_tokens: Tokens reserved for query + instructions (default 500)

    Methods:
        build_prompt: Build formatted prompt from query and chunks
    """

    def __init__(self, max_tokens: int = 3000, reserved_tokens: int = 500):
        """
        Initialize prompt builder.

        Args:
            max_tokens: Maximum tokens for entire prompt (default 3000)
            reserved_tokens: Tokens reserved for query/instructions (default 500)
        """
        self.max_tokens = max_tokens
        self.reserved_tokens = reserved_tokens
        self.context_budget = max_tokens - reserved_tokens

        logger.info(
            f"PromptBuilder initialized",
            extra={
                "max_tokens": max_tokens,
                "reserved_tokens": reserved_tokens,
                "context_budget": self.context_budget
            }
        )

    def build_prompt(self, query: str, chunks: List[SearchResult], model: str = "gpt-3.5-turbo") -> str:
        """
        Build formatted prompt with context and query.

        Creates a prompt with the following structure:
        ```
        <context>
        [chunk 1 text]
        Source: [url 1]

        [chunk 2 text]
        Source: [url 2]
        ...
        </context>

        <query>
        [user query]
        </query>

        Please answer the query using ONLY the information provided...
        ```

        The method:
        1. Selects chunks that fit within token budget
        2. Formats each chunk with source URL
        3. Wraps in XML tags
        4. Adds system instructions

        Args:
            query: User's natural language question
            chunks: Retrieved documentation chunks (sorted by relevance)
            model: OpenAI model for token counting (default: gpt-3.5-turbo)

        Returns:
            Formatted prompt string ready for OpenAI API

        Example:
            >>> builder = PromptBuilder(max_tokens=3000)
            >>> prompt = builder.build_prompt("What is ROS 2?", chunks)
            >>> print(prompt[:100])
            <context>
            ROS 2 is a set of software libraries and tools...
        """
        logger.info(
            f"Building prompt",
            extra={
                "query": query[:50] + "..." if len(query) > 50 else query,
                "total_chunks": len(chunks),
                "model": model
            }
        )

        # Select chunks that fit within budget (T036)
        selected_chunks = self._select_chunks_within_budget(chunks, model)

        # Format context with chunks and source URLs (T037)
        context = self._format_context_with_sources(selected_chunks)

        # Build full prompt with XML template (T034)
        prompt = self._format_prompt_with_xml(context, query)

        # Count final tokens
        final_tokens = count_tokens(prompt, model)

        logger.info(
            f"Prompt built successfully",
            extra={
                "query": query[:50] + "..." if len(query) > 50 else query,
                "selected_chunks": len(selected_chunks),
                "total_chunks": len(chunks),
                "final_tokens": final_tokens,
                "max_tokens": self.max_tokens
            }
        )

        return prompt

    def _select_chunks_within_budget(
        self,
        chunks: List[SearchResult],
        model: str
    ) -> List[SearchResult]:
        """
        Select top-scoring chunks that fit within token budget.

        Iterates through chunks (already sorted by relevance) and adds
        them to context until budget is exhausted. Prioritizes higher-scoring
        chunks when space is limited.

        Args:
            chunks: Retrieved chunks sorted by relevance (highest first)
            model: Model for token counting

        Returns:
            List of chunks that fit within context_budget
        """
        selected = []
        total_tokens = 0

        for chunk in chunks:
            # Format chunk with source for accurate token count
            chunk_text = chunk.text
            source_url = chunk.metadata.get('url', '')
            formatted_chunk = f"{chunk_text}\nSource: {source_url}\n"

            # Count tokens for this chunk
            chunk_tokens = count_tokens(formatted_chunk, model)

            # Check if adding this chunk would exceed budget
            if total_tokens + chunk_tokens > self.context_budget:
                logger.warning(
                    f"Hit token budget, truncating context at {len(selected)} chunks",
                    extra={
                        "selected_chunks": len(selected),
                        "total_chunks": len(chunks),
                        "current_tokens": total_tokens,
                        "context_budget": self.context_budget
                    }
                )
                break

            selected.append(chunk)
            total_tokens += chunk_tokens

        logger.debug(
            f"Selected {len(selected)} chunks using {total_tokens} tokens"
        )

        return selected

    def _format_context_with_sources(self, chunks: List[SearchResult]) -> str:
        """
        Format chunks with source URLs.

        Each chunk is formatted as:
        ```
        [chunk text]
        Source: [url]
        ```

        Args:
            chunks: Selected chunks to format

        Returns:
            Formatted context string with sources
        """
        context_parts = []

        for chunk in chunks:
            chunk_text = chunk.text
            source_url = chunk.metadata.get('url', 'Unknown')

            # Format: chunk text + source URL
            formatted_chunk = f"{chunk_text}\nSource: {source_url}"
            context_parts.append(formatted_chunk)

        # Join with blank line between chunks
        context = "\n\n".join(context_parts)

        return context

    def _format_prompt_with_xml(self, context: str, query: str) -> str:
        """
        Format prompt with XML tags and system instructions.

        Creates the final prompt structure:
        ```
        <context>
        [formatted context with sources]
        </context>

        <query>
        [user query]
        </query>

        [system instructions]
        ```

        Args:
            context: Formatted context with chunks and sources
            query: User query

        Returns:
            Complete formatted prompt
        """
        # Build prompt with XML delimiters (T034)
        prompt = f"""<context>
{context}
</context>

<query>
{query}
</query>

Please answer the query using ONLY the information provided in the context above. If the context doesn't contain enough information to answer the query, say so. Be concise and cite specific details from the context."""

        return prompt
