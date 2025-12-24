"""
Token Counter Utility

Provides token counting functionality using tiktoken library.
Used for managing OpenAI API token limits.

Functions:
    count_tokens: Count tokens in text using specified model encoding
"""

import tiktoken
from typing import Optional


def count_tokens(text: str, model: str = "gpt-3.5-turbo") -> int:
    """
    Count tokens in text using tiktoken for specified model.

    This function uses the tiktoken library to accurately count tokens
    as OpenAI's API would count them. This is essential for:
    - Staying within context limits
    - Estimating API costs
    - Managing prompt sizes

    Args:
        text: Text to count tokens in
        model: OpenAI model name (default: "gpt-3.5-turbo")
               Supported models: gpt-3.5-turbo, gpt-4, gpt-4-turbo-preview

    Returns:
        Number of tokens in the text

    Raises:
        ValueError: If model encoding is not available

    Example:
        >>> count_tokens("Hello, world!")
        4
        >>> count_tokens("How do I create a ROS 2 publisher?")
        10
        >>> count_tokens("Long text...", model="gpt-4")
        ...
    """
    try:
        encoder = tiktoken.encoding_for_model(model)
    except KeyError:
        # Fallback to cl100k_base encoding (used by gpt-3.5-turbo and gpt-4)
        encoder = tiktoken.get_encoding("cl100k_base")

    tokens = encoder.encode(text)
    return len(tokens)
