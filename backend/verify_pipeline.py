#!/usr/bin/env python3
"""
Pipeline Verification Script

Tests the website embedding pipeline end-to-end with a sample URL.
Validates that:
- Text extraction works
- Chunking produces valid chunks
- Embeddings are generated with correct dimensions
- All components work together

Usage:
    python verify_pipeline.py [--url URL]
"""

import sys
import argparse
from typing import List
from dotenv import load_dotenv
import os

# Import pipeline functions
from main import (
    extract_text_from_url,
    chunk_text,
    embed,
    log
)
from cohere import Client as CohereClient

# Load environment variables
load_dotenv()


def verify_text_extraction(url: str) -> bool:
    """
    Verify that text can be extracted from a URL.

    Args:
        url: URL to test

    Returns:
        True if extraction successful, False otherwise
    """
    log("INFO", "VERIFY", f"Testing text extraction from {url}")

    text = extract_text_from_url(url)

    if not text:
        log("ERROR", "VERIFY", "✗ Text extraction failed - no content returned")
        return False

    if len(text) < 100:
        log("WARNING", "VERIFY", f"✗ Text extraction returned very short content ({len(text)} chars)")
        return False

    log("INFO", "VERIFY", f"✓ Text extraction successful ({len(text)} chars, {len(text.split())} words)")
    return True


def verify_chunking(text: str, chunk_size: int = 512) -> bool:
    """
    Verify that text chunking works correctly.

    Args:
        text: Text to chunk
        chunk_size: Target chunk size in words

    Returns:
        True if chunking successful, False otherwise
    """
    log("INFO", "VERIFY", f"Testing chunking with chunk_size={chunk_size}")

    try:
        chunks = chunk_text(text, chunk_size)

        if not chunks:
            log("ERROR", "VERIFY", "✗ Chunking failed - no chunks returned")
            return False

        # Validate chunk sizes
        for i, chunk in enumerate(chunks):
            word_count = len(chunk.split())
            if word_count < 1:
                log("ERROR", "VERIFY", f"✗ Chunk {i} is empty")
                return False
            if word_count > chunk_size + 100:  # Allow some overflow for word boundaries
                log("WARNING", "VERIFY", f"✗ Chunk {i} exceeds size limit ({word_count} words)")

        avg_size = sum(len(c.split()) for c in chunks) / len(chunks)
        log("INFO", "VERIFY", f"✓ Chunking successful ({len(chunks)} chunks, avg {avg_size:.0f} words/chunk)")
        return True

    except Exception as e:
        log("ERROR", "VERIFY", f"✗ Chunking failed with error: {str(e)}")
        return False


def verify_embeddings(chunks: List[str], cohere_client: CohereClient) -> bool:
    """
    Verify that embeddings can be generated.

    Args:
        chunks: Text chunks to embed
        cohere_client: Cohere API client

    Returns:
        True if embedding generation successful, False otherwise
    """
    log("INFO", "VERIFY", f"Testing embedding generation for {len(chunks)} chunks")

    # Test with first 3 chunks to save API calls
    test_chunks = chunks[:min(3, len(chunks))]

    try:
        embeddings = embed(test_chunks, cohere_client)

        if not embeddings:
            log("ERROR", "VERIFY", "✗ Embedding generation failed - no embeddings returned")
            return False

        if len(embeddings) != len(test_chunks):
            log("ERROR", "VERIFY", f"✗ Embedding count mismatch: expected {len(test_chunks)}, got {len(embeddings)}")
            return False

        # Validate embedding dimensions
        for i, embedding in enumerate(embeddings):
            if not isinstance(embedding, list):
                log("ERROR", "VERIFY", f"✗ Embedding {i} is not a list")
                return False

            if len(embedding) != 1024:
                log("ERROR", "VERIFY", f"✗ Embedding {i} has incorrect dimensions: {len(embedding)} (expected 1024)")
                return False

            # Check that embedding contains valid floats
            if not all(isinstance(x, (int, float)) for x in embedding[:10]):
                log("ERROR", "VERIFY", f"✗ Embedding {i} contains non-numeric values")
                return False

        log("INFO", "VERIFY", f"✓ Embedding generation successful ({len(embeddings)} embeddings, 1024-dim each)")
        return True

    except Exception as e:
        log("ERROR", "VERIFY", f"✗ Embedding generation failed with error: {str(e)}")
        return False


def verify_chunk_size_validation() -> bool:
    """
    Verify that chunk size validation works.

    Returns:
        True if validation works correctly, False otherwise
    """
    log("INFO", "VERIFY", "Testing chunk size validation")

    test_text = "word " * 1000  # 1000 words

    # Test invalid chunk sizes
    invalid_sizes = [100, 255, 1025, 2000]

    for size in invalid_sizes:
        try:
            chunk_text(test_text, chunk_size=size)
            log("ERROR", "VERIFY", f"✗ Validation failed - chunk_size={size} should have raised ValueError")
            return False
        except ValueError:
            pass  # Expected

    # Test valid chunk sizes
    valid_sizes = [256, 512, 1024]

    for size in valid_sizes:
        try:
            chunks = chunk_text(test_text, chunk_size=size)
            if not chunks:
                log("ERROR", "VERIFY", f"✗ Validation failed - chunk_size={size} returned no chunks")
                return False
        except ValueError:
            log("ERROR", "VERIFY", f"✗ Validation failed - chunk_size={size} should be valid")
            return False

    log("INFO", "VERIFY", "✓ Chunk size validation working correctly")
    return True


def main():
    """
    Run all pipeline verification tests.
    """
    parser = argparse.ArgumentParser(description="Verify website embedding pipeline")
    parser.add_argument(
        "--url",
        type=str,
        default="https://umar-30.github.io/Physical-AI-Humanoid-Book/",
        help="URL to test (default: Physical AI book homepage)"
    )
    args = parser.parse_args()

    log("INFO", "VERIFY", "=" * 60)
    log("INFO", "VERIFY", "Website Embedding Pipeline Verification")
    log("INFO", "VERIFY", "=" * 60)

    # Check environment variables
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        log("ERROR", "VERIFY", "COHERE_API_KEY environment variable not set")
        log("ERROR", "VERIFY", "Please set it in your .env file")
        sys.exit(1)

    # Initialize Cohere client
    try:
        cohere_client = CohereClient(api_key=cohere_api_key)
    except Exception as e:
        log("ERROR", "VERIFY", f"Failed to initialize Cohere client: {str(e)}")
        sys.exit(1)

    # Run tests
    results = {}

    # Test 1: Chunk size validation
    results["chunk_validation"] = verify_chunk_size_validation()

    # Test 2: Text extraction
    results["text_extraction"] = verify_text_extraction(args.url)

    if not results["text_extraction"]:
        log("ERROR", "VERIFY", "Skipping remaining tests due to text extraction failure")
    else:
        # Extract text for subsequent tests
        text = extract_text_from_url(args.url)

        # Test 3: Chunking
        results["chunking"] = verify_chunking(text)

        if results["chunking"]:
            # Test 4: Embeddings
            chunks = chunk_text(text)
            results["embeddings"] = verify_embeddings(chunks, cohere_client)
        else:
            results["embeddings"] = False
            log("ERROR", "VERIFY", "Skipping embedding test due to chunking failure")

    # Print summary
    log("INFO", "VERIFY", "=" * 60)
    log("INFO", "VERIFY", "Verification Summary")
    log("INFO", "VERIFY", "=" * 60)

    all_passed = True
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        log("INFO", "VERIFY", f"{test_name:20s}: {status}")
        if not passed:
            all_passed = False

    log("INFO", "VERIFY", "=" * 60)

    if all_passed:
        log("INFO", "VERIFY", "✓ All verification tests PASSED")
        sys.exit(0)
    else:
        log("ERROR", "VERIFY", "✗ Some verification tests FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
