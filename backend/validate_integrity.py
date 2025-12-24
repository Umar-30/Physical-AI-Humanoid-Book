"""
Validate that retrieved chunks match original embedded text.
"""
import json
import os
from retrieval import embed_query, search_qdrant
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()


def validate_chunk_integrity(chunk_id: int, expected_text: str) -> bool:
    """
    Validate a specific chunk matches expected text.

    Args:
        chunk_id: Qdrant point ID
        expected_text: Expected chunk text

    Returns:
        True if match, False otherwise
    """
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Retrieve specific point
    point = client.retrieve(
        collection_name="website_embeddings",
        ids=[chunk_id],
        with_payload=True
    )[0]

    actual_text = point.payload.get('text', '')

    return actual_text == expected_text


def sample_validation_test():
    """
    Run sample validation on multiple retrieved chunks.
    """
    # Sample queries
    test_queries = [
        "What is DDS in ROS 2?",
        "How to create a publisher?",
        "Unity integration"
    ]

    results = []

    for query in test_queries:
        print(f"\nTesting: {query}")

        # Retrieve chunks
        embedding = embed_query(query)
        chunks = search_qdrant(embedding, top_k=3)

        for chunk in chunks:
            # Verify text is not empty
            if not chunk.text:
                results.append({
                    'query': query,
                    'rank': chunk.rank,
                    'issue': 'Empty text',
                    'passed': False
                })
                continue

            # Verify metadata URL is valid
            if not chunk.metadata['url'].startswith('http'):
                results.append({
                    'query': query,
                    'rank': chunk.rank,
                    'issue': 'Invalid URL',
                    'passed': False
                })
                continue

            # Verify chunk index consistency
            if chunk.metadata['chunk_index'] < 0 or \
               chunk.metadata['chunk_index'] >= chunk.metadata['total_chunks']:
                results.append({
                    'query': query,
                    'rank': chunk.rank,
                    'issue': 'Inconsistent chunk indices',
                    'passed': False
                })
                continue

            results.append({
                'query': query,
                'rank': chunk.rank,
                'url': chunk.metadata['url'],
                'chunk_index': chunk.metadata['chunk_index'],
                'text_length': len(chunk.text),
                'passed': True
            })

    # Summary
    passed = sum(1 for r in results if r['passed'])
    total = len(results)

    print(f"\n{'='*60}")
    print(f"Integrity Validation Results")
    print(f"{'='*60}")
    print(f"Total chunks validated: {total}")
    print(f"Passed: {passed}")
    print(f"Failed: {total - passed}")
    print(f"Success rate: {passed/total*100:.1f}%")

    # Show failures
    failures = [r for r in results if not r['passed']]
    if failures:
        print(f"\nFailures:")
        for f in failures:
            print(f"  - {f['query']} (rank {f['rank']}): {f['issue']}")

    return results


if __name__ == '__main__':
    sample_validation_test()
