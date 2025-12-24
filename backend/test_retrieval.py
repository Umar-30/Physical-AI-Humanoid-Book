"""
Command-Line Interface for Retrieval Pipeline Testing

Usage:
    python test_retrieval.py --query "What is ROS 2?"
    python test_retrieval.py --query "Unity integration" --top-k 10
    python test_retrieval.py --test-suite test_queries.json
    python test_retrieval.py --query "DDS explained" --output results.json
"""

import argparse
import json
import sys
import logging
from retrieval import (
    embed_query,
    search_qdrant,
    format_results,
    format_error,
    setup_logging
)

logger = logging.getLogger('test_retrieval')


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description='Test retrieval pipeline for RAG system'
    )
    parser.add_argument(
        '--query', '-q',
        type=str,
        help='Query text to search for'
    )
    parser.add_argument(
        '--top-k', '-k',
        type=int,
        default=5,
        help='Number of results to return (default: 5, max: 50)'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        help='Output file path (default: stdout)'
    )
    parser.add_argument(
        '--test-suite',
        type=str,
        help='Path to JSON file with test queries'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )
    return parser.parse_args()


def run_single_query(query: str, top_k: int, output_file: str = None) -> dict:
    """
    Execute a single query and return/save results.

    Args:
        query: Query text
        top_k: Number of results
        output_file: Optional output file path

    Returns:
        Results dict
    """
    try:
        # Embed query
        logger.info(f"Embedding query: {query[:50]}...")
        embedding = embed_query(query)

        # Search Qdrant
        logger.info(f"Searching Qdrant for top {top_k} results...")
        results = search_qdrant(embedding, top_k=top_k)

        # Format results
        formatted = format_results(query, results, top_k)

        # Output
        if output_file:
            with open(output_file, 'w') as f:
                json.dump(formatted, f, indent=2)
            logger.info(f"Results saved to {output_file}")
        else:
            print(json.dumps(formatted, indent=2))

        return formatted

    except Exception as e:
        logger.error(f"Query failed: {e}")
        error_response = format_error(e, query)

        if output_file:
            with open(output_file, 'w') as f:
                json.dump(error_response, f, indent=2)
        else:
            print(json.dumps(error_response, indent=2))

        return error_response


def run_test_suite(suite_file: str, output_file: str = None) -> dict:
    """
    Run multiple test queries from JSON file.

    Args:
        suite_file: Path to JSON file with test queries
        output_file: Optional output file for results

    Returns:
        Summary dict with all results
    """
    # Load test queries
    with open(suite_file, 'r') as f:
        test_suite = json.load(f)

    queries = test_suite.get('queries', [])
    top_k = test_suite.get('default_top_k', 5)

    logger.info(f"Running test suite with {len(queries)} queries...")

    results = []
    success_count = 0

    for i, test in enumerate(queries, 1):
        query_text = test.get('query', '')
        query_top_k = test.get('top_k', top_k)

        logger.info(f"[{i}/{len(queries)}] Testing: {query_text[:50]}...")

        result = run_single_query(query_text, query_top_k, output_file=None)

        results.append({
            'test_id': i,
            'query': query_text,
            'success': not result.get('error', False),
            'result_count': result.get('result_count', 0),
            'top_score': result['results'][0]['score'] if result.get('results') else 0.0
        })

        if not result.get('error'):
            success_count += 1

    # Create summary
    summary = {
        'test_suite': suite_file,
        'total_tests': len(queries),
        'successful': success_count,
        'failed': len(queries) - success_count,
        'success_rate': success_count / len(queries) if queries else 0,
        'results': results
    }

    # Output
    if output_file:
        with open(output_file, 'w') as f:
            json.dump(summary, f, indent=2)
        logger.info(f"Test results saved to {output_file}")
    else:
        print(json.dumps(summary, indent=2))

    return summary


def main():
    """Main entry point"""
    args = parse_args()
    setup_logging()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if args.query:
        result = run_single_query(args.query, args.top_k, args.output)
        sys.exit(0 if not result.get('error') else 2)
    elif args.test_suite:
        summary = run_test_suite(args.test_suite, args.output)
        sys.exit(0 if summary['failed'] == 0 else 2)
    else:
        print("Error: Must specify --query or --test-suite")
        sys.exit(1)


if __name__ == '__main__':
    main()
