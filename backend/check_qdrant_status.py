#!/usr/bin/env python3
"""
Quick check script to test Qdrant connection and configuration
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Get environment variables
QDRANT_URL = os.getenv("QDRANT_URL", "localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

print("Qdrant Configuration Check")
print("=" * 40)
print(f"QDRANT_URL: {QDRANT_URL}")
print(f"QDRANT_API_KEY: {'Set' if QDRANT_API_KEY else 'Not set (None)'}")

# Connect to Qdrant
try:
    if QDRANT_URL == "localhost:6333":
        print("Connecting to local Qdrant...")
        client = QdrantClient(host="localhost", port=6333)
    elif QDRANT_API_KEY:
        print("Connecting to remote Qdrant with API key...")
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        print("Connecting to remote Qdrant without API key...")
        client = QdrantClient(url=QDRANT_URL)

    print("✓ Successfully connected to Qdrant!")

    # Check collections
    collections = client.get_collections()
    print(f"\nFound {len(collections.collections)} collection(s):")
    for collection in collections.collections:
        collection_info = client.get_collection(collection.name)
        print(f"  - {collection.name}: {collection_info.points_count} points")

    # Check for expected collections
    expected_collections = ["rag_embedding", "website_embeddings"]
    found_expected = [name for name in expected_collections if any(c.name == name for c in collections.collections)]

    if found_expected:
        print(f"\n✓ Found expected collection(s): {found_expected}")
        for col_name in found_expected:
            col_info = client.get_collection(col_name)
            if col_info.points_count > 0:
                print(f"  - {col_name}: ✓ Has {col_info.points_count} points (ready for queries)")
            else:
                print(f"  - {col_name}: ⚠ Empty (needs embedding)")
    else:
        print(f"\n⚠ Expected collections {expected_collections} not found. You need to run the embedding pipeline first.")

except Exception as e:
    print(f"\n✗ Error connecting to Qdrant: {e}")
    print("\nPossible solutions:")
    print("1. Make sure Qdrant is running (try: docker run -p 6333:6333 qdrant/qdrant)")
    print("2. Check your QDRANT_URL in .env file")
    print("3. If using remote Qdrant, make sure QDRANT_API_KEY is set correctly")

print("\nDone!")