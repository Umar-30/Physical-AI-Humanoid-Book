#!/usr/bin/env python3
"""
Check what's stored in Qdrant
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "localhost:6333")

# Connect to Qdrant
if QDRANT_URL == "localhost:6333":
    client = QdrantClient(host="localhost", port=6333)
else:
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Check collections
print("=" * 60)
print("Checking Qdrant Collections")
print("=" * 60)

try:
    collections = client.get_collections()
    print(f"\nFound {len(collections.collections)} collection(s):")
    for collection in collections.collections:
        print(f"  - {collection.name}")
except Exception as e:
    print(f"Error getting collections: {e}")
    exit(1)

# Check website_embeddings collection
collection_name = "website_embeddings"
print(f"\n{'=' * 60}")
print(f"Checking '{collection_name}' collection")
print(f"{'=' * 60}")

try:
    collection_info = client.get_collection(collection_name)
    print(f"\nCollection Info:")
    print(f"  - Vector size: {collection_info.config.params.vectors.size}")
    print(f"  - Distance: {collection_info.config.params.vectors.distance}")
    print(f"  - Points count: {collection_info.points_count}")
    print(f"  - Segments count: {collection_info.segments_count}")

    if collection_info.points_count > 0:
        print(f"\n{'=' * 60}")
        print("Sample Points (first 3)")
        print(f"{'=' * 60}")

        # Get some sample points
        points = client.scroll(
            collection_name=collection_name,
            limit=3,
            with_payload=True,
            with_vectors=False
        )[0]

        for i, point in enumerate(points, 1):
            print(f"\nPoint {i} (ID: {point.id}):")
            print(f"  URL: {point.payload.get('url', 'N/A')}")
            print(f"  Chunk Index: {point.payload.get('chunk_index', 'N/A')}")
            print(f"  Text Preview: {point.payload.get('text', '')[:100]}...")
    else:
        print("\n⚠️  WARNING: Collection exists but has 0 points!")
        print("Data was not stored successfully.")

except Exception as e:
    print(f"Error: {e}")
    exit(1)

print(f"\n{'=' * 60}")
print("Done!")
print(f"{'=' * 60}")
