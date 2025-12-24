#!/usr/bin/env python3
"""
RAG Document Ingestion Pipeline

This script ingests documents from a data/ directory into Qdrant vector database
for use with the RAG chatbot.

Required Environment Variables:
    COHERE_API_KEY   - Cohere API key for generating embeddings
    QDRANT_URL       - Qdrant instance URL (e.g., https://xxx.cloud.qdrant.io:6333)
    QDRANT_API_KEY   - Qdrant API key (required for cloud instances)

Optional Environment Variables:
    COLLECTION_NAME  - Qdrant collection name (default: rag_embedding)

Usage:
    python ingest.py                    # Ingest all files from data/ directory
    python ingest.py --reset            # Delete existing collection and re-ingest
    python ingest.py --dir ./my_docs    # Ingest from custom directory

Supported file types: .txt, .md, .pdf
"""

import os
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime
from dataclasses import dataclass

from dotenv import load_dotenv
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct

# Load environment variables
load_dotenv()

# =============================================================================
# CONFIGURATION
# =============================================================================

# Environment variables
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "rag_embedding")

# Embedding configuration
EMBEDDING_MODEL = "embed-english-v3.0"  # Cohere model
EMBEDDING_DIMENSION = 1024              # embed-english-v3.0 produces 1024-dim vectors

# Chunking configuration
CHUNK_SIZE = 500          # Target words per chunk
CHUNK_OVERLAP = 50        # Overlap words between chunks
MAX_CHUNK_SIZE = 1000     # Maximum words per chunk

# Batch configuration
EMBEDDING_BATCH_SIZE = 96  # Cohere recommends max 96 texts per batch
UPSERT_BATCH_SIZE = 100    # Qdrant upsert batch size


# =============================================================================
# LOGGING
# =============================================================================

def log(level: str, component: str, message: str):
    """Structured logging with timestamps."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] [{component}] {message}")


# =============================================================================
# DOCUMENT LOADING
# =============================================================================

@dataclass
class Document:
    """Represents a loaded document."""
    content: str
    metadata: Dict[str, Any]
    file_path: str
    file_type: str


def load_text_file(file_path: Path) -> str:
    """Load content from a text or markdown file."""
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        return f.read()


def load_pdf_file(file_path: Path) -> str:
    """Load content from a PDF file using PyMuPDF."""
    try:
        import fitz  # PyMuPDF
        doc = fitz.open(file_path)
        text_parts = []
        for page_num, page in enumerate(doc):
            text = page.get_text()
            if text.strip():
                text_parts.append(f"[Page {page_num + 1}]\n{text}")
        doc.close()
        return "\n\n".join(text_parts)
    except ImportError:
        log("WARNING", "LOAD", f"PyMuPDF not installed. Skipping PDF: {file_path}")
        log("INFO", "LOAD", "Install with: pip install pymupdf")
        return ""
    except Exception as e:
        log("ERROR", "LOAD", f"Error loading PDF {file_path}: {e}")
        return ""


def load_documents(directory: str) -> List[Document]:
    """
    Load all supported documents from a directory.

    Args:
        directory: Path to directory containing documents

    Returns:
        List of Document objects
    """
    documents = []
    dir_path = Path(directory)

    if not dir_path.exists():
        log("ERROR", "LOAD", f"Directory not found: {directory}")
        return documents

    # Supported file extensions
    supported_extensions = {'.txt', '.md', '.pdf'}

    # Find all supported files
    files = []
    for ext in supported_extensions:
        files.extend(dir_path.rglob(f"*{ext}"))

    log("INFO", "LOAD", f"Found {len(files)} files in {directory}")

    for file_path in files:
        try:
            # Load content based on file type
            ext = file_path.suffix.lower()

            if ext in ['.txt', '.md']:
                content = load_text_file(file_path)
            elif ext == '.pdf':
                content = load_pdf_file(file_path)
            else:
                continue

            if not content.strip():
                log("WARNING", "LOAD", f"Empty content in: {file_path}")
                continue

            # Create document with metadata
            doc = Document(
                content=content,
                metadata={
                    'source': str(file_path),
                    'file_name': file_path.name,
                    'file_type': ext,
                    'ingested_at': datetime.now().isoformat()
                },
                file_path=str(file_path),
                file_type=ext
            )
            documents.append(doc)
            log("INFO", "LOAD", f"Loaded: {file_path.name} ({len(content)} chars)")

        except Exception as e:
            log("ERROR", "LOAD", f"Error loading {file_path}: {e}")

    log("INFO", "LOAD", f"Successfully loaded {len(documents)} documents")
    return documents


# =============================================================================
# TEXT CHUNKING
# =============================================================================

@dataclass
class Chunk:
    """Represents a text chunk for embedding."""
    text: str
    metadata: Dict[str, Any]
    chunk_index: int
    total_chunks: int


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
    """
    Split text into overlapping chunks.

    Uses word-based chunking with overlap for better context preservation.

    Args:
        text: Input text to chunk
        chunk_size: Target words per chunk
        overlap: Number of overlapping words between chunks

    Returns:
        List of text chunks
    """
    if not text or not text.strip():
        return []

    # Split into words
    words = text.split()

    if len(words) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(words):
        # Get chunk
        end = min(start + chunk_size, len(words))
        chunk_words = words[start:end]
        chunk_text = ' '.join(chunk_words)

        if chunk_text.strip():
            chunks.append(chunk_text)

        # Move start with overlap
        start = end - overlap if end < len(words) else end

        # Prevent infinite loop
        if start >= len(words):
            break

    return chunks


def create_chunks(documents: List[Document]) -> List[Chunk]:
    """
    Create chunks from all documents with metadata.

    Args:
        documents: List of Document objects

    Returns:
        List of Chunk objects with metadata
    """
    all_chunks = []

    for doc in documents:
        text_chunks = chunk_text(doc.content)
        total = len(text_chunks)

        for idx, text in enumerate(text_chunks):
            chunk = Chunk(
                text=text,
                metadata={
                    **doc.metadata,
                    'chunk_index': idx,
                    'total_chunks': total,
                    'url': doc.metadata.get('source', ''),
                    'page_title': doc.metadata.get('file_name', 'Unknown')
                },
                chunk_index=idx,
                total_chunks=total
            )
            all_chunks.append(chunk)

    log("INFO", "CHUNK", f"Created {len(all_chunks)} chunks from {len(documents)} documents")
    return all_chunks


# =============================================================================
# EMBEDDING GENERATION
# =============================================================================

def generate_embeddings(texts: List[str], cohere_client: CohereClient) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere.

    Args:
        texts: List of text strings to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors (1024 dimensions each)
    """
    try:
        response = cohere_client.embed(
            texts=texts,
            model=EMBEDDING_MODEL,
            input_type="search_document"  # Use search_document for indexing
        )
        return response.embeddings
    except Exception as e:
        log("ERROR", "EMBED", f"Embedding generation failed: {e}")
        raise


def embed_chunks(chunks: List[Chunk], cohere_client: CohereClient) -> List[tuple]:
    """
    Generate embeddings for all chunks in batches.

    Args:
        chunks: List of Chunk objects
        cohere_client: Initialized Cohere client

    Returns:
        List of (chunk, embedding) tuples
    """
    results = []
    total_batches = (len(chunks) + EMBEDDING_BATCH_SIZE - 1) // EMBEDDING_BATCH_SIZE

    for i in range(0, len(chunks), EMBEDDING_BATCH_SIZE):
        batch = chunks[i:i + EMBEDDING_BATCH_SIZE]
        batch_num = i // EMBEDDING_BATCH_SIZE + 1

        log("INFO", "EMBED", f"Processing batch {batch_num}/{total_batches} ({len(batch)} chunks)")

        texts = [chunk.text for chunk in batch]
        embeddings = generate_embeddings(texts, cohere_client)

        for chunk, embedding in zip(batch, embeddings):
            results.append((chunk, embedding))

    log("INFO", "EMBED", f"Generated {len(results)} embeddings")
    return results


# =============================================================================
# QDRANT STORAGE
# =============================================================================

def create_qdrant_client() -> QdrantClient:
    """Create and return a Qdrant client based on environment configuration."""
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL environment variable is required")

    # Local instance
    if QDRANT_URL == "localhost:6333":
        return QdrantClient(host="localhost", port=6333)

    # Remote instance (Qdrant Cloud)
    if not QDRANT_API_KEY:
        raise ValueError("QDRANT_API_KEY is required for remote Qdrant instances")

    return QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)


def ensure_collection(qdrant_client: QdrantClient, reset: bool = False):
    """
    Ensure the Qdrant collection exists with correct configuration.

    Args:
        qdrant_client: Initialized Qdrant client
        reset: If True, delete and recreate the collection
    """
    try:
        collections = qdrant_client.get_collections()
        collection_exists = any(c.name == COLLECTION_NAME for c in collections.collections)

        if collection_exists:
            if reset:
                log("INFO", "QDRANT", f"Deleting existing collection: {COLLECTION_NAME}")
                qdrant_client.delete_collection(COLLECTION_NAME)
            else:
                # Check if collection has correct dimensions
                info = qdrant_client.get_collection(COLLECTION_NAME)
                log("INFO", "QDRANT", f"Collection exists with {info.points_count} points")
                return

        # Create new collection
        log("INFO", "QDRANT", f"Creating collection: {COLLECTION_NAME}")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=Distance.COSINE
            )
        )
        log("INFO", "QDRANT", f"Collection created successfully (dim={EMBEDDING_DIMENSION}, distance=COSINE)")

    except Exception as e:
        log("ERROR", "QDRANT", f"Error managing collection: {e}")
        raise


def store_embeddings(
    qdrant_client: QdrantClient,
    chunk_embedding_pairs: List[tuple]
):
    """
    Store chunk embeddings in Qdrant.

    Args:
        qdrant_client: Initialized Qdrant client
        chunk_embedding_pairs: List of (Chunk, embedding) tuples
    """
    points = []

    for idx, (chunk, embedding) in enumerate(chunk_embedding_pairs):
        # Generate unique point ID
        point_id = abs(hash(chunk.metadata['source'] + str(chunk.chunk_index))) % (2**63)

        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                'text': chunk.text,
                'url': chunk.metadata.get('url', ''),
                'page_title': chunk.metadata.get('page_title', 'Unknown'),
                'source': chunk.metadata.get('source', ''),
                'file_name': chunk.metadata.get('file_name', ''),
                'file_type': chunk.metadata.get('file_type', ''),
                'chunk_index': chunk.chunk_index,
                'total_chunks': chunk.total_chunks,
                'ingested_at': chunk.metadata.get('ingested_at', '')
            }
        )
        points.append(point)

    # Upsert in batches
    total_batches = (len(points) + UPSERT_BATCH_SIZE - 1) // UPSERT_BATCH_SIZE

    for i in range(0, len(points), UPSERT_BATCH_SIZE):
        batch = points[i:i + UPSERT_BATCH_SIZE]
        batch_num = i // UPSERT_BATCH_SIZE + 1

        log("INFO", "QDRANT", f"Upserting batch {batch_num}/{total_batches} ({len(batch)} points)")

        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )

    log("INFO", "QDRANT", f"Successfully stored {len(points)} points in collection '{COLLECTION_NAME}'")


# =============================================================================
# MAIN INGESTION PIPELINE
# =============================================================================

def validate_environment():
    """Validate required environment variables."""
    errors = []

    if not COHERE_API_KEY:
        errors.append("COHERE_API_KEY is not set")

    if not QDRANT_URL:
        errors.append("QDRANT_URL is not set")

    if QDRANT_URL and QDRANT_URL != "localhost:6333" and not QDRANT_API_KEY:
        errors.append("QDRANT_API_KEY is required for remote Qdrant instances")

    if errors:
        log("ERROR", "CONFIG", "Missing required environment variables:")
        for err in errors:
            log("ERROR", "CONFIG", f"  - {err}")
        log("INFO", "CONFIG", "\nRequired environment variables:")
        log("INFO", "CONFIG", "  COHERE_API_KEY   - Get from https://dashboard.cohere.com/api-keys")
        log("INFO", "CONFIG", "  QDRANT_URL       - Your Qdrant instance URL")
        log("INFO", "CONFIG", "  QDRANT_API_KEY   - Your Qdrant API key (for cloud)")
        sys.exit(1)


def run_ingestion(data_dir: str = "data", reset: bool = False):
    """
    Run the complete ingestion pipeline.

    Args:
        data_dir: Directory containing documents to ingest
        reset: If True, delete and recreate the collection
    """
    log("INFO", "MAIN", "=" * 60)
    log("INFO", "MAIN", "RAG Document Ingestion Pipeline")
    log("INFO", "MAIN", "=" * 60)

    # Step 1: Validate environment
    log("INFO", "MAIN", "Step 1: Validating environment...")
    validate_environment()
    log("INFO", "MAIN", f"  Collection: {COLLECTION_NAME}")
    log("INFO", "MAIN", f"  Qdrant URL: {QDRANT_URL[:50]}...")

    # Step 2: Load documents
    log("INFO", "MAIN", f"\nStep 2: Loading documents from '{data_dir}'...")
    documents = load_documents(data_dir)

    if not documents:
        log("ERROR", "MAIN", f"No documents found in '{data_dir}'")
        log("INFO", "MAIN", f"Please add .txt, .md, or .pdf files to the '{data_dir}' directory")
        sys.exit(1)

    # Step 3: Create chunks
    log("INFO", "MAIN", "\nStep 3: Chunking documents...")
    chunks = create_chunks(documents)

    if not chunks:
        log("ERROR", "MAIN", "No chunks created from documents")
        sys.exit(1)

    # Step 4: Initialize clients
    log("INFO", "MAIN", "\nStep 4: Initializing clients...")
    cohere_client = CohereClient(api_key=COHERE_API_KEY)
    qdrant_client = create_qdrant_client()

    # Step 5: Ensure collection exists
    log("INFO", "MAIN", "\nStep 5: Setting up Qdrant collection...")
    ensure_collection(qdrant_client, reset=reset)

    # Step 6: Generate embeddings
    log("INFO", "MAIN", "\nStep 6: Generating embeddings...")
    chunk_embedding_pairs = embed_chunks(chunks, cohere_client)

    # Step 7: Store in Qdrant
    log("INFO", "MAIN", "\nStep 7: Storing vectors in Qdrant...")
    store_embeddings(qdrant_client, chunk_embedding_pairs)

    # Verify
    log("INFO", "MAIN", "\nStep 8: Verifying ingestion...")
    info = qdrant_client.get_collection(COLLECTION_NAME)
    log("INFO", "MAIN", f"  Collection '{COLLECTION_NAME}' now has {info.points_count} points")

    log("INFO", "MAIN", "\n" + "=" * 60)
    log("INFO", "MAIN", "Ingestion complete! Your chatbot is now ready to answer questions.")
    log("INFO", "MAIN", "=" * 60)


# =============================================================================
# CLI ENTRY POINT
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Ingest documents into Qdrant for RAG chatbot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python ingest.py                     # Ingest from data/ directory
  python ingest.py --dir ./docs        # Ingest from custom directory
  python ingest.py --reset             # Delete collection and re-ingest
  python ingest.py --dir ./docs --reset

Environment Variables:
  COHERE_API_KEY   - Required for embeddings
  QDRANT_URL       - Qdrant instance URL
  QDRANT_API_KEY   - Required for Qdrant Cloud
  COLLECTION_NAME  - Optional (default: rag_embedding)
        """
    )

    parser.add_argument(
        '--dir', '-d',
        default='data',
        help='Directory containing documents to ingest (default: data)'
    )

    parser.add_argument(
        '--reset', '-r',
        action='store_true',
        help='Delete existing collection and re-ingest all documents'
    )

    args = parser.parse_args()

    try:
        run_ingestion(data_dir=args.dir, reset=args.reset)
    except KeyboardInterrupt:
        log("INFO", "MAIN", "\nIngestion cancelled by user")
        sys.exit(1)
    except Exception as e:
        log("ERROR", "MAIN", f"Ingestion failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
