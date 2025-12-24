#!/usr/bin/env python3
"""
Local Documentation Embedding Pipeline
Processes local markdown files and embeds them into Qdrant
"""
import os
import re
from pathlib import Path
from typing import List, Dict, Tuple
from datetime import datetime
import time

from dotenv import load_dotenv
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http import models
import tiktoken

# Load environment variables
load_dotenv()

# Configuration
DOCS_PATH = Path("..") / "docs"
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "website_embeddings"
BASE_URL = "https://umar-30.github.io/Physical-AI-Humanoid-Book"

# Chunking configuration
TARGET_TOKENS = 600  # ~600 tokens per chunk
OVERLAP_TOKENS = 100  # Overlap between chunks


def log(level: str, component: str, message: str):
    """Structured logging with timestamps"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] [{component}] {message}")


def count_tokens(text: str) -> int:
    """Count tokens in text using tiktoken (GPT tokenizer as approximation)"""
    try:
        encoding = tiktoken.get_encoding("cl100k_base")
        return len(encoding.encode(text))
    except:
        # Fallback: rough approximation
        return len(text.split()) * 1.3


def extract_text_from_markdown(file_path: Path) -> Tuple[str, str]:
    """
    Extract clean text from markdown file.

    Returns:
        Tuple of (title, clean_text)
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract title from first heading or filename
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if title_match:
            title = title_match.group(1).strip()
        else:
            title = file_path.stem.replace('-', ' ').title()

        # Remove frontmatter (YAML between ---)
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Remove code blocks but keep their content
        content = re.sub(r'```[\w]*\n(.*?)\n```', r'\1', content, flags=re.DOTALL)

        # Remove HTML tags
        content = re.sub(r'<[^>]+>', '', content)

        # Remove markdown links but keep text [text](url) -> text
        content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

        # Remove markdown images
        content = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', content)

        # Remove markdown formatting
        content = re.sub(r'[*_`#]', '', content)

        # Clean up whitespace
        content = re.sub(r'\n\s*\n', '\n\n', content)
        content = content.strip()

        return title, content

    except Exception as e:
        log("ERROR", "EXTRACT", f"Failed to extract text from {file_path}: {str(e)}")
        return "", ""


def chunk_text_with_overlap(text: str, target_tokens: int = TARGET_TOKENS, overlap_tokens: int = OVERLAP_TOKENS) -> List[str]:
    """
    Chunk text into roughly equal-sized chunks with overlap.

    Args:
        text: Text to chunk
        target_tokens: Target tokens per chunk
        overlap_tokens: Tokens to overlap between chunks

    Returns:
        List of text chunks
    """
    # Split into sentences
    sentences = re.split(r'(?<=[.!?])\s+', text)

    chunks = []
    current_chunk = []
    current_tokens = 0

    for sentence in sentences:
        sentence_tokens = count_tokens(sentence)

        if current_tokens + sentence_tokens > target_tokens and current_chunk:
            # Save current chunk
            chunk_text = ' '.join(current_chunk)
            chunks.append(chunk_text)

            # Start new chunk with overlap
            overlap_sentences = []
            overlap_token_count = 0

            for sent in reversed(current_chunk):
                sent_tokens = count_tokens(sent)
                if overlap_token_count + sent_tokens <= overlap_tokens:
                    overlap_sentences.insert(0, sent)
                    overlap_token_count += sent_tokens
                else:
                    break

            current_chunk = overlap_sentences
            current_tokens = overlap_token_count

        current_chunk.append(sentence)
        current_tokens += sentence_tokens

    # Add final chunk
    if current_chunk:
        chunks.append(' '.join(current_chunk))

    return chunks


def find_all_docs(docs_path: Path) -> List[Tuple[Path, str]]:
    """
    Find all markdown documentation files.

    Returns:
        List of (file_path, url) tuples
    """
    docs = []

    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith(('.md', '.mdx')) and file != 'DOCUSAURUS_RESEARCH_SUMMARY.md':
                file_path = Path(root) / file

                # Generate URL path
                rel_path = file_path.relative_to(docs_path)
                url_path = str(rel_path).replace('\\', '/').replace('.mdx', '').replace('.md', '')

                # Handle index files
                if url_path.endswith('/index'):
                    url_path = url_path[:-6]

                # Create full URL
                if url_path:
                    url = f"{BASE_URL}/docs/{url_path}"
                else:
                    url = f"{BASE_URL}/docs"

                docs.append((file_path, url))

    return sorted(docs)


def embed_documents():
    """Main pipeline to embed local documentation"""

    log("INFO", "INIT", "Starting local documentation embedding pipeline")
    log("INFO", "INIT", f"Docs path: {DOCS_PATH.resolve()}")

    # Initialize clients
    log("INFO", "INIT", "Initializing Cohere client...")
    cohere_client = CohereClient(api_key=COHERE_API_KEY)

    log("INFO", "INIT", "Initializing Qdrant client...")
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Delete old collection if exists
    try:
        collections = qdrant_client.get_collections()
        if any(c.name == COLLECTION_NAME for c in collections.collections):
            log("INFO", "QDRANT", f"Deleting existing collection: {COLLECTION_NAME}")
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
            log("INFO", "QDRANT", "Collection deleted successfully")
    except Exception as e:
        log("WARNING", "QDRANT", f"Error checking/deleting collection: {str(e)}")

    # Create new collection
    log("INFO", "QDRANT", f"Creating collection: {COLLECTION_NAME}")
    qdrant_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=1024,  # Cohere embed-english-v3.0 dimension
            distance=models.Distance.COSINE
        )
    )
    log("INFO", "QDRANT", "Collection created successfully")

    # Find all documentation files
    log("INFO", "DISCOVER", "Discovering documentation files...")
    docs = find_all_docs(DOCS_PATH)
    log("INFO", "DISCOVER", f"Found {len(docs)} documentation files")

    # Process each document
    total_chunks = 0
    total_pages_processed = 0
    total_pages_failed = 0

    for idx, (file_path, url) in enumerate(docs, 1):
        log("INFO", "PROCESS", f"[{idx}/{len(docs)}] Processing: {file_path.name}")

        try:
            # Extract text
            title, text = extract_text_from_markdown(file_path)

            if not text:
                log("WARNING", "EXTRACT", f"No text extracted from {file_path.name}")
                total_pages_failed += 1
                continue

            # Chunk text
            chunks = chunk_text_with_overlap(text)
            log("INFO", "CHUNK", f"Created {len(chunks)} chunks from {file_path.name}")

            # Generate embeddings in batches
            batch_size = 96  # Cohere API limit
            for batch_idx in range(0, len(chunks), batch_size):
                batch_chunks = chunks[batch_idx:batch_idx + batch_size]

                try:
                    # Generate embeddings
                    response = cohere_client.embed(
                        texts=batch_chunks,
                        model='embed-english-v3.0',
                        input_type='search_document'
                    )

                    embeddings = response.embeddings

                    # Prepare points for Qdrant
                    points = []
                    for chunk_idx, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                        global_chunk_idx = batch_idx + chunk_idx

                        point = models.PointStruct(
                            id=total_chunks + global_chunk_idx,
                            vector=embedding,
                            payload={
                                "url": url,
                                "page_title": title,
                                "chunk_index": global_chunk_idx,
                                "total_chunks": len(chunks),
                                "text": chunk,
                                "source": "website",
                                "file_path": str(file_path.relative_to(DOCS_PATH))
                            }
                        )
                        points.append(point)

                    # Upload to Qdrant
                    qdrant_client.upsert(
                        collection_name=COLLECTION_NAME,
                        points=points
                    )

                    log("INFO", "STORE", f"Stored batch {batch_idx // batch_size + 1} ({len(batch_chunks)} chunks)")

                    # Small delay to respect API rate limits
                    time.sleep(0.5)

                except Exception as e:
                    log("ERROR", "EMBED", f"Failed to embed/store batch: {str(e)}")
                    total_pages_failed += 1
                    continue

            total_chunks += len(chunks)
            total_pages_processed += 1

        except Exception as e:
            log("ERROR", "PROCESS", f"Failed to process {file_path.name}: {str(e)}")
            total_pages_failed += 1
            continue

    # Summary
    log("INFO", "SUMMARY", "=" * 60)
    log("INFO", "SUMMARY", "Embedding pipeline completed!")
    log("INFO", "SUMMARY", f"Total pages processed: {total_pages_processed}/{len(docs)}")
    log("INFO", "SUMMARY", f"Total pages failed: {total_pages_failed}")
    log("INFO", "SUMMARY", f"Total chunks created: {total_chunks}")
    log("INFO", "SUMMARY", f"Collection: {COLLECTION_NAME}")
    log("INFO", "SUMMARY", "=" * 60)

    # Verify collection
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        log("INFO", "VERIFY", f"Vectors in collection: {collection_info.points_count}")

        # Get sample point
        if collection_info.points_count > 0:
            points = qdrant_client.scroll(
                collection_name=COLLECTION_NAME,
                limit=1,
                with_payload=True,
                with_vectors=False
            )[0]

            if points:
                sample = points[0].payload
                log("INFO", "VERIFY", f"Sample entry - Title: {sample.get('page_title')}, URL: {sample.get('url')}")
    except Exception as e:
        log("ERROR", "VERIFY", f"Failed to verify collection: {str(e)}")


if __name__ == "__main__":
    embed_documents()
