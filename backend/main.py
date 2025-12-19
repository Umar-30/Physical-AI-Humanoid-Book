import os
import requests
from bs4 import BeautifulSoup
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import re
from urllib.parse import urljoin, urlparse
import xml.etree.ElementTree as ET
from dotenv import load_dotenv
from datetime import datetime
import time

# Load environment variables from .env file
load_dotenv()


def log(level: str, component: str, message: str):
    """
    Structured logging with timestamps and component labels.

    Format: [TIMESTAMP] [LEVEL] [COMPONENT] Message

    Args:
        level: Log level (INFO, WARNING, ERROR)
        component: Component name (SITEMAP, EXTRACT, CHUNK, EMBED, STORE)
        message: Log message
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] [{component}] {message}")


def retry_with_backoff(func, max_retries=3, initial_delay=1.0, component="API"):
    """
    Retry a function with exponential backoff.

    Args:
        func: Function to retry (should be a callable with no args)
        max_retries: Maximum number of retry attempts (default: 3)
        initial_delay: Initial delay in seconds (default: 1.0)
        component: Component name for logging (default: "API")

    Returns:
        Result of the function if successful

    Raises:
        Last exception if all retries fail
    """
    delay = initial_delay
    last_exception = None

    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            last_exception = e
            if attempt < max_retries - 1:
                log("WARNING", component, f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
                time.sleep(delay)
                delay *= 2  # Exponential backoff: 1s, 2s, 4s
            else:
                log("ERROR", component, f"All {max_retries} attempts failed: {str(e)}")

    raise last_exception


def parse_sitemap(sitemap_url: str) -> Dict[str, List[str]]:
    """
    Parse a sitemap and determine if it's a sitemap index or regular sitemap.

    Args:
        sitemap_url: URL to the sitemap XML file

    Returns:
        Dict with 'type' (either 'index', 'urlset', or 'error') and 'urls' (list of URLs)
    """
    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse XML
        try:
            root = ET.fromstring(response.content)
        except ET.ParseError as e:
            log("ERROR", "SITEMAP", f"XML parse error for {sitemap_url}: {str(e)}")
            return {'type': 'error', 'urls': []}

        # Define namespaces
        namespaces = {
            'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'
        }

        # Check if it's a sitemap index (contains <sitemap> tags)
        sitemap_elements = root.findall('.//sitemap:sitemap', namespaces)

        if sitemap_elements:
            # This is a sitemap index
            sitemap_urls = []
            for sitemap_elem in sitemap_elements:
                loc = sitemap_elem.find('sitemap:loc', namespaces)
                if loc is not None and loc.text:
                    sitemap_urls.append(loc.text.strip())

            log("INFO", "SITEMAP", f"Parsed sitemap index with {len(sitemap_urls)} sitemaps")
            return {
                'type': 'index',
                'urls': sitemap_urls
            }
        else:
            # This is a regular sitemap (contains <url> tags)
            url_elements = root.findall('.//sitemap:url', namespaces)
            page_urls = []

            for url_elem in url_elements:
                loc = url_elem.find('sitemap:loc', namespaces)
                if loc is not None and loc.text:
                    page_urls.append(loc.text.strip())

            log("INFO", "SITEMAP", f"Parsed regular sitemap with {len(page_urls)} URLs")
            return {
                'type': 'urlset',
                'urls': page_urls
            }

    except requests.exceptions.Timeout:
        log("ERROR", "SITEMAP", f"Timeout fetching sitemap {sitemap_url}")
        return {'type': 'error', 'urls': []}
    except requests.exceptions.HTTPError as e:
        log("ERROR", "SITEMAP", f"HTTP error {e.response.status_code} for {sitemap_url}")
        return {'type': 'error', 'urls': []}
    except requests.exceptions.RequestException as e:
        log("ERROR", "SITEMAP", f"Network error fetching {sitemap_url}: {str(e)}")
        return {'type': 'error', 'urls': []}
    except Exception as e:
        log("ERROR", "SITEMAP", f"Unexpected error parsing {sitemap_url}: {str(e)}")
        return {'type': 'error', 'urls': []}


def get_all_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """
    Handle both sitemap index and regular sitemap with retry logic.

    If the sitemap is an index, it will recursively fetch all URLs from child sitemaps.
    If it's a regular sitemap, it will return all URLs directly.
    Uses retry logic with exponential backoff for network errors.

    Args:
        sitemap_url: URL to the sitemap or sitemap index

    Returns:
        List of all URLs found
    """
    all_urls = []

    # Use retry logic for sitemap parsing to handle transient network errors
    try:
        parsed = retry_with_backoff(
            lambda: parse_sitemap(sitemap_url),
            max_retries=3,
            initial_delay=1.0,
            component="SITEMAP"
        )
    except Exception as e:
        log("ERROR", "SITEMAP", f"Failed to parse sitemap after retries: {sitemap_url}")
        return []

    if parsed['type'] == 'index':
        # Sitemap index - recursively fetch URLs from each sitemap
        log("INFO", "SITEMAP", f"Found sitemap index with {len(parsed['urls'])} child sitemaps")
        for child_sitemap_url in parsed['urls']:
            log("INFO", "SITEMAP", f"Processing child sitemap: {child_sitemap_url}")
            child_urls = get_all_urls_from_sitemap(child_sitemap_url)
            all_urls.extend(child_urls)

    elif parsed['type'] == 'urlset':
        # Regular sitemap - return all URLs
        log("INFO", "SITEMAP", f"Found regular sitemap with {len(parsed['urls'])} URLs")
        all_urls = parsed['urls']

    else:
        log("ERROR", "SITEMAP", f"Error processing sitemap: {sitemap_url}")

    return all_urls


def get_all_urls(base_url: str) -> List[str]:
    """
    Fetch all URLs from the given base URL.

    Tries to find and parse sitemap.xml first. If not found, returns base URL.
    Handles both sitemap index and regular sitemap formats.
    """
    # Try common sitemap locations
    parsed_url = urlparse(base_url)
    base_domain = f"{parsed_url.scheme}://{parsed_url.netloc}"

    sitemap_locations = [
        urljoin(base_domain, 'sitemap.xml'),
        urljoin(base_domain, 'sitemap_index.xml'),
        urljoin(base_url, 'sitemap.xml'),
    ]

    for sitemap_url in sitemap_locations:
        try:
            log("INFO", "SITEMAP", f"Checking for sitemap at: {sitemap_url}")
            response = requests.head(sitemap_url, timeout=5)
            if response.status_code == 200:
                log("INFO", "SITEMAP", f"Found sitemap at: {sitemap_url}")
                urls = get_all_urls_from_sitemap(sitemap_url)
                if urls:
                    return urls
        except Exception as e:
            log("WARNING", "SITEMAP", f"No sitemap found at {sitemap_url}: {str(e)}")
            continue

    # If no sitemap found, return base URL
    log("WARNING", "SITEMAP", "No sitemap found, returning base URL only")
    return [base_url]


def extract_text_from_url(url: str) -> str:
    """
    Extract text content from a given URL.

    Args:
        url: URL to extract text from

    Returns:
        Extracted and cleaned text content, or empty string on error
    """
    try:
        response = requests.get(url, timeout=15)
        response.raise_for_status()

        # Parse HTML with BeautifulSoup
        try:
            soup = BeautifulSoup(response.content, 'html.parser')
        except Exception as e:
            log("ERROR", "EXTRACT", f"HTML parsing error for {url}: {str(e)}")
            return ""

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        if not text.strip():
            log("WARNING", "EXTRACT", f"No text content extracted from {url}")
            return ""

        return text

    except requests.exceptions.Timeout:
        log("ERROR", "EXTRACT", f"Timeout fetching {url}")
        return ""
    except requests.exceptions.HTTPError as e:
        status_code = e.response.status_code
        if status_code == 404:
            log("WARNING", "EXTRACT", f"Page not found (404): {url}")
        else:
            log("ERROR", "EXTRACT", f"HTTP error {status_code} for {url}")
        return ""
    except requests.exceptions.RequestException as e:
        log("ERROR", "EXTRACT", f"Network error fetching {url}: {str(e)}")
        return ""
    except Exception as e:
        log("ERROR", "EXTRACT", f"Unexpected error extracting text from {url}: {str(e)}")
        return ""


def chunk_text(text: str, chunk_size: int = 512) -> List[str]:
    """
    Split text into chunks of specified size with validation.

    Args:
        text: Input text to chunk
        chunk_size: Target words per chunk (default: 512, must be 256-1024)

    Returns:
        List of text chunks

    Raises:
        ValueError: If chunk_size is outside valid bounds
    """
    # Validate chunk size bounds (256-1024 words)
    if chunk_size < 256 or chunk_size > 1024:
        raise ValueError(
            f"Chunk size must be between 256 and 1024 words, got {chunk_size}. "
            "Adjust chunk_size parameter to be within valid bounds."
        )

    if not text or not text.strip():
        log("WARNING", "CHUNK", "Empty or whitespace-only text provided")
        return []

    words = text.split()

    # If text is shorter than chunk size, return as single chunk
    if len(words) <= chunk_size:
        return [text]

    chunks = []
    current_chunk = []
    current_length = 0

    for word in words:
        if current_length + len(word) > chunk_size and current_chunk:
            chunks.append(' '.join(current_chunk))
            current_chunk = [word]
            current_length = len(word)
        else:
            current_chunk.append(word)
            current_length += len(word)

    if current_chunk:
        chunks.append(' '.join(current_chunk))

    # Log validation info
    if chunks:
        avg_chunk_size = sum(len(c.split()) for c in chunks) / len(chunks)
        log("INFO", "CHUNK", f"Created {len(chunks)} chunks (avg {avg_chunk_size:.0f} words/chunk)")

    return chunks


def embed(texts: List[str], cohere_client: CohereClient) -> List[List[float]]:
    """
    Generate embeddings for the given texts using Cohere with rate limit handling.

    Args:
        texts: List of text strings to embed (max 96 per batch recommended)
        cohere_client: Cohere API client

    Returns:
        List of 1024-dimensional embedding vectors, or empty list on error
    """
    def _embed_with_retry():
        try:
            response = cohere_client.embed(
                texts=texts,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            return response.embeddings
        except Exception as e:
            # Check for rate limit error (429)
            error_str = str(e).lower()
            if '429' in error_str or 'rate limit' in error_str or 'too many requests' in error_str:
                log("WARNING", "EMBED", f"Rate limit hit, will retry with backoff...")
                raise  # Re-raise to trigger retry logic
            else:
                log("ERROR", "EMBED", f"API error generating embeddings: {str(e)}")
                raise

    try:
        # Use retry logic with exponential backoff for rate limits
        embeddings = retry_with_backoff(
            _embed_with_retry,
            max_retries=3,
            initial_delay=2.0,  # Longer initial delay for rate limits
            component="EMBED"
        )
        log("INFO", "EMBED", f"Successfully generated {len(embeddings)} embeddings")
        return embeddings

    except Exception as e:
        log("ERROR", "EMBED", f"Failed to generate embeddings after retries: {str(e)}")
        return []


def create_collection(qdrant_client: QdrantClient, collection_name: str):
    """
    Create a Qdrant collection for storing embeddings.

    Args:
        qdrant_client: Qdrant client instance
        collection_name: Name of the collection to create
    """
    try:
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0 produces 1024-dim vectors
                distance=models.Distance.COSINE
            )
        )
        log("INFO", "STORE", f"Collection '{collection_name}' created successfully (1024-dim, COSINE)")
    except Exception as e:
        log("ERROR", "STORE", f"Error creating collection '{collection_name}': {str(e)}")
        raise


def save_chunk_to_qdrant(
    qdrant_client: QdrantClient,
    collection_name: str,
    chunk: str,
    embedding: List[float],
    url: str,
    chunk_index: int
):
    """
    Save a text chunk with its embedding to Qdrant.

    Args:
        qdrant_client: Qdrant client instance
        collection_name: Name of the collection
        chunk: Text chunk to store
        embedding: 1024-dim embedding vector
        url: Source URL
        chunk_index: Position of chunk in page
    """
    try:
        point_id = hash(url + str(chunk_index)) % (10**9)
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "text": chunk,
                        "url": url,
                        "chunk_index": chunk_index
                    }
                )
            ]
        )
    except Exception as e:
        log("ERROR", "STORE", f"Error saving chunk {chunk_index} from {url}: {str(e)}")
        raise


def main():
    """
    Main function to execute the embedding pipeline.

    Environment Variables Required:
    - COHERE_API_KEY: Cohere API key for embedding generation
    - QDRANT_URL: Qdrant instance URL (default: localhost:6333)
    - QDRANT_API_KEY: Qdrant API key (for cloud instances only)
    - TARGET_URL: Target website URL to embed (optional)

    Raises:
        ValueError: If required environment variables are not set
    """
    # Configuration - Load environment variables
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL", "localhost:6333")
    TARGET_URL = os.getenv("TARGET_URL")

    # Environment variable validation - fail fast with clear errors
    if not COHERE_API_KEY:
        raise ValueError(
            "COHERE_API_KEY environment variable must be set.\n"
            "Get your API key from: https://dashboard.cohere.com/api-keys\n"
            "Add it to your .env file: COHERE_API_KEY=your_key_here"
        )

    if not QDRANT_URL:
        raise ValueError(
            "QDRANT_URL environment variable must be set.\n"
            "For local Qdrant: QDRANT_URL=localhost:6333\n"
            "For Qdrant Cloud: QDRANT_URL=your_cluster_url"
        )

    # Initialize clients
    cohere_client = CohereClient(api_key=COHERE_API_KEY)

    # Connect to Qdrant (using local instance by default, or cloud if URL is provided)
    if QDRANT_URL == "localhost:6333":
        qdrant_client = QdrantClient(host="localhost", port=6333)
    else:
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )

    # Target URL - use environment variable or default
    target_url = TARGET_URL or "https://umar-30.github.io/Physical-AI-Humanoid-Book/"
    log("INFO", "MAIN", f"Target website: {target_url}")

    # Step 1: Get all URLs
    log("INFO", "MAIN", "Fetching URLs from sitemap...")
    urls = get_all_urls(target_url)
    total_urls = len(urls)
    log("INFO", "MAIN", f"Found {total_urls} URLs to process")

    # Step 2: Create Qdrant collection
    collection_name = "rag_embedding"
    log("INFO", "MAIN", f"Creating collection '{collection_name}'...")
    create_collection(qdrant_client, collection_name)

    # Step 3: Process each URL with progress tracking
    for url_idx, url in enumerate(urls, 1):
        # Progress: Page X of Y (Z%)
        progress_pct = (url_idx / total_urls) * 100
        log("INFO", "MAIN", f"Processing page {url_idx}/{total_urls} ({progress_pct:.1f}%): {url}")

        # Extract text
        text = extract_text_from_url(url)
        if not text:
            log("WARNING", "MAIN", f"Could not extract text from {url}, skipping...")
            continue

        # Chunk text
        chunks = chunk_text(text)
        log("INFO", "CHUNK", f"Text split into {len(chunks)} chunks")

        # Process chunks in batches to avoid rate limits
        batch_size = 10
        total_batches = (len(chunks) + batch_size - 1) // batch_size
        for i in range(0, len(chunks), batch_size):
            batch_chunks = chunks[i:i + batch_size]
            batch_num = i // batch_size + 1

            # Generate embeddings
            log("INFO", "EMBED", f"Generating embeddings for batch {batch_num}/{total_batches} ({len(batch_chunks)} chunks)")
            embeddings = embed(batch_chunks, cohere_client)

            # Save to Qdrant
            for idx, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                chunk_idx = i + idx
                save_chunk_to_qdrant(
                    qdrant_client, collection_name, chunk, embedding, url, chunk_idx
                )

            # Progress update after each batch
            chunks_processed = min(i + batch_size, len(chunks))
            log("INFO", "STORE", f"Stored {chunks_processed}/{len(chunks)} chunks for page {url_idx}/{total_urls}")

    log("INFO", "MAIN", "Embedding pipeline completed successfully!")


if __name__ == "__main__":
    main()
