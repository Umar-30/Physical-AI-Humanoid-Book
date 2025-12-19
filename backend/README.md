# Website Embedding Pipeline

A robust pipeline that converts the deployed Docusaurus book website into searchable vector embeddings for RAG (Retrieval-Augmented Generation) usage.

## Features

- 🗺️ **Sitemap Discovery**: Automatically discovers all pages via sitemap parsing (supports both sitemap index and regular formats)
- 📄 **Text Extraction**: Extracts clean text content from HTML pages with automatic cleanup
- ✂️ **Smart Chunking**: Splits text into optimal 512-word chunks for embedding
- 🤖 **Cohere Integration**: Generates high-quality 1024-dimensional embeddings using Cohere's API
- 💾 **Qdrant Storage**: Stores embeddings in Qdrant vector database with comprehensive metadata
- 🔄 **Error Handling**: Graceful handling of network errors, rate limits, and failures
- 📊 **Progress Tracking**: Detailed logging with structured format

## Prerequisites

- Python 3.10+
- Cohere API key ([Get one here](https://dashboard.cohere.com/api-keys))
- Qdrant instance (local or cloud)

## Setup

### 1. Install Dependencies

Using pip:
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

Or using uv:
```bash
cd backend
uv sync
```

### 2. Configure Environment Variables

Copy the example environment file:
```bash
cp .env.example .env
```

Edit `.env` and add your API keys:
```bash
# Required
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=localhost:6333  # Or your Qdrant Cloud URL
QDRANT_API_KEY=your_qdrant_api_key_here  # Only for cloud

# Target Website
TARGET_URL=https://umar-30.github.io/Physical-AI-Humanoid-Book/
```

### 3. Set Up Qdrant

**Option A: Local Qdrant (Recommended for development)**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

**Option B: Qdrant Cloud**
1. Sign up at https://cloud.qdrant.io
2. Create a cluster
3. Copy the cluster URL and API key to `.env`

## Usage

Run the complete pipeline:
```bash
python main.py
```

The pipeline will:
1. Discover all pages from the website sitemap
2. Extract and clean text content from each page
3. Split content into 512-word chunks
4. Generate embeddings using Cohere
5. Store embeddings with metadata in Qdrant

### Expected Output

```
Checking for sitemap at: https://umar-30.github.io/sitemap.xml
Found sitemap at: https://umar-30.github.io/sitemap.xml
Found regular sitemap with 47 URLs
Found 47 URLs to process
Creating collection 'rag_embedding'...
Collection 'rag_embedding' created successfully.
Processing https://umar-30.github.io/Physical-AI-Humanoid-Book/...
Text split into 12 chunks
Generating embeddings for batch 1...
Processed 10/12 chunks
Embedding pipeline completed successfully!
```

## Architecture

### Pipeline Components

- **`parse_sitemap()`**: Parses XML sitemap and identifies type (index vs. urlset)
- **`get_all_urls_from_sitemap()`**: Handles both sitemap types with recursive processing
- **`get_all_urls()`**: Discovers sitemap and extracts all page URLs
- **`extract_text_from_url()`**: Extracts clean text from HTML pages
- **`chunk_text()`**: Splits text into 512-word chunks with word boundaries
- **`embed()`**: Generates 1024-dim embeddings using Cohere
- **`create_collection()`**: Creates Qdrant collection with proper configuration
- **`save_chunk_to_qdrant()`**: Stores chunks with embeddings and metadata
- **`main()`**: Orchestrates the complete pipeline

### Data Flow

```
Website → Sitemap Discovery → URL List → Text Extraction → Chunking
   ↓
Cohere API → Embeddings (1024-dim) → Qdrant Storage with Metadata
```

### Metadata Schema

Each stored chunk includes:
```python
{
    "id": int,              # Unique hash(url + chunk_index)
    "vector": [float],      # 1024-dimensional embedding
    "payload": {
        "text": str,        # Original chunk text
        "url": str,         # Source page URL
        "chunk_index": int  # Position in page (0, 1, 2...)
    }
}
```

## Troubleshooting

### Cohere Rate Limits (429 Errors)

**Symptom**: Pipeline fails with "429 Too Many Requests"

**Solutions**:
1. Wait for rate limit reset (usually 1 minute)
2. Reduce batch size in code (change `batch_size` from 10 to 5)
3. Add delays between batches (`time.sleep(2)` between batches)

### Qdrant Connection Issues

**Symptom**: "Connection refused" or timeout errors

**Solutions**:
1. Verify Qdrant is running: `curl http://localhost:6333/collections`
2. Check `QDRANT_URL` in `.env` matches your Qdrant instance
3. For cloud Qdrant, verify API key is correct

### Missing Environment Variables

**Symptom**: "COHERE_API_KEY environment variable must be set"

**Solution**: Ensure `.env` file exists and contains required variables

### Sitemap Not Found

**Symptom**: "No sitemap found, returning base URL only"

**Solutions**:
1. Verify target website is accessible
2. Check website has sitemap.xml at root or standard locations
3. Manually specify sitemap URL in code if needed

### Empty Text Extraction

**Symptom**: "Could not extract text from [URL], skipping..."

**Solutions**:
1. Check page is publicly accessible (not behind auth)
2. Verify page contains actual content (not just navigation)
3. Review page HTML structure (may need custom extraction logic)

## Performance

- **Throughput**: 50-100 pages/hour (depends on page size)
- **Memory Usage**: <500MB RAM
- **Processing Time**: ~2 hours for 100-150 pages
- **Embedding Cost**: ~$0.065 per full site run (Cohere pricing)

## Development

### Project Structure

```
backend/
├── main.py              # Main pipeline implementation
├── requirements.txt     # Python dependencies
├── .env.example        # Example environment variables
├── .env                # Your environment variables (git-ignored)
├── .gitignore          # Git ignore patterns
└── README.md           # This file
```

### Running Tests

Validation scripts (coming soon):
```bash
python verify_pipeline.py      # End-to-end pipeline test
python verify_metadata.py      # Metadata completeness check
python test_embeddings.py      # Embedding quality validation
```

## References

- [Cohere Documentation](https://docs.cohere.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Feature Specification](../specs/004-website-embedding/spec.md)
- [Implementation Plan](../specs/004-website-embedding/plan.md)
- [Task List](../specs/004-website-embedding/tasks.md)

## License

Part of the Physical AI & Humanoid Robotics educational project.
