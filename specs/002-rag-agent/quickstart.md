# Quickstart: RAG Agent API Service

**Feature**: 002-rag-agent
**Purpose**: Quick guide for developers to set up and use the RAG Agent API

## Prerequisites

Before starting, ensure you have:

- ✅ Python 3.9 or higher installed
- ✅ OpenAI API key (get from https://platform.openai.com/api-keys)
- ✅ Access to the Qdrant vector database with embedded documentation (from Spec-1/004-website-embedding)
- ✅ Cohere API key for query embedding (from Spec-2/001-retrieval-pipeline-testing)
- ✅ Git repository cloned locally

## Installation

### 1. Install Dependencies

```bash
cd backend
pip install fastapi uvicorn openai pydantic python-dotenv cohere qdrant-client tiktoken
```

**Installed Packages**:
- `fastapi`: Web framework for the API
- `uvicorn`: ASGI server to run FastAPI
- `openai`: OpenAI SDK for LLM calls
- `pydantic`: Data validation
- `python-dotenv`: Environment variable management
- `cohere`: Query embedding (reused from retrieval pipeline)
- `qdrant-client`: Vector database client (reused from retrieval pipeline)
- `tiktoken`: Token counting for context management

### 2. Configure Environment Variables

Create or update `.env` file in the `backend/` directory:

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-your-openai-api-key-here

# Qdrant Configuration (from Spec-1)
QDRANT_URL=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# Cohere Configuration (from Spec-2)
COHERE_API_KEY=your-cohere-api-key-here

# API Configuration (optional)
DEFAULT_MODEL=gpt-3.5-turbo
DEFAULT_TOP_K=5
MAX_TOKENS=3000
```

### 3. Verify Existing Components

Ensure the retrieval pipeline from Spec-2 is available:

```bash
# Check that retrieval.py exists
ls -la backend/retrieval.py

# Verify Qdrant collection has data
python backend/check_qdrant.py
```

## Running the API

### Start the Development Server

```bash
cd backend
uvicorn rag_agent:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

The API is now running at: `http://localhost:8000`

### Access Interactive Documentation

FastAPI automatically generates interactive API documentation:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc
- **OpenAPI JSON**: http://localhost:8000/openapi.json

## Usage Examples

### Example 1: Basic Query (cURL)

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I create a ROS 2 publisher in Python?"
  }'
```

**Expected Response**:
```json
{
  "query": "How do I create a ROS 2 publisher in Python?",
  "answer": "To create a ROS 2 publisher in Python, you need to:\n\n1. Import the rclpy library\n2. Create a node class that inherits from Node\n3. In the constructor, call create_publisher() with your message type and topic name...",
  "sources": [
    {
      "url": "https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher.html",
      "page_title": "Writing a simple publisher (Python)",
      "relevance_score": 0.8642,
      "chunk_index": 2
    }
  ],
  "model_used": "gpt-3.5-turbo",
  "tokens_used": 456,
  "retrieval_count": 5
}
```

### Example 2: Custom Parameters (Python)

```python
import requests
import json

# API endpoint
url = "http://localhost:8000/query"

# Request payload
payload = {
    "query": "What is a URDF file and how is it used in ROS 2?",
    "top_k": 10,  # Retrieve more sources
    "model": "gpt-4"  # Use GPT-4 for better quality
}

# Make request
response = requests.post(url, json=payload)

# Parse response
if response.status_code == 200:
    data = response.json()
    print(f"Query: {data['query']}")
    print(f"\nAnswer:\n{data['answer']}")
    print(f"\nSources ({len(data['sources'])}):")
    for i, source in enumerate(data['sources'], 1):
        print(f"  {i}. {source['page_title']} (score: {source['relevance_score']:.4f})")
        print(f"     {source['url']}")
    print(f"\nTokens used: {data['tokens_used']}")
else:
    error = response.json()
    print(f"Error: {error['error_message']}")
```

### Example 3: Using JavaScript (Fetch API)

```javascript
async function askQuestion(query) {
  const response = await fetch('http://localhost:8000/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      query: query,
      top_k: 5,
      model: 'gpt-3.5-turbo'
    })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error_message);
  }

  const data = await response.json();
  return data;
}

// Usage
askQuestion("How do I create a ROS 2 subscriber?")
  .then(data => {
    console.log("Answer:", data.answer);
    console.log("Sources:", data.sources.length);
  })
  .catch(error => {
    console.error("Error:", error.message);
  });
```

### Example 4: Health Check

```bash
curl http://localhost:8000/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-21T10:30:00Z"
}
```

## Testing

### Manual Testing Checklist

Test the API with these scenarios:

- [ ] **Basic query**: Simple question about ROS 2
- [ ] **No results**: Query about unrelated topic (should return "no information" answer)
- [ ] **Custom top_k**: Request with `top_k=10`
- [ ] **GPT-4 model**: Request with `model="gpt-4"`
- [ ] **Empty query**: Should return 422 validation error
- [ ] **Invalid top_k**: `top_k=0` or `top_k=100` should return 422 error
- [ ] **Concurrent requests**: Send 5 queries simultaneously (should all succeed)
- [ ] **Long query**: 1000-character question (should work)
- [ ] **Special characters**: Query with code snippets, special chars

### Automated Testing

Run the test suite (once implemented in tasks phase):

```bash
cd backend
pytest test_rag_agent.py -v
```

## Troubleshooting

### Issue: "OpenAI API authentication error"

**Symptom**: 401 error from OpenAI

**Solution**:
```bash
# Verify OPENAI_API_KEY is set
echo $OPENAI_API_KEY

# Check key is valid on OpenAI platform
# https://platform.openai.com/api-keys
```

### Issue: "Vector database unavailable"

**Symptom**: 503 error, "Qdrant is currently unavailable"

**Solution**:
```bash
# Verify Qdrant credentials
python backend/check_qdrant.py

# Check QDRANT_URL and QDRANT_API_KEY in .env
```

### Issue: "Module not found: retrieval"

**Symptom**: `ImportError: cannot import name 'embed_query' from 'retrieval'`

**Solution**:
```bash
# Ensure retrieval.py is in the same directory
ls backend/retrieval.py

# Verify Python path
echo $PYTHONPATH

# Add backend to Python path if needed
export PYTHONPATH="${PYTHONPATH}:$(pwd)/backend"
```

### Issue: "Token limit exceeded"

**Symptom**: OpenAI error about context length

**Solution**:
- Reduce `top_k` to retrieve fewer chunks
- Use `gpt-4` which has larger context window (8k vs 4k)
- Adjust `MAX_TOKENS` in `.env` to lower value

### Issue: "CORS error in browser"

**Symptom**: Browser console shows CORS policy error

**Solution**:
CORS is enabled by default in the API. If still blocked, check:
```python
# Verify CORS middleware in rag_agent.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins in development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Next Steps

1. **Review API Documentation**: Visit http://localhost:8000/docs
2. **Test Core Scenarios**: Use Swagger UI to test different queries
3. **Integrate with Frontend**: Use the JavaScript example to build a UI
4. **Monitor Performance**: Check logs for latency and token usage
5. **Run Test Suite**: Execute automated tests for validation

## API Endpoints Reference

| Endpoint | Method | Purpose | Status Codes |
|----------|--------|---------|--------------|
| `/query` | POST | Submit question and get answer | 200, 422, 429, 502, 503 |
| `/health` | GET | Check API health | 200 |
| `/docs` | GET | Interactive API documentation (Swagger UI) | 200 |
| `/redoc` | GET | Alternative API documentation (ReDoc) | 200 |

## Configuration Options

Environment variables you can customize in `.env`:

| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | (required) | Your OpenAI API key |
| `QDRANT_URL` | (required) | Qdrant instance URL |
| `QDRANT_API_KEY` | (required) | Qdrant API key |
| `COHERE_API_KEY` | (required) | Cohere API key for embeddings |
| `DEFAULT_MODEL` | `gpt-3.5-turbo` | Default OpenAI model |
| `DEFAULT_TOP_K` | `5` | Default number of chunks to retrieve |
| `MAX_TOKENS` | `3000` | Max tokens for context (leaves room for completion) |
| `LOG_LEVEL` | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |

## Performance Expectations

- **Response Time**: 2-4 seconds (95th percentile)
- **Throughput**: 10+ concurrent requests
- **Token Usage**: ~300-600 tokens per query (depending on context length)
- **Cost**: ~$0.001-0.003 per query with gpt-3.5-turbo

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review logs in terminal for error details
3. Consult the plan.md and spec.md for design decisions
4. Test with Swagger UI at http://localhost:8000/docs
