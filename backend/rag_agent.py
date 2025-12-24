"""
RAG Agent API Service

FastAPI application providing a RAG (Retrieval-Augmented Generation) endpoint
for querying documentation and generating grounded answers.

Endpoints:
    POST /query - Submit a question and get an answer with sources
    GET /health - Health check endpoint

Features:
    - Query validation with Pydantic
    - Integration with Qdrant for document retrieval
    - OpenAI integration for answer generation
    - Source attribution and deduplication
    - Structured logging
    - CORS support for web clients

Usage:
    uvicorn rag_agent:app --reload --host 0.0.0.0 --port 8000
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging
from datetime import datetime

from models.request_models import QueryRequest
from models.response_models import QueryResponse, ErrorResponse
from services.integration_service import IntegrationService
from exceptions import RAGException, ValidationError, RetrievalError, GenerationError, RateLimitError
from config.logging_config import setup_logging, get_logger

# Try to import translation router (optional - requires PyTorch)
translation_router = None
try:
    from routers.translation import router as translation_router
except Exception as e:
    print(f"[WARNING] Translation router not available: {e}")
    print("[INFO] RAG agent will run without translation support")

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="Retrieval-Augmented Generation API for documentation Q&A",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Configure CORS for development (allow all origins)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
setup_logging(level="INFO", json_format=False)
logger = get_logger('rag_agent')

# Initialize integration service
integration_service = IntegrationService()

# Register routers (only if available)
if translation_router is not None:
    app.include_router(translation_router)
    print("[INFO] Translation router registered")
else:
    print("[INFO] Running without translation support - use mock_translation_server.py for translations")


@app.get("/health")
async def health_check():
    """
    Health check endpoint.

    Returns:
        dict: Health status with version and timestamp
    """
    return {
        "status": "healthy",
        "version": "1.0.0",
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }


@app.get("/")
async def root():
    """
    Root endpoint with API information.

    Returns:
        dict: API info and documentation links
    """
    return {
        "name": "RAG Agent API",
        "version": "1.0.0",
        "description": "Retrieval-Augmented Generation API for documentation Q&A",
        "docs": "/docs",
        "redoc": "/redoc",
        "health": "/health"
    }


@app.post("/query", response_model=QueryResponse, responses={
    422: {"model": ErrorResponse, "description": "Validation Error"},
    503: {"model": ErrorResponse, "description": "Retrieval Error"},
    502: {"model": ErrorResponse, "description": "Generation Error"},
    429: {"model": ErrorResponse, "description": "Rate Limit Error"}
})
async def query_documentation(request: QueryRequest):
    """
    Submit a question and get an answer with source attribution.

    This endpoint implements the full RAG pipeline:
    1. Validates the query
    2. Retrieves relevant documentation chunks from Qdrant
    3. Generates a grounded answer using OpenAI
    4. Returns the answer with deduplicated source references

    Args:
        request: QueryRequest with query, top_k, and model

    Returns:
        QueryResponse with answer, sources, and metadata

    Raises:
        HTTPException: For validation, retrieval, generation, or rate limit errors
    """
    start_time = datetime.utcnow()

    logger.info(
        "Received query request",
        extra={
            "query": request.query[:50] + "..." if len(request.query) > 50 else request.query,
            "top_k": request.top_k,
            "model": request.model
        }
    )

    try:
        # Process query through RAG pipeline
        response = await integration_service.process_query(
            query=request.query,
            top_k=request.top_k,
            model=request.model
        )

        # Log success
        latency_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)
        logger.info(
            "Query processed successfully",
            extra={
                "query": request.query[:50] + "..." if len(request.query) > 50 else request.query,
                "latency_ms": latency_ms,
                "tokens_used": response.tokens_used,
                "sources_count": len(response.sources)
            }
        )

        return response

    except ValidationError as e:
        # Log validation error
        logger.warning(
            f"Validation error: {e.message}",
            extra={
                "query": request.query,
                "error_type": e.error_type,
                "status_code": e.status_code
            }
        )
        return JSONResponse(
            status_code=e.status_code,
            content=ErrorResponse(
                error_type=e.error_type,
                error_message=e.message,
                query=request.query,
                status_code=e.status_code
            ).model_dump()
        )

    except RetrievalError as e:
        # Log retrieval error
        logger.error(
            f"Retrieval error: {e.message}",
            extra={
                "query": request.query,
                "error_type": e.error_type,
                "status_code": e.status_code
            }
        )
        return JSONResponse(
            status_code=e.status_code,
            content=ErrorResponse(
                error_type=e.error_type,
                error_message=e.message,
                query=request.query,
                status_code=e.status_code
            ).model_dump()
        )

    except GenerationError as e:
        # Log generation error
        logger.error(
            f"Generation error: {e.message}",
            extra={
                "query": request.query,
                "error_type": e.error_type,
                "status_code": e.status_code
            }
        )
        return JSONResponse(
            status_code=e.status_code,
            content=ErrorResponse(
                error_type=e.error_type,
                error_message=e.message,
                query=request.query,
                status_code=e.status_code
            ).model_dump()
        )

    except RateLimitError as e:
        # Log rate limit error
        logger.warning(
            f"Rate limit error: {e.message}",
            extra={
                "query": request.query,
                "error_type": e.error_type,
                "status_code": e.status_code
            }
        )
        return JSONResponse(
            status_code=e.status_code,
            content=ErrorResponse(
                error_type=e.error_type,
                error_message=e.message,
                query=request.query,
                status_code=e.status_code
            ).model_dump()
        )

    except Exception as e:
        # Log unexpected error
        logger.error(
            f"Unexpected error: {str(e)}",
            extra={
                "query": request.query,
                "error_type": "InternalServerError"
            },
            exc_info=True
        )
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                error_type="InternalServerError",
                error_message=f"An unexpected error occurred: {str(e)}",
                query=request.query,
                status_code=500
            ).model_dump()
        )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
