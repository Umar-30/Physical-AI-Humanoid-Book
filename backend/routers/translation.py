"""
Translation API Router

Endpoints:
- POST /api/translate: Translate text from English to Urdu
- GET /api/translate/health: Health check for translation service
- DELETE /api/cache: Clear translation cache (admin)
- GET /api/cache/stats: Get cache statistics
"""

from fastapi import APIRouter, HTTPException, status
from datetime import datetime
import logging
import hashlib
from typing import Dict

from models.translation_models import (
    TranslationRequest,
    TranslationResponse,
    TranslationError,
    ErrorCodes
)
from services.translation_service import get_translation_service

# Initialize router
router = APIRouter(
    prefix="/api",
    tags=["translation"],
    responses={
        503: {"model": TranslationError, "description": "Translation service unavailable"},
        429: {"model": TranslationError, "description": "Rate limit exceeded"},
        400: {"model": TranslationError, "description": "Invalid request"}
    }
)

logger = logging.getLogger(__name__)


@router.post(
    "/translate",
    response_model=TranslationResponse,
    summary="Translate text from English to Urdu",
    description="Translates English text to Urdu using mBART model with server-side caching"
)
async def translate_text(request: TranslationRequest) -> TranslationResponse:
    """
    Translate text from English to Urdu.

    - **text**: English text to translate (1-50000 chars)
    - **source_language**: Must be "en"
    - **target_language**: Must be "ur"
    - **content_hash**: Optional MD5 hash for cache lookup optimization
    - **preserve_html**: Whether to preserve HTML tags (default: true)

    Returns translated text with metadata including cache status and model info.
    """
    try:
        service = get_translation_service()

        # Check if content_hash matches the text (validate cache key)
        if request.content_hash:
            computed_hash = hashlib.md5(request.text.encode('utf-8')).hexdigest()
            if computed_hash != request.content_hash:
                logger.warning(
                    f"Content hash mismatch: provided={request.content_hash}, "
                    f"computed={computed_hash}"
                )
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail={
                        "error": "Content hash does not match text content",
                        "error_code": ErrorCodes.INVALID_INPUT,
                        "timestamp": datetime.utcnow().isoformat() + "Z"
                    }
                )

        # Check cache first
        cache_key = hashlib.md5(request.text.encode('utf-8')).hexdigest()
        cached_result = service.cache.get(cache_key)

        if cached_result:
            logger.info(f"Cache hit for content_hash={cache_key}")
            return TranslationResponse(
                translated_text=cached_result,
                source_language=request.source_language,
                target_language=request.target_language,
                cached=True,
                model_used=service.model_name,
                timestamp=datetime.utcnow().isoformat() + "Z"
            )

        # Extract translatable content if HTML preservation is enabled
        if request.preserve_html:
            processable_html, placeholders = service.extract_translatable_content(request.text)
            text_to_translate = processable_html
        else:
            text_to_translate = request.text
            placeholders = {}

        # Translate text
        logger.info(f"Translating {len(text_to_translate)} chars from {request.source_language} to {request.target_language}")

        # Convert language codes (en -> en_XX, ur -> ur_PK for mBART)
        source_lang_code = "en_XX" if request.source_language == "en" else request.source_language
        target_lang_code = "ur_PK" if request.target_language == "ur" else request.target_language

        translated = await service.translate_text(
            text=text_to_translate,
            source_lang=source_lang_code,
            target_lang=target_lang_code,
            timeout_seconds=10
        )

        # Restore excluded content if HTML preservation was enabled
        if request.preserve_html and placeholders:
            translated = service.restore_excluded_content(translated, placeholders)

        logger.info(f"Translation completed successfully (cached={False})")

        return TranslationResponse(
            translated_text=translated,
            source_language=request.source_language,
            target_language=request.target_language,
            cached=False,
            model_used=service.model_name,
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

    except TimeoutError as e:
        logger.error(f"Translation timeout: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_408_REQUEST_TIMEOUT,
            detail={
                "error": str(e),
                "error_code": ErrorCodes.TIMEOUT,
                "retry_after": 5,
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )

    except RuntimeError as e:
        logger.error(f"Translation service error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error": "Translation service temporarily unavailable",
                "error_code": ErrorCodes.SERVICE_DOWN,
                "retry_after": 60,
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )

    except ValueError as e:
        logger.error(f"Invalid input: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": str(e),
                "error_code": ErrorCodes.INVALID_INPUT,
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )

    except Exception as e:
        logger.error(f"Unexpected error during translation: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error during translation",
                "error_code": "INTERNAL_ERROR",
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )


@router.get(
    "/translate/health",
    summary="Translation service health check",
    description="Check if translation service and model are loaded and ready"
)
async def translation_health_check() -> Dict:
    """
    Health check for translation service.

    Returns:
    - **status**: "healthy" if model loaded, "degraded" if not
    - **model**: Model identifier
    - **model_loaded**: Whether model is loaded and ready
    - **cache_stats**: Current cache statistics
    - **timestamp**: Current timestamp
    """
    try:
        service = get_translation_service()
        cache_stats = service.cache.get_stats()

        return {
            "status": "healthy" if service._model_loaded else "degraded",
            "model": service.model_name,
            "model_loaded": service._model_loaded,
            "cache_stats": cache_stats,
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }


@router.get(
    "/cache/stats",
    summary="Get cache statistics",
    description="Retrieve current translation cache statistics"
)
async def get_cache_stats() -> Dict:
    """
    Get translation cache statistics.

    Returns:
    - **total_entries**: Number of cached translations
    - **total_size_mb**: Cache size in megabytes
    - **oldest_entry_age_hours**: Age of oldest cache entry in hours
    - **max_size_mb**: Maximum cache size limit
    - **ttl_hours**: Cache TTL in hours
    """
    try:
        service = get_translation_service()
        stats = service.cache.get_stats()

        return {
            **stats,
            "max_size_mb": round(service.cache.max_size_bytes / (1024 * 1024), 2),
            "ttl_hours": round(service.cache.ttl_seconds / 3600, 2),
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    except Exception as e:
        logger.error(f"Failed to get cache stats: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Failed to retrieve cache statistics",
                "error_code": "INTERNAL_ERROR",
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )


@router.delete(
    "/cache",
    summary="Clear translation cache",
    description="Clear all cached translations (admin endpoint)"
)
async def clear_cache() -> Dict:
    """
    Clear all cached translations.

    **Note**: This is an admin endpoint. In production, this should be
    protected with authentication/authorization.

    Returns:
    - **cleared_entries**: Number of entries removed from cache
    - **status**: Success status
    - **timestamp**: Operation timestamp
    """
    try:
        service = get_translation_service()
        cleared_count = service.cache.clear()

        logger.info(f"Cache cleared: {cleared_count} entries removed")

        return {
            "status": "success",
            "cleared_entries": cleared_count,
            "message": f"Successfully cleared {cleared_count} cached translations",
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
    except Exception as e:
        logger.error(f"Failed to clear cache: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Failed to clear cache",
                "error_code": "INTERNAL_ERROR",
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )
