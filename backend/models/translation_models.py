"""
Pydantic Models for Translation API

Models:
- TranslationRequest: Request payload for translation
- TranslationResponse: Successful translation response
- TranslationError: Error response with error codes
"""

from pydantic import BaseModel, Field, validator
from typing import Optional
from datetime import datetime


class TranslationRequest(BaseModel):
    """Request model for translation endpoint"""

    text: str = Field(..., min_length=1, max_length=50000, description="Text content to translate")
    source_language: str = Field(..., pattern="^en$", description="Source language code (only 'en' supported)")
    target_language: str = Field(..., pattern="^ur$", description="Target language code (only 'ur' supported)")
    content_hash: Optional[str] = Field(None, pattern="^[a-f0-9]{32}$", description="MD5 hash for cache lookup")
    preserve_html: bool = Field(True, description="Whether to preserve HTML tags")

    @validator('text')
    def text_not_empty(cls, v):
        """Validate text is not empty or whitespace-only"""
        if not v or v.isspace():
            raise ValueError('Text cannot be empty or whitespace-only')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "text": "<p>Welcome to the <strong>Physical AI</strong> guide</p>",
                "source_language": "en",
                "target_language": "ur",
                "content_hash": "a3f5e8c9d2b14f6c8e7a9b2c3d4e5f6g",
                "preserve_html": True
            }
        }


class TranslationResponse(BaseModel):
    """Response model for successful translation"""

    translated_text: str = Field(..., description="Translated text in target language")
    source_language: str = Field(..., description="Source language code (echoed)")
    target_language: str = Field(..., description="Target language code (echoed)")
    cached: bool = Field(..., description="Whether result was served from cache")
    model_used: str = Field(..., description="Translation model identifier")
    timestamp: str = Field(..., description="Timestamp of translation (ISO 8601)")

    class Config:
        json_schema_extra = {
            "example": {
                "translated_text": "<p>فزیکل AI گائیڈ میں <strong>خوش آمدید</strong></p>",
                "source_language": "en",
                "target_language": "ur",
                "cached": False,
                "model_used": "facebook/mbart-large-50",
                "timestamp": "2025-12-22T10:30:00Z"
            }
        }


class TranslationError(BaseModel):
    """Error response model for translation failures"""

    error: str = Field(..., description="Human-readable error message")
    error_code: str = Field(..., description="Machine-readable error code")
    retry_after: Optional[int] = Field(None, description="Seconds to wait before retry (for rate limits)")
    timestamp: str = Field(..., description="Timestamp of error (ISO 8601)")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Translation service temporarily unavailable",
                "error_code": "SERVICE_DOWN",
                "retry_after": 60,
                "timestamp": "2025-12-22T10:30:00Z"
            }
        }


# Error code constants
class ErrorCodes:
    """Standard error codes for translation failures"""
    SERVICE_DOWN = "SERVICE_DOWN"  # Translation model not loaded or crashed
    RATE_LIMIT = "RATE_LIMIT"      # Too many requests
    INVALID_LANGUAGE = "INVALID_LANGUAGE"  # Unsupported language pair
    TIMEOUT = "TIMEOUT"             # Translation exceeded time limit
    INVALID_INPUT = "INVALID_INPUT" # Malformed HTML or empty text
