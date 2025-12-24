"""
OpenAI Translation Server
FastAPI server for real translations using OpenAI/OpenRouter API.
Uses the same free model as the chatbot: xiaomi/mimo-v2-flash:free
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import hashlib
from datetime import datetime
import os
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(title="OpenAI Translation API")

# Initialize OpenAI client (using OpenRouter)
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
if not OPENROUTER_API_KEY:
    print("[WARNING] OPENROUTER_API_KEY not found in environment")
    print("[INFO] Set it in .env file for real translations")

client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=OPENROUTER_API_KEY or "dummy-key"
)

# Model to use (same as chatbot)
TRANSLATION_MODEL = "xiaomi/mimo-v2-flash:free"

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Simple in-memory cache
translation_cache = {}

class TranslationRequest(BaseModel):
    text: str
    source_language: str = "en"
    target_language: str = "ur"
    content_hash: Optional[str] = None
    preserve_html: bool = True

class TranslationResponse(BaseModel):
    translated_text: str
    source_language: str
    target_language: str
    cached: bool
    model_used: str
    timestamp: str

def translate_with_openai(text: str, source_lang: str, target_lang: str) -> str:
    """
    Translate text using OpenAI API.

    Args:
        text: Text to translate
        source_lang: Source language code (e.g., 'en')
        target_lang: Target language code (e.g., 'ur')

    Returns:
        Translated text
    """
    # Language mapping
    lang_names = {
        "en": "English",
        "ur": "Urdu"
    }

    source_name = lang_names.get(source_lang, source_lang)
    target_name = lang_names.get(target_lang, target_lang)

    # Create translation prompt
    system_prompt = f"You are a professional translator. Translate the following text from {source_name} to {target_name}. Only return the translation, nothing else."

    try:
        response = client.chat.completions.create(
            model=TRANSLATION_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=2000
        )

        translated = response.choices[0].message.content.strip()
        return translated

    except Exception as e:
        print(f"[ERROR] Translation failed: {e}")
        # Fallback: return original text with marker
        return f"[Translation Error] {text}"

@app.post("/api/translate", response_model=TranslationResponse)
async def translate(request: TranslationRequest):
    """
    Real translation endpoint using OpenAI API.
    Includes caching for performance.
    """
    # Generate cache key
    cache_key = hashlib.md5(
        f"{request.text}:{request.source_language}:{request.target_language}".encode()
    ).hexdigest()

    # Check cache
    if cache_key in translation_cache:
        cached_result = translation_cache[cache_key]
        print(f"[CACHE HIT] Returning cached translation")
        return TranslationResponse(
            translated_text=cached_result,
            source_language=request.source_language,
            target_language=request.target_language,
            cached=True,
            model_used=TRANSLATION_MODEL,
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

    # Translate using OpenAI
    print(f"[TRANSLATE] {request.source_language} -> {request.target_language}: {request.text[:50]}...")
    translated = translate_with_openai(
        request.text,
        request.source_language,
        request.target_language
    )

    # Store in cache
    translation_cache[cache_key] = translated

    return TranslationResponse(
        translated_text=translated,
        source_language=request.source_language,
        target_language=request.target_language,
        cached=False,
        model_used=TRANSLATION_MODEL,
        timestamp=datetime.utcnow().isoformat() + "Z"
    )

@app.get("/api/translate/health")
async def health_check():
    """Health check endpoint."""
    api_key_set = bool(OPENROUTER_API_KEY)
    return {
        "status": "healthy",
        "model_loaded": api_key_set,
        "model_name": TRANSLATION_MODEL,
        "cache_entries": len(translation_cache),
        "api_key_configured": api_key_set,
        "message": "OpenAI translation server running" if api_key_set else "API key not set - translations will fail"
    }

@app.get("/api/cache/stats")
async def cache_stats():
    """Get cache statistics."""
    return {
        "total_entries": len(translation_cache),
        "cache_size_bytes": sum(len(v.encode()) for v in translation_cache.values()),
        "cache_keys": list(translation_cache.keys())[:10]  # First 10 keys
    }

@app.delete("/api/cache")
async def clear_cache():
    """Clear the translation cache."""
    count = len(translation_cache)
    translation_cache.clear()
    return {
        "message": "Cache cleared successfully",
        "cleared_entries": count
    }

@app.get("/")
async def root():
    """Root endpoint with API info."""
    return {
        "name": "OpenAI Translation API",
        "version": "2.0.0",
        "status": "running",
        "model": TRANSLATION_MODEL,
        "endpoints": {
            "translate": "POST /api/translate",
            "health": "GET /api/translate/health",
            "cache_stats": "GET /api/cache/stats",
            "clear_cache": "DELETE /api/cache"
        },
        "note": "Real translations using OpenAI/OpenRouter API (same model as chatbot)"
    }

if __name__ == "__main__":
    import uvicorn
    print("=" * 60)
    print("OPENAI TRANSLATION SERVER")
    print("=" * 60)
    print("Status: Starting...")
    print(f"Model: {TRANSLATION_MODEL}")
    print("API Key:", "[OK] Set" if OPENROUTER_API_KEY else "[MISSING] Not Set")
    print("Server URL: http://localhost:8001")
    print("API Docs: http://localhost:8001/docs")
    print("=" * 60)
    uvicorn.run(app, host="0.0.0.0", port=8001)
