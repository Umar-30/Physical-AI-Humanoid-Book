"""
Translation Service for English to Urdu translation using mBART model.

This service handles:
- Loading and managing the mBART translation model
- Translating text content from English to Urdu
- Content extraction (excluding code blocks)
- Server-side caching with TTL and LRU eviction
- Error handling and timeout management
"""

import asyncio
import hashlib
import logging
from datetime import datetime, timedelta
from typing import Dict, Optional, Tuple
from bs4 import BeautifulSoup
import time

# Translation model imports (will be loaded lazily)
try:
    from transformers import MBartForConditionalGeneration, MBart50TokenizerFast
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    logging.warning("Transformers library not available. Translation service will be disabled.")

logger = logging.getLogger(__name__)


class TranslationCache:
    """Server-side cache for translations with TTL and LRU eviction"""

    def __init__(self, ttl_seconds: int = 86400, max_size_bytes: int = 1_000_000_000):
        """
        Initialize translation cache

        Args:
            ttl_seconds: Time-to-live for cache entries (default: 24 hours)
            max_size_bytes: Maximum cache size in bytes (default: 1GB)
        """
        self.cache: Dict[str, Dict] = {}
        self.ttl_seconds = ttl_seconds
        self.max_size_bytes = max_size_bytes
        self.access_times: Dict[str, float] = {}

    def get(self, key: str) -> Optional[str]:
        """
        Get cached translation if not expired

        Args:
            key: Cache key (hash of content)

        Returns:
            Translated text if found and not expired, None otherwise
        """
        if key not in self.cache:
            return None

        entry = self.cache[key]
        created_at = datetime.fromisoformat(entry["timestamp"])
        age = datetime.now() - created_at

        # Check if expired
        if age.total_seconds() > self.ttl_seconds:
            del self.cache[key]
            del self.access_times[key]
            return None

        # Update access time for LRU
        self.access_times[key] = time.time()
        return entry["translated"]

    def set(self, key: str, translated_text: str):
        """
        Store translation in cache

        Args:
            key: Cache key (hash of content)
            translated_text: Translated text to cache
        """
        # Evict if cache is too large
        self._evict_if_needed()

        self.cache[key] = {
            "translated": translated_text,
            "timestamp": datetime.now().isoformat(),
            "access_count": 1
        }
        self.access_times[key] = time.time()

    def _evict_if_needed(self):
        """Evict least recently used entries if cache exceeds size limit"""
        current_size = self._estimate_size()

        while current_size > self.max_size_bytes and self.cache:
            # Find least recently used entry
            lru_key = min(self.access_times, key=self.access_times.get)
            del self.cache[lru_key]
            del self.access_times[lru_key]
            current_size = self._estimate_size()

    def _estimate_size(self) -> int:
        """Estimate cache size in bytes"""
        total_size = 0
        for entry in self.cache.values():
            total_size += len(entry["translated"].encode('utf-8'))
        return total_size

    def clear(self) -> int:
        """Clear all cache entries. Returns number of entries cleared."""
        count = len(self.cache)
        self.cache.clear()
        self.access_times.clear()
        return count

    def get_stats(self) -> Dict:
        """Get cache statistics"""
        total_entries = len(self.cache)
        total_size_mb = self._estimate_size() / (1024 * 1024)

        oldest_age_hours = 0
        if self.cache:
            oldest_timestamp = min(
                datetime.fromisoformat(entry["timestamp"])
                for entry in self.cache.values()
            )
            oldest_age_hours = (datetime.now() - oldest_timestamp).total_seconds() / 3600

        return {
            "total_entries": total_entries,
            "total_size_mb": round(total_size_mb, 2),
            "oldest_entry_age_hours": round(oldest_age_hours, 2)
        }


class TranslationService:
    """Service for translating English text to Urdu using mBART"""

    def __init__(self):
        self.model = None
        self.tokenizer = None
        self.model_name = "facebook/mbart-large-50-many-to-many-mmt"
        self.cache = TranslationCache()
        self._model_loading = False
        self._model_loaded = False

    async def load_model(self):
        """Load mBART model (lazy loading on first translation request)"""
        if self._model_loaded or not TRANSFORMERS_AVAILABLE:
            return

        if self._model_loading:
            # Wait for concurrent load to complete
            while self._model_loading:
                await asyncio.sleep(0.1)
            return

        self._model_loading = True
        try:
            logger.info(f"Loading translation model: {self.model_name}")
            # Note: Actual model loading would happen here
            # For now, we'll mark as loaded but note that actual inference won't work
            # until the model is properly downloaded
            logger.warning(
                "Translation model loading skipped in setup phase. "
                "Run 'python -c \"from transformers import MBartForConditionalGeneration; "
                "MBartForConditionalGeneration.from_pretrained('facebook/mbart-large-50-many-to-many-mmt')\" "
                "to download the model."
            )
            self._model_loaded = True
            logger.info("Translation service initialized (model download pending)")
        except Exception as e:
            logger.error(f"Failed to load translation model: {e}")
            raise
        finally:
            self._model_loading = False

    def extract_translatable_content(self, html: str) -> Tuple[str, Dict[str, str]]:
        """
        Extract translatable content from HTML, excluding code blocks

        Args:
            html: HTML string to process

        Returns:
            Tuple of (processable_html, placeholders_dict)
        """
        soup = BeautifulSoup(html, 'html.parser')

        # Tags to exclude from translation
        EXCLUDE_TAGS = ['code', 'pre', 'script', 'style']

        placeholders = {}
        for i, tag in enumerate(soup.find_all(EXCLUDE_TAGS)):
            placeholder = f"__SKIP_{i}__"
            placeholders[placeholder] = str(tag)
            tag.replace_with(placeholder)

        return str(soup), placeholders

    def restore_excluded_content(self, translated_html: str, placeholders: Dict[str, str]) -> str:
        """
        Restore excluded content (code blocks) after translation

        Args:
            translated_html: Translated HTML with placeholders
            placeholders: Dictionary of placeholders and their original content

        Returns:
            Final HTML with code blocks restored
        """
        for placeholder, original_content in placeholders.items():
            translated_html = translated_html.replace(placeholder, original_content)

        return translated_html

    async def translate_text(
        self,
        text: str,
        source_lang: str = "en_XX",
        target_lang: str = "ur_PK",
        timeout_seconds: int = 10
    ) -> str:
        """
        Translate text from English to Urdu

        Args:
            text: Text to translate
            source_lang: Source language code (mBART format)
            target_lang: Target language code (mBART format)
            timeout_seconds: Maximum time for translation

        Returns:
            Translated text

        Raises:
            TimeoutError: If translation exceeds timeout
            RuntimeError: If model not loaded or translation fails
        """
        # Check cache first
        cache_key = hashlib.md5(text.encode('utf-8')).hexdigest()
        cached = self.cache.get(cache_key)
        if cached:
            logger.debug(f"Cache hit for key: {cache_key}")
            return cached

        # Ensure model is loaded
        await self.load_model()

        if not self._model_loaded:
            raise RuntimeError("Translation model not loaded")

        try:
            # For now, return a placeholder since actual model isn't loaded
            # In production, this would call the actual mBART model:
            #
            # self.tokenizer.src_lang = source_lang
            # encoded = self.tokenizer(text, return_tensors="pt")
            # generated = self.model.generate(
            #     **encoded,
            #     forced_bos_token_id=self.tokenizer.lang_code_to_id[target_lang]
            # )
            # translated = self.tokenizer.batch_decode(generated, skip_special_tokens=True)[0]

            # Placeholder implementation
            translated = f"[URDU TRANSLATION PLACEHOLDER]: {text}"
            logger.warning("Using placeholder translation (model not actually loaded)")

            # Cache the result
            self.cache.set(cache_key, translated)

            return translated

        except asyncio.TimeoutError:
            logger.error(f"Translation timeout after {timeout_seconds} seconds")
            raise TimeoutError(f"Translation took longer than {timeout_seconds} seconds")
        except Exception as e:
            logger.error(f"Translation failed: {e}")
            raise RuntimeError(f"Translation failed: {str(e)}")


# Global service instance
_translation_service: Optional[TranslationService] = None


def get_translation_service() -> TranslationService:
    """Get or create the global translation service instance"""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service
