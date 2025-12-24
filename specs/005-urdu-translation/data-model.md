# Data Model: Urdu Translation Agent Integration

**Feature**: 005-urdu-translation
**Created**: 2025-12-22
**Status**: Draft

## Overview

This document defines the data structures, entities, and their relationships for the Urdu Translation feature. The feature involves minimal persistent data (mostly ephemeral cache), with primary focus on request/response contracts and state management.

## Core Entities

### 1. TranslationRequest

**Description**: Request payload sent from frontend to backend translation service

**Fields**:
| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `text` | string | Yes | Original text to translate (HTML or plain text) | Min length: 1, Max length: 50,000 chars |
| `source_language` | string | Yes | Source language code (ISO 639-1) | Must be "en" |
| `target_language` | string | Yes | Target language code (ISO 639-1) | Must be "ur" |
| `content_hash` | string | No | MD5 hash of text for cache lookup | 32-char hex string |
| `preserve_html` | boolean | No | Whether to preserve HTML tags | Default: true |

**Validation Rules**:
- `text` cannot be empty or whitespace-only
- `source_language` must be "en" (English only source in MVP)
- `target_language` must be "ur" (Urdu only target in MVP)
- `content_hash` if provided must be valid MD5 (32 hex chars)

**Example**:
```json
{
  "text": "<p>Welcome to the <strong>Physical AI</strong> guide</p>",
  "source_language": "en",
  "target_language": "ur",
  "content_hash": "a3f5e8c9d2b14f6c8e7a9b2c3d4e5f6g",
  "preserve_html": true
}
```

---

### 2. TranslationResponse

**Description**: Response payload returned from backend after successful translation

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `translated_text` | string | Yes | Translated text in target language |
| `source_language` | string | Yes | Echoed source language code |
| `target_language` | string | Yes | Echoed target language code |
| `cached` | boolean | Yes | Whether result was served from cache |
| `model_used` | string | Yes | Translation model identifier |
| `timestamp` | string (ISO 8601) | Yes | Timestamp of translation |

**Example**:
```json
{
  "translated_text": "<p>فزیکل AI گائیڈ میں <strong>خوش آمدید</strong></p>",
  "source_language": "en",
  "target_language": "ur",
  "cached": false,
  "model_used": "facebook/mbart-large-50",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

---

### 3. TranslationError

**Description**: Error response when translation fails

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `error` | string | Yes | Human-readable error message |
| `error_code` | string | Yes | Machine-readable error code |
| `retry_after` | integer | No | Seconds to wait before retry (for rate limits) |
| `timestamp` | string (ISO 8601) | Yes | Timestamp of error |

**Error Codes**:
- `SERVICE_DOWN`: Translation model not loaded or crashed
- `RATE_LIMIT`: Too many requests (external API quota)
- `INVALID_LANGUAGE`: Unsupported language pair
- `TIMEOUT`: Translation exceeded 10-second limit
- `INVALID_INPUT`: Malformed HTML or empty text

**Example**:
```json
{
  "error": "Translation service temporarily unavailable",
  "error_code": "SERVICE_DOWN",
  "retry_after": 60,
  "timestamp": "2025-12-22T10:30:00Z"
}
```

---

### 4. CachedTranslation (Client Storage)

**Description**: Translation cached in browser localStorage

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `translated` | string | Translated HTML content |
| `timestamp` | number | Unix timestamp (milliseconds) of cache creation |
| `version` | string | Cache schema version (for migrations) |

**Storage Key Format**: `en-ur-{contentHash}`

**Example**:
```json
{
  "translated": "<div>فزیکل AI...</div>",
  "timestamp": 1703260800000,
  "version": "1.0"
}
```

**Eviction Policy**:
- LRU (Least Recently Used) when total cache size > 8MB
- Manual clear via browser settings

---

### 5. ServerCacheEntry (Backend Storage)

**Description**: Translation cached in server memory (Python dict)

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `translated` | string | Translated HTML content |
| `timestamp` | string (ISO 8601) | Cache creation timestamp |
| `ttl` | integer | Time-to-live in seconds (default: 86400 = 24h) |
| `access_count` | integer | Number of times accessed (for LRU) |

**Storage Key Format**: `en-ur-{contentHash}`

**Example**:
```python
{
  "en-ur-a3f5e8c9d2b1": {
    "translated": "<div>فزیکل AI...</div>",
    "timestamp": "2025-12-22T10:30:00Z",
    "ttl": 86400,
    "access_count": 15
  }
}
```

**Eviction Policy**:
- TTL-based: Expire after 24 hours
- LRU: Evict least accessed when memory > 1GB
- Manual: Admin endpoint `DELETE /api/cache`

---

### 6. LanguagePreference (Client State)

**Description**: User's current language selection stored in localStorage

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `language` | string | Current language code ("en" or "ur") |
| `timestamp` | number | Unix timestamp of last change |

**Storage Key**: `docusaurus-language-preference`

**Example**:
```json
{
  "language": "ur",
  "timestamp": 1703260800000
}
```

**Persistence**: Survives page reloads, cleared on browser storage clear

---

### 7. LanguageContext (React State)

**Description**: React Context managing language state across components

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `currentLanguage` | 'en' \| 'ur' | Currently active language |
| `isTranslating` | boolean | Whether translation is in progress |
| `translationError` | string \| null | Current error message (if any) |

**Methods**:
| Method | Parameters | Description |
|--------|------------|-------------|
| `setLanguage` | `lang: 'en' \| 'ur'` | Change active language |
| `clearError` | none | Clear current error message |

**Example**:
```typescript
interface LanguageContextValue {
  currentLanguage: 'en' | 'ur';
  setLanguage: (lang: 'en' | 'ur') => void;
  isTranslating: boolean;
  translationError: string | null;
  clearError: () => void;
}
```

---

### 8. ChatbotQueryRequest (Extended)

**Description**: Existing chatbot request extended with language parameter

**New Field**:
| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `language` | string | No | Target language for response | "en" or "ur", default: "en" |

**Existing Fields** (unchanged):
- `query`: string - User's question
- `top_k`: integer - Number of sources to retrieve
- `model`: string - LLM model to use

**Example**:
```json
{
  "query": "ROS 2 کیا ہے؟",
  "top_k": 5,
  "model": "xiaomi/mimo-v2-flash:free",
  "language": "ur"
}
```

---

### 9. ChatbotQueryResponse (Unchanged Structure, Translated Content)

**Description**: Chatbot response with content in requested language

**Fields** (unchanged):
| Field | Type | Description |
|-------|------|-------------|
| `answer` | string | Generated answer (in requested language) |
| `sources` | array | Retrieved source references |
| `query` | string | Echoed user query |
| `model` | string | Model used for generation |
| `timestamp` | string (ISO 8601) | Response timestamp |

**Source Object**:
| Field | Type | Description |
|-------|------|-------------|
| `url` | string | Source documentation URL |
| `page_title` | string | Page title (in requested language) |
| `chunk_id` | string | Unique chunk identifier |
| `relevance_score` | number | Similarity score (0.0-1.0) |

**Example (Urdu Response)**:
```json
{
  "answer": "ROS 2 ایک جدید روبوٹک آپریٹنگ سسٹم ہے...",
  "sources": [
    {
      "url": "http://localhost:3000/docs/module-1-ros2",
      "page_title": "ماڈیول 1: روبوٹک اعصابی نظام",
      "chunk_id": "chunk_042",
      "relevance_score": 0.92
    }
  ],
  "query": "ROS 2 کیا ہے؟",
  "model": "xiaomi/mimo-v2-flash:free",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

---

## Entity Relationships

```
┌─────────────────────────┐
│   User (Browser)        │
└───────────┬─────────────┘
            │
            │ Interacts with
            ↓
┌─────────────────────────┐      ┌────────────────────────┐
│  LanguageContext        │◄─────│  LanguagePreference    │
│  (React State)          │      │  (localStorage)        │
└───────────┬─────────────┘      └────────────────────────┘
            │
            │ Triggers
            ↓
┌─────────────────────────┐
│  TranslationRequest     │
│  (API Call)             │
└───────────┬─────────────┘
            │
            │ HTTP POST
            ↓
┌─────────────────────────┐      ┌────────────────────────┐
│  Translation Service    │◄─────│  ServerCacheEntry      │
│  (Backend)              │      │  (Memory Cache)        │
└───────────┬─────────────┘      └────────────────────────┘
            │
            │ Returns
            ↓
┌─────────────────────────┐      ┌────────────────────────┐
│  TranslationResponse    │─────►│  CachedTranslation     │
│  or TranslationError    │      │  (localStorage)        │
└─────────────────────────┘      └────────────────────────┘
```

**Flow Description**:
1. User toggles language via UI → Updates `LanguageContext`
2. `LanguageContext` updates → Saves `LanguagePreference` to localStorage
3. Component requests translation → Sends `TranslationRequest` to backend
4. Backend checks `ServerCacheEntry` → Hit: Return cached, Miss: Translate
5. Backend returns `TranslationResponse` or `TranslationError`
6. Frontend stores result in `CachedTranslation` (localStorage)
7. Component renders translated content

---

## State Transitions

### Language State Machine

```
┌─────────┐
│  IDLE   │ Initial state, English content displayed
└────┬────┘
     │ User clicks "اردو" button
     ↓
┌──────────────┐
│ TRANSLATING  │ Loading spinner, disabled toggle
└────┬────┬────┘
     │    │ Translation fails
     │    ↓
     │  ┌───────┐
     │  │ ERROR │ Error banner, fallback to English, retry button
     │  └───┬───┘
     │      │ User clicks retry or switch language
     │      ↓
     │    (back to TRANSLATING or IDLE)
     │
     │ Translation succeeds
     ↓
┌──────────────┐
│  TRANSLATED  │ Urdu content displayed, toggle enabled
└────┬────┬────┘
     │    │ User clicks "English" button
     │    ↓
     │  ┌─────────┐
     │  │  IDLE   │
     │  └─────────┘
     │
     │ User navigates to new page
     ↓
  (Check cache → TRANSLATED or TRANSLATING)
```

**States**:
- `IDLE`: Default state, English content, no translation activity
- `TRANSLATING`: Translation in progress, loading indicator visible
- `TRANSLATED`: Urdu content displayed, user can interact normally
- `ERROR`: Translation failed, error message shown, original content visible

**Triggers**:
- `toggleLanguage('ur')`: IDLE → TRANSLATING
- `toggleLanguage('en')`: TRANSLATED → IDLE
- `translationSuccess`: TRANSLATING → TRANSLATED
- `translationError`: TRANSLATING → ERROR
- `retry`: ERROR → TRANSLATING
- `pageNavigation`: Check cache → TRANSLATED (hit) or TRANSLATING (miss)

---

## Validation Rules

### Input Validation (Backend)

```python
from pydantic import BaseModel, Field, validator

class TranslationRequest(BaseModel):
    text: str = Field(..., min_length=1, max_length=50000)
    source_language: str = Field(..., regex="^en$")
    target_language: str = Field(..., regex="^ur$")
    content_hash: str = Field(None, regex="^[a-f0-9]{32}$")
    preserve_html: bool = True

    @validator('text')
    def text_not_empty(cls, v):
        if not v or v.isspace():
            raise ValueError('Text cannot be empty or whitespace-only')
        return v
```

### Cache Size Validation (Frontend)

```javascript
const MAX_CACHE_SIZE_MB = 8;
const MAX_CACHE_SIZE_BYTES = MAX_CACHE_SIZE_MB * 1024 * 1024;

function getCacheSize(cache) {
  return JSON.stringify(cache).length;
}

function evictOldestEntries(cache) {
  const entries = Object.entries(cache).sort((a, b) =>
    a[1].timestamp - b[1].timestamp
  );

  while (getCacheSize(cache) > MAX_CACHE_SIZE_BYTES && entries.length > 0) {
    const [key] = entries.shift();
    delete cache[key];
  }

  return cache;
}
```

---

## Data Migration Strategy

### Cache Version Migration

**Scenario**: Upgrade from cache v1.0 to v2.0 (hypothetical future)

**v1.0 Schema**:
```json
{
  "translated": "<html>",
  "timestamp": 1703260800000,
  "version": "1.0"
}
```

**v2.0 Schema**:
```json
{
  "translated": "<html>",
  "timestamp": 1703260800000,
  "version": "2.0",
  "model": "facebook/mbart-large-50",
  "quality_score": 0.85
}
```

**Migration Logic**:
```javascript
function migrateCache(cacheEntry) {
  if (cacheEntry.version === "1.0") {
    // Invalidate old cache, force retranslation
    return null;
  }
  return cacheEntry;
}

function loadFromCache(key) {
  const cached = localStorage.getItem(key);
  if (!cached) return null;

  const entry = JSON.parse(cached);
  const migrated = migrateCache(entry);

  if (!migrated) {
    localStorage.removeItem(key); // Evict old version
    return null;
  }

  return migrated;
}
```

**Strategy**: Invalidate-on-mismatch (no complex migrations, just retranslate)

---

## Performance Considerations

### Cache Hit Rates (Expected)

| Cache Layer | Expected Hit Rate | Latency on Hit |
|-------------|-------------------|----------------|
| Client (localStorage) | 60-70% | <50ms |
| Server (memory) | 80-90% | ~200ms |
| Miss (translation) | 10-20% | ~2000ms |

### Storage Limits

| Storage | Limit | Eviction Policy |
|---------|-------|-----------------|
| Client localStorage | 8MB | LRU (oldest first) |
| Server memory | 1GB | LRU + TTL (24h) |

### Typical Page Sizes

| Content Type | Avg Characters | Translation Time (Cold) |
|--------------|----------------|-------------------------|
| Short page (<1000 words) | 5,000 | ~800ms |
| Medium page (1000-3000 words) | 15,000 | ~1500ms |
| Long page (3000-5000 words) | 25,000 | ~2500ms |

---

## Security Considerations

### Data Privacy

- **No PII Storage**: Translation requests contain only documentation content (no user data)
- **Local Cache Only**: Client cache stored in browser localStorage (not cloud-synced)
- **No Logging of Content**: Backend logs only metadata (language pair, length, success/failure)

### Input Sanitization

- **HTML Escaping**: Translated content sanitized before rendering (prevent XSS)
- **Content Length Limits**: Max 50,000 characters per request (prevent DoS)
- **Rate Limiting**: 100 requests/minute per IP (backend firewall)

### API Key Security (if using Google Translate)

- **Backend Proxy**: API key stored in `.env`, never exposed to client
- **Secret Rotation**: API key rotatable without frontend changes
- **Access Control**: Backend endpoint accessible only from same origin

---

**End of Data Model Document**
