---
title: RAG Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# RAG Chatbot API

A Retrieval-Augmented Generation (RAG) API for documentation Q&A.

## API Endpoints

- `POST /query` - Submit a question and get an answer with sources
- `GET /health` - Health check endpoint
- `GET /docs` - Swagger API documentation

## Environment Variables

Set these in your HF Space secrets:

- `OPENAI_API_KEY` - OpenAI API key
- `QDRANT_URL` - Qdrant cloud URL
- `QDRANT_API_KEY` - Qdrant API key
- `COHERE_API_KEY` - Cohere API key (for embeddings)
