"""
Logging Configuration for RAG Agent API

Provides structured JSON logging for all components of the RAG Agent.
Logs include timestamp, level, component name, message, and contextual data.

Usage:
    from config.logging_config import setup_logging, get_logger

    setup_logging()
    logger = get_logger('rag_agent')
    logger.info("Message", extra={"query": "test", "latency_ms": 123})
"""

import logging
import json
import sys
from datetime import datetime
from typing import Any, Dict


class JSONFormatter(logging.Formatter):
    """
    Custom formatter that outputs logs as JSON.

    Format:
        {
            "timestamp": "2025-12-21T10:30:00Z",
            "level": "INFO",
            "component": "rag_agent",
            "message": "Query processed",
            "context": {"query": "...", "latency_ms": 123}
        }
    """

    def format(self, record: logging.LogRecord) -> str:
        """
        Format log record as JSON string.

        Args:
            record: LogRecord to format

        Returns:
            JSON-formatted log string
        """
        log_data: Dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "component": record.name,
            "message": record.getMessage(),
        }

        # Add any extra context from the log call
        if hasattr(record, 'query'):
            log_data['query'] = record.query
        if hasattr(record, 'latency_ms'):
            log_data['latency_ms'] = record.latency_ms
        if hasattr(record, 'tokens_used'):
            log_data['tokens_used'] = record.tokens_used
        if hasattr(record, 'model'):
            log_data['model'] = record.model
        if hasattr(record, 'status_code'):
            log_data['status_code'] = record.status_code
        if hasattr(record, 'error_type'):
            log_data['error_type'] = record.error_type

        # Include exception info if present
        if record.exc_info:
            log_data['exception'] = self.formatException(record.exc_info)

        return json.dumps(log_data)


def setup_logging(level: str = "INFO", json_format: bool = False) -> None:
    """
    Configure logging for the application.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR)
        json_format: If True, use JSON formatter; otherwise use standard format

    Example:
        >>> setup_logging(level="DEBUG", json_format=True)
    """
    log_level = getattr(logging, level.upper(), logging.INFO)

    # Create handler for stdout
    handler = logging.StreamHandler(sys.stdout)

    if json_format:
        handler.setFormatter(JSONFormatter())
    else:
        # Standard format for development
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)

    # Configure root logger
    logging.root.setLevel(log_level)
    logging.root.handlers = [handler]

    # Set levels for noisy libraries
    logging.getLogger('httpx').setLevel(logging.WARNING)
    logging.getLogger('httpcore').setLevel(logging.WARNING)
    logging.getLogger('openai').setLevel(logging.WARNING)
    logging.getLogger('urllib3').setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger instance for a component.

    Args:
        name: Component name (e.g., 'rag_agent', 'retrieval', 'generation')

    Returns:
        Logger instance

    Example:
        >>> logger = get_logger('rag_agent')
        >>> logger.info("Processing query", extra={"query": "test"})
    """
    return logging.getLogger(name)


# Default setup for imports
if __name__ != "__main__":
    setup_logging()
