"""OpenNear Python SDK — zero-dependency client for OpenNear Gateway REST API."""

from .client import (
    OpenNearAuthError,
    OpenNearClient,
    OpenNearError,
    RuleTrigger,
    __version__,
)

__all__ = [
    "__version__",
    "OpenNearClient",
    "OpenNearError",
    "OpenNearAuthError",
    "RuleTrigger",
]
