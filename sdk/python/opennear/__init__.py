"""OpenNear Python SDK — zero-dependency client for OpenNear Gateway REST API."""

from .client import (
    LongRangeMode,
    OpenNearAuthError,
    OpenNearClient,
    OpenNearError,
    RuleTrigger,
    __version__,
)

__all__ = [
    "__version__",
    "LongRangeMode",
    "OpenNearClient",
    "OpenNearError",
    "OpenNearAuthError",
    "RuleTrigger",
]
