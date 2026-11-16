"""Explicitly selected Motion backend, separate from the default v23 mode."""

from .errors import MotionError
from .service import MotionService


def create_router(service):
    from .router import create_router as factory

    return factory(service)


__all__ = ["MotionService", "MotionError", "create_router"]
