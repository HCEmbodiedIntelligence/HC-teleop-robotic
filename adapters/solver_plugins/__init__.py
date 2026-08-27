"""Selectable solver backends for the HC teleoperation controller."""

from .registry import SolverPlugin, available_plugins, resolve_plugin

__all__ = ["SolverPlugin", "available_plugins", "resolve_plugin"]
