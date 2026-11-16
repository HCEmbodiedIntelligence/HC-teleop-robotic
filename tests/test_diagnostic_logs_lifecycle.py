"""A failed middleware startup must not leak global handlers or workers."""
import asyncio
import json
import logging
from unittest.mock import AsyncMock

import pytest

from middleware.core import app as app_module
from middleware.core.config import ConfigStore
from middleware.core.diagnostic_logs import DiagnosticLogs, DiagnosticLogHandler


def test_failed_runtime_start_drains_diagnostics_and_detaches_handler(tmp_path, monkeypatch):
    service = DiagnosticLogs(tmp_path, poll_interval=.02)
    monkeypatch.setattr(app_module, 'DiagnosticLogs', lambda *args, **kwargs: service)
    monkeypatch.setattr(app_module.MiddlewareRuntime, 'start', AsyncMock(
        side_effect=RuntimeError('startup unavailable password=never-persist')))
    monkeypatch.setattr(app_module.MiddlewareRuntime, 'stop', AsyncMock())
    before = list(logging.getLogger().handlers)

    async def exercise():
        app = app_module.create_app(ConfigStore(tmp_path / 'config.yaml'))
        app.freeze()
        with pytest.raises(RuntimeError, match='startup unavailable'):
            await app.startup()
        await app.cleanup()

    asyncio.run(exercise())
    assert logging.getLogger().handlers == before
    assert not any(isinstance(h, DiagnosticLogHandler) and h.service is service
                   for h in logging.getLogger().handlers)
    assert service._thread is not None and not service._thread.is_alive()
    saved = service.journal.read_text()
    assert 'startup unavailable' in saved
    assert 'never-persist' not in saved
    entries = [json.loads(line) for line in saved.splitlines()]
    assert any(e['level'] == 'error' and 'middleware startup failed' in e['message'] for e in entries)
