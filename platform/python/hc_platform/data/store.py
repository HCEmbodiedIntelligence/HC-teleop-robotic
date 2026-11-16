"""A process-exclusive SQLite writer, owned by one dedicated thread."""

import asyncio
import fcntl
import sqlite3
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .errors import DataError

SCHEMA = """
CREATE TABLE IF NOT EXISTS sessions (
 id TEXT PRIMARY KEY, request_id TEXT UNIQUE NOT NULL, state TEXT NOT NULL,
 spec TEXT NOT NULL, root TEXT NOT NULL, started_ns INTEGER NOT NULL,
 ended_ns INTEGER, stats TEXT NOT NULL DEFAULT '{}', error TEXT
);
CREATE TABLE IF NOT EXISTS artifacts (
 id TEXT PRIMARY KEY, session_id TEXT, state TEXT NOT NULL, format TEXT NOT NULL,
 root TEXT NOT NULL UNIQUE, manifest TEXT NOT NULL, created REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS artifact_refs (
 artifact_id TEXT NOT NULL REFERENCES artifacts(id), owner TEXT NOT NULL,
 kind TEXT NOT NULL, expires REAL, PRIMARY KEY(artifact_id,owner)
);
CREATE TABLE IF NOT EXISTS jobs (
 id TEXT PRIMARY KEY, kind TEXT NOT NULL, artifact_id TEXT NOT NULL REFERENCES artifacts(id),
 request_id TEXT UNIQUE NOT NULL, state TEXT NOT NULL, params TEXT NOT NULL,
 result TEXT NOT NULL DEFAULT '{}', error TEXT, attempts INTEGER NOT NULL DEFAULT 0,
 next_run REAL NOT NULL DEFAULT 0, cancel_requested INTEGER NOT NULL DEFAULT 0,
 created REAL NOT NULL, updated REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS jobs_due ON jobs(state,next_run);
CREATE TABLE IF NOT EXISTS job_events (
 seq INTEGER PRIMARY KEY AUTOINCREMENT, job_id TEXT NOT NULL,
 timestamp REAL NOT NULL, state TEXT NOT NULL, detail TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS settings (key TEXT PRIMARY KEY, value TEXT NOT NULL);
"""


class MetadataStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="data-sqlite"
        )
        self.connection = None
        self.lock_file = None

    async def open(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock_file = (self.directory / "writer.lock").open("a+")
        try:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock_file.close()
            self.lock_file = None
            raise DataError(
                "DATA_WRITER_EXISTS", "Another data service owns this state directory"
            ) from None

        def initialize():
            self.connection = sqlite3.connect(
                self.directory / "metadata.sqlite3", timeout=5
            )
            self.connection.row_factory = sqlite3.Row
            self.connection.execute("PRAGMA journal_mode=WAL")
            self.connection.execute("PRAGMA synchronous=FULL")
            self.connection.execute("PRAGMA foreign_keys=ON")
            self.connection.executescript(SCHEMA)
            self.connection.commit()

        try:
            await asyncio.get_running_loop().run_in_executor(self.executor, initialize)
        except BaseException:
            await self.close()
            raise

    async def run(self, operation):
        if self.connection is None:
            raise DataError("DATA_SERVICE_CLOSED")

        def transaction():
            with self.connection:
                return operation(self.connection)

        return await asyncio.get_running_loop().run_in_executor(
            self.executor, transaction
        )

    async def close(self):
        if self.connection is not None:

            def finish():
                self.connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
                self.connection.close()
                self.connection = None

            await asyncio.get_running_loop().run_in_executor(self.executor, finish)
        self.executor.shutdown(wait=True)
        if self.lock_file is not None:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_UN)
            self.lock_file.close()
            self.lock_file = None
