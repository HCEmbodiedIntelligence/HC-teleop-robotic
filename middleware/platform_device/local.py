"""Private same-user middleware interface for physical output and task-bound recording."""

import asyncio
import fcntl
import json
import logging
from pathlib import Path
import sqlite3
import threading
from uuid import UUID

from aiohttp import web
from .sink import local_directory

LOG = logging.getLogger("platform-device")


class Recording:
    def __init__(self, runtime, project):
        self.rt, self.project = runtime, project
        self.lock = threading.RLock()
        self.db = None

    def selection(self):
        platform = self.rt.platform
        return platform.recording_selection() if platform and self.project else {}

    def capability(self):
        s = self.selection()
        if s.get("mode") != "platform" or not s.get("task_id"):
            return {}
        return {"capture_project": self.project, "capture_tasks": [str(s["task_id"])]}

    def connect(self):
        if self.db is None:
            store = Path(self.rt.platform.directory) / "remote-capture.sqlite3"
            self.db = sqlite3.connect(store, check_same_thread=False)
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS commands(id TEXT PRIMARY KEY, command TEXT NOT NULL, receipt TEXT,sid TEXT NOT NULL,sent INTEGER NOT NULL DEFAULT 0)"
            )
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS ended(session TEXT PRIMARY KEY)"
            )
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS owners(session TEXT PRIMARY KEY, project TEXT, task TEXT, record TEXT)"
            )
            self.db.commit()
            self.recover_pending()
        return self.db

    def recover_pending(self):
        # A crash can separate a native side effect from its durable receipt.
        # Reconcile from the native journal without repeating an actuator action.
        rec = self.rt.recorder
        pending = self.db.execute(
            "SELECT id,sid,command FROM commands WHERE receipt IS NULL ORDER BY rowid"
        ).fetchall()
        with rec.lock, self.db:
            for rid, sid, body in pending:
                c = json.loads(body)
                result = dict(
                    request_id=rid,
                    state="error",
                    detail="command interrupted before confirmation",
                )
                owner = self.db.execute(
                    "SELECT record FROM owners WHERE session=? AND project=? AND task=?",
                    (sid, c["project_id"], c["task_id"]),
                ).fetchone()
                if c["action"] == "start":
                    ident = next(
                        (i for i, j in rec.jobs.items() if j.get("request_id") == rid),
                        None,
                    )
                    if ident is not None:
                        self.db.execute(
                            "INSERT OR REPLACE INTO owners VALUES(?,?,?,?)",
                            (sid, c["project_id"], c["task_id"], ident),
                        )
                        result.update(
                            record_id=ident,
                            state="recording" if rec.active == ident else "interrupted",
                            detail="recovered from native journal",
                        )
                elif owner and c["action"] == "stop":
                    state = rec.jobs.get(owner[0], {}).get("state")
                    if state in (
                        "finalizing",
                        "upload_pending",
                        "queued",
                        "ready",
                        "needs_attention",
                    ):
                        result.update(
                            record_id=owner[0],
                            state="saved",
                            upload="pending",
                            detail="local recording closed",
                        )
                self.db.execute(
                    "UPDATE commands SET receipt=?,sent=0 WHERE id=?",
                    (json.dumps(result), rid),
                )

    def handle(self, c):
        rid = str(UUID(c["request_id"]))
        sid = str(UUID(c["session_id"]))
        task = c["task_id"]
        serial = json.dumps(c, sort_keys=True)
        with self.lock:
            caps = self.capability()
            if not self.rt.platform:
                return dict(
                    request_id=rid,
                    state="error",
                    detail="recording integration unavailable",
                )
            db = self.connect()
            row = db.execute(
                "SELECT command,receipt FROM commands WHERE id=?", (rid,)
            ).fetchone()
            if row and row[0] != serial:
                raise ValueError("request id conflicts with earlier command")
            if row and row[1]:
                return json.loads(row[1])
            receipt = dict(request_id=rid, state="error")
            rec = self.rt.recorder
            if not row:
                with db:
                    db.execute(
                        "INSERT INTO commands(id,command,sid) VALUES(?,?,?)",
                        (rid, serial, sid),
                    )
            try:
                if c["project_id"] != self.project:
                    raise ValueError("project mismatch")
                if db.execute("SELECT 1 FROM ended WHERE session=?", (sid,)).fetchone():
                    raise ValueError("session has ended")
                with rec.lock:
                    owner = db.execute(
                        "SELECT project,task,record FROM owners WHERE session=?", (sid,)
                    ).fetchone()
                    action = c["action"]
                    if action == "start":
                        if task not in caps.get("capture_tasks", []):
                            raise ValueError(
                                "select this platform task on the robot first"
                            )
                        selection = dict(self.selection(), request_id=rid)
                        if rec.active and rec.jobs[rec.active]["request_id"] != rid:
                            raise ValueError("another recording is active")
                        if not rec.active:
                            bound = self.rt.platform.preflight(selection)
                            if (
                                str(bound["task"]["ownership"].get("project_id", ""))
                                != self.project
                            ):
                                raise ValueError(
                                    "device task belongs to a different project"
                                )
                            rec.start(
                                "teleop-" + rid,
                                self.rt.session_profile_metadata,
                                selection,
                            )
                        ident = rec.active
                        if not ident or rec.jobs[ident]["request_id"] != rid:
                            raise ValueError("recording is no longer active")
                        with db:
                            db.execute(
                                "INSERT OR REPLACE INTO owners VALUES(?,?,?,?)",
                                (sid, self.project, task, ident),
                            )
                        receipt.update(state="recording", record_id=ident)
                    elif owner and owner[:2] == (self.project, task):
                        ident = owner[2]
                        job = rec.jobs.get(ident, {})
                        receipt["record_id"] = ident
                        if action == "mark" and rec.active == ident:
                            if row:
                                raise ValueError(
                                    "marker outcome unknown after restart; not repeated"
                                )
                            marked = rec.mark_current_or_latest("platform:" + rid)
                            if not marked.get("marked"):
                                raise ValueError("marker not saved")
                            receipt.update(state="recording", detail="marker saved")
                        elif action == "stop":
                            if rec.active == ident:
                                rec.stop()
                            if job.get("state") not in (
                                "finalizing",
                                "upload_pending",
                                "queued",
                                "ready",
                                "needs_attention",
                            ):
                                receipt.update(
                                    state="interrupted",
                                    detail="local recording did not close normally",
                                )
                            else:
                                receipt.update(
                                    state="saved",
                                    upload="pending",
                                    detail="local recording closed",
                                )
                        else:
                            raise ValueError("requested recording is not active")
                    else:
                        raise ValueError("recording belongs to another session or task")
            except Exception as exc:
                LOG.warning("capture %s failed (%s)", rid, type(exc).__name__)
                receipt["detail"] = (
                    str(exc)[:300]
                    if isinstance(exc, ValueError)
                    else "local recorder rejected the command"
                )
            with db:
                db.execute(
                    "UPDATE commands SET receipt=? WHERE id=?",
                    (json.dumps(receipt), rid),
                )
            return receipt

    def end_session(self, sid):
        if not self.rt.platform or not self.project:
            return
        with self.lock:
            db = self.connect()
            with db:
                db.execute("INSERT OR IGNORE INTO ended VALUES(?)", (sid,))
            row = db.execute(
                "SELECT record FROM owners WHERE session=?", (sid,)
            ).fetchone()
            if not row:
                return
            rec = self.rt.recorder
            with rec.lock:
                if rec.active != row[0]:
                    return
                rec.stop()
                latest = db.execute(
                    "SELECT id,receipt FROM commands WHERE sid=? ORDER BY rowid DESC LIMIT 1",
                    (sid,),
                ).fetchone()
                if latest:
                    result = dict(
                        request_id=latest[0],
                        record_id=row[0],
                        state="interrupted",
                        detail="session ended; local capture stopped",
                    )
                    with db:
                        db.execute(
                            "UPDATE commands SET receipt=?,sent=0 WHERE id=?",
                            (json.dumps(result), latest[0]),
                        )

    def next_receipt(self):
        if not self.rt.platform or not self.project:
            return None
        with self.lock:
            db = self.connect()
            # The current platform contract has one command slot per device.
            item = db.execute(
                "SELECT id,sid,receipt,sent FROM commands WHERE receipt IS NOT NULL ORDER BY sent, rowid DESC LIMIT 1"
            ).fetchone()
            if not item:
                return None
            rid, sid, stored, sent = item
            receipt = json.loads(stored)
            state = receipt["state"]
            rec = self.rt.recorder
            with rec.lock:
                job = dict(rec.jobs.get(receipt.get("record_id"), {}))
                artifacts = list(job.get("artifacts", {}).values())
            if state == "recording" and job and job.get("state") == "interrupted":
                receipt.update(state="interrupted", detail="local recorder interrupted")
            elif state == "saved" and receipt.get("upload") == "pending":
                if job.get("state") == "needs_attention":
                    receipt.update(
                        upload="failed",
                        detail="artifact finalization requires attention",
                    )
                elif job.get("state") == "ready":
                    receipt.update(
                        upload="", detail="local artifacts ready; upload not queued"
                    )
                elif job.get("state") == "queued" and artifacts:
                    ids = {a["artifact_id"] for a in artifacts}
                    rows = [
                        r
                        for r in self.rt.platform.upload.queue.jobs()
                        if r.get("artifact_id") in ids
                    ]
                    if ids <= {
                        r["artifact_id"] for r in rows if r.get("raw_source_id")
                    }:
                        receipt.update(
                            upload="uploaded",
                            detail="platform returned source identifiers",
                        )
                    elif any(
                        r.get("terminal") and not r.get("raw_source_id") for r in rows
                    ):
                        receipt.update(
                            upload="failed", detail="upload queue reports failure"
                        )
            if receipt != json.loads(stored):
                with db:
                    db.execute(
                        "UPDATE commands SET receipt=?,sent=0 WHERE id=?",
                        (json.dumps(receipt), rid),
                    )
                sent = 0
            return None if sent else [sid, receipt]

    def confirm(self, receipt):
        if not self.rt.platform or not self.project:
            return
        with self.lock:
            db = self.connect()
            rid = receipt["request_id"]
            row = db.execute(
                "SELECT receipt FROM commands WHERE id=?", (rid,)
            ).fetchone()
            if row and row[0] and json.loads(row[0]) == receipt:
                with db:
                    db.execute("UPDATE commands SET sent=1 WHERE id=?", (rid,))


class LocalServer:
    def __init__(self, rt, cfg):
        self.rt = rt
        self.capture = Recording(rt, cfg.get("capture_project", ""))
        self.runner = None
        self.lock = None

    def snapshot(self):
        vr = self.rt.vr.status() if self.rt.vr else {}
        mux = self.rt.ros.status().get("command_mux", {}) if self.rt.ros else {}
        ready = (
            vr.get("state") == "running"
            and vr.get("active_peer") in (None, "127.0.0.2")
            and self.rt.config["vr"].get("exclusive_mode", True)
            and self.rt.config["vr"].get("publish_to_ros", True)
            and mux.get("source") == "vr"
            and mux.get("output_enabled")
            and mux.get("hardware_ready")
            and not mux.get("replay_active")
        )
        return dict(
            ready=bool(ready),
            pose_port=self.rt.config["vr"]["pose_port"],
            capture=self.capture.capability(),
        )

    async def status(self, request):
        return web.json_response(await asyncio.to_thread(self.snapshot))

    async def receipt(self, request):
        return web.json_response(await asyncio.to_thread(self.capture.next_receipt))

    async def confirm(self, request):
        await asyncio.to_thread(self.capture.confirm, await request.json())
        return web.json_response({})

    async def end(self, request):
        sid = str(UUID((await request.json())["session_id"]))
        await asyncio.to_thread(self.capture.end_session, sid)
        return web.json_response({})

    async def halt(self, request):
        if self.rt.vr and self.rt.vr.status().get("active_peer") == "127.0.0.2":
            self.rt._on_vr_timeout("platform operator stopped")
        return web.json_response({})

    async def command(self, request):
        c = await request.json()
        return web.json_response(await asyncio.to_thread(self.capture.handle, c))

    async def start(self):
        directory = local_directory()
        self.lock = (directory / "owner.lock").open("a")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        p = directory / "device.sock"
        p.unlink(missing_ok=True)
        app = web.Application(client_max_size=8192)
        app.add_routes(
            [
                web.get("/status", self.status),
                web.post("/halt", self.halt),
                web.get("/receipt", self.receipt),
                web.post("/confirm", self.confirm),
                web.post("/end", self.end),
                web.post("/capture", self.command),
            ]
        )
        self.runner = web.AppRunner(app, access_log=None)
        await self.runner.setup()
        await web.UnixSite(self.runner, str(p)).start()

    async def close(self):
        if self.runner:
            await self.runner.cleanup()
        if self.lock:
            self.lock.close()
