from __future__ import annotations

import asyncio
import copy
from typing import Any

from aiohttp import web

from .config import ConfigStore
from .robot_profiles import RobotProfileError, RobotProfileManager, STANDARD_TOPICS


def register_profile_routes(
    app: web.Application,
    store: ConfigStore,
    runtime: Any,
    profiles: RobotProfileManager,
) -> None:
    """Register profile management without coupling it to the runtime module."""

    async def get_robot_profiles(_request: web.Request) -> web.Response:
        active = str(store.value["robot_profiles"].get("active", ""))
        values = profiles.list()
        active_topics = dict(STANDARD_TOPICS)
        if active:
            try:
                active_topics = profiles.get_profile_topics(active)
            except Exception:
                pass
        for value in values:
            value["active"] = value["id"] == active
        return web.json_response(
            {
                "active": active,
                "root": str(profiles.root),
                "profiles": values,
                "standard_topics": active_topics,
            }
        )

    async def import_robot_profile(request: web.Request) -> web.Response:
        if not request.content_type.startswith("multipart/"):
            raise web.HTTPBadRequest(text="multipart form data is required")
        fields: dict[str, str] = {}
        uploads: dict[str, tuple[str, bytes]] = {}
        try:
            reader = await request.multipart()
            async for part in reader:
                if part.name in {"archive", "file", "zip", "urdf", "config"}:
                    if not part.filename:
                        raise RobotProfileError(f"{part.name} file is required")
                    uploads[part.name] = (
                        part.filename,
                        await part.read(decode=False),
                    )
                elif part.name in {"id", "display_name"}:
                    fields[part.name] = (await part.text()).strip()

            archive_key = next(
                (key for key in ("archive", "file", "zip") if key in uploads), None
            )
            if archive_key:
                archive_name, archive_payload = uploads[archive_key]
                profile = await asyncio.to_thread(
                    profiles.import_archive,
                    fields.get("id", ""),
                    fields.get("display_name", ""),
                    archive_payload,
                    archive_name,
                )
            elif "urdf" in uploads and "config" in uploads:
                urdf_name, urdf_payload = uploads["urdf"]
                config_name, config_payload = uploads["config"]
                profile = await asyncio.to_thread(
                    profiles.import_profile,
                    fields.get("id", ""),
                    fields.get("display_name", ""),
                    urdf_name,
                    urdf_payload,
                    config_name,
                    config_payload,
                )
            else:
                raise RobotProfileError("robot zip archive is required")
            return web.json_response({"ok": True, "profile": profile}, status=201)
        except RobotProfileError as exc:
            if "already exists" in str(exc):
                raise web.HTTPConflict(text=str(exc)) from exc
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def activate_robot_profile(request: web.Request) -> web.Response:
        profile_id = request.match_info["profile_id"]
        try:
            profile = profiles.get(profile_id)
        except RobotProfileError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        if profile.get("schema") == "invalid":
            raise web.HTTPBadRequest(text="invalid robot profile cannot be activated")
        previous = str(store.value["robot_profiles"].get("active", ""))
        if profile_id != previous:
            proposed = copy.deepcopy(store.value)
            proposed["robot_profiles"]["active"] = profile_id
            saved = store.save(proposed)
            runtime.config = saved
            runtime._on_safety_event(
                f"robot profile changed from {previous or 'none'} to {profile_id}; restart teleop"
            )
        return web.json_response(
            {
                "ok": True,
                "active": profile_id,
                "profile": profile,
                "restart_simulation_required": profile_id != previous,
            }
        )

    async def delete_robot_profile(request: web.Request) -> web.Response:
        profile_id = request.match_info["profile_id"]
        try:
            await asyncio.to_thread(profiles.delete_profile, profile_id)
            active = str(store.value["robot_profiles"].get("active", ""))
            cleared = profile_id == active
            if cleared:
                proposed = copy.deepcopy(store.value)
                proposed["robot_profiles"]["active"] = ""
                saved = store.save(proposed)
                runtime.config = saved
            return web.json_response(
                {"ok": True, "deleted": profile_id, "cleared_active": cleared}
            )
        except RobotProfileError as exc:
            if "does not exist" in str(exc):
                raise web.HTTPNotFound(text=str(exc)) from exc
            raise web.HTTPBadRequest(text=str(exc)) from exc

    app.router.add_get("/api/robot-profiles", get_robot_profiles)
    app.router.add_post("/api/robot-profiles/import", import_robot_profile)
    app.router.add_post(
        "/api/robot-profiles/{profile_id}/activate", activate_robot_profile
    )
    app.router.add_delete(
        "/api/robot-profiles/{profile_id}", delete_robot_profile
    )
