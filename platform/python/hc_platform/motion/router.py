from fastapi import APIRouter
from fastapi.responses import JSONResponse
from .errors import MotionError


def create_router(service):
    router = APIRouter(prefix="/api/motion", tags=["motion"])

    async def invoke(method, *args):
        try:
            return await method(*args)
        except MotionError as exc:
            return JSONResponse(exc.as_dict(), status_code=409)

    @router.get("/capabilities")
    async def capabilities():
        return await service.capabilities()

    @router.get("/info")
    async def info():
        return await invoke(service.describe)

    @router.post("/servo-j")
    async def servo_j(payload: dict):
        return await invoke(service.servo_j, payload)

    @router.post("/servo-p")
    async def servo_p(payload: dict):
        return await invoke(service.servo_p, payload)

    @router.post("/move-j", status_code=202)
    async def move_j(payload: dict):
        return await invoke(service.move_j, payload)

    @router.post("/move-l", status_code=202)
    async def move_l(payload: dict):
        return await invoke(service.move_l, payload)

    @router.post("/move-p", status_code=202)
    async def move_p(payload: dict):
        return await invoke(service.move_p, payload)

    @router.get("/tasks")
    async def tasks():
        return await service.list_tasks()

    @router.get("/tasks/{task_id}")
    async def task(task_id: str):
        return await invoke(service.get_task, task_id)

    @router.post("/tasks/{task_id}/cancel")
    async def cancel(task_id: str):
        return await invoke(service.cancel, task_id)

    return router
