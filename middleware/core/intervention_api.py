"""DAgger session API; all capture data uses the normal recording pipeline."""
import asyncio
from aiohttp import web


def register_intervention_routes(app, runtime):
    lock = asyncio.Lock()

    def snapshot():
        return dict(ownership=runtime.ros.ownership.snapshot(),
                    capture=runtime.recorder.corrections.status() if runtime.recorder and runtime.recorder.corrections else None)

    async def status(_request):
        if runtime.ros is None:
            raise web.HTTPServiceUnavailable(text='ROS bridge not running')
        result = snapshot()
        return web.json_response({**result['ownership'], 'capture': result['capture']})

    async def action(request):
        if runtime.ros is None or runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text='control and recording services are required')
        data = await request.json()
        if not isinstance(data, dict):
            raise web.HTTPBadRequest(text='request must be an object')
        name = request.match_info['action']
        async with lock:
            control = runtime.ros.ownership
            capture = runtime.recorder.corrections
            operations = {
                'enable': control.enable, 'takeover': control.takeover, 'hold': control.hold,
                'disable': control.disable, 'recover': runtime.ros.recover_intervention,
                'return': lambda: control.start_auto(recording_ready=bool(capture and capture.status()['ready']),
                                                    control_ready=runtime.ros.control_ready()),
            }
            if name not in operations:
                raise web.HTTPNotFound(text='unknown intervention operation')
            try:
                if name == 'enable':
                    if runtime.player and runtime.player.status().get('is_active'):
                        raise ValueError('stop replay before enabling intervention')
                    if not {'motion', 'teleop'} <= control.peers.keys():
                        raise ValueError('DAgger requires the Motion backend and its teleop receiver')
                    if runtime.recorder.is_recording():
                        raise ValueError('stop ordinary recording before enabling intervention')
                control.request(data.get('request_id'), data.get('session_id', ''), data.get('expected_epoch'), operations[name])
                if name == 'enable' and runtime.recorder.corrections is None:
                    await asyncio.to_thread(runtime.recorder.prepare_corrections, runtime.session_profile_metadata)
                elif name == 'disable':
                    await asyncio.to_thread(runtime.recorder.stop_corrections)
            except (ValueError, RuntimeError, OSError) as exc:
                raise web.HTTPConflict(text=str(exc)) from exc
            return web.json_response(dict(accepted=True, **snapshot()), status=202)

    app.router.add_get('/api/control/ownership', status)
    app.router.add_post('/api/control/intervention/{action}', action)
