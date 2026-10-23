"""DAgger session API; all capture data uses the normal recording pipeline."""
import asyncio
from aiohttp import web


def register_intervention_routes(app, runtime):
    lock = asyncio.Lock()
    pending_capture = None

    def snapshot():
        return dict(ownership=runtime.ros.ownership.snapshot(),
                    capture=runtime.recorder.corrections.status() if runtime.recorder and runtime.recorder.corrections else None)

    async def status(_request):
        if runtime.ros is None:
            raise web.HTTPServiceUnavailable(text='ROS bridge not running')
        result = snapshot()
        return web.json_response({**result['ownership'], 'capture': result['capture']})

    async def action(request):
        nonlocal pending_capture
        if runtime.ros is None or runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text='control and recording services are required')
        try:
            data = await request.json()
        except ValueError as exc:
            raise web.HTTPBadRequest(text='request must contain valid JSON') from exc
        if not isinstance(data, dict):
            raise web.HTTPBadRequest(text='request must be an object')
        name = request.match_info['action']
        capture_mode = data.get('capture_mode', 'corrections_only')
        if name == 'enable' and capture_mode not in ('corrections_only', 'full_session'):
            raise web.HTTPBadRequest(text='capture_mode must be corrections_only or full_session')
        async with lock:
            control = runtime.ros.ownership
            capture = runtime.recorder.corrections
            operations = {
                'enable': lambda: control.enable(preparing_capture=True),
                'takeover': control.takeover, 'hold': control.hold,
                'disable': control.disable, 'recover': runtime.ros.recover_intervention,
                'return': lambda: control.start_auto(recording_ready=bool(capture and capture.status()['ready']),
                                                    control_ready=runtime.ros.control_ready()),
            }
            if name not in operations:
                raise web.HTTPNotFound(text='unknown intervention operation')
            try:
                if name == 'enable':
                    if capture is not None and capture.capture_mode != capture_mode:
                        raise ValueError('exit the intervention session before changing capture mode')
                    if runtime.player and runtime.player.status().get('is_active'):
                        raise ValueError('stop replay before enabling intervention')
                    if not {'motion', 'teleop'} <= control.peers.keys():
                        raise ValueError('DAgger requires the Motion backend and its teleop receiver')
                    if runtime.recorder.is_recording() and capture is None:
                        raise ValueError('stop ordinary recording before enabling intervention')
                def apply():
                    nonlocal pending_capture
                    operations[name]()
                    pending_capture = (name, data['request_id'], control.session, control.epoch, capture_mode)

                control.request(data.get('request_id'), data.get('session_id', ''), data.get('expected_epoch'), apply)
                # The authority transaction deduplicates request IDs. Its disk
                # effect must also be deduplicated: a delayed old disable must
                # never stop a newer capture. A failed disk operation may only
                # retry while its original authority generation is current.
                pending = pending_capture == (name, data.get('request_id'), control.session, control.epoch, capture_mode)
                if pending and name == 'enable' and runtime.recorder.corrections is None:
                    metadata = {**runtime.session_profile_metadata,
                                'controller_instance_id': control.instance, 'session_id': control.session}
                    await asyncio.to_thread(runtime.recorder.prepare_corrections, metadata, capture_mode=capture_mode)
                    capture = runtime.recorder.corrections
                    if capture and capture.status()['ready']:
                        control.finish_capture_preparation(metadata['session_id'])
                elif pending and name == 'disable':
                    await asyncio.to_thread(runtime.recorder.stop_corrections)
                if pending:
                    pending_capture = None
            except (ValueError, RuntimeError, OSError) as exc:
                raise web.HTTPConflict(text=str(exc)) from exc
            return web.json_response(dict(accepted=True, **snapshot()), status=202)

    app.router.add_get('/api/control/ownership', status)
    app.router.add_post('/api/control/intervention/{action}', action)
