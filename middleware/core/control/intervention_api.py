"""DAgger session API; all capture data uses the normal recording pipeline."""
import asyncio
from aiohttp import web


async def _capture_work(operation, *args, **kwargs):
    """Do not release the lifecycle lease while a cancelled disk job still runs."""
    job = asyncio.create_task(asyncio.to_thread(operation, *args, **kwargs))
    try:
        return await asyncio.shield(job)
    except asyncio.CancelledError:
        # Cancelling to_thread only cancels the waiter. Its thread can still
        # create capture processes, so shutdown must wait for actual completion.
        while not job.done():
            try:
                await asyncio.shield(job)
            except asyncio.CancelledError:
                continue
            except Exception:
                break
        # Retrieve a failure so a cancelled HTTP request cannot leave an
        # unobserved task error; the original cancellation still propagates.
        if not job.cancelled():
            job.exception()
        raise


def register_intervention_routes(app, runtime):
    lock = getattr(runtime, '_intervention_lifecycle_lock', None)
    if lock is None:
        lock = runtime._intervention_lifecycle_lock = asyncio.Lock()
    pending_capture = None

    def snapshot():
        ownership = runtime.ros.ownership.snapshot()
        if getattr(runtime.ros, '_legacy_enabled', False) and runtime.ros._boundary:
            ownership['legacy_control'] = runtime.ros._boundary.status()
        return dict(ownership=ownership,
                    capture=runtime.recorder.corrections.status() if runtime.recorder and runtime.recorder.corrections else None)

    async def status(_request):
        if runtime.ros is None:
            raise web.HTTPServiceUnavailable(text='ROS bridge not running')
        result = snapshot()
        return web.json_response({**result['ownership'], 'capture': result['capture']})

    async def action(request):
        nonlocal pending_capture
        if runtime.ros is None:
            raise web.HTTPServiceUnavailable(text='control service is required')
        if getattr(runtime, '_stopping', False):
            raise web.HTTPServiceUnavailable(text='middleware is stopping')
        isolated = getattr(runtime.ros, 'isolated_control_process', False) is True
        # Capture at arrival, before body decoding or waiting on disk work. A
        # later stop must invalidate this authorization, even while queued.
        expected_fence = runtime.ros.authorization_fence() if isolated else None
        try:
            data = await request.json()
        except ValueError as exc:
            raise web.HTTPBadRequest(text='request must contain valid JSON') from exc
        if not isinstance(data, dict):
            raise web.HTTPBadRequest(text='request must be an object')
        name = request.match_info['action']
        capture_mode = data.get('capture_mode', 'corrections_only')
        if name in {'enable', 'enable_dagger'} and capture_mode not in ('corrections_only', 'full_session'):
            raise web.HTTPBadRequest(text='capture_mode must be corrections_only or full_session')
        async with lock:
            if getattr(runtime, '_stopping', False):
                raise web.HTTPServiceUnavailable(text='middleware is stopping')
            control = runtime.ros.ownership
            capture = runtime.recorder.corrections if runtime.recorder else None
            recording = control.recording_enabled
            if recording and name in {'enable', 'enable_dagger'} and runtime.recorder is None:
                raise web.HTTPServiceUnavailable(text='recording service is required when automatic recording is enabled')
            operations = {
                'enable': lambda: control.enable(preparing_capture=recording),
                'start_model': lambda: control.start_model(control_ready=runtime.ros.control_ready(),
                    recording_ready=bool(capture and capture.status()['ready'])),
                'enable_dagger': lambda: control.set_dagger(True, preparing_capture=recording),
                'disable_dagger': lambda: control.set_dagger(False),
                'takeover': control.takeover, 'hold': control.hold,
                'disable': control.disable, 'recover': runtime.ros.recover_intervention,
                'return': lambda: control.start_auto(recording_ready=bool(capture and capture.status()['ready']),
                                                    control_ready=runtime.ros.control_ready()),
            }
            if name not in operations:
                raise web.HTTPNotFound(text='unknown intervention operation')
            try:
                if name == 'start_model' and runtime.player and runtime.player.status().get('is_active'):
                    raise ValueError('stop replay before starting model control')
                if name in {'enable', 'enable_dagger'}:
                    if capture is not None and capture.capture_mode != capture_mode:
                        raise ValueError('exit the intervention session before changing capture mode')
                    if runtime.player and runtime.player.status().get('is_active'):
                        raise ValueError('stop replay before enabling intervention')
                    if not control.required_peers <= control.peers.keys():
                        raise ValueError('DAgger requires control participants: ' + ', '.join(sorted(control.required_peers)))
                    if recording and runtime.recorder.is_recording() and capture is None:
                        raise ValueError('stop ordinary recording before enabling intervention')
                def apply():
                    nonlocal pending_capture
                    operations[name]()
                    pending_capture = (name, data['request_id'], control.session, control.epoch, capture_mode)

                if isolated:
                    result = await asyncio.to_thread(runtime.ros.apply_intervention,
                        name, data, recording_ready=bool(capture and capture.status().get('ready')),
                        preparing_capture=recording, expected_fence=expected_fence)
                    generation = result['ownership']
                    session_id, epoch = generation['session_id'], generation['epoch']
                    instance = generation['controller_instance_id']
                    recording = generation['recording_enabled']
                    if result['applied']:
                        pending_capture = (name, data['request_id'], session_id, epoch, capture_mode)
                else:
                    generation = control.request(data.get('request_id'), data.get('session_id', ''),
                                                 data.get('expected_epoch'), apply)
                    session_id, epoch, instance = control.session, control.epoch, control.instance
                # The authority transaction deduplicates request IDs. Its disk
                # effect must also be deduplicated: a delayed old disable must
                # never stop a newer capture. A failed disk operation may only
                # retry while its original authority generation is current.
                pending = pending_capture == (name, data.get('request_id'), session_id, epoch, capture_mode)
                if pending and recording and name in {'enable', 'enable_dagger'} and runtime.recorder.corrections is None:
                    metadata = {**runtime.session_profile_metadata,
                                'controller_instance_id': instance, 'session_id': session_id}
                    await _capture_work(runtime.recorder.prepare_corrections, metadata, capture_mode=capture_mode)
                    capture = runtime.recorder.corrections
                    if capture and capture.status()['ready']:
                        if isolated:
                            await asyncio.to_thread(control.finish_capture_preparation, metadata['session_id'])
                        else:
                            control.finish_capture_preparation(metadata['session_id'])
                elif pending and recording and name in {'disable', 'disable_dagger'} and runtime.recorder is not None:
                    await _capture_work(runtime.recorder.stop_corrections)
                if pending:
                    pending_capture = None
            except (ValueError, RuntimeError, OSError) as exc:
                raise web.HTTPConflict(text=str(exc)) from exc
            return web.json_response(dict(accepted=True, **snapshot()), status=202)

    app.router.add_get('/api/control/ownership', status)
    app.router.add_post('/api/control/intervention/{action}', action)
