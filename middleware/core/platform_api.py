"""Local browser facade. Only this service exchanges tickets and reads credentials."""
import asyncio
from aiohttp import web
from urllib.parse import urlsplit

from humanoid_upload.common import UploadError


def register_platform_routes(app, runtime):
    async def dispatch(request):
        if getattr(runtime, 'platform', None) is None:
            return web.json_response({'code': 'PLATFORM_UNAVAILABLE', 'reason': '平台连接服务尚未就绪'}, status=503)
        platform = runtime.platform
        operation = request.match_info['operation']
        try:
            if request.method == 'GET':
                if operation == 'status':
                    result = platform.status()
                elif operation == 'tasks':
                    query = dict(request.query)
                    query['page'] = max(1, int(query.get('page', 1)))
                    query['page_size'] = 20
                    if set(query) - {'connection_id', 'page', 'page_size', 'q', 'project_id', 'source_format'}:
                        raise ValueError()
                    if query.get('source_format', 'MCAP') not in ('MCAP', 'LEROBOT_V3', 'CAPTURE_BUNDLE'):
                        raise ValueError()
                    result = await asyncio.to_thread(platform.tasks, **query)
                elif operation == 'recordings':
                    result = {'items': runtime.recorder.records(), 'jobs': platform.upload.jobs()}
                else:
                    raise web.HTTPNotFound()
            else:
                origin = request.headers.get('Origin')
                if origin and urlsplit(origin).netloc != request.host:
                    raise web.HTTPForbidden(text='origin mismatch')
                if request.content_type != 'application/json':
                    raise web.HTTPUnsupportedMediaType()
                body = await request.json()
                if not isinstance(body, dict):
                    raise ValueError()
                if operation == 'connect':
                    result = await asyncio.to_thread(platform.connect, body['link'])
                elif operation == 'verify':
                    result = await asyncio.to_thread(platform.verify)
                elif operation == 'auto-upload':
                    result = await asyncio.to_thread(platform.set_auto, body['enabled'])
                elif operation == 'preflight':
                    result = await asyncio.to_thread(platform.freeze, body, remember=False)
                elif operation == 'selection':
                    result = await asyncio.to_thread(runtime.recorder.set_selection, body)
                elif operation == 'upload-recording':
                    result = await asyncio.to_thread(runtime.recorder.upload_recording,
                                                      body['recording_id'], body.get('selection'))
                elif operation == 'retry-upload':
                    await asyncio.to_thread(platform.upload.queue.retry, body['job_id'])
                    with platform._lock:
                        platform.state['worker_enabled'] = True
                        platform.save()
                    platform.upload.start()
                    result = {'ok': True}
                else:
                    raise web.HTTPNotFound()
            return web.json_response(result, headers={'Cache-Control': 'no-store'})
        except UploadError as error:
            platform.error = error.code
            return web.json_response({'code': error.code, 'reason': error.action}, status=409,
                                     headers={'Cache-Control': 'no-store'})
        except (ValueError, KeyError, TypeError):
            return web.json_response({'code': 'PLATFORM_REQUEST_INVALID', 'reason': '请求格式无效'}, status=400)

    app.router.add_get('/api/platform/{operation}', dispatch)
    app.router.add_post('/api/platform/{operation}', dispatch)
