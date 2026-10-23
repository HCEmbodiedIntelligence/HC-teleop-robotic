"""Local browser facade. Only this service exchanges tickets and reads credentials."""
import asyncio
import logging
from aiohttp import web
from urllib.parse import urlsplit

from humanoid_upload.common import UploadError

logger = logging.getLogger(__name__)


def register_platform_routes(app, runtime):
    async def dispatch(request):
        if getattr(runtime, 'platform', None) is None:
            return web.json_response({
                'code': 'PLATFORM_UNAVAILABLE',
                'reason': '采集端的平台连接服务未启动，请重启采集端服务；如果仍然失败，请更新采集端程序',
                'retryable': True,
            }, status=503, headers={'Cache-Control': 'no-store'})
        platform = runtime.platform
        connection_generation = getattr(platform, '_connection_generation', None)
        operation = request.match_info['operation']
        try:
            if request.method == 'GET':
                if operation == 'status':
                    result = await asyncio.to_thread(platform.status)
                    local_fmt = getattr(runtime.recorder, 'save_format', None)
                    if not local_fmt and hasattr(runtime, 'config'):
                        local_fmt = runtime.config.get('ros', {}).get('recording', {}).get('format')
                    result = {**result, 'local_format': local_fmt or 'both'}
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
                    result = {'items': runtime.recorder.upload_records(), 'jobs': platform.upload.jobs()}
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
                elif operation == 'upload-format':
                    result = await asyncio.to_thread(runtime.recorder.set_upload_format, body['format'])
                    local_fmt = getattr(runtime.recorder, 'save_format', None)
                    if not local_fmt and hasattr(runtime, 'config'):
                        local_fmt = runtime.config.get('ros', {}).get('recording', {}).get('format')
                    result['local_format'] = local_fmt or 'both'
                elif operation == 'auto-upload':
                    result = await asyncio.to_thread(platform.set_auto, body['enabled'])
                elif operation == 'preflight':
                    result = await asyncio.to_thread(platform.preflight, body, remember=False)
                elif operation == 'selection':
                    result = await asyncio.to_thread(runtime.recorder.set_selection, body)
                elif operation == 'upload-recording':
                    result = await asyncio.to_thread(runtime.recorder.upload_recording,
                                                      body['recording_id'], body.get('selection'),
                                                      body.get('source_format'), body.get('new_upload') is True)
                elif operation == 'upload-recordings':
                    recordings = body.get('recordings')
                    selection = body.get('selection')
                    if (not isinstance(recordings, list) or not 1 <= len(recordings) <= 100
                            or not isinstance(selection, dict)):
                        raise ValueError()
                    uploaded, failed = [], []
                    for item in recordings:
                        try:
                            if not isinstance(item, dict) or set(item) != {'recording_id', 'source_format'}:
                                raise ValueError()
                            response = await asyncio.to_thread(
                                runtime.recorder.upload_recording, item['recording_id'], selection,
                                item['source_format'], True)
                            uploaded.append({'recording_id': item['recording_id'], **response})
                        except UploadError as error:
                            failed.append({'recording_id': item.get('recording_id', ''),
                                           'code': error.code, 'reason': error.action})
                    result = {'ok': not failed, 'uploaded': uploaded, 'failed': failed}
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
            if connection_generation == getattr(platform, '_connection_generation', None):
                platform.error = error.code
            # Public code/action only: never log a link, ticket, token or response body.
            logger.warning('Platform operation %s failed: %s; %s', operation, error.code, error.action)
            return web.json_response({'code': error.code, 'reason': error.action,
                                      'retryable': error.retry}, status=409,
                                     headers={'Cache-Control': 'no-store'})
        except (ValueError, KeyError, TypeError):
            return web.json_response({'code': 'PLATFORM_REQUEST_INVALID', 'reason': '请求格式无效'}, status=400)

    app.router.add_get('/api/platform/{operation}', dispatch)
    app.router.add_post('/api/platform/{operation}', dispatch)
