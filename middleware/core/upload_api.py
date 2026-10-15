"""HC upload routes; file import remains /api/recordings/upload."""
import asyncio
from aiohttp import web
from humanoid_upload.common import UploadError


def register_upload_routes(app, service):
    async def dispatch(request):
        operation = request.match_info.get('operation', 'status')
        try:
            if request.method == 'GET':
                if operation != 'status':
                    raise web.HTTPNotFound()
                result = await asyncio.to_thread(service.status)
            else:
                body = await request.json()
                if not isinstance(body, dict):
                    raise ValueError()
                service.require_enabled()
                if operation == 'binding':
                    result = await asyncio.to_thread(service.set_binding, body)
                elif operation == 'enqueue':
                    result = await asyncio.to_thread(service.enqueue, body['artifact_id'], body.get('binding_id'))
                elif operation == 'seal-raw':
                    result = await asyncio.to_thread(service.seal_raw, body['recording_id'], body.get('binding_id'))
                elif operation == 'retry':
                    if any(type(body[k]) is not bool for k in ('resume', 'processing') if k in body):
                        raise ValueError()
                    result = await asyncio.to_thread(service.queue.retry, body['job_id'],
                                                     body.get('resume', False), body.get('processing', False))
                    from humanoid_upload.queue import public_job
                    result = public_job(result)
                elif operation == 'cancel':
                    result = await asyncio.to_thread(service.cancel_artifact, body['artifact_id'])
                elif operation == 'scan':
                    result = await asyncio.to_thread(service.scan)
                else:
                    raise web.HTTPNotFound()
            return web.json_response(result, headers={'Cache-Control': 'no-store'})
        except UploadError as error:
            return web.json_response({'code': error.code, 'action': error.action},
                status=409 if error.code in {'ARTIFACT_BUSY', 'WORKER_ALREADY_RUNNING'} else 400)
        except (KeyError, ValueError, TypeError):
            return web.json_response({'code': 'UPLOAD_REQUEST_INVALID'}, status=400)

    app.router.add_get('/api/uploads', dispatch)
    app.router.add_post('/api/uploads/{operation}', dispatch)
