"""Disposable HTTP protocol service; no platform database or production credentials."""
import copy
import io
import json
import socket
import threading
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from capture_contract_support import snapshot_document, task_document
from humanoid_upload.capture_plan import reference
from humanoid_upload.common import canonical
from upload_client.tests.test_hc_artifacts import OfflinePlatform


class CaptureServer:
    def __init__(self, local):
        self.local = local
        self.token = 'hcri_' + 'a' * 32 + '.' + 'b' * 64
        self.snapshots, self.uploads = {}, {}
        self.calls = Counter()
        self.commands = []
        self.revisions = {'A': 1, 'B': 1}
        self.drop_prepare = False
        self.before_prepare = lambda command: None
        self.receipt_annotation = 'NOT_CONFIGURED'
        self.default_scheme = 'irrelevant'
        self.latest_version = 99
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_): pass
            def do_GET(self): owner.handle(self)
            def do_POST(self): owner.handle(self)
            def do_PUT(self): owner.handle(self)
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.base = f'http://127.0.0.1:{self.server.server_port}'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def task(self, task, source_format):
        return task_document(task, source_format, version=1 if task == 'A' else 2, revision=self.revisions[task])

    def send(self, request, body, status=200):
        value = canonical(body)
        request.send_response(status)
        request.send_header('Content-Type', 'application/json')
        request.send_header('Content-Length', str(len(value)))
        request.send_header('ETag', 'test')
        request.end_headers()
        request.wfile.write(value)

    def handle(self, r):
        path = urlsplit(r.path).path
        data = r.rfile.read(int(r.headers.get('Content-Length', 0)))
        if r.command == 'PUT':
            _, _, uid, aid, number = path.split('/')
            self.uploads[uid].put(aid+'/'+number, io.BytesIO(data), len(data))
            return self.send(r, {})
        body = json.loads(data) if data else {}
        if not path.endswith('connections:redeem') and r.headers.get('Authorization') != 'Bearer ' + self.token:
            return self.send(r, {'code': 'ROBOT_CREDENTIAL_INVALID'}, 401)
        if path.endswith('connections:redeem') or path.endswith('/me'):
            return self.send(r, dict(token=self.token, robot_id='robot', robot_name='Synthetic Robot',
                organization_id='org', ingest_identity_id='identity', platform_name='Isolated contract', local_robot=self.local))
        if '/tasks' in path:
            if path.endswith('/tasks'):
                self.calls['tasks'] += 1
                fmt = parse_qs(urlsplit(r.path).query)['source_format'][0]
                return self.send(r, {'items': [self.task(t, fmt) for t in 'AB'], 'total': 2,
                    'page': 1, 'page_size': 20, 'projects': []})
            task, operation = path.rsplit('/', 1)[1].split(':')
            self.calls[operation] += 1
            current = self.task(task, body['source_format'])
            if operation == 'preflight': return self.send(r, current)
            assert operation == 'prepare-recording'
            assert set(body) == {'expected_plan_revision', 'source_format', 'recording_id'}
            self.before_prepare(body)
            self.commands.append(copy.deepcopy(body))
            identity = body['recording_id']
            previous = self.snapshots.get(identity)
            if previous:
                if previous['collection_task_id'] != task or previous['plan_revision'] != body['expected_plan_revision'] or previous['source_format'] != body['source_format']:
                    return self.send(r, {'code': 'CAPTURE_RECORDING_CONFLICT'}, 409)
            else:
                if body['expected_plan_revision'] != self.revisions[task]:
                    return self.send(r, {'code': 'CAPTURE_REVISION_CONFLICT'}, 409)
                self.snapshots[identity] = snapshot_document(current, body)
            if self.drop_prepare:
                self.drop_prepare = False
                r.connection.shutdown(socket.SHUT_RDWR)
                r.connection.close()
                return
            return self.send(r, self.snapshots[identity])
        if path.endswith('/uploads'):
            self.calls['create'] += 1
            assert body['capture_plan'] == reference(self.snapshots[body['capture_plan']['recording_id']])
            assert 'capture_snapshot' not in body and 'effective_adapter' not in body
            uid = body['client_upload_id']
            backend = self.uploads.setdefault(uid, OfflinePlatform())
            return self.send(r, backend.api('POST', payload=body))
        suffix = path.split('/uploads/', 1)[1]
        uid = suffix.split('/')[0].split(':')[0]
        backend = self.uploads[uid]
        if path.endswith('/processing'):
            self.calls['processing'] += 1
            return self.send(r, dict(schema_version='openarm-processing-result/v1', upload_id=uid,
                raw_source_id='raw-test', processing_status='READY', quality_status='PASS', terminal=True,
                poll_after_seconds=0, data_status='READABLE', annotation_status=self.receipt_annotation,
                training_status='NOT_READY', episodes=[dict(source_episode_id='episode-0', source_episode_index=0,
                    episode_id='episode-0', dataset_id='dataset-'+backend.upload['manifest']['collection_task_id'],
                    status='READY', quality_status='PASS', error_code=None, retryable=False, next_action='open_episode')]))
        result = backend.api(r.command, '/'+suffix, body)
        if path.endswith(':authorize-parts'):
            for grant in result['authorizations']:
                grant['url'] = self.base+'/objects/'+uid+'/'+grant['url']
        return self.send(r, result)
