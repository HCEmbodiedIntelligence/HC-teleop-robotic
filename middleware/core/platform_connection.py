"""Service-owned platform connections; browser requests never carry long-lived tokens."""
from __future__ import annotations

import copy
import hashlib
import json
import os
import threading
from pathlib import Path
from urllib.parse import parse_qs, quote, urlencode, urlsplit, urlunsplit
from uuid import uuid4

from humanoid_upload.common import UploadError, atomic_json, canonical, read_json
from humanoid_upload.http import Http

from .upload_service import UploadService


def local_robot(config, config_dir):
    """Read the profile actually selected by the running service, never a recording file."""
    import yaml
    profile_id = str(os.environ.get('HC_ROBOT_NAME') or config.get('robot_profiles', {}).get('active', '')).strip()
    root = Path(os.environ.get('HC_ROBOT_CONFIG_ROOT') or config.get('robot_profiles', {}).get('root', '../adapters/robots'))
    root = root if root.is_absolute() else Path(config_dir) / root
    path = root / profile_id
    try:
        profile = yaml.safe_load((path / 'profile.yaml').read_text())
        controller_name = profile.get('primary_config', 'controller_v23.yml')
        controller = yaml.safe_load((path / controller_name).read_text())
        # Fingerprint the hardware model and selected profile, excluding connection
        # settings, camera rates and other operational preferences.
        hardware = {'profile_id': profile_id, 'robot_name': profile.get('robot_name'),
                    'model': controller.get('model', {})}
        urdf = path / profile.get('urdf', '')
        if urdf.is_file():
            hardware['urdf_sha256'] = hashlib.sha256(urdf.read_bytes()).hexdigest()
        identity = str(profile.get('local_robot_id') or profile.get('robot_name') or profile_id)
        return dict(local_robot_id=identity, profile_id=profile_id,
                    config_fingerprint=hashlib.sha256(canonical(hardware)).hexdigest(),
                    serial_no=profile.get('serial_no'), model_version_id=profile.get('model_version_id'))
    except (OSError, ValueError, TypeError, AttributeError):
        raise UploadError('LOCAL_ROBOT_CONFIG_INVALID', '无法读取正在使用的机器人配置，请先检查机器人配置') from None


class PlatformConnection:
    def __init__(self, config, config_dir, data_directory, *, identity_provider=None):
        self.config, self.config_dir = copy.deepcopy(config), Path(config_dir)
        legacy = copy.deepcopy(config.get('uploads', {}))
        queue = Path(legacy.get('queue_directory', '../runtime/upload_queue')).expanduser()
        queue = queue if queue.is_absolute() else self.config_dir / queue
        self.directory = queue.resolve()
        self._lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self.activity_provider = lambda: 'IDLE'
        self.identity_provider = identity_provider or (lambda: local_robot(self.config, self.config_dir))
        artifact_dir = config['ros']['recording'].get('artifact_state_dir')
        artifact_dir = Path(artifact_dir) if artifact_dir else self.directory / 'artifacts'
        if not artifact_dir.is_absolute(): artifact_dir = self.config_dir / artifact_dir
        self.legacy_auto = bool(legacy.get('enabled') and legacy.get('auto_enqueue'))
        self.upload = UploadService({**legacy, 'enabled': True, 'auto_enqueue': False,
                                    'queue_directory': str(self.directory)},
                                   artifact_state_dir=artifact_dir, data_directory=data_directory)
        self.path = self.directory / 'connections.json'
        self.state = read_json(self.path) if self.path.exists() else {
            'version': 1, 'connections': {}, 'current': None, 'recent': None,
            'auto_upload': bool(legacy.get('auto_enqueue', False)),
            'worker_enabled': bool(legacy.get('enabled', False)),
        }
        self.error = None
        for entry in self.state['connections'].values():
            entry['verified'] = False
        if not self.state['current'] and self.upload.default_binding_id:
            binding = self.upload.bindings[self.upload.default_binding_id]
            connection = {k: binding[k] for k in self.upload.binding_fields if k != 'task_id'}
            connection.update(robot_name=binding['robot_id'], platform_name='HC DataPlatform',
                              organization_id=None, local_robot=None, verified=False)
            identity = hashlib.sha256(canonical(connection)).hexdigest()
            self.state['connections'][identity] = connection
            self.state['current'] = identity
            self.state['recent'] = {'connection_id': identity, 'task_id': binding['task_id']}
            self.save()

    def save(self):
        atomic_json(self.path, self.state)

    def connection(self, identity=None):
        with self._lock:
            identity = identity or self.state['current']
            if identity not in self.state['connections']:
                raise UploadError('CONNECTION_REQUIRED', '请先粘贴平台连接链接')
            return copy.deepcopy(self.state['connections'][identity])

    def request(self, connection, method, endpoint, payload=None, *, redeem=False):
        cfg = {**self.upload.cfg, **connection}
        http = Http(cfg)
        url = connection['platform_base_url'] + '/api/v1/robot-ingest/' + endpoint
        if redeem:
            # One-time ticket in POST body, without old platform Authorization.
            return http.request(method, url, payload=payload, anonymous=True)
        return http.request(method, url, payload=payload)

    def connect(self, link):
        try:
            parts = urlsplit(link.strip())
            ticket = parse_qs(parts.fragment, strict_parsing=True)['robot-connect'][0]
            if (parts.scheme not in ('http', 'https') or not parts.hostname or parts.username
                    or parts.password or parts.query or len(ticket) < 32 or len(link) > 4096):
                raise ValueError()
            base = urlunsplit((parts.scheme, parts.netloc, parts.path.rstrip('/'), '', ''))
        except (ValueError, KeyError, TypeError, AttributeError):
            raise UploadError('CONNECTION_LINK_INVALID', '请粘贴平台“接入详情”复制的完整连接链接') from None
        with self._lock:
            local = self.identity_provider()
            key = hashlib.sha256(canonical([base, ticket])).hexdigest()
            pending_path = self.directory / 'connection-attempts' / (key + '.json')
            # Persist only a hash of the ticket. The operator can paste the same
            # link after a crash; the original request ID and configuration survive.
            if pending_path.exists():
                pending = read_json(pending_path)
                if pending['local_robot'] != local:
                    raise UploadError('LOCAL_ROBOT_MISMATCH', '连接重试期间机器人配置已变化')
            else:
                pending = {'request_id': uuid4().hex, 'local_robot': local}
                atomic_json(pending_path, pending)
            response = self.request({'platform_base_url': base}, 'POST', 'connections:redeem',
                                    {**pending, 'ticket': ticket}, redeem=True)
            token = response.get('token')
            if (not isinstance(token, str) or not token.startswith('hcri_')
                    or response.get('local_robot') != local or not response.get('robot_id')):
                raise UploadError('CONNECTION_RESPONSE_INVALID')
            # A managed credential slot is scoped to one platform/org/identity.
            # Renewing that identity repairs old jobs without rewriting their
            # frozen target. External legacy credential references stay intact.
            credential_key = hashlib.sha256(canonical([base, response['organization_id'],
                response['robot_id'], response.get('ingest_identity_id') or key])).hexdigest()
            credential_path = self.directory / 'credentials' / (credential_key + '.token')
            if credential_path.parent.is_symlink() or not credential_path.parent.resolve().is_relative_to(self.directory):
                raise UploadError('CREDENTIAL_FILE_PERMISSIONS', '凭据目录不能使用外部链接')
            credential_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if credential_path.is_symlink():
                raise UploadError('CREDENTIAL_FILE_PERMISSIONS')
            import tempfile
            fd, tmp = tempfile.mkstemp(dir=credential_path.parent)
            try:
                with os.fdopen(fd, 'w') as out:
                    out.write(token); out.flush(); os.fsync(out.fileno())
                os.replace(tmp, credential_path)
                parent_fd = os.open(credential_path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try: os.fsync(parent_fd)
                finally: os.close(parent_fd)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            connection = {k: response[k] for k in ('organization_id', 'robot_id', 'robot_name', 'platform_name', 'local_robot')}
            connection.update(platform_base_url=base, credential_ref='file:' + str(credential_path),
                              local_robot_id=local['local_robot_id'], verified=True,
                              ingest_identity_id=response.get('ingest_identity_id'))
            identity = hashlib.sha256(canonical(connection)).hexdigest()
            first = not self.state['connections']
            next_state = copy.deepcopy(self.state)
            next_state['connections'][identity] = connection
            next_state['current'] = identity
            if identity != self.state['current']:
                next_state['recent'] = None
                next_state['recording_selection'] = {'mode': 'local'}
            if first:
                next_state['auto_upload'] = True
            next_state['worker_enabled'] = True
            atomic_json(self.path, next_state)
            self.state = next_state
            self.error = None
            self.upload.start()
        # Failure to list tasks doesn't destroy a successfully saved connection.
        return self.status()

    def verify(self):
        connection_id = self.state['current']
        if not connection_id:
            return self.status()
        connection = self.connection(connection_id)
        info = self.request(connection, 'GET', 'me')
        if info['robot_id'] != connection['robot_id']:
            raise UploadError('ROBOT_IDENTITY_MISMATCH')
        local = self.identity_provider()
        if info.get('local_robot') and info['local_robot'] != local:
            raise UploadError('LOCAL_ROBOT_MISMATCH', '请检查当前机器人配置，不能以连接绕过模型校验')
        with self._lock:
            connection.update({k: info[k] for k in ('organization_id', 'robot_name', 'platform_name')})
            connection['verified'] = True
            self.state['connections'][connection_id] = connection
            self.save()
            self.error = None
        return self.status()

    def tasks(self, **query):
        identity = query.pop('connection_id', None) or self.state['current']
        connection = self.connection(identity)
        source_format = self.source_format
        result = self.request(connection, 'GET', 'tasks?' + urlencode({'source_format': source_format, **query}))
        return {**result, 'connection_id': identity}

    @property
    def source_format(self):
        return 'LEROBOT_V3' if self.config['ros']['recording'].get('format') == 'lerobot' else 'MCAP'

    def freeze(self, selection, *, remember=True, source_format=None):
        if not isinstance(selection, dict) or selection.get('mode') not in ('local', 'platform'):
            raise UploadError('RECORDING_SELECTION_REQUIRED', '请选择平台任务或“暂不关联平台任务”')
        if selection['mode'] == 'local':
            return None
        identity = selection.get('connection_id')
        if not identity or not selection.get('task_id'):
            raise UploadError('TASK_REQUIRED', '请先选择采集任务')
        if remember and identity != self.state['current']:
            raise UploadError('CONNECTION_CHANGED', '平台连接已变化，请刷新页面并重新选择任务')
        connection = self.connection(identity)
        local = self.identity_provider()
        if connection.get('local_robot') and local != connection['local_robot']:
            raise UploadError('LOCAL_ROBOT_MISMATCH', '当前配置与连接设备不一致，请检查机器人配置')
        task = self.request(connection, 'POST', 'tasks/' + quote(selection['task_id'], safe='') + ':preflight',
                            {'source_format': source_format or self.source_format, 'local_robot': local})
        if not task.get('selectable') or task['collection_task_id'] != selection['task_id']:
            raise UploadError('TASK_UNAVAILABLE', '任务已不可用，请刷新；不会自动选择其他任务')
        ownership = task['ownership']
        if connection.get('organization_id') and ownership['organization_id'] != connection['organization_id']:
            raise UploadError('ROBOT_IDENTITY_MISMATCH')
        with self._lock:
            if remember and identity != self.state['current']:
                raise UploadError('CONNECTION_CHANGED', '平台连接已变化，请刷新页面并重新选择任务')
            values = {k: connection[k] for k in self.upload.binding_fields if k != 'task_id'}
            binding = self.upload.set_binding({**values, 'task_id': selection['task_id']})
            if remember:
                self.state['recent'] = {'connection_id': identity, 'task_id': selection['task_id']}
                self.save()
            return {**binding, 'connection_id': identity, 'task': task, 'local_robot': local,
                    'auto_upload': self.state['auto_upload'] and self.state['worker_enabled']}

    def recording_selection(self):
        with self._lock:
            # Older releases only remembered a recent upload task. That is not
            # consent to use it for future captures, so default to local capture.
            return copy.deepcopy(self.state.get('recording_selection', {'mode': 'local'}))

    def set_recording_selection(self, selection):
        with self._lock:
            self.freeze(selection)
            value = {'mode': selection['mode']}
            if value['mode'] == 'platform':
                value.update(connection_id=selection['connection_id'], task_id=selection['task_id'])
            next_state = {**self.state, 'recording_selection': value}
            atomic_json(self.path, next_state)
            self.state = next_state
            return {'recording_selection': copy.deepcopy(value)}

    def set_auto(self, enabled):
        if type(enabled) is not bool:
            raise UploadError('UPLOAD_REQUEST_INVALID')
        with self._lock:
            self.state['auto_upload'] = enabled
            if enabled: self.state['worker_enabled'] = True
            self.save()
        if enabled: self.upload.start()
        return self.status()

    def status(self):
        with self._lock:
            current = self.state['current']
            connection = self.state['connections'].get(current)
            return {'connection': ({k: connection[k] for k in
                    ('platform_base_url', 'platform_name', 'robot_id', 'robot_name', 'organization_id', 'verified')}
                    | {'id': current}) if connection else None,
                    'recent': copy.deepcopy(self.state['recent']), 'error': self.error,
                    'recording_selection': self.recording_selection(),
                    'auto_upload': self.state['auto_upload'], 'jobs': self.upload.jobs()}

    def start(self):
        self._resume_legacy()
        if self.state['worker_enabled']: self.upload.start()
        if self._heartbeat_thread is None or not self._heartbeat_thread.is_alive():
            self._heartbeat_stop.clear()
            self._heartbeat_thread = threading.Thread(target=self._heartbeat, name='platform-heartbeat', daemon=True)
            self._heartbeat_thread.start()

    def _heartbeat(self):
        while not self._heartbeat_stop.wait(60):
            self._resume_legacy()
            if not self.state['current']: continue
            try:
                self.request(self.connection(), 'POST', 'heartbeat', {'activity': self.activity_provider()})
            except UploadError as error:
                self.error = error.code
            except Exception:
                self.error = 'HEARTBEAT_UNAVAILABLE'

    def _resume_legacy(self):
        if not self.legacy_auto or not self.state['worker_enabled']: return
        for path in self.upload.state_dir.glob('*.json'):
            try:
                descriptor = read_json(path)
                # Only complete legacy captures with an existing frozen binding.
                if not descriptor.get('binding_id'): continue
                if (self.directory/'task-recordings'/(descriptor['recording_id']+'.json')).exists(): continue
                self.upload.enqueue(descriptor['artifact_id'], descriptor['binding_id'])
            except (UploadError, KeyError):
                continue  # Preserve the original evidence for operator review.

    def stop(self):
        self._heartbeat_stop.set()
        if self._heartbeat_thread: self._heartbeat_thread.join(timeout=1)
        self.upload.stop()
