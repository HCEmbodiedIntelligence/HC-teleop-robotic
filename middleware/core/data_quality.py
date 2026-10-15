"""Persist live annotation edges and recording anomalies independently of the UI."""
import json
import time
import uuid

from aiohttp import web

from .data_tools import ANNOTATION_TOPIC
from .artifact_state import atomic_json


class DataQuality:
    def __init__(self, runtime):
        self.runtime = runtime
        self.active_segment = None
        self.events = []
        self.issues = []
        self.last_issue = None
        self.last_buttons = None
        self.last_buttons_at = 0

    def reset(self):
        self.active_segment = None
        self.events = []
        self.issues = []
        self.last_issue = None

    def mark(self, scope=None, source='web', note=''):
        if not self.runtime.recorder.is_recording():
            raise web.HTTPConflict(text='当前没有录制；已有数据请在数据文件中添加标记')
        scope = scope or self.runtime.config['data_quality']['scope']
        if scope not in {'segment', 'recording', 'frame'}:
            raise ValueError('标记范围无效')
        if not isinstance(note, str) or len(note) > 10000:
            raise ValueError('标记备注过长')
        ident = self.active_segment if scope == 'segment' and self.active_segment else uuid.uuid4().hex
        action = 'end' if scope == 'segment' and self.active_segment else {'segment': 'start', 'recording': 'recording', 'frame': 'point'}[scope]
        event = {'id': ident, 'action': action, 'source': source, 'note': note, 'stamp_ns': time.time_ns()}
        if not self.runtime.recorder.record({'topic': ANNOTATION_TOPIC, 'stamp_ns': event['stamp_ns'], 'payload': event}):
            raise web.HTTPServiceUnavailable(text='标记未写入：录制队列已满或写入进程已停止')
        if scope == 'segment':
            self.active_segment = None if action == 'end' else ident
        self.events.append(event)
        self.persist()
        self.runtime.emit({'kind': 'data_annotation', 'payload': event})
        return event

    def buttons(self, event):
        try:
            message = event['payload']
            payload = json.loads(message['data']) if isinstance(message, dict) and 'data' in message else message
            self.last_buttons = payload
            self.last_buttons_at = time.monotonic()
            config = self.runtime.config['data_quality']
            capture=getattr(self.runtime,'capture',None)
            aligned=bool(capture and capture.session and not capture.session.closed.is_set())
            if not config['button_enabled'] or not (self.runtime.recorder.is_recording() or aligned):
                return
            for edge in payload.get('edges', []):
                if edge.get('controller') == config['controller'] and edge.get('button') == config['button'] and edge.get('action') == 'pressed':
                    if aligned:
                        capture.session.mark(config['scope'],source='button:'+edge['controller']+':'+edge['button'])
                        continue
                    self.mark(source=f"{config['controller']}/{config['button']}")
        except (ValueError, TypeError, KeyError, web.HTTPException) as error:
            self.issues.append({'kind': 'button_marker_error', 'stamp_ns': time.time_ns(), 'detail': str(error)})
            self.persist()

    def check(self):
        recorder = self.runtime.recorder
        if not recorder or not recorder.path:
            return
        status = recorder.status()
        if not status['recording'] and not status['error']:
            return
        health = self.runtime.recording_ros.get_topic_health()
        state = {'error': status['error'], 'dropped': status['dropped'],
                 'topics': {key: value['state'] for key, value in health.items() if value['state'] != 'ok'}}
        if state != self.last_issue:
            self.last_issue = state
            if state['error'] or state['dropped'] or state['topics']:
                event = {'kind': 'recording_health', 'stamp_ns': time.time_ns(), **state}
                self.issues.append(event)
                recorder.record({'topic': '/humanoid/data_quality', 'stamp_ns': event['stamp_ns'], 'payload': event})
                self.persist()

    def persist(self):
        path = self.runtime.recorder.path
        if path:
            atomic_json(path.with_suffix('.mcap.session.json'), {'events': self.events, 'issues': self.issues})

    def status(self):
        return {'active_segment': self.active_segment, 'events': self.events[-20:], 'issues': self.issues[-20:],
                'buttons': self.last_buttons, 'buttons_fresh': time.monotonic()-self.last_buttons_at < 2}
