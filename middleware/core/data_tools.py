"""Non-destructive MCAP inspection, annotations, trimming and browser playback."""
from __future__ import annotations

import asyncio
import copy
import hashlib
import json
import math
import shutil
from pathlib import Path
import threading
import time
import uuid

from aiohttp import web
from mcap.reader import make_reader
from mcap.writer import Writer

from .artifact_state import atomic_json, fingerprint

ANNOTATION_TOPIC = '/humanoid/annotations'


def finite_number(value, name, low=0, high=1e12):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} 必须在 {low:g}–{high:g} 之间')
    return float(value)


def plain(value, limit=2048):
    if isinstance(value, dict):
        return {str(k): plain(v, limit) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return {'binary_bytes': len(value)}
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        result = [plain(x, limit) for x in value[:limit]]
        if len(value) > limit:
            result.append({'omitted_elements': len(value)-limit})
        return result
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def decode(schema, channel, message):
    if channel.message_encoding == 'json':
        return json.loads(message.data)
    if channel.message_encoding == 'cdr' and schema:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        decoded = deserialize_message(message.data, get_message(schema.name))
        if schema.name in {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}:
            # uint8 image payloads cannot contain non-finite numbers. Avoid turning
            # millions of bytes into Python objects just to audit/preview metadata.
            # The complete CDR buffer has already been decoded and stays in MCAP.
            size = len(decoded.data)
            decoded.data = b''
            payload = message_to_ordereddict(decoded)
            payload['data'] = {'binary_bytes': size}
            return payload
        return message_to_ordereddict(decoded)
    raise ValueError(f'尚不能预览 {channel.message_encoding} / {schema.name if schema else "无类型"}；原始消息仍可裁剪导出')


def nonfinite(value, path=''):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from nonfinite(child, path+'/'+str(key))
    elif isinstance(value, (list, tuple)) or hasattr(value, 'tolist'):
        for index, child in enumerate(value):
            yield from nonfinite(child, path+'/'+str(index))
    elif isinstance(value, float) and not math.isfinite(value):
        yield path


class Datasets:
    def __init__(self, recorder):
        self.recorder = recorder
        self.cache = {}

    def path(self, filename, *, idle=True):
        if not isinstance(filename, str) or Path(filename).name != filename or '\\' in filename or filename.startswith('.') or not filename.endswith('.mcap'):
            raise ValueError('请选择录制目录中的 MCAP 文件')
        path = self.recorder.directory / filename
        if not path.is_file() or path.is_symlink():
            raise ValueError('数据文件不存在或是符号链接')
        if idle and hasattr(self.recorder, 'assert_file_idle'):
            self.recorder.assert_file_idle(path)
        if idle and self.recorder.is_recording() and path == self.recorder.path:
            raise web.HTTPConflict(text='请结束当前录制后再查看和编辑完整数据')
        return path

    def sidecar(self, path):
        return path.with_suffix('.mcap.edit.json')

    def identity(self, path):
        stat = path.stat()
        return {'size': stat.st_size, 'mtime_ns': stat.st_mtime_ns}

    def overview(self, filename):
        path = self.path(filename)
        identity = self.identity(path)
        key = (str(path), identity['size'], identity['mtime_ns'])
        if key not in self.cache:
            with path.open('rb') as stream:
                reader = make_reader(stream, validate_crcs=True)
                summary = reader.get_summary()
                if not summary or not summary.statistics:
                    raise ValueError('MCAP 缺少完整索引，文件可能未正常结束；请先执行修复导出')
                stats = summary.statistics
                result = {'filename': filename, 'source': identity, 'start_ns': stats.message_start_time,
                    'duration': max(0, (stats.message_end_time-stats.message_start_time)/1e9),
                    'message_count': stats.message_count, 'channels': [
                        {'topic': c.topic, 'type': summary.schemas[c.schema_id].name if c.schema_id else '',
                         'encoding': c.message_encoding, 'count': stats.channel_message_counts.get(c.id, 0)}
                        for c in summary.channels.values()]}
                exported_edit = None
                for metadata in reader.iter_metadata():
                    if metadata.name == 'humanoid_manager.edit' and 'annotations_absolute' in metadata.metadata:
                        candidate = metadata.metadata
                        if exported_edit is None or int(candidate.get('exported_at_ns', 0)) > int(exported_edit.get('exported_at_ns', 0)):
                            exported_edit = candidate
                if exported_edit is not None:
                    result['recorded_annotations'] = []
                    result['provenance'] = exported_edit['source']
                    result['recorded_notes'] = json.loads(exported_edit['edit']).get('notes', '')
                    for item in json.loads(exported_edit['annotations_absolute']):
                        item['start'] = max(0, (item.pop('start_ns')-result['start_ns'])/1e9)
                        item['end'] = min(result['duration'], (item.pop('end_ns')-result['start_ns'])/1e9)
                        if item['start'] <= item['end']:
                            result['recorded_annotations'].append(item)
                else:
                    result['recorded_annotations'] = self._recorded_annotations(reader, result)
            if len(self.cache) > 50:
                self.cache.clear()
            self.cache[key] = result
        base = copy.deepcopy(self.cache[key])
        try:
            edit = json.loads(self.sidecar(path).read_text())
            if edit['source'] != identity:
                raise web.HTTPConflict(text='原文件已被外部修改，请先另存原标记文件并重新检查数据')
        except FileNotFoundError:
            edit = {'source': identity, 'notes': base.get('recorded_notes', ''), 'trim': {'start': 0, 'end': base['duration']},
                    'annotations': base['recorded_annotations'], 'revision': ''}
        base.update(edit=edit, etag=fingerprint(edit))
        try:
            quality = json.loads(path.with_suffix('.mcap.quality.json').read_text())
            base['quality'] = quality if quality.get('source') == identity else None
        except FileNotFoundError:
            base['quality'] = None
        return base

    def _recorded_annotations(self, reader, info):
        annotations, opened = [], {}
        points = []
        duration = info['duration']
        for _, channel, message in reader.iter_messages(topics=[ANNOTATION_TOPIC]):
            try:
                event = json.loads(message.data)
                position = max(0, min(duration, (event.get('stamp_ns', message.log_time)-info['start_ns'])/1e9))
                action = event.get('action')
                if action in {'start', 'point', 'recording'}:
                    annotation = {'id': event['id'], 'label': 'invalid', 'note': event.get('note', ''),
                        'start': 0 if action == 'recording' else position,
                        'end': duration if action in {'start','recording'} else position,
                        'source': event.get('source', 'button')}
                    annotations.append(annotation)
                    if action == 'point':
                        points.append(annotation)
                    if action == 'start':
                        opened[event['id']] = annotation
                elif action == 'end' and event['id'] in opened:
                    opened.pop(event['id'])['end'] = position
            except (ValueError, KeyError, TypeError):
                continue
        # A button timestamp seldom equals a sensor timestamp exactly. Bind a point
        # marker to the closest recorded data sample in a bounded 0.5 s window.
        for annotation in points:
            requested = annotation['start']
            closest = None
            for _, channel, message in reader.iter_messages(
                    start_time=info['start_ns']+round(max(0, requested-.5)*1e9),
                    end_time=info['start_ns']+round((requested+.5)*1e9)+1):
                if channel.topic in {ANNOTATION_TOPIC, '/humanoid/data_quality'}:
                    continue
                position = (message.log_time-info['start_ns'])/1e9
                if closest is None or abs(position-requested) < abs(closest-requested):
                    closest = position
            annotation['requested_time'] = requested
            if closest is not None:
                annotation['start'] = annotation['end'] = closest
        return annotations

    def save(self, filename, data):
        info = self.overview(filename)
        if data.get('etag') != info['etag']:
            raise web.HTTPConflict(text='数据标记已在其他窗口更新，请重新打开文件')
        edit = {'source': info['source'], 'notes': data.get('notes', ''), 'trim': data.get('trim', {}),
                'annotations': data.get('annotations', []), 'revision': uuid.uuid4().hex}
        if not isinstance(edit['notes'], str) or len(edit['notes']) > 10000:
            raise ValueError('备注需为不超过 10000 字符的文本')
        if not isinstance(edit['trim'], dict):
            raise ValueError('裁剪范围格式错误')
        for key in ('start', 'end'):
            edit['trim'][key] = finite_number(edit['trim'].get(key), '裁剪时间', high=info['duration'])
        if edit['trim']['end'] < edit['trim']['start']:
            raise ValueError('裁剪结束时间不能早于开始时间')
        if not isinstance(edit['annotations'], list) or len(edit['annotations']) > 10000:
            raise ValueError('标记必须为列表，最多 10000 条')
        ids = set()
        for item in edit['annotations']:
            if not isinstance(item, dict) or item.get('label') not in {'valid', 'invalid', 'note'}:
                raise ValueError('标记类型应为 valid、invalid 或 note')
            if not isinstance(item.get('id'), str) or not 1 <= len(item['id']) <= 100 or item['id'] in ids:
                raise ValueError('标记 ID 为空或重复')
            ids.add(item['id'])
            for key in ('start', 'end'):
                item[key] = finite_number(item.get(key), '标记时间', high=info['duration'])
            if item['end'] < item['start'] or not isinstance(item.get('note', ''), str) or len(item.get('note','')) > 10000:
                raise ValueError('标记结束时间或备注无效')
        atomic_json(self.sidecar(self.path(filename)), edit)
        return self.overview(filename)

    def frames(self, filename, start=0, end=None, limit=200):
        info = self.overview(filename)
        start = finite_number(start, '起始时间', high=info['duration'])
        end = info['duration'] if end is None else finite_number(end, '结束时间', high=info['duration'])
        frames = []
        with self.path(filename).open('rb') as stream:
            for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages(
                    start_time=info['start_ns']+round(start*1e9), end_time=info['start_ns']+round(end*1e9)+1):
                frame = {'position': (message.log_time-info['start_ns'])/1e9, 'topic': channel.topic,
                         'type': schema.name if schema else '', 'size': len(message.data)}
                try:
                    frame['payload'] = plain(decode(schema, channel, message))
                except Exception as error:
                    frame['error'] = str(error)
                frames.append(frame)
                if len(frames) >= limit:
                    break
        return {'frames': frames, 'limit': limit, 'truncated': len(frames) == limit}

    def recover(self, filename, data):
        """Copy the readable prefix into a new MCAP; never guess lost bytes."""
        from mcap.stream_reader import StreamReader
        from mcap.records import Schema, Channel, Message, Metadata
        path = self.path(filename)
        output = data.get('filename', '')
        if not isinstance(output, str) or Path(output).name != output or '\\' in output or output.startswith('.') or not output.endswith('.mcap'):
            raise ValueError('修复副本文件名无效')
        destination = self.recorder.directory / output
        if destination.exists():
            raise web.HTTPConflict(text='目标文件已存在')
        temporary = destination.with_name('.recover-'+uuid.uuid4().hex)
        recovered, stopped_at = 0, ''
        try:
            with path.open('rb') as source, temporary.open('xb') as target:
                writer = Writer(target)
                writer.start()
                schemas, channels = {}, {}
                try:
                    for record in StreamReader(source, validate_crcs=True).records:
                        if isinstance(record, Schema):
                            schemas[record.id] = writer.register_schema(record.name, record.encoding, record.data)
                        elif isinstance(record, Channel):
                            channels[record.id] = writer.register_channel(record.topic, record.message_encoding,
                                schemas.get(record.schema_id, 0), record.metadata)
                        elif isinstance(record, Message):
                            writer.add_message(channels[record.channel_id], record.log_time, record.data,
                                record.publish_time, record.sequence)
                            recovered += 1
                        elif isinstance(record, Metadata):
                            writer.add_metadata(record.name, record.metadata)
                except Exception as error:
                    stopped_at = f'{type(error).__name__}: {error}'
                writer.add_metadata('humanoid_manager.recovery', {'source': filename,
                    'recovered_messages': str(recovered), 'stopped_at': stopped_at,
                    'complete_original_data': 'unknown'})
                writer.finish()
            if not recovered:
                raise ValueError('没有可恢复的完整消息；原文件已保留。'+stopped_at)
            destination.hardlink_to(temporary)
            return {'ok': True, 'filename': output, 'recovered_messages': recovered,
                    'stopped_at': stopped_at, 'source_preserved': True,
                    'message': '仅恢复完整可读消息；损坏块及之后的数据可能缺失，请重新检查副本。'}
        finally:
            temporary.unlink(missing_ok=True)

    def audit(self, filename):
        info = self.overview(filename)
        issues, counts, previous, gaps, unsupported = [], {}, {}, {}, set()
        def issue(kind, topic, position, detail):
            counts[kind] = counts.get(kind, 0)+1
            if len(issues) < 200:
                issues.append({'kind': kind, 'topic': topic, 'position': position, 'detail': detail})
        messages = 0
        try:
            with self.path(filename).open('rb') as stream:
                for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False):
                    messages += 1
                    position = (message.log_time-info['start_ns'])/1e9
                    prev = previous.get(channel.id)
                    if prev is not None:
                        if message.log_time < prev:
                            issue('timestamp_regression', channel.topic, position, '同一通道时间戳倒退')
                        gaps[channel.topic] = max(gaps.get(channel.topic, 0), (message.log_time-prev)/1e9)
                    previous[channel.id] = message.log_time
                    if channel.id in unsupported:
                        continue
                    try:
                        payload = decode(schema, channel, message)
                        if channel.topic == '/humanoid/data_quality':
                            issue('recording_health', channel.topic, position, json.dumps(payload, ensure_ascii=False))
                        invalid = list(nonfinite(payload))
                        if invalid:
                            issue('non_finite', channel.topic, position, ', '.join(invalid[:8]))
                    except (ImportError, ModuleNotFoundError, AttributeError) as error:
                        unsupported.add(channel.id)
                        issue('unsupported_type', channel.topic, position, str(error))
                    except Exception as error:
                        issue('decode_error', channel.topic, position, str(error))
        except Exception as error:
            issue('corrupt_file', '', 0, str(error))
        if not messages:
            issue('empty_recording', '', 0, '录制文件中没有消息')
        rules = getattr(self.recorder, 'quality_rules', [])
        for rule in rules:
            threshold = rule.get('max_gap_seconds', 0)
            gap = gaps.get(rule['topic'], 0)
            if threshold and gap > threshold:
                issue('data_gap', rule['topic'], 0, f'最大间隔 {gap:.3f}s，阈值 {threshold:.3f}s')
        result = {'source': info['source'], 'checked_at': time.time(), 'message_count': messages,
                  'ok': not counts, 'counts': counts, 'issues': issues, 'max_gap_seconds': gaps,
                  'scope': 'CRC、消息解码、非有限数值、通道时间戳；频率间断按配置阈值检查'}
        atomic_json(self.path(filename).with_suffix('.mcap.quality.json'), result)
        return result

    def export(self, filename, data):
        info = self.overview(filename)
        if data.get('etag') != info['etag']:
            raise web.HTTPConflict(text='标记版本已变化，请刷新后再导出')
        output = data.get('filename', '')
        if not isinstance(output, str) or Path(output).name != output or '\\' in output or output.startswith('.') or not output.endswith('.mcap'):
            raise ValueError('导出文件名必须以 .mcap 结尾且不能包含目录')
        destination = self.recorder.directory/output
        if destination.exists():
            raise web.HTTPConflict(text='导出文件已存在，请使用新名称')
        edit = info['edit']
        start, end = edit['trim']['start'], edit['trim']['end']
        excluded = [x for x in edit['annotations'] if x['label'] == 'invalid'] if data.get('exclude_invalid', True) else []
        output_annotations = [{**item, 'start_ns': info['start_ns']+round(max(start, item['start'])*1e9),
            'end_ns': info['start_ns']+round(min(end, item['end'])*1e9)} for item in edit['annotations']
            if item not in excluded and item['start'] <= end and item['end'] >= start]
        temporary = destination.with_name('.export-'+uuid.uuid4().hex)
        count = 0
        try:
            with self.path(filename).open('rb') as source, temporary.open('xb') as target:
                reader = make_reader(source, validate_crcs=True)
                writer = Writer(target)
                writer.start(profile=reader.get_header().profile)
                for metadata in reader.iter_metadata():
                    writer.add_metadata(metadata.name, metadata.metadata)
                for attachment in reader.iter_attachments():
                    writer.add_attachment(attachment.create_time, attachment.log_time, attachment.name,
                                          attachment.media_type, attachment.data)
                writer.add_metadata('humanoid_manager.edit', {'source': filename, 'edit': json.dumps(edit, ensure_ascii=False),
                    'annotations_absolute': json.dumps(output_annotations, ensure_ascii=False), 'exported_at_ns': str(time.time_ns()),
                    'excluded_invalid': str(bool(data.get('exclude_invalid', True))).lower()})
                schemas, channels = {}, {}
                for schema, channel, message in reader.iter_messages(
                        start_time=info['start_ns']+round(start*1e9), end_time=info['start_ns']+round(end*1e9)+1):
                    position = (message.log_time-info['start_ns'])/1e9
                    if any(x['start']-1e-9 <= position <= x['end']+1e-9 for x in excluded):
                        continue
                    if schema and schema.id not in schemas:
                        schemas[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
                    if channel.id not in channels:
                        channels[channel.id] = writer.register_channel(channel.topic, channel.message_encoding,
                            schemas.get(channel.schema_id, 0), channel.metadata)
                    writer.add_message(channels[channel.id], message.log_time, message.data,
                                       message.publish_time, message.sequence)
                    count += 1
                writer.finish()
            if count == 0:
                raise ValueError('裁剪并排除无效数据后没有剩余消息，请调整范围')
            # link is an atomic no-overwrite commit, even if an external program creates the destination.
            destination.hardlink_to(temporary)
            return {'ok': True, 'filename': output, 'message_count': count, 'source_preserved': True}
        finally:
            temporary.unlink(missing_ok=True)


class DatasetPlayer:
    """Read-only preview. Events are sent to browsers, never to ROS command topics."""
    def __init__(self, datasets, emit):
        self.datasets, self.emit = datasets, emit
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.state = {'is_active': False, 'paused': False, 'filename': '', 'position': 0, 'duration': 0, 'speed': 1, 'error': None}

    def status(self):
        with self.lock:
            return dict(self.state)

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=5)
        if self.thread and self.thread.is_alive():
            raise RuntimeError('回放尚未退出，请稍后再试')
        with self.lock:
            self.state['is_active'] = False
            self.state['paused'] = False

    def play(self, filename, position=0, speed=1):
        info = self.datasets.overview(filename)
        position = finite_number(position, '回放位置', high=info['duration'])
        speed = finite_number(speed, '回放速度', .1, 4)
        self.stop()
        self.stop_event = threading.Event()
        with self.lock:
            self.state.update(is_active=True, paused=False, filename=filename, position=position,
                              duration=info['duration'], speed=speed, error=None)
        self.thread = threading.Thread(target=self.run, args=(info, position, speed), daemon=True, name='mcap-preview')
        self.thread.start()
        return self.status()

    def pause(self):
        with self.lock:
            self.state['paused'] = not self.state['paused']
        return self.status()

    def run(self, info, position, speed):
        previous = position
        last_sent = {}
        try:
            with self.datasets.path(info['filename']).open('rb') as stream:
                for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages(
                        start_time=info['start_ns']+round(position*1e9)):
                    current = (message.log_time-info['start_ns'])/1e9
                    remaining = max(0, (current-previous)/speed)
                    while remaining > 0 or self.status()['paused']:
                        before = time.monotonic()
                        if self.stop_event.wait(min(.02, remaining) if remaining > 0 else .02):
                            return
                        if not self.status()['paused']:
                            remaining -= time.monotonic()-before
                    if self.stop_event.is_set():
                        return
                    previous = current
                    with self.lock:
                        self.state['position'] = current
                    # Do not decode image buffers on the browser preview path.
                    if schema and schema.name in {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/PointCloud2'}:
                        continue
                    now = time.monotonic()
                    if now-last_sent.get(channel.topic, 0) < .04 and channel.topic != ANNOTATION_TOPIC:
                        continue
                    last_sent[channel.topic] = now
                    try:
                        payload, error = plain(decode(schema, channel, message)), None
                    except Exception as exc:
                        payload, error = None, str(exc)
                    self.emit({'kind': 'data_preview_message', 'filename': info['filename'], 'topic': channel.topic, 'position': current,
                               'type': schema.name if schema else '', 'payload': payload, 'error': error})
        except Exception as error:
            with self.lock:
                self.state['error'] = str(error)
        finally:
            with self.lock:
                self.state['is_active'] = False
                self.state['paused'] = False



def register_data_routes(app, runtime):
    """Browser-only preview. This module never calls TopicPlayer or ROS publish."""
    from concurrent.futures import ThreadPoolExecutor
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='data-tools')
    tasks = {}
    context = {}

    def datasets():
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text='recorder is not initialized')
        if context.get('recorder') is not runtime.recorder:
            if 'player' in context:
                context['player'].stop()
            context.update(recorder=runtime.recorder, datasets=Datasets(runtime.recorder))
            for file in (runtime.recorder.state_dir/'data_tasks').glob('*.json'):
                task = json.loads(file.read_text())
                if task['job_id'] in tasks:
                    continue
                if task['state'] in {'queued', 'running'}:
                    task.update(state='interrupted', error='process stopped before file operation committed')
                    atomic_json(file, task)
                tasks[task['job_id']] = task
            context['player'] = DatasetPlayer(context['datasets'], lambda event: runtime.emit(event, ['websocket']))
        return context['datasets']

    async def overview(request):
        return web.json_response(await asyncio.to_thread(datasets().overview, request.match_info['filename']))

    async def frames(request):
        return web.json_response(await asyncio.to_thread(datasets().frames, request.match_info['filename'],
            float(request.query.get('start', 0)), None, min(500, max(1, int(request.query.get('limit', 200))))))

    async def action(request):
        filename, operation = request.match_info['filename'], request.match_info['operation']
        data = await request.json() if request.can_read_body else {}
        target = datasets()
        recorder = target.recorder
        if operation == 'edit':
            def save():
                with recorder.file_operation(filename):
                    return target.save(filename, data)
            return web.json_response(await asyncio.to_thread(save))
        if operation not in {'audit', 'recover', 'export'}:
            raise web.HTTPNotFound()
        if sum(t['state'] in {'queued', 'running'} for t in tasks.values()) >= 8:
            raise web.HTTPConflict(text='数据处理队列已满')
        ident = uuid.uuid4().hex
        task = {'job_id': ident, 'state': 'queued', 'operation': operation, 'filename': filename}
        tasks[ident] = task
        task_path = recorder.state_dir/'data_tasks'/(ident+'.json')
        atomic_json(task_path, task)
        def run():
            task['state'] = 'running'
            atomic_json(task_path, task)
            try:
                with recorder.file_operation(filename):
                    if operation in {'recover', 'export'}:
                        needed = target.path(filename).stat().st_size + recorder.config.get('min_free_bytes', 268435456)
                        if shutil.disk_usage(recorder.directory).free < needed:
                            raise ValueError('磁盘可用空间不足以创建副本并保留预留空间')
                    before = target.identity(target.path(filename))
                    result = target.audit(filename) if operation == 'audit' else getattr(target, operation)(filename, data)
                    if target.identity(target.path(filename)) != before:
                        raise ValueError('源文件在处理期间发生变化；结果不再有效')
                    if operation in {'recover', 'export'}:
                        recorder.register_derived(filename, result['filename'], operation)
                    task.update(state='completed', result=result)
            except Exception as exc:
                task.update(state='failed', error=str(exc))
            finally:
                atomic_json(task_path, task)
        pool.submit(run)
        return web.json_response(task, status=202)

    async def preview(request):
        datasets()
        player = context['player']
        data = await request.json() if request.can_read_body else {}
        operation = request.match_info['operation']
        if operation in {'play', 'seek'}:
            result = await asyncio.to_thread(player.play, data['filename'], data.get('position', 0), data.get('speed', 1))
        elif operation == 'pause':
            result = player.pause()
        elif operation == 'stop':
            await asyncio.to_thread(player.stop)
            result = player.status()
        else:
            raise web.HTTPNotFound()
        return web.json_response(result)

    async def jobs(request):
        datasets()
        return web.json_response(list(tasks.values()))

    async def cleanup(app):
        if 'player' in context:
            await asyncio.to_thread(context['player'].stop)
        await asyncio.to_thread(pool.shutdown, wait=True)

    app.on_cleanup.append(cleanup)
    app.router.add_get('/api/data/jobs', jobs)
    app.router.add_get('/api/data/{filename}', overview)
    app.router.add_get('/api/data/{filename}/frames', frames)
    app.router.add_post('/api/data/{filename}/{operation}', action)
    app.router.add_post('/api/data-preview/{operation}', preview)
