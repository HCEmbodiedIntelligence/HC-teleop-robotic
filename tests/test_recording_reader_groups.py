"""Independent DDS readers retain one writer and one source-time window."""
import queue
from types import SimpleNamespace

import pytest

from middleware.core.recording.ros_recording_executor import RosRecordingExecutor, recording_reader_groups


SMALL = dict(topic='/observation/joint_states', type='sensor_msgs/msg/JointState', outputs=['record'])
COLOR = dict(topic='/observation/cameras/head/color/compressed', type='sensor_msgs/msg/CompressedImage', outputs=['record'])
DEPTH = {**COLOR, 'topic': COLOR['topic'].replace('/color/', '/depth/')}


@pytest.fixture(autouse=True)
def clear_mode_override(monkeypatch):
    monkeypatch.delenv('HC_ISOLATED_CONTROL_PROCESS', raising=False)


@pytest.mark.parametrize('profile,isolated,expected', [
    ('x1', True, ['small', 'images']), ('x1', False, ['all']),
    ('other', True, ['all']), ('', True, ['all']),
])
def test_only_isolated_x1_separates_images_and_keeps_rgbd_together(profile, isolated, expected):
    items = [SMALL, COLOR, DEPTH]
    groups = recording_reader_groups({'legacy_control_profile': profile,
        'isolated_control_process': isolated}, items)
    assert [name for name, _items in groups] == expected
    assert [item for _name, selected in groups for item in selected] == items
    if len(groups) == 2:
        assert groups[0][1] == [SMALL]
        assert groups[1][1] == [COLOR, DEPTH]


@pytest.mark.parametrize('profile,configured,override,expected', [
    ('x1', False, '1', ['small', 'images']),
    ('x1', True, '0', ['all']),
    ('other', False, '1', ['all']),
    ('other', True, '0', ['all']),
    ('', False, '1', ['all']),
])
def test_reader_mode_uses_same_environment_resolution_as_control_bridge(monkeypatch, profile, configured, override, expected):
    monkeypatch.setenv('HC_ISOLATED_CONTROL_PROCESS', override)
    config = dict(legacy_control_profile=profile, isolated_control_process=configured,
                  subscriptions=[SMALL, COLOR, DEPTH])
    assert [name for name, _items in recording_reader_groups(config, config['subscriptions'])] == expected
    receiver = RosRecordingExecutor(config, SimpleNamespace())
    assert [worker['name'] for worker in receiver._workers] == expected
    assert receiver._isolated_readers is (expected != ['all'])
    assert receiver.status()['priority'] == ('normal' if expected != ['all'] else 'low')


class FakeProcess:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.alive = False
        self.exitcode = 0
    def start(self): self.alive = True
    def is_alive(self): return self.alive
    def join(self, timeout=None): self.alive = False


def reader():
    resources = (object(), object(), object())
    recorder = SimpleNamespace(ipc_resources=lambda: resources, status=lambda:{})
    config = dict(legacy_control_profile='x1', isolated_control_process=True,
        node_name='test_reader', subscriptions=[SMALL, COLOR, DEPTH])
    result = RosRecordingExecutor(config, recorder)
    result._mp = SimpleNamespace(Process=FakeProcess)
    for worker in result._workers:
        worker['status_queue'] = queue.Queue()
    return result, resources


def test_processes_have_independent_wait_sets_and_share_original_ingress_and_stop():
    receiver, resources = reader()
    receiver.start()
    small, images = receiver._workers
    assert small['process'] is receiver._process
    assert small['process'] is not images['process']
    small_args, image_args = [worker['process'].kwargs['args'] for worker in receiver._workers]
    assert small_args[1] == [SMALL] and image_args[1] == [COLOR, DEPTH]
    assert small_args[0]['node_name'] != image_args[0]['node_name']
    assert all(args[0]['_isolated_recording_reader'] for args in (small_args, image_args))
    assert small_args[5] is image_args[5] is receiver._stop_event
    assert small_args[9] is image_args[9] is resources
    assert small_args[6] is not image_args[6]
    assert small_args[7] is not image_args[7]
    receiver.stop()
    assert receiver._process is None
    assert all(worker['process'] is None for worker in receiver._workers)
    assert receiver.status()['error'] is None


def test_health_and_counts_merge_without_losing_a_reader_failure():
    receiver, _resources = reader()
    receiver.start()
    small, images = receiver._workers
    small['status_queue'].put(dict(state='ready', active=False, pid=11,
        received=20, recorded=19, topic_health={SMALL['topic']: {'hz':100}},
        warm_qos_override=None))
    images['status_queue'].put(dict(state='ready', active=False, pid=12,
        received=8, recorded=8, topic_health={COLOR['topic']: {'hz':30}},
        warm_qos_override=dict(topics=[COLOR['topic'], DEPTH['topic']])))
    status = receiver.status()
    assert status['state'] == 'ready'
    assert status['received'] == 28 and status['recorded'] == 27
    assert set(status['topic_health']) == {SMALL['topic'], COLOR['topic']}
    assert [worker['pid'] for worker in status['readers']] == [11, 12]
    images['process'].alive = False
    images['process'].exitcode = 17
    failed = receiver.status()
    assert failed['state'] == 'error'
    assert 'images recording process exited with code 17' in failed['error']
    assert failed['topic_health'][SMALL['topic']]['hz'] == 100


def test_preparation_resets_every_reader_and_waits_for_all_before_open():
    receiver, _resources = reader()
    receiver.start()
    waits = []
    class Ready:
        def clear(self): pass
        def wait(self, timeout):
            assert all(worker['reset'].is_set() for worker in receiver._workers)
            waits.append(timeout)
            return len(waits) == 1
    for worker in receiver._workers:
        worker['ready'] = Ready()
    with pytest.raises(TimeoutError, match='images recording subscriptions'):
        receiver.prepare_recording()
    assert len(waits) == 2

