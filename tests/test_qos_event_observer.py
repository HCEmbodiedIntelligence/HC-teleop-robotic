"""Default warning handlers move; their native endpoints and QoS do not."""
import threading
import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
import middleware.core.control.qos_event_observer as module


class Group:
    def __init__(self):
        self.members = []
    def add_entity(self, entity):
        self.members.append(entity)


class Handler:
    def __init__(self, group, event_type, callback):
        self.callback_group, self.event_type, self.callback = group, event_type, callback


class Owner:
    def __init__(self, context):
        self.context = context
        self.waitables, self.publishers, self.subscriptions = [], [], []
        self.destroyed = False
    def get_name(self):
        return 'control'
    def add_waitable(self, item):
        self.waitables.append(item)
    def remove_waitable(self, item):
        self.waitables.remove(item)
    def destroy_node(self):
        # This mirrors Humble's endpoint destruction requirement.
        for endpoint in self.publishers+self.subscriptions:
            for handler in endpoint.event_handlers:
                self.waitables.remove(handler)
        self.destroyed = True


class Executor:
    def __init__(self, context):
        self.stopped = threading.Event()
        self.entered = threading.Event()
    def add_node(self, node):
        self.node = node
    def spin(self):
        self.entered.set()
        self.stopped.wait(3)
    def wait_for_ready_callbacks(self, **kwargs):
        self.entered.set()
        raise module.TimeoutException()
    def shutdown(self, timeout_sec):
        self.stopped.set()
        return True


@pytest.fixture
def fake(monkeypatch):
    context, group = object(), Group()
    owner = Owner(context)
    observer = Owner(context)
    factory = Mock(return_value=observer)
    monkeypatch.setattr(module, 'Node', factory)
    monkeypatch.setattr(module, 'QoSEventHandler', Handler)
    monkeypatch.setattr(module, 'ReentrantCallbackGroup', Group)
    monkeypatch.setattr(module, 'SingleThreadedExecutor', Executor)
    def _default_incompatible_qos_callback(event):
        return event
    _default_incompatible_qos_callback.__module__ = 'rclpy.qos_event'
    _default_incompatible_qos_callback.__qualname__ = (
        'PublisherEventCallbacks.create_event_handlers.<locals>._default_incompatible_qos_callback')
    custom = lambda event: event
    default = Handler(group, module.QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS,
                      _default_incompatible_qos_callback)
    explicit = Handler(group, module.QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS, custom)
    deadline = Handler(group, module.QoSPublisherEventType.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED,
                       _default_incompatible_qos_callback)
    qos = object()
    endpoint = SimpleNamespace(event_handlers=[default, explicit, deadline], qos_profile=qos)
    owner.publishers = [endpoint]
    owner.waitables = endpoint.event_handlers.copy()+[object()]
    return SimpleNamespace(owner=owner, observer=observer, group=group, default=default,
                           explicit=explicit, deadline=deadline, endpoint=endpoint, qos=qos, factory=factory)


def test_only_default_incompatible_moves_and_close_restores_original_destruction_contract(fake):
    observer = module.QosEventObserver([fake.owner], enabled=True)
    callback = fake.default.callback
    assert observer.start()
    assert observer._executor.entered.wait(1)
    assert observer.status()['moved'] == 1
    assert fake.default not in fake.owner.waitables
    assert fake.default in fake.observer.waitables
    assert fake.default.callback_group is not fake.group
    assert fake.default.callback is callback
    assert fake.explicit in fake.owner.waitables and fake.explicit.callback_group is fake.group
    assert fake.deadline in fake.owner.waitables and fake.deadline.callback_group is fake.group
    assert fake.endpoint.qos_profile is fake.qos
    assert fake.endpoint.event_handlers == [fake.default, fake.explicit, fake.deadline]
    fake.default._executor_event = True  # selected just before diagnostic stop
    assert fake.factory.call_args.kwargs == dict(context=fake.owner.context, enable_rosout=False,
                                                 start_parameter_services=False)
    observer.close()
    observer.close()
    assert not observer.status()['thread_alive']
    assert observer.status()['moved'] == 0
    assert observer.status()['restored'] == 1
    assert fake.default in fake.owner.waitables and fake.default.callback_group is fake.group
    assert fake.default._executor_event is False
    fake.owner.destroy_node()
    assert fake.owner.destroyed


def test_environment_fallback_keeps_original_events(fake, monkeypatch):
    monkeypatch.setenv('HC_QOS_EVENT_OBSERVER', '0')
    observer = module.QosEventObserver([fake.owner])
    assert observer.start() is False
    assert observer.status()['enabled'] is False
    assert fake.default in fake.owner.waitables
    fake.factory.assert_not_called()
    observer.close()
    fake.owner.destroy_node()


def test_partial_migration_failure_restores_every_handler(fake, monkeypatch):
    monkeypatch.setattr(fake.observer, 'add_waitable', Mock(side_effect=RuntimeError('cannot attach')))
    observer = module.QosEventObserver([fake.owner], enabled=True)
    assert observer.start() is False
    assert 'cannot attach' in observer.status()['error']
    assert fake.default in fake.owner.waitables
    assert fake.default.callback_group is fake.group
    observer.close()
    fake.owner.destroy_node()


def test_diagnostic_executor_failure_restores_original_events(fake, monkeypatch):
    class FailedExecutor(Executor):
        def wait_for_ready_callbacks(self, **kwargs):
            raise RuntimeError('diagnostic executor failed')
    monkeypatch.setattr(module, 'SingleThreadedExecutor', FailedExecutor)
    observer = module.QosEventObserver([fake.owner], enabled=True)
    observer.start()
    observer._thread.join(1)
    assert not observer.status()['thread_alive']
    assert 'diagnostic executor failed' in observer.status()['error']
    assert fake.default in fake.owner.waitables and fake.default.callback_group is fake.group
    observer.close()
    fake.owner.destroy_node()


def test_different_contexts_fall_back_without_migration(fake):
    observer = module.QosEventObserver([fake.owner, Owner(object())], enabled=True)
    assert observer.start() is False
    assert fake.default in fake.owner.waitables
    assert 'share one ROS context' in observer.status()['error']
    fake.factory.assert_not_called()


def test_busy_diagnostic_thread_defers_endpoint_destruction_until_join(fake, monkeypatch):
    release = threading.Event()
    class BusyExecutor(Executor):
        def wait_for_ready_callbacks(self, **kwargs):
            self.entered.set()
            release.wait(5)
            raise module.TimeoutException()
    monkeypatch.setattr(module, 'SingleThreadedExecutor', BusyExecutor)
    observer = module.QosEventObserver([fake.owner], enabled=True)
    assert observer.start()
    assert observer._executor.entered.wait(1)
    try:
        with pytest.raises(RuntimeError, match='endpoint destruction deferred'):
            observer.close()
        assert fake.default not in fake.owner.waitables
        assert observer.status()['thread_alive']
    finally:
        release.set()
        observer._thread.join(1)
        observer.close()
    assert fake.default in fake.owner.waitables
    fake.owner.destroy_node()


def test_custom_callback_with_similar_name_does_not_move(fake):
    fake.explicit.callback.__qualname__ = 'custom.default_incompatible_qos_callback'
    observer = module.QosEventObserver([fake.owner], enabled=True)
    assert observer.start()
    observer.close()
    assert fake.explicit in fake.owner.waitables
    assert fake.explicit.callback_group is fake.group


def test_diagnostic_poll_drains_ready_batch_then_sleeps_instead_of_busy_spin(fake, monkeypatch):
    done = threading.Event()
    calls, executed = [], []
    for _ in range(2):
        handler = Handler(fake.group, fake.default.event_type, fake.default.callback)
        fake.endpoint.event_handlers.append(handler)
        fake.owner.waitables.append(handler)
    def task():
        executed.append(threading.get_ident())
        if len(executed) == 3:
            done.set()
    task.exception = lambda: None
    class ReadyExecutor(Executor):
        def __init__(self, context):
            super().__init__(context)
            self.tasks = deque([task]*3)
        def wait_for_ready_callbacks(self, **kwargs):
            calls.append(kwargs['timeout_sec'])
            if not self.tasks:
                raise module.TimeoutException()
            return self.tasks.popleft(), None, None
    monkeypatch.setattr(module, 'SingleThreadedExecutor', ReadyExecutor)
    observer = module.QosEventObserver([fake.owner], enabled=True)
    try:
        assert observer.start()
        assert done.wait(1)
        time.sleep(.02)
        assert calls == [0., 0., 0.]
        assert all(ident != threading.get_ident() for ident in executed)
    finally:
        observer.close()
    fake.owner.destroy_node()


def test_adapter_sends_base_zero_before_waiting_for_diagnostic_shutdown(monkeypatch):
    import adapters.core.arm_teleop_node as arm_module
    order = []
    node = arm_module.RobotArmTeleopNode.__new__(arm_module.RobotArmTeleopNode)
    def stop_base(message):
        assert list(message.data) == [0., 0., 0.]
        order.append('base zero')
    def blocked_close():
        order.append('diagnostic close')
        raise RuntimeError('endpoint destruction deferred')
    node.base_pub = SimpleNamespace(publish=stop_base)
    node._qos_event_observer = SimpleNamespace(close=blocked_close)
    destroy = Mock()
    monkeypatch.setattr(arm_module.Node, 'destroy_node', destroy)
    with pytest.raises(RuntimeError, match='deferred'):
        node.destroy_node()
    assert order == ['base zero', 'diagnostic close']
    destroy.assert_not_called()
