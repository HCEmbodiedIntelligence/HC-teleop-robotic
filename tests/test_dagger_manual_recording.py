"""X/Y use the normal recorder while DAgger owns robot motion."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

@pytest.mark.parametrize('state', ['AUTO','MANUAL','HOLD'])
def test_xy_recording_during_dagger_writes_mcap(tmp_path, state):
    from mcap.reader import make_reader
    from middleware.core.app import MiddlewareRuntime
    from middleware.core.config import validate_config
    from middleware.core.topic_recorder import TopicRecorder
    from middleware.core.ros_bridge import RosBridge
    cfg = validate_config({'vr':{'publish_to_ros':False}})
    cfg['ros']['recording'].update(directory=str(tmp_path/'data'),min_free_bytes=0)
    recorder=TopicRecorder(cfg['ros']['recording'], tmp_path)
    runtime=MiddlewareRuntime(cfg,tmp_path)
    runtime.recorder=recorder
    bridge=RosBridge({'enabled':True,'intervention':{'automatic_recording':False}},lambda *a:None)
    runtime.ros=bridge
    bridge.ownership.state=state
    bridge._enqueue=Mock(return_value=True)
    runtime.recording_ros=SimpleNamespace(prepare_recording=Mock())
    runtime.emit=Mock()
    try:
        with ThreadPoolExecutor(max_workers=1) as commands:
            runtime._record_commands=commands
            def button(mask,pressed=0):
                runtime._on_pose(SimpleNamespace(left_input=SimpleNamespace(held_mask=mask,pressed_mask=pressed)))
                commands.submit(lambda:None).result(timeout=5)
            button(1,1)
            assert recorder.is_recording()
            path=Path(recorder.status()['path'])
            button(1)
            assert str(path)==recorder.status()['path']
            for i in range(5):
                recorder.record({'topic':'/test/manual_recording','payload':{'state':state,'frame':i}})
            button(0)
            button(2,2)
            assert not recorder.is_recording()
            with path.open('rb') as stream:
                messages=list(make_reader(stream).iter_messages())
            assert len(messages)>=5
            assert bridge.ownership.state==state
            assert not any(call.args[0]['kind']=='recording_error' for call in runtime.emit.call_args_list)
            notifications=[c for c in bridge._enqueue.call_args_list if c.args[0]=='publish' and c.args[1][0]=='/teleop/recording_state']
            assert len(notifications)==2
    finally:
        recorder.shutdown()

def test_exit_dagger_selects_vr_without_enabling_motion():
    from middleware.core.ros_bridge import RosBridge
    bridge=RosBridge({'enabled':True,'command_mux':{'source':'motion'},'intervention':{'automatic_recording':False}},lambda *a:None)
    bridge._enqueue=Mock(return_value=True)
    bridge.ownership.enable()
    bridge._command_source='motion'
    bridge._command_output_enabled=True
    bridge.ownership.disable()
    assert bridge._command_source=='vr'
    assert not bridge._command_output_enabled
    assert bridge.ownership.state=='OFF'
    assert any(c.args==('control_source','vr') for c in bridge._enqueue.call_args_list)

def test_home_queue_rejects_auto_and_accepts_only_current_manual_authority():
    from middleware.core.ros_bridge import RosBridge
    from middleware.core.control_ownership import ControlToken
    bridge=RosBridge({'enabled':True},lambda *a:None)
    bridge._legacy_enabled=True
    bridge._enqueue=Mock(return_value=True)
    bridge._ready_locked=lambda now:True
    bridge._command_output_enabled=True
    bridge.ownership.state='AUTO'
    assert not bridge.request_manual_home()
    bridge.ownership.state='MANUAL'
    bridge.ownership.token=ControlToken('c','s',2,'vr','i')
    assert bridge.request_manual_home()
    bridge._enqueue.assert_called_once_with('manual_home',bridge.ownership.token)


def test_home_http_routes_to_current_authority_and_uses_runtime_source(tmp_path, monkeypatch):
    import asyncio
    from unittest.mock import AsyncMock
    from aiohttp.test_utils import TestClient, TestServer
    import middleware.core.app as app_module
    from middleware.core.config import ConfigStore, validate_config
    runtime=app_module.MiddlewareRuntime(validate_config({}),tmp_path)
    runtime.start=AsyncMock()
    runtime.stop=AsyncMock()
    ros=SimpleNamespace(ownership=SimpleNamespace(active=True),request_manual_home=Mock(return_value=False),
                        status=lambda:{'command_mux':{'source':'vr'}},publish=Mock(return_value=True))
    runtime.ros=ros
    monkeypatch.setattr(app_module,'MiddlewareRuntime',lambda *a:runtime)
    store=ConfigStore(tmp_path/'config.yaml')
    async def check():
        app=app_module.create_app(store)
        # Stored source intentionally differs after a DAgger session exits.
        store.value['ros']['command_mux']['source']='motion'
        async with TestClient(TestServer(app)) as client:
            response=await client.post('/api/teleop/home')
            assert response.status==409
            ros.request_manual_home.return_value=True
            response=await client.post('/api/teleop/home')
            assert response.status==202
            ros.publish.assert_not_called()
            ros.ownership.active=False
            response=await client.post('/api/teleop/home')
            assert response.status==202
            ros.publish.assert_called_once_with('/teleop/arm/home','std_msgs/msg/Bool',{'data':True})
    asyncio.run(check())
