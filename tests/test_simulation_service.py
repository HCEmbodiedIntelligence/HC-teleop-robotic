import asyncio
import copy
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer

from middleware.core.app import create_app
from middleware.core.config import ConfigError, ConfigStore, validate_config
from middleware.core.robot_profiles import RobotProfileManager
from middleware.core.simulation import SimulationError, SimulationService
from tests.test_robot_profiles import URDF, io_yaml


class SimulationServiceTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.store = ConfigStore(root / 'middleware.yaml')
        cfg = self.store.load()
        cfg['robot_profiles'] = {'root': str(root / 'profiles'), 'active': 'test_bot'}
        cfg['simulation'] = {'headless': True, 'with_control': False, 'debug_joints': False}
        for section in ('ros', 'vr', 'camera', 'safety'):
            cfg[section]['enabled'] = False
        cfg['safety']['stop_on_startup'] = False
        self.store.save(cfg)
        self.profiles = RobotProfileManager(root / 'profiles')
        for name in ('test_bot', 'other_bot'):
            self.profiles.import_profile(name, name, 'robot.urdf', URDF, 'vr_configs.yml', io_yaml())
        self.service = SimulationService(self.store, self.profiles)
        self.env = patch.dict(os.environ)
        self.env.start()
        os.environ.pop('HC_EXTERNAL_STACK', None)
        os.environ.pop('ROS_DOMAIN_ID', None)
        os.environ.pop('HC_RUNTIME_MODE', None)
        os.environ.pop('HC_SIMULATION_DOMAIN_ID', None)
        os.environ.pop('HC_HARDWARE_DOMAIN_ID', None)
        self.commands = []
        real_spawn = asyncio.create_subprocess_exec

        async def spawn(*args, **kwargs):
            self.commands.append((args, kwargs['env']))
            # Exercise the real process supervisor, signals and log reader with
            # a bounded child, without requiring ROS or a graphics server.
            prefix = args[:args.index('--') + 1]
            return await real_spawn(*prefix, sys.executable, '-u', '-c',
                'import time; print("HC_SIMULATION_READY", flush=True); print(\'HC_SIMULATION_FRAME {"positions":{"a":0.1},"base_pose":[[0,0,0],[0,0,0,1]]}\', flush=True); time.sleep(60)', **kwargs)
        self.spawn_patch = patch('middleware.core.simulation.asyncio.create_subprocess_exec', side_effect=spawn)
        self.spawn_patch.start()

    async def asyncTearDown(self):
        async with self.service.lock:
            await self.service.stop()
        self.spawn_patch.stop()
        self.env.stop()
        self.temp.cleanup()

    async def wait_state(self, state):
        for _ in range(100):
            if self.service.state == state:
                return
            await asyncio.sleep(0.03)
        self.fail(self.service.status())

    async def test_start_stop_is_independent_and_duplicate_start_rejected(self):
        async with self.service.lock:
            await self.service.start()
            with self.assertRaises(SimulationError):
                await self.service.start()
        await self.wait_state('running')
        process = self.service.processes[0]
        args, env = self.commands[0]
        self.assertIn(str(self.profiles.root / 'test_bot'), args)
        self.assertIn('--headless', args)
        self.assertEqual(env['HC_ROBOT_NAME'], 'test_bot')
        self.assertNotEqual(env['ROS_DOMAIN_ID'], '14')
        self.assertEqual(env['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(env['HC_TELEOP_MODE'], 'sim')
        async with self.service.lock:
            await self.service.stop()
        self.assertIsNotNone(process.returncode)
        self.assertEqual(self.service.state, 'stopped')

    async def test_launcher_domain_override_reaches_simulation_and_controller(self):
        os.environ['ROS_DOMAIN_ID'] = '229'
        self.store.value['simulation']['with_control'] = True
        self.assertEqual(self.store.value['ros']['domain_id'], 14)
        async with self.service.lock:
            await self.service.start()
        await self.wait_state('running')
        self.assertEqual(len(self.commands), 2)
        self.assertEqual(len({env['ROS_DOMAIN_ID'] for _, env in self.commands}), 1)
        self.assertTrue(all(env['ROS_DOMAIN_ID'] not in ('14', '229') for _, env in self.commands))
        self.assertTrue(all(env['ROS_LOCALHOST_ONLY'] == '1' for _, env in self.commands))

    async def test_child_exit_is_reported_and_owned_peers_stop(self):
        self.store.value['simulation']['with_control'] = True
        async with self.service.lock:
            await self.service.start()
        await self.wait_state('running')
        processes = list(self.service.processes)
        processes[0].terminate()
        await self.wait_state('failed')
        self.assertTrue(self.service.error)
        self.assertTrue(all(p.returncode is not None for p in processes))

    async def test_missing_sim_config_and_external_stack_are_rejected(self):
        (self.profiles.root / 'test_bot/vr_configs.yml').unlink(missing_ok=True)
        (self.profiles.root / 'test_bot/arm_teleop.yaml').unlink(missing_ok=True)
        async with self.service.lock:
            with self.assertRaisesRegex(SimulationError, 'arm_teleop.yaml 或 vr_configs.yml'):
                await self.service.start()
        os.environ['HC_EXTERNAL_STACK'] = '1'
        self.assertIn('run.sh middleware', self.service.status()['unavailable_reason'])
        self.assertFalse(self.commands)

    async def test_api_options_launch_profile_switch_and_shutdown(self):
        client = TestClient(TestServer(create_app(self.store)))
        await client.start_server()
        try:
            response = await client.put('/api/simulation/options', json={'headless':'yes'})
            self.assertEqual(response.status, 400)
            config = await (await client.get('/api/config')).json()
            self.assertEqual(config['ros']['domain_id'], 14)
            self.assertNotEqual(os.environ['ROS_DOMAIN_ID'], '14')
            response = await client.put('/api/simulation/options', json={
                'headless':True, 'with_control':False, 'debug_joints':True})
            self.assertEqual(response.status, 200)
            self.assertTrue(ConfigStore(self.store.path).load()['simulation']['debug_joints'])
            self.assertEqual(ConfigStore(self.store.path).load()['simulation']['startup_timeout_sec'], 120)
            response = await client.post('/api/simulation/start')
            self.assertEqual(response.status, 200, await response.text())
            response = await client.post('/api/simulation/start')
            self.assertEqual(response.status, 409)
            unchanged = await (await client.get('/api/config')).json()
            unchanged['ros']['recording']['directory'] = 'runtime/test_recordings'
            response = await client.put('/api/config', json=unchanged)
            self.assertEqual(response.status, 200)
            status = await (await client.get('/api/simulation')).json()
            self.assertIn(status['state'], ('starting', 'running'))
            response = await client.post('/api/robot-profiles/other_bot/activate')
            self.assertEqual(response.status, 200)
            status = await (await client.get('/api/simulation')).json()
            self.assertEqual(status['state'], 'stopped')
            self.assertEqual(status['active_profile'], 'other_bot')
            response = await client.post('/api/simulation/restart')
            self.assertEqual(response.status, 200)
            self.assertEqual(self.commands[-1][1]['HC_ROBOT_NAME'], 'other_bot')
            response = await client.get('/api/status')
            self.assertEqual(response.status, 200)
            response = await client.post('/api/simulation/stop')
            self.assertEqual(response.status, 200)
            response = await client.get('/api/status')
            self.assertEqual(response.status, 200)
        finally:
            await client.close()

    async def test_native_viewer_uses_headless_backend_and_receives_frames(self):
        self.store.value['simulation']['headless'] = False
        self.store.save(self.store.value)
        os.environ.pop('DISPLAY', None)
        client = TestClient(TestServer(create_app(self.store)))
        await client.start_server()
        ws = await client.ws_connect('/ws')
        try:
            await ws.send_json({'type': 'simulation_viewer', 'data': {'enabled': True}})
            while (await ws.receive_json()) .get('type') != 'simulation_viewer_ack':
                pass
            response = await client.post('/api/simulation/start')
            self.assertEqual(response.status, 200, await response.text())
            status = await response.json()
            self.assertEqual(status['display_mode'], 'local_viewer')
            self.assertIn('--headless', self.commands[0][0])
            self.assertIn('--stream-viewer', self.commands[0][0])
            async def frame():
                while True:
                    message = await ws.receive_json()
                    if message.get('type') == 'simulation_frame':
                        return message
            message = await asyncio.wait_for(frame(), timeout=3)
            self.assertEqual(message['data']['positions'], {'a': 0.1})
            self.assertEqual(message['data']['profile_id'], 'test_bot')
            await ws.close()
            await asyncio.sleep(0.01)
            status = await (await client.get('/api/simulation')).json()
            self.assertEqual(status['viewer_clients'], 0)
        finally:
            await ws.close()
            await client.close()

    async def test_stalled_viewer_close_does_not_freeze_healthy_viewer(self):
        received = asyncio.Queue()

        class StalledViewer:
            async def send_json(self, event):
                await asyncio.sleep(60)

            async def close(self):
                await asyncio.sleep(60)

        class HealthyViewer:
            async def send_json(self, event):
                await received.put(event)

        stalled, healthy = StalledViewer(), HealthyViewer()
        self.service.viewers.update((stalled, healthy))
        self.service.state = 'running'
        self.service.latest_frame = {'positions': {'a': 0.1}, 'base_pose': [[0, 0, 0], [0, 0, 0, 1]]}
        with patch('middleware.core.app.SimulationService', return_value=self.service):
            client = TestClient(TestServer(create_app(self.store)))
        await client.start_server()
        try:
            await asyncio.wait_for(received.get(), timeout=2)
            self.service.latest_frame = {'positions': {'a': 0.2}, 'base_pose': [[0, 0, 0], [0, 0, 0, 1]]}
            event = await asyncio.wait_for(received.get(), timeout=2)
            self.assertEqual(event['data']['positions'], {'a': 0.2})
            self.assertNotIn(stalled, self.service.viewers)
        finally:
            await client.close()

    def test_simulation_options_are_strict_booleans(self):
        for key in ('headless', 'with_control', 'debug_joints'):
            cfg = copy.deepcopy(self.store.value)
            cfg['simulation'][key] = 'false'
            with self.assertRaises(ConfigError):
                validate_config(cfg)


def test_startup_timeout_validation():
    for invalid in (True, 0, -1, 601, float("nan"), float("inf"), "120"):
        with unittest.TestCase().assertRaises(ConfigError):
            validate_config({"simulation": {"startup_timeout_sec": invalid}})
    assert validate_config({"simulation": {"startup_timeout_sec": 180}})["simulation"]["startup_timeout_sec"] == 180


class TeleopSimulationBoundaryTests(unittest.IsolatedAsyncioTestCase):
    async def test_teleop_cannot_start_simulation_without_external_stack_flag(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(os.environ, {"HC_RUNTIME_MODE": "teleop"}):
            os.environ.pop("HC_EXTERNAL_STACK", None)
            store = ConfigStore(Path(root) / "config.yaml")
            store.load()
            service = SimulationService(store, RobotProfileManager(Path(root) / "profiles"))
            self.assertFalse(service.status()["available"])
            with self.assertRaises(SimulationError):
                await service.start()
            self.assertEqual(service.processes, [])
