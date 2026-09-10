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
        self.commands = []
        real_spawn = asyncio.create_subprocess_exec

        async def spawn(*args, **kwargs):
            self.commands.append((args, kwargs['env']))
            # Exercise the real process supervisor, signals and log reader with
            # a bounded child, without requiring ROS or a graphics server.
            prefix = args[:args.index('--') + 1]
            return await real_spawn(*prefix, sys.executable, '-u', '-c',
                'import time; print("HC_SIMULATION_READY", flush=True); time.sleep(60)', **kwargs)
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
        self.assertEqual(env['ROS_DOMAIN_ID'], str(self.store.value['ros']['domain_id']))
        self.assertEqual(env['HC_TELEOP_MODE'], 'sim')
        async with self.service.lock:
            await self.service.stop()
        self.assertIsNotNone(process.returncode)
        self.assertEqual(self.service.state, 'stopped')

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
            response = await client.put('/api/simulation/options', json={
                'headless':True, 'with_control':False, 'debug_joints':True})
            self.assertEqual(response.status, 200)
            self.assertTrue(ConfigStore(self.store.path).load()['simulation']['debug_joints'])
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

    def test_simulation_options_are_strict_booleans(self):
        for key in ('headless', 'with_control', 'debug_joints'):
            cfg = copy.deepcopy(self.store.value)
            cfg['simulation'][key] = 'false'
            with self.assertRaises(ConfigError):
                validate_config(cfg)
