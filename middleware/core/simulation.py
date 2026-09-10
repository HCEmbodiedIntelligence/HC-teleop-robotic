"""Own independently launched simulation processes for the Dashboard."""
from __future__ import annotations

import asyncio
import copy
import json
from collections import deque
import os
from pathlib import Path
import signal
import uuid

import yaml
from aiohttp import web

from .config import ConfigError
from .robot_profiles import RobotProfileError, UrdfModel


class SimulationError(ValueError):
    pass


class SimulationService:
    def __init__(self, store, profiles):
        self.store, self.profiles = store, profiles
        self.project = Path(__file__).resolve().parents[2]
        self.lock = asyncio.Lock()
        self.processes = []
        self.readers = []
        self.watcher = None
        self.state = 'stopped'
        self.profile_id = ''
        self.error = ''
        self.logs = deque(maxlen=100)
        self.options = None
        self.log_dir = None

    def status(self):
        active = self.store.value['robot_profiles'].get('active', '')
        reason = ''
        configured_root = Path(self.store.value['robot_profiles']['root']).expanduser()
        if not configured_root.is_absolute():
            configured_root = self.store.path.parent / configured_root

        if os.environ.get('HC_EXTERNAL_STACK'):
            reason = '当前由 run.sh sim/teleop 管理控制进程；请使用 ./run.sh middleware 启动后在此管理仿真。'
        elif configured_root.resolve() != self.profiles.root:
            reason = '机器人配置目录已变更，请先重启中间件。'
        elif not active:
            reason = '请先导入并应用机器人 ZIP 配置。'
        elif not (self.profiles.root / active / 'vr_configs.yml').is_file():
            reason = '当前机器人包缺少 vr_configs.yml，无法启动 PyBullet。'
        logs = list(self.logs)
        if self.state == 'failed' and self.log_dir:
            for name in ('v23_solver.log', 'teleop_controller.log'):
                try:
                    with (self.log_dir / name).open('rb') as stream:
                        stream.seek(0, 2)
                        stream.seek(max(0, stream.tell() - 6000))
                        logs.extend(f'[{name}] {line}' for line in stream.read().decode(errors='replace').splitlines()[-35:])
                except OSError:
                    pass
        return dict(state=self.state, profile_id=self.profile_id, active_profile=active,
                    options=self.options, error=self.error, logs=logs,
                    available=not reason, unavailable_reason=reason,
                    pids=[p.pid for p in self.processes if p.returncode is None])

    async def _read(self, label, process):
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            text = line.decode(errors='replace').rstrip()
            self.logs.append(f'[{label}] {text}'[-2000:])
            if 'HC_SIMULATION_READY' in text and self.state == 'starting':
                self.state = 'running'

    async def _spawn(self, label, command, env):
        supervisor = self.project / 'tools/runtime/process_supervisor.py'
        # Arguments are passed separately; profile paths never become shell code.
        process = await asyncio.create_subprocess_exec(
            '/usr/bin/python3', str(supervisor), '--parent', str(os.getpid()),
            '--grace', '8', '--', '/bin/bash', '-c',
            'source /opt/ros/humble/setup.bash; exec "$@"', 'hc-simulation', *command,
            cwd=self.project, env=env, start_new_session=True,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            limit=1024 * 1024,
        )
        self.processes.append(process)
        self.readers.append(asyncio.create_task(self._read(label, process)))

    async def start(self):
        # Caller holds lock, also shared with profile/config mutations.
        if self.processes:
            raise SimulationError('仿真已启动，请先停止或使用重启。')
        status = self.status()
        if not status['available']:
            raise SimulationError(status['unavailable_reason'])
        options = dict(self.store.value['simulation'])
        name = status['active_profile']
        profile = self.profiles.root / name
        try:
            self.profiles.get(name)
            config = yaml.safe_load((profile / 'vr_configs.yml').read_text())
            urdf = (profile / config['urdf_path']).resolve()
            urdf.relative_to(profile.resolve())
            UrdfModel(urdf.read_bytes())
            if options['with_control']:
                for filename in ('controller_v23.yml', 'arm_teleop.yaml'):
                    if not (profile / filename).is_file():
                        raise SimulationError(f'当前机器人包缺少 {filename}')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise SimulationError(f'机器人仿真配置无效：{exc}') from exc
        if not options['headless'] and not os.environ.get('DISPLAY'):
            raise SimulationError('中间件没有可用的桌面 DISPLAY，请选择无窗口模式，或从桌面终端启动中间件。')
        self.profile_id, self.options = name, options
        self.state, self.error = 'starting', ''
        self.logs.clear()
        self.log_dir = self.project / 'runtime/simulation_logs' / uuid.uuid4().hex
        self.log_dir.mkdir(parents=True, mode=0o700)
        env = os.environ.copy()
        for key in ('HC_CONTROLLER_CONFIG', 'HC_ARM_TELEOP_CONFIG'):
            env.pop(key, None)
        env.update(HC_ROBOT_CONFIG_ROOT=str(self.profiles.root), HC_ROBOT_NAME=name,
                   HC_MIDDLEWARE_CONFIG=str(self.store.path.resolve()), HC_TELEOP_MODE='sim',
                   ROS_DOMAIN_ID=str(self.store.value['ros']['domain_id']), PYTHONUNBUFFERED='1',
                   PYTHONPATH=f'{self.project}:{self.project / ".deps"}:' + env.get('PYTHONPATH', ''))
        command = ['/usr/bin/python3', str(self.project / 'simulation/general_sim_robot_control_node_ros2.py'),
                   '--profile', str(profile)]
        if options['headless']:
            command.append('--headless')
        if options['debug_joints']:
            command.append('--init_debug')
        command += ['--ros-args']
        for topic in ('joint_states','joint_cmd','target_joint_from_vr','target_finger_joints',
                      'target_ee_poses','target_gripper_status','target_base_move','hardware_ready'):
            command += ['-r', f'/io_teleop/{topic}:=/hc_teleop/{topic}']
        try:
            await self._spawn('PyBullet', command, env)
            if options['with_control']:
                await self._spawn('控制', [str(self.project / 'adapters/start.sh'),
                                         '--config', str(self.store.path.resolve()),
                                         '--log-dir', str(self.log_dir)], env)
        except Exception as exc:
            await self.stop()
            self.state, self.error = 'failed', str(exc)
            raise SimulationError(f'启动失败：{exc}') from exc
        self.watcher = asyncio.create_task(self._watch())
        return self.status()

    async def _watch(self):
        waits = [asyncio.create_task(p.wait()) for p in self.processes]
        try:
            done, _ = await asyncio.wait(waits, timeout=45, return_when=asyncio.FIRST_COMPLETED)
            if not done and self.state == 'running':
                done, _ = await asyncio.wait(waits, return_when=asyncio.FIRST_COMPLETED)
            code = next(iter(done)).result() if done else '启动超时（45 秒）' 
            async with self.lock:
                self.watcher = None
                await self.stop()
                self.state = 'failed'
                self.error = f'仿真或控制进程已退出（退出码 {code}），请查看日志后重新启动。'
        finally:
            for wait in waits:
                wait.cancel()
            await asyncio.gather(*waits, return_exceptions=True)

    async def stop(self):
        self.state = 'stopping' if self.processes else 'stopped'
        if self.watcher:
            self.watcher.cancel()
            await asyncio.gather(self.watcher, return_exceptions=True)
            self.watcher = None
        for process in self.processes:
            if process.returncode is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for process in self.processes:
            try:
                await asyncio.wait_for(process.wait(), timeout=12)
            except asyncio.TimeoutError:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await process.wait()
        for reader in self.readers:
            try:
                await asyncio.wait_for(reader, timeout=2)
            except Exception:
                reader.cancel()
        self.processes.clear()
        self.readers.clear()
        self.state = 'stopped'
        return self.status()


def register_simulation_routes(app, service):
    async def status(_request):
        return web.json_response(service.status())

    async def action(request):
        async with service.lock:
            try:
                operation = request.match_info['action']
                if operation not in ('start', 'stop', 'restart'):
                    raise web.HTTPNotFound()
                if operation in ('stop', 'restart'):
                    await service.stop()
                if operation in ('start', 'restart'):
                    await service.start()
                return web.json_response(service.status())
            except (SimulationError, RobotProfileError) as exc:
                raise web.HTTPConflict(text=str(exc)) from exc

    async def options(request):
        async with service.lock:
            try:
                value = await request.json()
                if not isinstance(value, dict) or set(value) != {'headless', 'with_control', 'debug_joints'}:
                    raise ConfigError('需要 headless、with_control、debug_joints 三个布尔选项')
                proposed = copy.deepcopy(service.store.value)
                proposed['simulation'] = value
                saved = service.store.save(proposed)
                return web.json_response(saved['simulation'])
            except (ConfigError, json.JSONDecodeError) as exc:
                raise web.HTTPBadRequest(text=str(exc)) from exc

    app.router.add_put('/api/simulation/options', options)
    app.router.add_get('/api/simulation', status)
    app.router.add_post('/api/simulation/{action}', action)
