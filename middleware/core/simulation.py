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
import math
import xml.etree.ElementTree as ET
from urllib.parse import quote

import yaml
from aiohttp import web

from .config import ConfigError
from .runtime_mode import configure_environment
from .robot_profiles import RobotProfileError, UrdfModel, validate_profile_id


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
        self.viewers = set()
        self.latest_frame = None
        self.display_mode = "web"
        self.web_viewers = set()
        self.demand_file = None
        self.viewer_frames = 0

    def set_viewer(self, viewer, enabled, render_mode='native'):
        if enabled:
            self.viewers.add(viewer)
            if render_mode == 'web':
                self.web_viewers.add(viewer)
        else:
            self.viewers.discard(viewer)
            self.web_viewers.discard(viewer)
        self._sync_viewer_demand()

    def _sync_viewer_demand(self):
        if self.demand_file is not None:
            if self.viewers and self.state in ('starting', 'running'):
                self.demand_file.touch(exist_ok=True)
            else:
                self.demand_file.unlink(missing_ok=True)
        if not self.viewers:
            self.latest_frame = None

    def status(self):
        active = self.store.value['robot_profiles'].get('active', '')
        reason = ''
        configured_root = Path(self.store.value['robot_profiles']['root']).expanduser()
        if not configured_root.is_absolute():
            configured_root = self.store.path.parent / configured_root

        if os.environ.get('HC_RUNTIME_MODE') == 'teleop' or os.environ.get('HC_EXTERNAL_STACK'):
            reason = '当前为真机遥操或命令行托管模式，禁止从网页启动仿真；请停止后使用 ./run.sh middleware。'
        elif configured_root.resolve() != self.profiles.root:
            reason = '机器人配置目录已变更，请先重启中间件。'
        elif not active:
            reason = '请先导入并应用机器人 ZIP 配置。'
        elif not ((self.profiles.root / active / 'arm_teleop.yaml').is_file() or (self.profiles.root / active / 'vr_configs.yml').is_file()):
            reason = '当前机器人包缺少 arm_teleop.yaml 或 vr_configs.yml，无法启动 PyBullet。'
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
                    runtime_mode=os.environ.get('HC_RUNTIME_MODE', 'simulation'),
                    ros_domain_id=int(os.environ.get('ROS_DOMAIN_ID', self.store.value['ros']['domain_id'])),
                    hardware_domain_id=os.environ.get('HC_HARDWARE_DOMAIN_ID'),
                    display_mode=self.display_mode, viewer_clients=len(self.viewers),
                    web_viewer_clients=len(self.web_viewers), viewer_frames=self.viewer_frames,
                    available=not reason, unavailable_reason=reason,
                    pids=[p.pid for p in self.processes if p.returncode is None])

    async def _read(self, label, process):
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            text = line.decode(errors='replace').rstrip()
            if text.startswith('HC_SIMULATION_FRAME '):
                if not self.viewers:
                    continue
                try:
                    self.latest_frame = json.loads(text.split(' ', 1)[1])
                    self.viewer_frames += 1
                except (ValueError, TypeError):
                    pass
                continue
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
            if (profile / 'arm_teleop.yaml').is_file():
                config = yaml.safe_load((profile / 'arm_teleop.yaml').read_text())
                urdf_rel = config.get('robot', {}).get('urdf_path') if isinstance(config, dict) else None
                if not urdf_rel and isinstance(config, dict):
                    urdf_rel = config.get('urdf_path')
            elif (profile / 'vr_configs.yml').is_file():
                config = yaml.safe_load((profile / 'vr_configs.yml').read_text())
                urdf_rel = config.get('urdf_path') if isinstance(config, dict) else None
            else:
                raise SimulationError('当前机器人包缺少 arm_teleop.yaml 或 vr_configs.yml')
            if not urdf_rel:
                raise SimulationError('配置文件中缺少 urdf_path')
            urdf = (profile / urdf_rel).resolve()
            urdf.relative_to(profile.resolve())
            UrdfModel(urdf.read_bytes())
            if options['with_control']:
                for filename in ('controller_v23.yml', 'arm_teleop.yaml'):
                    if not (profile / filename).is_file():
                        raise SimulationError(f'当前机器人包缺少 {filename}')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise SimulationError(f'机器人仿真配置无效：{exc}') from exc
        native_viewer = bool(self.viewers - self.web_viewers) and not options['headless']
        self.profile_id, self.options = name, options
        self.latest_frame = None
        self.display_mode = 'local_viewer' if native_viewer else 'web'
        self.state, self.error = 'starting', ''
        self.logs.clear()
        self.log_dir = self.project / 'runtime/simulation_logs' / uuid.uuid4().hex
        self.log_dir.mkdir(parents=True, mode=0o700)
        self.demand_file = self.log_dir / "viewer.enabled"
        self._sync_viewer_demand()
        domain_id = configure_environment(self.store.value['ros']['domain_id'])
        env = os.environ.copy()
        for key in ('HC_CONTROLLER_CONFIG', 'HC_ARM_TELEOP_CONFIG'):
            env.pop(key, None)
        env.update(HC_ROBOT_CONFIG_ROOT=str(self.profiles.root), HC_ROBOT_NAME=name,
                   HC_MIDDLEWARE_CONFIG=str(self.store.path.resolve()), HC_TELEOP_MODE='sim',
                   # The gateway and both children share the isolated simulation domain.
                   ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1', HC_RUNTIME_MODE='simulation', PYTHONUNBUFFERED='1',
                   PYTHONPATH=f'{self.project}:{self.project / ".deps"}:' + env.get('PYTHONPATH', ''))
        command = ['/usr/bin/python3', str(self.project / 'simulation/general_sim_robot_control_node_ros2.py'),
                   '--profile', str(profile)]
        # Browser display is independent from physics: every Dashboard simulator
        # runs DIRECT, and produces frames only while a viewer is subscribed.
        command += ['--headless', '--viewer-demand-file', str(self.demand_file)]
        if options['debug_joints']:
            command.append('--init_debug')
        command += ['--ros-args']
        for topic, public_topic in {'joint_states': '/observation/joint_states', 'joint_cmd': '/action/joint_states', 'target_ee_poses': '/action/ee_poses', 'target_base_move': '/action/base/command', 'hardware_ready': '/status/hardware/ready', 'target_finger_joints': '/action/hands/joint_targets', 'target_gripper_status': '/action/hands/gripper_status', 'target_joint_from_vr': '/control/solver/target_joint_from_vr'}.items():
            command += ['-r', f'/io_teleop/{topic}:={public_topic}']
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
            done, _ = await asyncio.wait(waits, timeout=self.options.get("startup_timeout_sec", 120), return_when=asyncio.FIRST_COMPLETED)
            if not done and self.state == 'running':
                done, _ = await asyncio.wait(waits, return_when=asyncio.FIRST_COMPLETED)
            code = next(iter(done)).result() if done else f'启动超时（{self.options.get("startup_timeout_sec", 120)} 秒）'
            async with self.lock:
                self.watcher = None
                await self.stop()
                self.state = 'failed'
                self.error = f'仿真或控制进程已退出（退出码 {code}），请查看日志后重新启动。'
                if any('X connection to ' in line and 'broken' in line for line in self.logs):
                    self.error = 'PyBullet 的 X11 显示连接已断开。请保持本机 ssh -Y 会话连接，从该会话重新启动网页服务后再启动仿真。'
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
        self._sync_viewer_demand()
        self.latest_frame = None
        return self.status()


def register_simulation_routes(app, service):
    async def status(_request):
        return web.json_response(service.status())

    def asset_path(profile_id, relative):
        root = (service.profiles.root / validate_profile_id(profile_id)).resolve()
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise web.HTTPForbidden(text='asset escapes robot package') from exc
        if path.suffix.lower() not in {'.urdf', '.stl', '.obj', '.dae', '.png', '.jpg', '.jpeg', '.mtl', '.glb', '.gltf', '.bin'} or not path.is_file():
            raise web.HTTPNotFound()
        return path

    async def model(_request):
        name = service.profile_id if service.state in ('starting', 'running') else service.store.value['robot_profiles'].get('active', '')
        try:
            meta = service.profiles.get(name)
            folder = service.profiles.root / validate_profile_id(name)
            config_path = folder / 'arm_teleop.yaml'
            if not config_path.is_file():
                config_path = folder / 'vr_configs.yml'
            config = yaml.safe_load(config_path.read_text()) or {}
            relative = config.get('robot', {}).get('urdf_path') or config.get('urdf_path') or meta.get('urdf')
            if not isinstance(relative, str):
                raise ValueError('profile has no URDF path')
            root = ET.parse(asset_path(name, relative)).getroot()
            child_links = {j.find('child').get('link') for j in root.findall('joint') if j.find('child') is not None}
            link = next((v for v in root.findall('link') if v.get('name') not in child_links), None)
            origin = link.find('inertial/origin') if link is not None else None
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0., 0., 0.]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0., 0., 0.]
            if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
                raise ValueError('invalid root inertial origin')
            sr, sp, sy = (math.sin(v / 2) for v in rpy)
            cr, cp, cy = (math.cos(v / 2) for v in rpy)
            quat = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
            base = '/api/simulation/assets/' + quote(name, safe='') + '/'
            return web.json_response({'profile_id': name, 'urdf_url': base + quote(relative, safe='/'), 'asset_base_url': base, 'base_inertial_pose': [xyz, quat]}, headers={'Cache-Control': 'no-store'})
        except (RobotProfileError, OSError, KeyError, ValueError, ET.ParseError, yaml.YAMLError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def asset(request):
        try:
            path = asset_path(request.match_info['profile_id'], request.match_info['path'])
        except RobotProfileError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return web.FileResponse(path)

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
                proposed['simulation'].update(value)
                saved = service.store.save(proposed)
                return web.json_response(saved['simulation'])
            except (ConfigError, json.JSONDecodeError) as exc:
                raise web.HTTPBadRequest(text=str(exc)) from exc

    app.router.add_get('/api/simulation/model', model)
    app.router.add_get('/api/simulation/assets/{profile_id}/{path:.*}', asset)
    app.router.add_put('/api/simulation/options', options)
    app.router.add_get('/api/simulation', status)
    app.router.add_post('/api/simulation/{action}', action)
