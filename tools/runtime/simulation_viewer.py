#!/usr/bin/env python3
"""Display remote simulation state locally; never connect to ROS or command a robot."""
from __future__ import annotations

import argparse
import asyncio
import math
import os
from pathlib import Path
import sys
import threading
import time

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / 'simulation'))


def normalize_frame(data):
    """Reject malformed/nonfinite remote state before passing it to PyBullet."""
    if not isinstance(data, dict) or not isinstance(data.get('positions'), dict):
        raise ValueError('frame requires a positions mapping')
    positions = {str(name): float(value) for name, value in data['positions'].items()}
    if not all(math.isfinite(value) for value in positions.values()):
        raise ValueError('nonfinite joint position')
    pose = data.get('base_pose')
    if not isinstance(pose, (list, tuple)) or len(pose) != 2 or len(pose[0]) != 3 or len(pose[1]) != 4:
        raise ValueError('base_pose requires position[3] and quaternion[4]')
    position, quaternion = tuple(map(float, pose[0])), tuple(map(float, pose[1]))
    if not all(math.isfinite(value) for value in position + quaternion):
        raise ValueError('nonfinite base pose')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-8:
        raise ValueError('invalid base quaternion')
    return dict(positions=positions, base_pose=(position, tuple(v / norm for v in quaternion)),
                profile_id=data.get('profile_id'))


class LatestState:
    """One immutable frame slot bounds latency even when GUI rendering slows."""
    def __init__(self):
        self.lock = threading.Lock()
        self.status = {}
        self.frame = None
        self.sequence = 0

    def set_status(self, status):
        with self.lock:
            previous = self.status.get('profile_id')
            if status.get('state') != 'running' or previous != status.get('profile_id'):
                self.frame = None
            self.status = dict(status)

    def set_frame(self, frame):
        with self.lock:
            if self.status.get('state') == 'running':
                self.frame = frame
                self.sequence += 1

    def snapshot(self):
        with self.lock:
            return dict(self.status), self.frame, self.sequence


async def receive_remote(server, state, stop):
    import aiohttp
    timeout = aiohttp.ClientTimeout(total=4)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        async def poll():
            while not stop.is_set():
                try:
                    async with session.get(server + '/api/simulation') as response:
                        response.raise_for_status()
                        state.set_status(await response.json())
                except (aiohttp.ClientError, asyncio.TimeoutError, ValueError):
                    state.set_status({'state': 'disconnected'})
                await asyncio.sleep(.5)

        task = asyncio.create_task(poll())
        try:
            while not stop.is_set():
                try:
                    async with session.ws_connect(server + '/ws', heartbeat=10) as ws:
                        await ws.send_json({'type': 'simulation_viewer', 'data': {'enabled': True}})
                        while not stop.is_set():
                            try:
                                message = await ws.receive(timeout=.5)
                            except asyncio.TimeoutError:
                                continue
                            if message.type == aiohttp.WSMsgType.TEXT:
                                try:
                                    payload = message.json()
                                    if payload.get('type') == 'simulation_frame':
                                        state.set_frame(normalize_frame(payload.get('data')))
                                except (ValueError, TypeError, AttributeError):
                                    continue
                            elif message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                                break
                except (aiohttp.ClientError, asyncio.TimeoutError):
                    pass
                # A broken stream must never keep replaying stale display state.
                state.set_status({'state': 'disconnected'})
                await asyncio.sleep(.5)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


def load_profile(path):
    import yaml
    path = Path(path).expanduser().resolve()
    if not path.is_dir():
        raise ValueError('--profile must name a local robot profile directory')
    meta_path = path / 'profile.yaml'
    meta = yaml.safe_load(meta_path.read_text()) if meta_path.exists() else {}
    profile_id = str((meta or {}).get('id', path.name))
    config_path = next((path / name for name in ('arm_teleop.yaml', 'vr_configs.yml') if (path / name).is_file()), None)
    if config_path is None:
        raise ValueError(f'no arm_teleop.yaml or vr_configs.yml in {path}')
    config = yaml.safe_load(config_path.read_text())
    urdf = config.get('urdf_path') or config.get('robot', {}).get('urdf_path')
    if not urdf:
        raise ValueError('profile has no urdf_path')
    config.setdefault('arm_teleop_path', str(path / 'arm_teleop.yaml'))
    return profile_id, config, (path / urdf).resolve()


class LocalDisplay:
    def __init__(self, config, urdf):
        import pybullet as p
        from robot_utils.robot_module import AssembledRobot
        self.p = p
        self.client = p.connect(p.GUI)
        if self.client < 0:
            raise RuntimeError('cannot open local PyBullet GUI')
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=self.client)
            self.robot = AssembledRobot(str(urdf), config, False, False, self.client)
            self.joints = {p.getJointInfo(self.robot.robot_id, i, physicsClientId=self.client)[1].decode(): i
                           for i in range(p.getNumJoints(self.robot.robot_id, physicsClientId=self.client))}
            p.resetDebugVisualizerCamera(1.5, 180, -30, [0, 0, .6], physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.client)
        except Exception:
            self.close()
            raise

    def connected(self):
        return self.p.isConnected(physicsClientId=self.client)

    def apply(self, frame):
        # PyBullet has no resetJointStates API: update the received batch locally.
        # No physics step is taken; the remote simulator owns all dynamics.
        for name, value in frame['positions'].items():
            index = self.joints.get(name)
            if index is not None:
                self.p.resetJointState(self.robot.robot_id, index, value, targetVelocity=0,
                                       physicsClientId=self.client)
        position, orientation = frame['base_pose']
        self.p.resetBasePositionAndOrientation(self.robot.robot_id, position, orientation,
                                              physicsClientId=self.client)

    def close(self):
        if self.connected():
            self.p.disconnect(physicsClientId=self.client)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', required=True, help='remote Dashboard URL, e.g. http://192.168.2.33:7876')
    parser.add_argument('--profile', default=str(PROJECT / 'adapters/robots/x1'))
    args = parser.parse_args(argv)
    display = os.environ.get('DISPLAY', '')
    if not display or display.startswith('localhost:'):
        parser.error('run this viewer on your local desktop with its native DISPLAY, not inside SSH -Y')
    profile_id, config, urdf = load_profile(args.profile)
    state, stop = LatestState(), threading.Event()
    worker = threading.Thread(target=lambda: asyncio.run(receive_remote(args.server.rstrip('/'), state, stop)), daemon=True)
    worker.start()
    gui = None
    applied = -1
    last_error = None
    print(f'Local viewer ready for profile {profile_id}; waiting for remote simulation.', flush=True)
    try:
        while True:
            status, frame, sequence = state.snapshot()
            remote_id = status.get('profile_id') or status.get('active_profile')
            running = status.get('state') == 'running' and status.get('display_mode') == 'local_viewer'
            if running and remote_id != profile_id:
                error = f'Remote profile {remote_id!r} differs from local {profile_id!r}; restart viewer with matching --profile.'
                if error != last_error:
                    print(error, file=sys.stderr, flush=True)
                    last_error = error
                running = False
            if running and gui is None:
                gui = LocalDisplay(config, urdf)
                applied = -1
                print('Remote simulation running; local display opened.', flush=True)
            if not running and gui is not None:
                gui.close()
                gui = None
            if gui is not None:
                if not gui.connected():
                    break  # Closing the local window only exits this viewer.
                if frame is not None and sequence != applied:
                    if frame.get('profile_id') in (None, profile_id):
                        try:
                            gui.apply(frame)
                        except gui.p.error:
                            if not gui.connected():
                                break
                            raise
                    applied = sequence
            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if gui is not None:
            gui.close()
        worker.join(timeout=5)


if __name__ == '__main__':
    main()
