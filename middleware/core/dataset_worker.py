"""Own dataset projection in a separate interpreter, never the control gate."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import threading


class DatasetTopicProcess:
    def __init__(self, config, domain_id, node_name):
        self.config = dict(config)
        self.domain_id = int(domain_id)
        self.node_name = node_name
        self.process = None
        self.reader = None
        self.ready = threading.Event()
        self.error = ''

    def start(self):
        env = os.environ.copy()
        # Preserve the selected domain and DDS transport configuration. The
        # explicit project path also supports launching server.py directly.
        env['ROS_DOMAIN_ID'] = str(self.domain_id)
        root = str(Path(__file__).resolve().parents[2])
        env['PYTHONPATH'] = os.pathsep.join(filter(None, [root, env.get('PYTHONPATH', '')]))
        self.process = subprocess.Popen(
            [sys.executable, '-u', '-m', 'middleware.core.dataset_worker'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
            env=env,
        )
        self.reader = threading.Thread(target=self._read_status, name='dataset-worker-status', daemon=True)
        self.reader.start()
        try:
            self.process.stdin.write(json.dumps(dict(config=self.config, domain_id=self.domain_id,
                                                    node_name=self.node_name), allow_nan=False)+'\n')
            self.process.stdin.flush()
            if not self.ready.wait(5.):
                raise RuntimeError('dataset projection process did not become ready')
            self.check()
        except Exception:
            self.close()
            raise

    def _read_status(self):
        for line in self.process.stdout:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            if message.get('state') == 'error':
                self.error = message.get('error', 'dataset projection failed')
                self.ready.set()
            elif message.get('state') == 'ready':
                self.ready.set()
        if not self.ready.is_set():
            self.error = 'dataset projection process exited before readiness'
            self.ready.set()

    def check(self):
        if self.error:
            raise RuntimeError(self.error)
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('dataset projection process exited')

    def close(self):
        process = self.process
        if process is None:
            return
        # EOF also stops the child if the parent unexpectedly exits. Only this
        # owned child may be terminated; never signal by process name or group.
        try:
            process.stdin.close()
        except (OSError, ValueError):
            pass
        try:
            process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.)
        if self.reader is not None:
            self.reader.join(timeout=1.)
        process.stdout.close()
        self.process = None


def main():
    import rclpy
    from .dataset_topics import DatasetTopics
    request = json.loads(sys.stdin.readline())
    domain = int(request['domain_id'])
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    stopped = threading.Event()
    # Start the lifetime guard before ROS initialization, so startup failures
    # or a disappearing parent never leave a detached projection publisher.
    def parent_lifetime():
        sys.stdin.read()
        stopped.set()
    threading.Thread(target=parent_lifetime, name='dataset-parent-lifetime', daemon=True).start()
    node = None
    try:
        rclpy.init(args=[], domain_id=domain)
        node = rclpy.create_node(request['node_name'], enable_rosout=False, start_parameter_services=False)
        projection = DatasetTopics(node, request['config'])
        print(json.dumps(dict(state='ready', pid=os.getpid())), flush=True)
        while not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as error:
        print(json.dumps(dict(state='error', error=f'{type(error).__name__}: {error}')), flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
