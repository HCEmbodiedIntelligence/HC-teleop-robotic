"""One reusable, low-priority child for expensive non-realtime work.

Only small job descriptions/results cross the pipe; recordings stay on disk.
Calls are serialized, children use spawn (never fork a running ROS runtime),
and shutdown is bounded even during a blocked HTTP request or native encoder.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import signal
import threading


class BackgroundProcessError(RuntimeError):
    pass


class BackgroundProcessStopped(BackgroundProcessError):
    pass


_stop_requested = None


def stopping():
    return _stop_requested is not None and _stop_requested.is_set()


def background_environment():
    return {**os.environ, **{key: '1' for key in (
        'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
        'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'BLIS_NUM_THREADS')}}


def _signal_process(process, sig):
    if os.name == 'posix':
        try:
            os.killpg(process.pid, sig)
            return
        except ProcessLookupError:
            pass  # A child still starting may not have called setsid yet.
    if process.is_alive():
        if sig == signal.SIGTERM:
            process.terminate()
        else:
            process.kill()


def _exit_with_parent():
    parent = mp.parent_process()
    if parent is not None:
        parent.join()
        # Also reap an external encoder in our private process group.
        if os.name == 'posix' and os.getpgrp() == os.getpid():
            os.killpg(os.getpid(), signal.SIGTERM)
        os._exit(1)


def _serve(connection, stop):
    global _stop_requested
    _stop_requested = stop
    if os.name == 'posix':
        os.setsid()
        os.nice(10)
    os.environ.update(background_environment())
    threading.Thread(target=_exit_with_parent, name='parent-watch', daemon=True).start()
    try:
        while not stop.is_set():
            if not connection.poll(.1):
                continue
            function, args, kwargs = connection.recv()
            try:
                result = {'result': function(*args, **kwargs)}
            except Exception as error:
                from humanoid_upload.common import UploadError
                if isinstance(error, UploadError):
                    result = {'upload_error': {key: getattr(error, key) for key in (
                        'code', 'action', 'retry', 'after', 'conflict')}}
                else:
                    # Never pass a transport body or credential into the UI.
                    result = {'error': type(error).__name__}
            connection.send(result)
    except (EOFError, BrokenPipeError, ConnectionResetError):
        pass
    finally:
        connection.close()


class BackgroundProcess:
    def __init__(self, name):
        self.name = name
        self._context = mp.get_context('spawn')
        self._calls = threading.Lock()
        self._shutdown = threading.Lock()
        self._state = threading.RLock()
        self._process = self._connection = self._stop = None
        self._closed = False
        self._busy = False
        self._broken = False

    @property
    def closed(self):
        return self._closed

    def _start_locked(self):
        if self._closed:
            raise BackgroundProcessStopped(self.name + ' stopped')
        if self._process is not None and self._process.is_alive() and not self._broken:
            return
        if self._process is not None:
            # A broken pipe is not reusable even if the OS has not yet
            # reported child exit. Include any encoder descendants in cleanup.
            _signal_process(self._process, signal.SIGTERM)
            self._process.join(1)
            if self._process.is_alive():
                _signal_process(self._process, signal.SIGKILL)
                self._process.join(1)
            if self._process.is_alive():
                raise BackgroundProcessError(self.name + ' could not be reaped')
            self._process.close()
            self._connection.close()
            self._process = self._connection = self._stop = None
        parent, child = self._context.Pipe()
        stop = self._context.Event()
        process = self._context.Process(target=_serve, args=(child, stop), name=self.name, daemon=True)
        try:
            process.start()
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self._process, self._connection, self._stop = process, parent, stop
        self._broken = False

    def call(self, function, *args, **kwargs):
        with self._calls:
            with self._state:
                self._start_locked()
                process, connection = self._process, self._connection
                self._busy = True
            try:
                connection.send((function, args, kwargs))
                while not connection.poll(.1):
                    if self._closed:
                        raise BackgroundProcessStopped(self.name + ' stopped')
                    if not process.is_alive():
                        raise BackgroundProcessError(self.name + ' exited')
                response = connection.recv()
            except (EOFError, OSError) as error:
                self._broken = True
                failure = BackgroundProcessStopped if self._closed else BackgroundProcessError
                raise failure(self.name + ' disconnected') from error
            finally:
                with self._state:
                    self._busy = False
            if 'upload_error' in response:
                from humanoid_upload.common import UploadError
                raise UploadError(**response['upload_error'])
            if 'error' in response:
                raise BackgroundProcessError(self.name + ': ' + response['error'])
            return response['result']

    def status(self):
        with self._state:
            running = bool(self._process is not None and self._process.is_alive())
            return {'pid': self._process.pid if running else None, 'running': running,
                    'busy': self._busy, 'priority': 'low', 'native_threads': 1}

    def close(self, timeout=3):
        with self._shutdown:
            self._close(timeout)

    def _close(self, timeout):
        with self._state:
            self._closed = True
            process = self._process
            if process is None:
                return
            self._stop.set()
        # Do not wait for the RPC lock: its owner may be in a long upload.
        process.join(timeout)
        if process.is_alive():
            _signal_process(process, signal.SIGTERM)
            process.join(1)
        if process.is_alive():
            _signal_process(process, signal.SIGKILL)
            process.join(1)
        if not process.is_alive() and self._calls.acquire(timeout=1):
            try:
                with self._state:
                    self._connection.close()
                    process.close()
                    self._process = self._connection = self._stop = None
            finally:
                self._calls.release()
