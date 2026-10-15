import argparse
import json
import os
import signal
import sqlite3
import sys
import threading
import time
from pathlib import Path

from .common import UploadError, load_config, read_json
from .queue import Queue, public_job
from .worker import Worker
from .http import Http


def emit(value):
    print(json.dumps(value, ensure_ascii=False), flush=True)


def check_connection(cfg):
    """Authenticate without creating an upload or touching the queue."""
    value = Http({**cfg, 'timeout_seconds': min(cfg['timeout_seconds'], 5)}).api('GET', ':capabilities')
    if not isinstance(value, dict) or value.get('schema_version') != 'robot-ingest-capabilities/v1':
        raise UploadError('CAPABILITIES_INVALID', '地址未返回平台上传接口，请检查平台地址')
    if value.get('robot_id') != cfg['robot_id']:
        raise UploadError('ROBOT_ID_MISMATCH', '上传凭据与所选平台机器人不一致')
    if not value.get('max_recording_assets'):
        raise UploadError('RECORDING_UPLOAD_DISABLED', '平台尚未启用整段录制上传')
    return {'connected': True, 'robot_id': value['robot_id']}


def scan(queue, cfg):
    if not cfg['auto_enqueue']:
        return []
    result = []
    # Skip known immutable roots cheaply. Explicit enqueue revalidates duplicate notices.
    known = {j["root"] for j in queue.jobs()}
    for current, dirs, files in os.walk(cfg["data_directory"], followlinks=False):
        dirs[:] = sorted(d for d in dirs if not d.startswith(".export-") and not (Path(current)/d).is_symlink())
        if "FAILED.json" in files:
            dirs[:] = []
            continue
        if not {'export-complete.json','recording-complete.json'}.intersection(files):
            continue
        dirs[:] = []
        if str(Path(current).resolve()) in known:
            continue
        try:
            result.append(public_job(queue.enqueue(current, cfg)))
        except UploadError as error:
            result.append({"code": error.code, "next_action": error.action})
    return result


def exports(cfg):
    result=[]
    for current,dirs,files in os.walk(cfg['data_directory'],followlinks=False):
        dirs[:]=sorted(d for d in dirs if not d.startswith('.export-') and not (Path(current)/d).is_symlink())
        if 'FAILED.json' in files:dirs[:]=[];continue
        marker=next((m for m in ('recording-complete.json','export-complete.json') if m in files),None)
        if marker is None:continue
        dirs[:]=[]
        try:
            value=read_json(Path(current)/marker)
            result.append({'root':str(Path(current).resolve()),'export_id':value['export_id'],
                'session_id':value['session_id'],'task_id':value['collection_task_id'],
                'format':'整段录制，平台切片' if marker=='recording-complete.json' else 'LeRobot 已切片数据'})
        except (UploadError,KeyError,TypeError):continue
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Independent robot-ingest uploader")
    parser.add_argument("--config", required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check-config")
    commands.add_parser('check-connection')
    commands.add_parser("scan")
    commands.add_parser("status")
    commands.add_parser('exports')
    commands.add_parser("step", help="Run due jobs once (persistent backoff applies)")
    commands.add_parser("run", help="Scan A markers and run until SIGINT/SIGTERM")
    enqueue = commands.add_parser("enqueue")
    enqueue.add_argument("root")
    enqueue.add_argument('--automatic',action='store_true')
    retry = commands.add_parser("retry", help="Stop worker first; retain the UUID; query processing without re-upload")
    retry.add_argument("id")
    retry.add_argument("--resume", action="store_true", help="Explicitly resume a server PAUSED upload")
    retry.add_argument("--processing", action="store_true", help="C API: explicitly retry failed processing with a durable request UUID")
    args = parser.parse_args(argv)
    try:
        cfg = load_config(args.config)
        if args.command == "check-config":
            emit(cfg)
            return 0
        if args.command == 'check-connection':
            emit(check_connection(cfg))
            return 0
        queue = Queue(cfg["queue_directory"])
        if args.command == "status":
            emit([public_job(j) for j in queue.jobs()])
        elif args.command == "enqueue":
            emit({'local_only':True} if args.automatic and not cfg['auto_enqueue'] else public_job(queue.enqueue(args.root, cfg)))
        elif args.command == 'exports':
            emit(exports(cfg))
        elif args.command == "scan":
            emit(scan(queue, cfg))
        elif args.command == "retry":
            if args.processing and not cfg["processing_endpoint"]:
                raise UploadError("PROCESSING_API_DISABLED", "C 服务就绪后启用 processing_endpoint")
            emit(public_job(queue.retry(args.id, args.resume, args.processing)))
        else:
            with queue.worker_lock():
                if args.command == "step":
                    Worker(queue, cfg).tick()
                    emit([public_job(j) for j in queue.jobs()])
                    return 0
                # Lower CPU priority; separate process means callbacks never wait on this worker.
                os.nice(10)
                stopped = threading.Event()
                for sig in (signal.SIGTERM, signal.SIGINT):
                    signal.signal(sig, lambda *_: stopped.set())
                next_scan = 0
                while not stopped.is_set():
                    current = load_config(args.config)
                    if current["queue_directory"] != cfg["queue_directory"]:
                        raise UploadError("QUEUE_CONFIG_CHANGED", "队列目录改变后需重新启动 worker")
                    cfg = current
                    if time.time() >= next_scan:
                        for result in scan(queue, cfg):
                            emit(result)
                        next_scan = time.time() + cfg["scan_seconds"]
                    Worker(queue, cfg).tick()
                    stopped.wait(0.25)
    except UploadError as error:
        emit({"code": error.code, "next_action": error.action})
        return 2
    except (OSError, sqlite3.Error):
        emit({"code": "QUEUE_IO_FAILED", "next_action": "检查队列目录权限/磁盘空间；保留数据库，修复后重启同一命令"})
        return 3
    except (ValueError, KeyError, TypeError):
        emit({"code": "INPUT_INVALID", "next_action": "检查配置和完成包契约"})
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
