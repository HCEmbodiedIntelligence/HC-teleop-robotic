import json
import os
import subprocess
import sys
import time
from pathlib import Path

from humanoid_upload.common import atomic_json
from conftest import due
from test_client import copied, reseal, tick


def test_multipart_kill_restart_and_control_sampler(tmp_path, queue, cfg, server):
    root = copied(tmp_path, cfg)
    (root/"synthetic-payload.bin").write_bytes(b"synthetic" * (700000))
    reseal(root)
    cfg.update(part_size_bytes=5*1024**2, bandwidth_bytes_per_second=2*1024**2)
    job = queue.enqueue(root, cfg)
    config = tmp_path/"config.json"
    atomic_json(config, cfg)
    command = [sys.executable, "-m", "humanoid_upload.cli", "--config", str(config), "step"]
    sampler = '''import json,time,resource
stop=time.perf_counter()+3
next_control=next_capture=time.perf_counter()
control=[];capture=[]
while time.perf_counter()<stop:
 now=time.perf_counter()
 if now>=next_control:
  control.append(max(0,now-next_control));next_control+=.01
 if now>=next_capture:
  capture.append(max(0,now-next_capture));next_capture+=1/30
 time.sleep(max(0,min(next_control,next_capture)-time.perf_counter()))
def facts(a,period):
 a=sorted(a);return {'samples':len(a),'p99_ms':a[int(len(a)*.99)]*1000,'max_ms':max(a)*1000,
                     'missed_deadlines':sum(v>period for v in a)}
usage=resource.getrusage(resource.RUSAGE_SELF)
print(json.dumps({'control_100hz':facts(control,.01),'capture_30hz':facts(capture,1/30),
                  'sampler_cpu_seconds':usage.ru_utime+usage.ru_stime}))
'''
    baseline = json.loads(subprocess.check_output([sys.executable, "-c", sampler], text=True))
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    loaded = json.loads(subprocess.check_output([sys.executable, "-c", sampler], text=True))
    def process_usage(pid):
        root=Path('/proc')/str(pid)
        try:
            stat=(root/'stat').read_text().split(')',1)[1].split()
            io=dict(line.split(':',1) for line in (root/'io').read_text().splitlines())
            return {'cpu_seconds':(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK'),
                    'rss_bytes':int(stat[21])*os.sysconf('SC_PAGE_SIZE'),
                    'io_read_bytes':int(io['read_bytes']),
                    'io_write_bytes':int(io['write_bytes']),
                    'io_rchar':int(io['rchar'])}
        except (FileNotFoundError,PermissionError):
            return {'unavailable':'client exited before process sample'}
    uploader_usage=process_usage(process.pid)
    # Kill a real client during a multipart transfer, with already durable server parts.
    assert process.poll() is None
    process.kill()
    process.communicate(timeout=10)
    assert server.parts
    existing = dict(server.parts)
    time.sleep(max(0, queue.jobs()[0]["next_at"] - time.time()) + .02)
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    assert queue.jobs()[0]["state"] == "raw_committed"
    assert len(server.raws) == 1 and queue.jobs()[0]["id"] == job["id"]
    for key, value in existing.items():
        assert server.parts[key] == value
        assert server.calls[f"put:{key[1]}:{key[2]}"] == 1
    large = next(a for a in job["manifest"]["assets"] if a["path"] == "synthetic-payload.bin")
    assert len([key for key in server.parts if key[1] == large["asset_id"]]) == 2
    for key in ('control_100hz','capture_30hz'):
        assert loaded[key]["samples"] >= baseline[key]["samples"] - 2
        assert loaded[key]["p99_ms"] < baseline[key]["p99_ms"] + 30
    artifact = Path(os.environ.get('OPENARM_E_EVIDENCE',
        Path(__file__).resolve().parents[4]/"artifacts/g0/synthetic-pressure.json"))
    atomic_json(artifact, {"kind": "synthetic scheduler only; not ROS or hardware acceptance",
                          "without_upload": baseline, "with_upload": loaded,
                          "uploader_process":uploader_usage,
                          "bandwidth_bytes_per_second": cfg["bandwidth_bytes_per_second"],
                          "client_killed_and_restarted": True, "raw_count": len(server.raws)})
