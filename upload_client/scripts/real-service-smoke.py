"""D side of real C service smoke. Requires C-issued robot identity/task/credential.

Uses only the installed robot client over HTTP; no platform sources or database.
C must independently attest Raw/episode counts from its persistence.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--export-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=int, default=600)
    args = parser.parse_args()
    base = [sys.executable, "-I", "-m", "humanoid_upload.cli", "--config", args.config]

    def cli(*parts):
        run = subprocess.run(base + list(parts), text=True, capture_output=True, timeout=120)
        if run.returncode:
            raise RuntimeError(run.stdout)  # CLI outputs only safe public diagnostics.
        return json.loads(run.stdout)

    initial = cli("enqueue", args.export_root)
    end = time.monotonic() + args.timeout
    while time.monotonic() < end:
        # Every iteration is a freshly started process using the same durable UUID.
        jobs = cli("step")
        result = next(j for j in jobs if j["id"] == initial["id"])
        if result["terminal"]:
            Path(args.output).write_text(json.dumps({"kind": "real C HTTP client receipt",
                "receipt": result, "raw_uniqueness_attestation": "must be supplied by C"}, indent=2, ensure_ascii=False)+"\n")
            return 0 if result["state"] == "ready" else 2
        time.sleep(1)
    raise RuntimeError("Real service smoke timed out; retain queue and inspect status")


if __name__ == "__main__":
    sys.exit(main())
