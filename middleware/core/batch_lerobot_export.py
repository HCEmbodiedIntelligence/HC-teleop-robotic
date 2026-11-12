"""Run validated historical MCAP conversions sequentially in one low-priority process."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from .export_progress import ExportProgress, progress_path
from .lerobot_export import export_mcap


def main() -> int:
    if len(sys.argv) != 2:
        return 2
    spec_path = Path(sys.argv[1]).resolve()
    try:
        spec = json.loads(spec_path.read_text(encoding="utf-8"))
        items = spec["items"]
        for item in items:
            ExportProgress(progress_path(Path(item["destination"]))).update(
                "排队等待批量转换", 0, state="exporting", force=True, reset=True
            )
        for index, item in enumerate(items, 1):
            destination = Path(item["destination"])
            progress = ExportProgress(progress_path(destination))
            progress.update(
                f"批量转换 {index}/{len(items)}", 0, state="exporting", force=True, reset=True
            )
            try:
                export_mcap(
                    item["source"], destination, int(item["fps"]), item["task"],
                    item["robot_type"], item.get("action_sources", []),
                )
            except Exception as error:
                progress.update(
                    "转换失败", 0, state="error", error=str(error)[-2000:], force=True
                )
        return 0
    finally:
        try:
            spec_path.unlink()
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
