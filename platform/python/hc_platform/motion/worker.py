"""JSON-lines subprocess entry point; CPU planning never runs in the API loop."""

import argparse
import contextlib
import json
import os
import sys

from .planner import Planner, MotionError


@contextlib.contextmanager
def quiet_native_stdout():
    sys.stdout.flush()
    original = os.dup(1)
    os.dup2(2, 1)
    try:
        with contextlib.redirect_stdout(sys.stderr):
            yield
    finally:
        sys.stderr.flush()
        os.dup2(original, 1)
        os.close(original)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    planner = None
    try:
        with quiet_native_stdout():
            with open(args.config) as stream:
                planner = Planner(json.load(stream))
        for line in sys.stdin:
            if len(line) > 4 * 1024 * 1024:
                raise ValueError("planner request too large")
            request = json.loads(line)
            try:
                op = request["op"]
                with quiet_native_stdout():
                    if op == "describe":
                        result = planner.describe()
                    elif op == "plan":
                        result = planner.plan(
                            request["kind"], request["request"], request["feedback"]
                        )
                    elif op == "servo":
                        result = planner.servo(
                            request["kind"], request["request"], request["feedback"]
                        )
                    elif op == "observe":
                        result = planner.observe(
                            request["request"], request["feedback"]
                        )
                    elif op == "validate_external":
                        result = planner.validate_external(
                            request["request"], request["feedback"]
                        )
                    elif op == "validate_home":
                        result = planner.validate_home(
                            request["request"], request["feedback"]
                        )
                    else:
                        raise MotionError("UNKNOWN_PLANNER_OPERATION")
                response = {"id": request["id"], "ok": True, "result": result}
            except Exception as exc:
                error = (
                    exc.as_dict()
                    if isinstance(exc, MotionError)
                    else {"code": "PLANNER_ERROR", "message": str(exc)}
                )
                response = {"id": request.get("id"), "ok": False, "error": error}
            sys.stdout.write(
                json.dumps(response, allow_nan=False, separators=(",", ":")) + "\n"
            )
            sys.stdout.flush()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if planner:
            planner.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
