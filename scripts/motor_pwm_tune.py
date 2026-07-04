#!/usr/bin/env python3
"""Launch deck robot PWM tune GUI (wrapper for ./run_motor_pwm_tune.sh)."""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script = os.path.join(root, "run_motor_pwm_tune.sh")
    if not os.path.isfile(script):
        print(f"Missing launcher: {script}", file=sys.stderr)
        return 1
    os.environ.setdefault("MOTOR_PWM_TUNE_NO_TERMINAL", "1")
    return subprocess.call(["bash", script])


if __name__ == "__main__":
    raise SystemExit(main())
