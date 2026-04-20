#!/usr/bin/env python3
"""
motorwasd.py — Web-driven manual motor controller.

Started by webserver.py when "Motors ON" is pressed.
Reads motor_command.json every 50 ms.
  - If the command timestamp is < 0.3 s old  → apply it to the motors.
  - If stale or file missing                  → safety-stop (motors = 0).

Command file format (written by webserver.py):
    {"fwd": 0.5, "turn": 0.0, "speed": 0.7, "ts": 1712345678.123}

    fwd   : -1.0 (full reverse) … +1.0 (full forward)
    turn  : -1.0 (full left)    … +1.0 (full right)
    speed :  0.0 … 1.0  (global speed scale / throttle)
    ts    : time.monotonic() timestamp written by the server

Motor mixing:
    left  = clamp((fwd - turn) * speed, -1, 1)
    right = clamp((fwd + turn) * speed, -1, 1)

Stop this process (from webserver) to stop manual control.
"""

import os
import sys
import json
import time
import signal

from hardware_interface_3_28 import set_motor_velocities, close_motor_connection

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SCRIPT_DIR        = os.path.dirname(os.path.abspath(__file__))
CMD_FILE          = os.path.join(SCRIPT_DIR, "motor_command.json")
POLL_INTERVAL_S   = 0.05     # 20 Hz read loop
CMD_STALE_S       = 0.3      # treat command as stale after this many seconds
PRINT_HZ          = 5        # how often to print status to stdout


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------
# Graceful shutdown on SIGTERM / Ctrl-C
# ---------------------------------------------------------------------------
_running = True

def _shutdown(signum, frame):
    global _running
    _running = False

signal.signal(signal.SIGTERM, _shutdown)
signal.signal(signal.SIGINT,  _shutdown)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    print(f"motorwasd.py started — reading {CMD_FILE}")
    print("Motors will stop if command goes stale (> 0.3 s old).\n")

    last_print  = 0.0
    print_every = 1.0 / PRINT_HZ

    while _running:
        loop_start = time.monotonic()

        # --- Read command file ---
        fwd = turn = 0.0
        speed = 0.5
        stale = True

        try:
            with open(CMD_FILE, "r") as f:
                cmd = json.load(f)
            age = time.monotonic() - float(cmd.get("ts", 0))
            if age < CMD_STALE_S:
                fwd   = float(cmd.get("fwd",   0.0))
                turn  = float(cmd.get("turn",  0.0))
                speed = float(cmd.get("speed", 0.5))
                stale = False
        except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError):
            stale = True

        # --- Motor mixing ---
        if stale:
            left_cmd = right_cmd = 0.0
        else:
            raw_left  = fwd - turn
            raw_right = fwd + turn
            # Normalize so diagonal inputs don't clip — preserves direction at any angle
            scale = max(abs(raw_left), abs(raw_right), 1.0)
            left_cmd  = clamp((raw_left  / scale) * speed, -1.0, 1.0)
            right_cmd = clamp((raw_right / scale) * speed, -1.0, 1.0)

        set_motor_velocities(left_cmd, right_cmd)

        # --- Periodic stdout status ---
        now = time.monotonic()
        if now - last_print >= print_every:
            status = "STALE/STOP" if stale else f"fwd={fwd:+.2f} turn={turn:+.2f} spd={speed:.2f}"
            sys.stdout.write(
                f"\r[motorwasd]  {status}   L={left_cmd:+.3f}  R={right_cmd:+.3f}   "
            )
            sys.stdout.flush()
            last_print = now

        # --- Sleep remainder of poll interval ---
        elapsed = time.monotonic() - loop_start
        remaining = POLL_INTERVAL_S - elapsed
        if remaining > 0:
            time.sleep(remaining)

    # --- Clean shutdown ---
    print("\n[motorwasd] Stopping motors and closing connection...")
    try:
        set_motor_velocities(0.0, 0.0)
        close_motor_connection()
    except Exception:
        pass
    print("[motorwasd] Done.")


if __name__ == "__main__":
    main()
