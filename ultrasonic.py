#!/usr/bin/env python3
"""
ultrasonic_bg.py — HC-SR04 ultrasonic sensor reader (background process).

Runs independently from sensors_Claude.py so ultrasonic timing (~12–25 ms
per pair of readings) never blocks the IMU/encoder 100 Hz cache writes.

Writes ultrasonic_cache.bin every cycle:
    [0:8]  float64  monotonic timestamp  (seconds)
    [8:12] float32  right distance (cm), -1 = timeout/error
   [12:16] float32  left  distance (cm), -1 = timeout/error

Pin assignments (same as sensorsHI.py):
    Right sensor: TRIG = GPIO 8,  ECHO = GPIO 11
    Left  sensor: TRIG = GPIO 22, ECHO = GPIO 10

Run as a separate terminal process alongside sensors_Claude.py:
    Terminal 1: python3 sensors_Claude.py
    Terminal 2: python3 ultrasonic_bg.py
    Terminal 3: python3 deployPID_Dylan_NoMPL.py
"""

import os
import sys
import struct
import time
import traceback

import RPi.GPIO as GPIO

# ---------------------------------------------------------------------------
# Pin assignments — match sensorsHI.py exactly
# ---------------------------------------------------------------------------
TRIG_RIGHT  = 8
ECHO_RIGHT  = 11
TRIG_LEFT   = 22
ECHO_LEFT   = 10

# ---------------------------------------------------------------------------
# Timing constants
# ---------------------------------------------------------------------------
ECHO_TIMEOUT_S        = 0.025   # 25 ms — max ~4.3 m range at 340 m/s
INTER_SENSOR_DELAY_S  = 0.012   # 12 ms gap between right and left fire
LOOP_RATE_S           = 0.05    # ~20 Hz target (two sensors take ~30 ms per pair)
CM_PER_US             = 1.0 / 58.3

# ---------------------------------------------------------------------------
# Cache file path — same directory as this script
# ---------------------------------------------------------------------------
_SCRIPT_DIR         = os.path.dirname(os.path.abspath(__file__))
ULTRASONIC_CACHE    = os.path.join(_SCRIPT_DIR, "ultrasonic_cache.bin")
US_STATS_FILE       = os.path.join(_SCRIPT_DIR, "us_stats.bin")
# us_stats.bin layout (28 bytes):
#   float64 ts, float32 loop_hz, float32 right_ok_pct,
#   float32 left_ok_pct, uint32 cycle


def _read_cm(trig_pin: int, echo_pin: int) -> float:
    """
    Fire one HC-SR04 pulse and return distance in cm.
    Returns -1.0 on timeout or GPIO error.

    Uses GPIO.wait_for_edge() instead of busy-wait loops so the process
    yields the CPU while waiting for the echo pulse.
    """
    timeout_ms = int(ECHO_TIMEOUT_S * 1000)
    try:
        GPIO.output(trig_pin, True)
        time.sleep(10e-6)
        GPIO.output(trig_pin, False)

        # Wait for echo rising edge (start of pulse)
        if GPIO.wait_for_edge(echo_pin, GPIO.RISING, timeout=timeout_ms) is None:
            return -1.0
        start = time.monotonic()

        # Wait for echo falling edge (end of pulse)
        if GPIO.wait_for_edge(echo_pin, GPIO.FALLING, timeout=timeout_ms) is None:
            return -1.0

        return (time.monotonic() - start) * 1e6 * CM_PER_US

    except Exception:
        return -1.0


def _write_cache(ts: float, right_cm: float, left_cm: float) -> None:
    """Atomic write to ultrasonic_cache.bin via a tmp file."""
    tmp = ULTRASONIC_CACHE + ".tmp"
    try:
        with open(tmp, "wb") as f:
            f.write(struct.pack("<d", ts))
            f.write(struct.pack("<ff", right_cm, left_cm))
        os.replace(tmp, ULTRASONIC_CACHE)
    except Exception:
        pass


def read_ultrasonic_cache() -> tuple:
    """
    Helper — call this from any other script to get the latest readings.
    Returns (timestamp, right_cm, left_cm).
    right_cm / left_cm are -1.0 when no valid reading available.
    Returns (None, -1, -1) if the cache file is missing or corrupt.
    """
    try:
        with open(ULTRASONIC_CACHE, "rb") as f:
            raw = f.read()
        if len(raw) < 16:
            return (None, -1.0, -1.0)
        ts,          = struct.unpack("<d",  raw[0:8])
        right, left  = struct.unpack("<ff", raw[8:16])
        return (ts, right, left)
    except Exception:
        return (None, -1.0, -1.0)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in (TRIG_RIGHT, TRIG_LEFT):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, False)
    for pin in (ECHO_RIGHT, ECHO_LEFT):
        GPIO.setup(pin, GPIO.IN)

    # Short settle after setting up pins
    time.sleep(0.1)

    print("ultrasonic_bg.py — HC-SR04 background reader started.")
    print(f"  Right sensor: TRIG={TRIG_RIGHT}  ECHO={ECHO_RIGHT}")
    print(f"  Left  sensor: TRIG={TRIG_LEFT}   ECHO={ECHO_LEFT}")
    print(f"  Writing: {ULTRASONIC_CACHE}")
    print("  Press Ctrl+C to stop.\n")

    cycle = 0
    right_ok = 0
    left_ok  = 0
    _stats_last = time.monotonic()
    _cycles_window = 0
    _measured_hz = 0.0

    try:
        while True:
            loop_start = time.monotonic()

            right_cm = _read_cm(TRIG_RIGHT, ECHO_RIGHT)
            time.sleep(INTER_SENSOR_DELAY_S)
            left_cm  = _read_cm(TRIG_LEFT, ECHO_LEFT)

            ts = time.monotonic()
            _write_cache(ts, right_cm, left_cm)
            cycle += 1
            _cycles_window += 1
            if right_cm >= 0: right_ok += 1
            if left_cm  >= 0: left_ok  += 1

            # Write us_stats.bin once per second
            _elapsed = ts - _stats_last
            if _elapsed >= 1.0:
                _measured_hz = _cycles_window / _elapsed
                _r_pct = (right_ok / cycle * 100.0) if cycle > 0 else 0.0
                _l_pct = (left_ok  / cycle * 100.0) if cycle > 0 else 0.0
                try:
                    _buf = struct.pack("<dffff I",
                        ts, float(_measured_hz), float(_r_pct), float(_l_pct), 0.0,
                        cycle)
                    _tmp = US_STATS_FILE + ".tmp"
                    with open(_tmp, "wb") as _f:
                        _f.write(_buf)
                    os.replace(_tmp, US_STATS_FILE)
                except Exception:
                    pass
                _stats_last    = ts
                _cycles_window = 0

            r_str = f"{right_cm:6.1f}" if right_cm >= 0 else "  ---"
            l_str = f"{left_cm:6.1f}"  if left_cm  >= 0 else "  ---"
            sys.stdout.write(
                f"\r[{cycle:6d}]  Right={r_str} cm   Left={l_str} cm      "
            )
            sys.stdout.flush()

            # Sleep for the remainder of the target loop period
            elapsed = time.monotonic() - loop_start
            remaining = LOOP_RATE_S - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\n\nStopped after {cycle} cycles.")

    except Exception as e:
        print(f"\nFatal error: {e}")
        traceback.print_exc()

    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
