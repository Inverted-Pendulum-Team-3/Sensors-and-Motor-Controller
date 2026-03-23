#!/usr/bin/env python3
"""
WASD keyboard control for Sabertooth 2x12 on Raspberry Pi.
GPIO 9  -> S1 (Motor 1),  GPIO 25 -> S2 (Motor 2)
Run on the Pi with:  sudo pigpiod   then   python3 motor_wasd.py

Controls: W=forward, S=backward, A=turn left, D=turn right, Space=stop, Q=quit
"""

import sys
import time
import select

# Optional: use termios/tty for raw keypress (Unix/Pi). On Windows, fall back to input().
try:
    import tty
    import termios
    _has_termios = True
except ImportError:
    _has_termios = False

try:
    import pigpio
except ImportError:
    print("pigpio not installed. Install: sudo apt install pigpio python3-pigpio")
    sys.exit(1)

# Same pins as motortest.py
MOTOR1_PIN = 9
MOTOR2_PIN = 25
PULSE_NEUTRAL = 1500
PULSE_MIN = 1000
PULSE_MAX = 2000
SPEED_PERCENT = 50  # 0–100 for W/S/A/D (adjust for safety)


def speed_to_pulse(speed):
    """speed in -100..100 -> pulse width us."""
    speed = max(-100, min(100, speed))
    if abs(speed) < 2:
        return PULSE_NEUTRAL
    if speed >= 0:
        return int(PULSE_NEUTRAL + (speed / 100.0) * (PULSE_MAX - PULSE_NEUTRAL))
    return int(PULSE_NEUTRAL + (speed / 100.0) * (PULSE_NEUTRAL - PULSE_MIN))


def get_key_nonblock():
    """Read one key if available (Unix raw mode). Returns None if no key."""
    if not _has_termios:
        return None
    if not select.select([sys.stdin], [], [], 0.0)[0]:
        return None
    return sys.stdin.read(1).lower()


def main():
    if not _has_termios:
        print("Run this script on the Raspberry Pi (Linux) for keyboard control.")
        print("On Windows, termios is not available; use the Pi or SSH.")
        sys.exit(1)
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        print("Stdin is not a TTY. Run from a terminal or SSH session.")
        sys.exit(1)
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        run_loop(old, fd)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_loop(old_termios, fd):
    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpio. Run: sudo pigpiod")
        sys.exit(1)

    def set_both(l_speed, r_speed):
        pi.set_servo_pulsewidth(MOTOR1_PIN, speed_to_pulse(l_speed))
        pi.set_servo_pulsewidth(MOTOR2_PIN, speed_to_pulse(r_speed))

    def stop():
        set_both(0, 0)

    print("WASD motor control (GPIO 9, 25)")
    print("W=forward  S=back  A=left  D=right  Space=stop  Q=quit")
    print("Press keys (no Enter). Speed:", SPEED_PERCENT, "%")
    print()

    stop()
    try:
        while True:
            key = get_key_nonblock()
            if key == "q":
                break
            if key == "w":
                set_both(SPEED_PERCENT, SPEED_PERCENT)
                print("\r Forward ", end="", flush=True)
            elif key == "s":
                set_both(-SPEED_PERCENT, -SPEED_PERCENT)
                print("\r Backward", end="", flush=True)
            elif key == "a":
                set_both(-SPEED_PERCENT, SPEED_PERCENT)
                print("\r Left    ", end="", flush=True)
            elif key == "d":
                set_both(SPEED_PERCENT, -SPEED_PERCENT)
                print("\r Right   ", end="", flush=True)
            elif key in (" ", "\x20"):
                stop()
                print("\r Stop    ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        pi.set_servo_pulsewidth(MOTOR1_PIN, 0)
        pi.set_servo_pulsewidth(MOTOR2_PIN, 0)
        pi.stop()
        if old_termios is not None and fd is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
        print("\nStopped. Goodbye.")


if __name__ == "__main__":
    main()
