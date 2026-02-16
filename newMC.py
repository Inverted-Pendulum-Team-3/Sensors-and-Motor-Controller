#!/usr/bin/env python3
"""
Sabertooth 2x12 Motor Driver Test — Raspberry Pi 4
GPIO 13 -> S1 (Motor 1)
GPIO 26 -> S2 (Motor 2)

R/C (PWM) mode: 1000 µs = full reverse, 1500 µs = stop, 2000 µs = full forward.
Uses pigpio for hardware-timed PWM. Run: sudo pigpiod  (once) before this script.

Usage:
  sudo python3 sabertooth_2x12_test.py
"""

import time
import sys

# Pin assignments (BCM)
GPIO_M1 = 13   # S1 — Motor 1
GPIO_M2 = 26   # S2 — Motor 2

# Pulse widths (microseconds) for Sabertooth R/C mode
PWM_STOP = 1500
PWM_FULL_FWD = 2000
PWM_FULL_REV = 1000
# Optional: reduced speed for safer testing
PWM_SLOW_FWD = 1650
PWM_SLOW_REV = 1350


def main():
    try:
        import pigpio
    except ImportError:
        print("pigpio not installed. Install with: sudo apt install pigpio python3-pigpio")
        sys.exit(1)

    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpio daemon. Start it with: sudo pigpiod")
        sys.exit(1)

    def stop_all():
        pi.set_servo_pulsewidth(GPIO_M1, PWM_STOP)
        pi.set_servo_pulsewidth(GPIO_M2, PWM_STOP)

    def set_m1(us):
        pi.set_servo_pulsewidth(GPIO_M1, us)

    def set_m2(us):
        pi.set_servo_pulsewidth(GPIO_M2, us)

    print("Sabertooth 2x12 test — GPIO 13 (M1), GPIO 26 (M2)")
    print("PWM: 1000 = full rev, 1500 = stop, 2000 = full fwd")
    print("Ensure Sabertooth is in R/C mode (DIP 1&2 on).")
    print()

    try:
        stop_all()
        time.sleep(0.5)

        # --- Motor 1 ---
        print("Motor 1: slow forward (2 s)...")
        set_m1(PWM_SLOW_FWD)
        time.sleep(2.0)
        print("Motor 1: stop (1 s)...")
        set_m1(PWM_STOP)
        time.sleep(1.0)
        print("Motor 1: slow reverse (2 s)...")
        set_m1(PWM_SLOW_REV)
        time.sleep(2.0)
        set_m1(PWM_STOP)
        time.sleep(1.0)

        # --- Motor 2 ---
        print("Motor 2: slow forward (2 s)...")
        set_m2(PWM_SLOW_FWD)
        time.sleep(2.0)
        print("Motor 2: stop (1 s)...")
        set_m2(PWM_STOP)
        time.sleep(1.0)
        print("Motor 2: slow reverse (2 s)...")
        set_m2(PWM_SLOW_REV)
        time.sleep(2.0)
        set_m2(PWM_STOP)
        time.sleep(1.0)

        # --- Both forward then stop ---
        print("Both motors: slow forward (2 s)...")
        set_m1(PWM_SLOW_FWD)
        set_m2(PWM_SLOW_FWD)
        time.sleep(2.0)
        print("Both: stop.")
        stop_all()
        time.sleep(1.0)

        print("Test done. Motors stopped.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_all()
        # Stop servo pulses (optional; stop_all already at 1500)
        pi.set_servo_pulsewidth(GPIO_M1, 0)
        pi.set_servo_pulsewidth(GPIO_M2, 0)
        pi.stop()


if __name__ == "__main__":
    main()


