import time
import math
import sys
import os
import struct
import threading
import traceback
from datetime import datetime

import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

# ---------------------------------------------------------------------------
# Encoder + robot geometry
# ---------------------------------------------------------------------------
PPR = 2048
COUNTS_PER_REV = PPR * 4          # full quadrature: 8192 counts per revolution
WHEEL_DIAMETER_IN = 8.0
WHEEL_RADIUS_M    = (WHEEL_DIAMETER_IN * 25.4 / 1000.0) / 2.0
TRACK_WIDTH_IN    = 13.0
ROBOT_TRACK_WIDTH_M = TRACK_WIDTH_IN * 0.0254

# GPIO pin assignments (BCM numbering)
LEFT_A,  LEFT_B,  LEFT_X  = 18, 17, 27
RIGHT_A, RIGHT_B, RIGHT_X = 14,  4, 15

# Sign conventions — set to +1 or -1 so that driving forward gives positive speed
# on both wheels. Adjust after physical verification.
LEFT_WHEEL_SIGN  = -1
RIGHT_WHEEL_SIGN =  1

# --- NO scale-factor hacks ---
# If one encoder reads 2× too high the root cause is almost certainly that both
# edges of one channel are being double-counted (wiring/config issue) or the
# wrong PPR is set for that encoder. Fix it in hardware/config, not in software.

_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
OBS_CACHE_FILE = os.path.join(_SCRIPT_DIR, "obs_cache.bin")


# ---------------------------------------------------------------------------
# Encoder class — hardware-interrupt driven, thread-safe
# ---------------------------------------------------------------------------
class Encoder:
    """
    Quadrature encoder using RPi.GPIO edge-detect interrupts.

    Why interrupts instead of software polling?
    --------------------------------------------
    Software polling in a Python thread is unreliable at high edge rates.
    The GIL and OS scheduler mean your "5 kHz" loop can easily run at 1–2 kHz
    in practice, causing missed edges and wrong (often zero) speed readings.

    RPi.GPIO interrupt callbacks are dispatched from a dedicated C-level thread
    that is not subject to the GIL in the same way, so edges are caught
    much more reliably.

    Speed computation
    -----------------
    - Position is updated atomically in the interrupt callback.
    - compute_speed() snapshots position, computes delta and dt, and derives
      angular speed in rad/s.
    - "Stopped" is declared only when dt > STOPPED_TIMEOUT_S with zero counts,
      preventing false zeroing of slow motion.

    Thread safety
    -------------
    self._position is read/written from both the interrupt thread and the main
    thread. On CPython, int reads/writes are atomic under the GIL, so no lock
    is needed for the simple increment/decrement pattern used here. If you ever
    port to MicroPython or PyPy, add a threading.Lock around position updates.
    """

    STOPPED_TIMEOUT_S = 0.15   # declare 'stopped' only after this many seconds of zero counts

    def __init__(self, pin_a: int, pin_b: int, pin_x: int, name: str = "ENC"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_x = pin_x
        self.name  = name

        self._position    = 0        # raw count — written by interrupt, read by main thread
        self._last_encoded = 0       # last 2-bit A/B state

        self.last_position = 0       # snapshot at last compute_speed() call
        self.last_time     = time.monotonic()

        self.speed_rad_s   = 0.0
        self.direction     = "stopped"
        self.connected     = False   # set True once interrupts are successfully attached

    # ------------------------------------------------------------------
    # Setup / teardown
    # ------------------------------------------------------------------
    def setup(self):
        """
        Configure GPIO pins and attach edge-detect interrupt callbacks.

        Both A and B channels watch BOTH edges (rising + falling) so all four
        quadrature transitions per physical step are captured, giving the full
        COUNTS_PER_REV = PPR × 4 resolution.
        """
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_x, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Seed last_encoded so the first callback computes a valid transition
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self._last_encoded = (a << 1) | b

        # bouncetime=0 is correct for encoders — debounce would eat real edges
        try:
            GPIO.add_event_detect(self.pin_a, GPIO.BOTH,
                                  callback=self._quadrature_callback,
                                  bouncetime=0)
            GPIO.add_event_detect(self.pin_b, GPIO.BOTH,
                                  callback=self._quadrature_callback,
                                  bouncetime=0)
            self.connected = True
            print(f"  {self.name}: interrupt-driven encoder ready "
                  f"(A=GPIO{self.pin_a}, B=GPIO{self.pin_b})")
        except RuntimeError as e:
            # GPIO.add_event_detect raises RuntimeError if the pin already has a
            # handler (e.g. called setup() twice). Clean up and re-raise.
            self.connected = False
            print(f"  {self.name}: ERROR attaching interrupts — {e}")
            raise

    def teardown(self):
        """Remove interrupt handlers.  Call before GPIO.cleanup()."""
        try:
            GPIO.remove_event_detect(self.pin_a)
            GPIO.remove_event_detect(self.pin_b)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Interrupt callback (called from RPi.GPIO C thread — keep it fast)
    # ------------------------------------------------------------------
    def _quadrature_callback(self, channel: int):
        """
        Called on every edge of pin_a or pin_b.

        Reads both channels immediately to get the current state, builds the
        4-bit transition word (last_state << 2 | current_state), and looks it
        up in the standard quadrature truth table to determine direction.

        Truth table (standard Gray-code quadrature):
            CW  transitions: 0b1101, 0b0100, 0b0010, 0b1011  → +1
            CCW transitions: 0b1110, 0b0111, 0b0001, 0b1000  → -1
        """
        a   = GPIO.input(self.pin_a)
        b   = GPIO.input(self.pin_b)
        enc = (a << 1) | b

        transition = (self._last_encoded << 2) | enc
        self._last_encoded = enc

        if transition in (0b1101, 0b0100, 0b0010, 0b1011):
            self._position += 1
        elif transition in (0b1110, 0b0111, 0b0001, 0b1000):
            self._position -= 1
        # All other values are invalid/noise — silently ignore

    # ------------------------------------------------------------------
    # Speed computation (call from main loop at desired control rate)
    # ------------------------------------------------------------------
    def compute_speed(self):
        """
        Compute angular wheel speed in rad/s from accumulated encoder counts.

        Call this at your control-loop rate (e.g. 100–500 Hz).  The method
        snapshots self._position atomically, computes delta counts and elapsed
        time since the last call, and derives speed.

        Stopped detection
        -----------------
        Speed is set to zero only when BOTH conditions are true:
            1. delta counts == 0  (no edges since last call)
            2. dt > STOPPED_TIMEOUT_S  (enough time has passed to be sure)

        This prevents false "stopped" readings when the wheel is moving slowly
        and only 1–2 counts accumulate per call interval — a critical
        correctness requirement for a balancing robot that must detect slow drift.
        """
        now     = time.monotonic()
        dt      = now - self.last_time

        # Guard: don't compute if called too quickly (avoids divide-by-very-small-dt noise)
        if dt < 0.002:   # minimum 2 ms between calls (~500 Hz max)
            return

        # Atomic snapshot — on CPython int assignment is GIL-protected
        current_pos = self._position
        delta       = current_pos - self.last_position

        self.last_position = current_pos
        self.last_time     = now

        if delta == 0:
            if dt > self.STOPPED_TIMEOUT_S:
                # Genuinely no movement for 150 ms — declare stopped
                self.speed_rad_s = 0.0
                self.direction   = "stopped"
            # else: keep the last speed estimate (wheel may just be moving slowly)
            return

        # rad/s = (counts / dt / counts_per_rev) * 2π
        self.speed_rad_s = (delta / dt / COUNTS_PER_REV) * (2.0 * math.pi)
        self.direction   = "forward" if delta > 0 else "backward"

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------
    def wheel_linear_speed(self) -> float:
        """Return wheel surface speed in m/s (positive = forward)."""
        return self.speed_rad_s * WHEEL_RADIUS_M

    @property
    def position(self) -> int:
        """Current raw count (useful for odometry)."""
        return self._position

    def reset_position(self):
        """Zero the position counter (e.g. at the start of a run)."""
        self._position = 0
        self.last_position = 0


# ---------------------------------------------------------------------------
# IMU (unchanged from your existing code — included here for completeness)
# ---------------------------------------------------------------------------
class BNO085_IMU:
    def __init__(self, sda_pin, scl_pin, address=0x4A, imu_name="IMU"):
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.address = address
        self.imu_name = imu_name
        self.bno = None
        self.i2c = None
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        self.reconnect_delay = 2.0
        self.pitch_offset_deg = None
        self._initialize_imu()

    def _initialize_imu(self):
        try:
            if self.i2c:
                try:
                    self.i2c.deinit()
                except Exception:
                    pass
            self.i2c = busio.I2C(self.scl_pin, self.sda_pin)
            self.bno = BNO08X_I2C(self.i2c, address=self.address)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.connection_attempts = 0
            print(f"{self.imu_name} initialized at 0x{self.address:02X}")
            return True
        except Exception as e:
            self.connection_attempts += 1
            print(f"Error initializing {self.imu_name}: {e}")
            self.bno = None
            self.i2c = None
            return False

    def _attempt_reconnection(self):
        if self.connection_attempts >= self.max_connection_attempts:
            time.sleep(self.reconnect_delay * 2.0)
            self.connection_attempts = 0
        print(f"Attempting to reconnect {self.imu_name}...")
        time.sleep(self.reconnect_delay)
        return self._initialize_imu()

    @staticmethod
    def quaternion_to_euler(q_i, q_j, q_k, q_real):
        try:
            sinr_cosp  = 2 * (q_real * q_i + q_j * q_k)
            cosr_cosp  = 1 - 2 * (q_i * q_i + q_j * q_j)
            roll       = math.atan2(sinr_cosp, cosr_cosp)
            sinp       = 2 * (q_real * q_j - q_k * q_i)
            pitch      = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
            siny_cosp  = 2 * (q_real * q_k + q_i * q_j)
            cosy_cosp  = 1 - 2 * (q_j * q_j + q_k * q_k)
            yaw        = math.atan2(siny_cosp, cosy_cosp)
            return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        except Exception:
            return 0.0, 0.0, 0.0

    def calibrate_pitch(self, duration_s=3.0):
        if self.bno is None and not self._attempt_reconnection():
            print(f"{self.imu_name}: cannot calibrate, IMU not connected.")
            return
        print(f"\n{'='*55}")
        print(f"  {self.imu_name}: Zero-point pitch calibration")
        print(f"{'='*55}")
        input("\n  >>> Hold the robot UPRIGHT and STILL, then press Enter...")
        print(f"  Sampling for {duration_s:.1f} s — keep it still...")
        t_end, samples = time.monotonic() + duration_s, []
        while time.monotonic() < t_end:
            try:
                qi, qj, qk, qr = self.bno.quaternion
                _, p, _ = self.quaternion_to_euler(qi, qj, qk, qr)
                samples.append(p)
            except Exception:
                pass
            time.sleep(0.01)
        if samples:
            self.pitch_offset_deg = sum(samples) / len(samples)
            print(f"  raw 0° = {self.pitch_offset_deg:+.3f}°  ({len(samples)} samples)")
        else:
            print("  WARNING: no samples! Offset set to 0.")
            self.pitch_offset_deg = 0.0
        print("  Calibration complete.\n")

    def get_sensor_data(self):
        if self.bno is None:
            if not self._attempt_reconnection():
                return None
        try:
            qi, qj, qk, qr = self.bno.quaternion
            roll_raw, pitch_raw, yaw_raw = self.quaternion_to_euler(qi, qj, qk, qr)
            offset = self.pitch_offset_deg or 0.0
            pitch  = pitch_raw - offset
            if   pitch >  180.0: pitch -= 360.0
            elif pitch < -180.0: pitch += 360.0
            gx, gy, gz = self.bno.gyro
            return {"pitch": pitch, "roll": roll_raw, "yaw": yaw_raw,
                    "gyro_x": gx, "gyro_y": gy, "gyro_z": gz, "connected": True}
        except (OSError, RuntimeError, ValueError) as e:
            print(f"Error reading {self.imu_name}: {e}")
            self.bno = None; self.i2c = None
            return {"connected": False, "error": str(e)}
        except Exception as e:
            traceback.print_exc()
            return {"connected": False, "error": str(e)}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    IMU1_SDA = board.D2
    IMU1_SCL = board.D3

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Create encoders with human-readable names for diagnostics
    left_enc  = Encoder(LEFT_A,  LEFT_B,  LEFT_X,  name="LEFT")
    right_enc = Encoder(RIGHT_A, RIGHT_B, RIGHT_X, name="RIGHT")

    # setup() attaches the interrupt callbacks — no polling thread needed
    left_enc.setup()
    right_enc.setup()

    successful_reads = 0
    failed_reads     = 0
    reading_count    = 0

    try:
        print("\nInitializing BNO085 IMU...")
        imu1 = BNO085_IMU(IMU1_SDA, IMU1_SCL, imu_name="IMU #1")
        imu1.calibrate_pitch(duration_s=3.0)

        print("Starting sensor loop.  Press Ctrl+C to stop.")
        print(f"Cache file: {OBS_CACHE_FILE}\n")

        while True:
            now = time.monotonic()

            # Compute speeds — interrupts have been updating _position continuously
            # in the background; compute_speed() just snapshots and differentiates.
            left_enc.compute_speed()
            right_enc.compute_speed()

            # Apply sign convention (no scale factor — fix hardware if readings diverge)
            v_l = left_enc.wheel_linear_speed()  * LEFT_WHEEL_SIGN
            v_r = right_enc.wheel_linear_speed() * RIGHT_WHEEL_SIGN

            v_robot     = 0.5 * (v_r + v_l)
            w_robot_enc = (v_r - v_l) / (ROBOT_TRACK_WIDTH_M + 1e-9)

            wv1 = left_enc.speed_rad_s  * LEFT_WHEEL_SIGN
            wv2 = right_enc.speed_rad_s * RIGHT_WHEEL_SIGN

            reading_count += 1
            data1 = imu1.get_sensor_data()

            if data1 and data1.get("connected", False):
                successful_reads += 1

                pitch_rate = data1["gyro_y"]
                pitch_rad  = math.radians(data1["pitch"])
                yaw_rate   = data1["gyro_z"]
                yaw_rad    = math.radians(data1["yaw"])

                obs9 = (v_robot, pitch_rad, pitch_rate, yaw_rate,
                        wv1, wv2, 0.0, 0.0, yaw_rad)
                try:
                    tmp = OBS_CACHE_FILE + ".tmp"
                    with open(tmp, "wb") as f:
                        f.write(struct.pack("<d", now))
                        f.write(struct.pack("<9f", *obs9))
                    os.replace(tmp, OBS_CACHE_FILE)
                except Exception:
                    pass

                line = (
                    "obs=[%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, 0.000, 0.000, %6.3f]"
                    " | pitch=%7.2f° | L=%6.3f R=%6.3f rad/s"
                    % (v_robot, pitch_rad, pitch_rate, yaw_rate,
                       wv1, wv2, yaw_rad,
                       data1["pitch"], left_enc.speed_rad_s, right_enc.speed_rad_s)
                )
                sys.stdout.write("\r" + line + "          \r")
                sys.stdout.flush()

            else:
                failed_reads += 1
                msg = "DISCONNECTED" if not data1 else data1.get("error", "unknown")
                sys.stdout.write(f"\rIMU: {msg} | ok={successful_reads} fail={failed_reads}          \r")
                sys.stdout.flush()

            time.sleep(0.01)   # ~100 Hz main loop

    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print("Stopped.")
        print(f"Total: {reading_count}  OK: {successful_reads}  Fail: {failed_reads}")
        if reading_count:
            print(f"Success rate: {successful_reads/reading_count*100:.1f}%")
        print("="*60)

    except Exception as e:
        print(f"\nFatal: {e}")
        traceback.print_exc()

    finally:
        # Always remove interrupt handlers before cleanup to avoid GPIO warnings
        left_enc.teardown()
        right_enc.teardown()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
