#!/usr/bin/env python3
"""
sensorsCALZero.py — sensorsENC.py with single-point 0° calibration.

Calibration procedure (run once at startup):
1. Hold the robot UPRIGHT at the balance point → press Enter
The average raw pitch over the sample window is set as the offset
so the balance point reads exactly 0° during operation.

All other behaviour (encoder scale fix, obs vector, cache/DB writes)
is identical to sensorsENC.py.

IMU mounting (robot): arrow pointing to X on the IMU is pointed FORWARD.
So: X = forward, Y = lateral (left/right), Z = up.
Pitch (forward/back tilt) = rotation about Y axis → pitch_rate = gyro_y.
Roll (side-to-side tilt)  = rotation about X axis → roll_rate  = gyro_x.
"""

import time
import math
import sys
import os
import struct
import sqlite3
import threading
import traceback
from datetime import datetime

import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

# ---------------------------------------------------------------------------
# Encoder + robot geometry (copied from sensorsNEW.py)
# ---------------------------------------------------------------------------
PPR = 2048
COUNTS_PER_REV = PPR * 4
WHEEL_DIAMETER_IN = 8.0
WHEEL_RADIUS_M = (WHEEL_DIAMETER_IN * 25.4 / 1000.0) / 2.0
TRACK_WIDTH_IN = 13.0   # center of one wheel to center of other (inches)
ROBOT_TRACK_WIDTH_M = TRACK_WIDTH_IN * 0.0254

LEFT_A, LEFT_B, LEFT_X = 18, 17, 27
RIGHT_A, RIGHT_B, RIGHT_X = 14, 4, 15
ENCODER_POLL_RATE = 0.0002
# If left wheel reads negative when driving forward, set to -1 so both wheels are positive forward
LEFT_WHEEL_SIGN = -1
RIGHT_WHEEL_SIGN = 1
# Temporary hardware fix: left encoder reads ~2x too high due to encoder hardware issue.
# Scale it down by 0.5 to roughly match the right encoder until hardware is fixed.
LEFT_WHEEL_SCALE = 0.5
RIGHT_WHEEL_SCALE = 1.0


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")
OBS_CACHE_FILE = os.path.join(_SCRIPT_DIR, "obs_cache.bin")


class Encoder:
    """Quadrature encoder for wheel speed in rad/s and m/s."""

    NOISE_COUNTS = 2

    def __init__(self, pin_a, pin_b, pin_x):
        self.pin_a, self.pin_b, self.pin_x = pin_a, pin_b, pin_x
        self.position = 0
        self.last_position = 0
        self.last_time = time.monotonic()
        self.speed_rad_s = 0.0
        self.direction = "stopped"
        self.last_encoded = 0
        self.connected = True

    def setup(self):
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_x, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        self.last_encoded = (a << 1) | b

    def update(self):
        try:
            a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
            enc = (a << 1) | b
            sum_val = (self.last_encoded << 2) | enc
            if sum_val in (0b1101, 0b0100, 0b0010, 0b1011):
                self.position += 1
            elif sum_val in (0b1110, 0b0111, 0b0001, 0b1000):
                self.position -= 1
            self.last_encoded = enc
            self.connected = True
        except Exception:
            self.connected = False

    def compute_speed(self):
        now = time.monotonic()
        dt = now - self.last_time
        if dt <= 0:
            return
        delta = self.position - self.last_position
        self.last_position = self.position
        self.last_time = now
        if abs(delta) <= self.NOISE_COUNTS:
            self.speed_rad_s = 0.0
            self.direction = "stopped"
            return
        self.speed_rad_s = (delta / dt / COUNTS_PER_REV) * (2.0 * math.pi)
        self.direction = "forward" if delta > 0 else "backward"

    def wheel_linear_speed(self):
        return self.speed_rad_s * WHEEL_RADIUS_M



# ---------------------------------------------------------------------------
# Database (same schema as sensorsNEW for hardware_interface / deploy)
# ---------------------------------------------------------------------------
def init_db(db_path):
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("""
        CREATE TABLE IF NOT EXISTS sensor_readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL NOT NULL,
            datetime TEXT NOT NULL,
            imu1_gx REAL, imu1_gy REAL, imu1_gz REAL,
            imu1_ax REAL, imu1_ay REAL, imu1_az REAL,
            imu1_body_pitch REAL, imu1_yaw_rate REAL, imu1_rot_vel REAL,
            imu1_vx REAL, imu1_vy REAL, imu1_vz REAL,
            imu2_gx REAL, imu2_gy REAL, imu2_gz REAL,
            imu2_ax REAL, imu2_ay REAL, imu2_az REAL,
            imu2_pendulum_angle REAL, imu2_pendulum_angle_deg REAL,
            imu2_pendulum_ang_vel REAL, imu2_rot_vel REAL,
            encoder_left_rad_s REAL, encoder_right_rad_s REAL,
            encoder_left_dir TEXT, encoder_right_dir TEXT,
            robot_v REAL, robot_w_enc REAL,
            ultrasonic_right_cm REAL, ultrasonic_left_cm REAL,
            imu1_connected INTEGER, imu2_connected INTEGER,
            encoder_left_connected INTEGER, encoder_right_connected INTEGER
        )
    """)
    conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON sensor_readings(timestamp)")
    conn.commit()
    return conn


def insert_reading(conn, data):
    conn.execute("""
        INSERT INTO sensor_readings (
            timestamp, datetime,
            imu1_gx, imu1_gy, imu1_gz, imu1_ax, imu1_ay, imu1_az,
            imu1_body_pitch, imu1_yaw_rate, imu1_rot_vel,
            imu1_vx, imu1_vy, imu1_vz,
            imu2_gx, imu2_gy, imu2_gz, imu2_ax, imu2_ay, imu2_az,
            imu2_pendulum_angle, imu2_pendulum_angle_deg, imu2_pendulum_ang_vel, imu2_rot_vel,
            encoder_left_rad_s, encoder_right_rad_s, encoder_left_dir, encoder_right_dir,
            robot_v, robot_w_enc, ultrasonic_right_cm, ultrasonic_left_cm,
            imu1_connected, imu2_connected, encoder_left_connected, encoder_right_connected
        ) VALUES (
            :timestamp, :datetime,
            :imu1_gx, :imu1_gy, :imu1_gz, :imu1_ax, :imu1_ay, :imu1_az,
            :imu1_body_pitch, :imu1_yaw_rate, :imu1_rot_vel,
            :imu1_vx, :imu1_vy, :imu1_vz,
            :imu2_gx, :imu2_gy, :imu2_gz, :imu2_ax, :imu2_ay, :imu2_az,
            :imu2_pendulum_angle, :imu2_pendulum_angle_deg, :imu2_pendulum_ang_vel, :imu2_rot_vel,
            :encoder_left_rad_s, :encoder_right_rad_s, :encoder_left_dir, :encoder_right_dir,
            :robot_v, :robot_w_enc, :ultrasonic_right_cm, :ultrasonic_left_cm,
            :imu1_connected, :imu2_connected, :encoder_left_connected, :encoder_right_connected
        )
    """, data)
    conn.commit()


# ---------------------------------------------------------------------------
# IMU class (from one_imu_sensor_input.py) with pitch calibration
# ---------------------------------------------------------------------------
class BNO085_IMU:
    def __init__(self, sda_pin, scl_pin, address=0x4A, imu_name="IMU"):
        """Initialize the BNO085 IMU with specified I2C pins."""
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.address = address
        self.imu_name = imu_name
        self.bno = None
        self.i2c = None
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        self.reconnect_delay = 2.0  # seconds

        # Calibration offset — set from 0° upright sample so balance point reads 0°
        self.pitch_offset_deg = None

        self._initialize_imu()

    def _initialize_imu(self):
        """Initialize or reinitialize the IMU connection."""
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
            print(f"{self.imu_name} initialized successfully at address 0x{self.address:02X}")
            return True

        except Exception as e:
            self.connection_attempts += 1
            print(f"Error initializing {self.imu_name} (attempt {self.connection_attempts}): {e}")
            self.bno = None
            self.i2c = None
            return False

    def _attempt_reconnection(self):
        """Attempt to reconnect to the IMU."""
        if self.connection_attempts >= self.max_connection_attempts:
            print(f"Max connection attempts reached for {self.imu_name}. Waiting before retry...")
            time.sleep(self.reconnect_delay * 2.0)
            self.connection_attempts = 0

        print(f"Attempting to reconnect {self.imu_name}...")
        time.sleep(self.reconnect_delay)
        return self._initialize_imu()

    @staticmethod
    def quaternion_to_euler(q_i, q_j, q_k, q_real):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        try:
            sinr_cosp = 2 * (q_real * q_i + q_j * q_k)
            cosr_cosp = 1 - 2 * (q_i * q_i + q_j * q_j)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (q_real * q_j - q_k * q_i)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)

            siny_cosp = 2 * (q_real * q_k + q_i * q_j)
            cosy_cosp = 1 - 2 * (q_j * q_j + q_k * q_k)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            return roll_deg, pitch_deg, yaw_deg

        except Exception as e:
            print(f"Error in quaternion conversion: {e}")
            return 0.0, 0.0, 0.0

    def calibrate_pitch(self, duration_s=3.0):
        """
        Single-point 0° calibration.

        Hold the robot upright at the balance point and press Enter.
        The average raw pitch over the sample window is stored as the offset
        so the balance point reads 0° during operation.
        """
        if self.bno is None and not self._attempt_reconnection():
            print(f"{self.imu_name}: cannot calibrate, IMU not connected.")
            return

        print(f"\n{'='*55}")
        print(f"  {self.imu_name}: Zero-point pitch calibration")
        print(f"  Hold the robot UPRIGHT at the balance point.")
        print(f"{'='*55}")
        input(f"\n  >>> Hold the robot UPRIGHT and STILL, then press Enter...")
        print(f"  Sampling for {duration_s:.1f} s — keep it still...")

        t_end = time.monotonic() + duration_s
        samples = []
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
            print(f"\n  raw 0° = {self.pitch_offset_deg:+.3f}°  ({len(samples)} samples)")
            print(f"  → pitch offset = {self.pitch_offset_deg:+.3f}°")
        else:
            print(f"  WARNING: no samples collected! Offset set to 0.")
            self.pitch_offset_deg = 0.0

        print(f"  Calibration complete.\n")

    def get_sensor_data(self):
        """Get calibrated pitch/roll/yaw and gyro rates."""
        if self.bno is None:
            success = self._attempt_reconnection()
            if not success:
                return None

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            roll_raw, pitch_raw, yaw_raw = self.quaternion_to_euler(
                quat_i, quat_j, quat_k, quat_real
            )

            offset = self.pitch_offset_deg or 0.0
            pitch = pitch_raw - offset
            if pitch > 180.0:
                pitch -= 360.0
            elif pitch < -180.0:
                pitch += 360.0

            gyro_x, gyro_y, gyro_z = self.bno.gyro

            return {
                "pitch": pitch,
                "roll": roll_raw,
                "yaw": yaw_raw,
                "gyro_x": gyro_x,
                "gyro_y": gyro_y,
                "gyro_z": gyro_z,
                "connected": True,
            }

        except (OSError, RuntimeError, ValueError) as e:
            print(f"Error reading {self.imu_name}: {e}")
            print(f"Attempting to reconnect {self.imu_name}...")
            self.bno = None
            self.i2c = None
            return {"connected": False, "error": str(e)}

        except Exception as e:
            print(f"Unexpected error with {self.imu_name}: {e}")
            traceback.print_exc()
            return {"connected": False, "error": str(e)}


def main():
    # IMU I2C pins (adjust if needed)
    IMU1_SDA = board.D2  # GPIO 2
    IMU1_SCL = board.D3  # GPIO 3

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    left_enc = Encoder(LEFT_A, LEFT_B, LEFT_X)
    right_enc = Encoder(RIGHT_A, RIGHT_B, RIGHT_X)
    left_enc.setup()
    right_enc.setup()
    successful_reads = 0
    failed_reads = 0
    reading_count = 0

    running = False
    try:
        print("Initializing BNO085 IMU...")
        imu1 = BNO085_IMU(IMU1_SDA, IMU1_SCL, imu_name="IMU #1")
        imu1.calibrate_pitch(duration_s=3.0)

        print("Starting IMU + encoder data reading. Press Ctrl+C to stop.")
        print("Writes to: %s and %s (run deploy.py in another terminal)" % (DB_FILE, OBS_CACHE_FILE))
        print("Main output (obs = 9-element vector for hardware_interface):")
        print("  [0] linear_velocity   (m/s)     robot forward speed from encoders")
        print("  [1] pitch             (rad)     angle of tilt forwards/backwards")
        print("  [2] pitch_rate       (rad/s)   rate of tilt forwards/backwards (gyro Y)")
        print("  [3] yaw_rate          (rad/s)   rotation rate of robot (turning, from encoders)")
        print("  [4] wheel_velocity_1 (rad/s)   left wheel angular speed")
        print("  [5] wheel_velocity_2 (rad/s)   right wheel angular speed")
        print("  [6] velocity_error    (m/s)     target_velocity - linear_velocity (set in get_sensor_data)")
        print("  [7] rotation_error    (rad/s)   target_rotation_rate - yaw_rate (set in get_sensor_data)")
        print("  [8] yaw               (rad)     absolute heading from IMU quaternion")
        print("  pitch_deg            (deg)     same as [1] in degrees")
        print()

        print("Cache: %s" % OBS_CACHE_FILE)
        print()

        # Encoders must be polled at high rate (~5 kHz) or quadrature edges are missed and speed stays 0
        running = True
        def encoder_loop():
            while running:
                left_enc.update()
                right_enc.update()
                time.sleep(ENCODER_POLL_RATE)
        enc_thread = threading.Thread(target=encoder_loop, daemon=True)
        enc_thread.start()

        while True:
            now = time.monotonic()

            # Compute speeds at ~100 Hz; position is updated by encoder_loop
            left_enc.compute_speed()
            right_enc.compute_speed()
            v_l = left_enc.wheel_linear_speed() * LEFT_WHEEL_SIGN * LEFT_WHEEL_SCALE
            v_r = right_enc.wheel_linear_speed() * RIGHT_WHEEL_SIGN * RIGHT_WHEEL_SCALE
            v_robot = 0.5 * (v_r + v_l)
            w_robot_enc = (v_r - v_l) / (ROBOT_TRACK_WIDTH_M + 1e-9)

            reading_count += 1
            data1 = imu1.get_sensor_data()

            if data1 and data1.get("connected", False):
                successful_reads += 1

                linear_velocity = v_robot
                # X forward: pitch = rotation about Y → pitch_rate = gyro_y; roll = about X → roll_rate = gyro_x
                pitch_rate = data1["gyro_y"]               # forward/back tilt rate (rad/s)
                pitch_rad = math.radians(data1["pitch"])   # calibrated pitch (rad)
                roll_rate = data1["gyro_x"]                # side-to-side tilt rate (rad/s)
                # Yaw rate now comes from IMU gyro Z instead of encoders
                yaw_rate = data1["gyro_z"]
                wheel_velocity_1 = left_enc.speed_rad_s * LEFT_WHEEL_SIGN * LEFT_WHEEL_SCALE
                wheel_velocity_2 = right_enc.speed_rad_s * RIGHT_WHEEL_SIGN * RIGHT_WHEEL_SCALE

                # Absolute yaw from IMU quaternion (rad), stored in obs[8]
                yaw_rad = math.radians(data1["yaw"])

                # Write cache for deploy before ultrasonics (minimal latency)
                obs9 = (linear_velocity, pitch_rad, pitch_rate, yaw_rate,
                        wheel_velocity_1, wheel_velocity_2, 0.0, 0.0, yaw_rad)
                try:
                    tmp = OBS_CACHE_FILE + ".tmp"
                    with open(tmp, "wb") as f:
                        f.write(struct.pack("<d", now))
                        f.write(struct.pack("<9f", *obs9))
                    os.replace(tmp, OBS_CACHE_FILE)
                except Exception:
                    pass


                # Order: linear_velocity, pitch, pitch_rate, yaw_rate, wheel_velocity_1, wheel_velocity_2, 0, 0, yaw
                obs = [
                    linear_velocity,
                    pitch_rad,
                    pitch_rate,
                    yaw_rate,
                    wheel_velocity_1,
                    wheel_velocity_2,
                    0.0,      # velocity_error (filled by hardware_interface)
                    0.0,      # rotation_error (filled by hardware_interface)
                    yaw_rad,  # absolute yaw from IMU quaternion (rad)
                ]

                line = (
                    "obs=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f] | pitch_deg=%7.2f | yaw_deg=%7.2f"
                    % (
                        obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8],
                        data1["pitch"], data1["yaw"],
                    )
                )
                pad = " " * 10
                sys.stdout.write("\r" + line + pad + "\r")
                sys.stdout.flush()

            else:
                failed_reads += 1
                msg = "DISCONNECTED" if not data1 else f"Error: {data1.get('error', 'unknown')}"
                line = f"IMU status: {msg} | successful={successful_reads} failed={failed_reads}"
                pad = " " * 10
                sys.stdout.write("\r" + line + pad + "\r")
                sys.stdout.flush()

            time.sleep(0.01)  # ~100 Hz

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("IMU + Encoder Data Collection Stopped")
        print(f"Total Readings: {reading_count}")
        print(f"Successful Reads: {successful_reads}")
        print(f"Failed Reads: {failed_reads}")
        if reading_count > 0:
            print(f"Success Rate: {(successful_reads / reading_count) * 100:.1f}%")
        print("=" * 60)

    except Exception as e:
        print(f"Fatal error occurred: {e}")
        traceback.print_exc()

    finally:
        running = False
        GPIO.cleanup()


if __name__ == "__main__":
    main()

