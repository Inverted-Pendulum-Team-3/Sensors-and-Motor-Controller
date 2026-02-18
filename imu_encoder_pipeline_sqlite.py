#!/usr/bin/env python3
"""
Improved Dual BNO085 IMU + AMT-103 Encoder Pipeline with SQLite Database
For Inverted Pendulum Robot (Team 3)

Key improvements:
- SQLite database for persistent, queryable sensor history
- Uses gravity for tilt/pendulum angle; linear_acceleration (gravity removed) for IMU1 velocity integration
- Clear complementary filters for body pitch and pendulum angle
- Fixed-rate main loop using POLLING_RATE
- Slightly stronger gyro deadband + larger moving-average window
- Cleaner encoder speed calculation and robot kinematics
"""

import time
import math
import board
import busio
import sys
import os
import io
import contextlib
import threading
import warnings
import sqlite3
from collections import deque
from datetime import datetime

import RPi.GPIO as GPIO
from adafruit_bno08x import (
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GRAVITY,
)
try:
    from adafruit_bno08x import BNO_REPORT_LINEAR_ACCELERATION
except ImportError:
    BNO_REPORT_LINEAR_ACCELERATION = None  # older library; velocity integration will stay 0
from adafruit_bno08x.i2c import BNO08X_I2C

# ----------------------------- CONFIG ---------------------------------

warnings.filterwarnings("ignore")

# IMU addresses
IMU1_ADDRESS = 0x4A  # chassis IMU
IMU2_ADDRESS = 0x4B  # pendulum IMU
I2C_FREQUENCY = 400_000

# Gyro filtering
GYRO_THRESHOLD = 0.02          # rad/s deadband (slightly stronger)
FILTER_WINDOW_SIZE = 10        # moving average window for gyro & accel

# Complementary filter
BODY_PITCH_ALPHA = 0.98        # gyro weight for body pitch angle
PENDULUM_ALPHA_MOVING = 0.98   # gyro weight when pendulum moving
PENDULUM_ALPHA_STABLE = 0.90   # gyro weight when pendulum accel small
ACCEL_STABILITY_THRESHOLD = 0.1  # m/s^2

# Robot geometry
PENDULUM_LENGTH_M = 0.65       # pendulum length [m] (~65 cm)
CHASSIS_HEIGHT_M = 0.05        # chassis height off ground [m] (5 cm)
WHEEL_DIAMETER_IN = 8.0        # wheel diameter [inches]
WHEEL_DIAMETER_MM = WHEEL_DIAMETER_IN * 25.4   # 203.2 mm for 8 in
WHEEL_RADIUS_M = (WHEEL_DIAMETER_MM / 1000.0) / 2.0

# Encoder config
PPR = 2048
COUNTS_PER_REV = PPR * 4       # 8192 counts/rev
ROBOT_TRACK_WIDTH_M = 0.25     # distance between wheel centers (tune!)

# GPIO pins (BCM) — rotary encoders
# Left encoder:  A=18 (CLK), B=17, X=27
# Right encoder: A=14 (TXD), B=4, X=15 (RXD)
LEFT_A = 18
LEFT_B = 17
LEFT_X = 27
RIGHT_A = 14
RIGHT_B = 4
RIGHT_X = 15

# Timing
POLLING_RATE = 0.03    # main loop period [s] ~33 Hz
ENCODER_POLL_RATE = 0.0002  # encoder polling thread period [s]

# Velocity drift compensation
VELOCITY_RESET_THRESHOLD = 0.1  # rad/s from encoders

# Database config (same folder as this script if you run from project dir)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")
BATCH_INSERT_SIZE = 50  # insert in batches for efficiency
TEXT_OUTPUT_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.txt")  # optional real-time text output


# -------------------------- UTILITIES ---------------------------------


@contextlib.contextmanager
def suppress_all_output():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr


class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.data = deque(maxlen=window_size)

    def update(self, value):
        self.data.append(value)
        return sum(self.data) / len(self.data) if self.data else 0.0

    def reset(self):
        self.data.clear()


# ------------------------- SQLite DATABASE ---------------------------


class SensorDatabase:
    """
    Manages SQLite database for sensor readings.
    Provides efficient batch inserts and query methods.
    """

    def __init__(self, db_path):
        self.db_path = db_path
        self.conn = None
        self.pending_inserts = []
        self._init_database()

    def _init_database(self):
        """Create database and table if they don't exist."""
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.execute("PRAGMA journal_mode=WAL")  # Write-Ahead Logging for better concurrency

        # Create sensor_readings table
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                datetime TEXT NOT NULL,

                -- IMU1 (Chassis) data
                imu1_gx REAL,
                imu1_gy REAL,
                imu1_gz REAL,
                imu1_ax REAL,
                imu1_ay REAL,
                imu1_az REAL,
                imu1_body_pitch REAL,
                imu1_yaw_rate REAL,
                imu1_rot_vel REAL,
                imu1_vx REAL,
                imu1_vy REAL,
                imu1_vz REAL,

                -- IMU2 (Pendulum) data
                imu2_gx REAL,
                imu2_gy REAL,
                imu2_gz REAL,
                imu2_ax REAL,
                imu2_ay REAL,
                imu2_az REAL,
                imu2_pendulum_angle REAL,
                imu2_pendulum_angle_deg REAL,
                imu2_pendulum_ang_vel REAL,
                imu2_rot_vel REAL,

                -- Encoder data
                encoder_left_rad_s REAL,
                encoder_right_rad_s REAL,
                encoder_left_dir TEXT,
                encoder_right_dir TEXT,

                -- Robot kinematics
                robot_v REAL,
                robot_w_enc REAL,

                -- Metadata
                imu1_connected INTEGER,
                imu2_connected INTEGER,
                encoder_left_connected INTEGER,
                encoder_right_connected INTEGER
            )
        """)

        # Create index on timestamp for fast time-based queries
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_timestamp ON sensor_readings(timestamp)
        """)

        # Create index on datetime for human-readable queries
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_datetime ON sensor_readings(datetime)
        """)

        self.conn.commit()
        print(f"[Database] Initialized: {self.db_path}")

    def add_reading(self, data_dict):
        """
        Queue a sensor reading for batch insert.
        data_dict should contain all sensor fields.
        """
        self.pending_inserts.append(data_dict)

        # Flush if batch size reached
        if len(self.pending_inserts) >= BATCH_INSERT_SIZE:
            self.flush()

    def flush(self):
        """Insert all pending readings into database."""
        if not self.pending_inserts:
            return

        try:
            cursor = self.conn.cursor()
            cursor.executemany("""
                INSERT INTO sensor_readings (
                    timestamp, datetime,
                    imu1_gx, imu1_gy, imu1_gz, imu1_ax, imu1_ay, imu1_az,
                    imu1_body_pitch, imu1_yaw_rate, imu1_rot_vel,
                    imu1_vx, imu1_vy, imu1_vz,
                    imu2_gx, imu2_gy, imu2_gz, imu2_ax, imu2_ay, imu2_az,
                    imu2_pendulum_angle, imu2_pendulum_angle_deg, imu2_pendulum_ang_vel,
                    imu2_rot_vel,
                    encoder_left_rad_s, encoder_right_rad_s,
                    encoder_left_dir, encoder_right_dir,
                    robot_v, robot_w_enc,
                    imu1_connected, imu2_connected,
                    encoder_left_connected, encoder_right_connected
                ) VALUES (
                    :timestamp, :datetime,
                    :imu1_gx, :imu1_gy, :imu1_gz, :imu1_ax, :imu1_ay, :imu1_az,
                    :imu1_body_pitch, :imu1_yaw_rate, :imu1_rot_vel,
                    :imu1_vx, :imu1_vy, :imu1_vz,
                    :imu2_gx, :imu2_gy, :imu2_gz, :imu2_ax, :imu2_ay, :imu2_az,
                    :imu2_pendulum_angle, :imu2_pendulum_angle_deg, :imu2_pendulum_ang_vel,
                    :imu2_rot_vel,
                    :encoder_left_rad_s, :encoder_right_rad_s,
                    :encoder_left_dir, :encoder_right_dir,
                    :robot_v, :robot_w_enc,
                    :imu1_connected, :imu2_connected,
                    :encoder_left_connected, :encoder_right_connected
                )
            """, self.pending_inserts)

            self.conn.commit()
            count = len(self.pending_inserts)
            self.pending_inserts.clear()
            return count
        except Exception as e:
            print(f"[Database] Error flushing: {e}")
            self.pending_inserts.clear()
            return 0

    def get_recent_readings(self, seconds=10, limit=1000):
        """
        Query recent readings within last N seconds.
        Returns list of dictionaries.
        """
        try:
            cutoff_time = time.time() - seconds
            cursor = self.conn.cursor()
            cursor.execute("""
                SELECT * FROM sensor_readings
                WHERE timestamp >= ?
                ORDER BY timestamp DESC
                LIMIT ?
            """, (cutoff_time, limit))

            columns = [desc[0] for desc in cursor.description]
            return [dict(zip(columns, row)) for row in cursor.fetchall()]
        except Exception as e:
            print(f"[Database] Error querying: {e}")
            return []

    def get_count(self):
        """Get total number of readings in database."""
        try:
            cursor = self.conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM sensor_readings")
            return cursor.fetchone()[0]
        except Exception as e:
            print(f"[Database] Error counting: {e}")
            return 0

    def close(self):
        """Flush pending inserts and close database connection."""
        self.flush()
        if self.conn:
            self.conn.close()


# --------------------------- IMU DATA ---------------------------------


class IMUState:
    def __init__(self):
        # raw filtered signals
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.ax = 0.0  # includes gravity or gravity-only, depending on config
        self.ay = 0.0
        self.az = 0.0

        # bias offsets
        self.gx_off = 0.0
        self.gy_off = 0.0
        self.gz_off = 0.0
        self.ax_off = 0.0
        self.ay_off = 0.0
        self.az_off = 0.0

        # filters
        self.gx_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.gy_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.gz_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.ax_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.ay_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.az_f = MovingAverageFilter(FILTER_WINDOW_SIZE)

        # derived states
        self.body_pitch = 0.0          # radians (for IMU1)
        self.pendulum_angle = 0.0      # radians (for IMU2)
        self.last_angle_update = time.monotonic()

        # linear acceleration (gravity removed), for velocity integration (IMU1 only)
        self.lax = 0.0
        self.lay = 0.0
        self.laz = 0.0
        # linear velocity integration (for IMU1)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_vel_update = time.monotonic()

        self.connected = False
        self.last_update = 0.0


class IMUManager:
    """
    Handles one BNO085: gyro + accel/gravity, calibration, filtering, and
    angle estimation (pitch / pendulum).
    """

    def __init__(self, i2c, address, name):
        self.i2c = i2c
        self.address = address
        self.name = name
        self.device = None
        self.state = IMUState()

    def initialize(self):
        try:
            with suppress_all_output():
                self.device = BNO08X_I2C(self.i2c, address=self.address)
                self.device.enable_feature(BNO_REPORT_GYROSCOPE)
                self.device.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.device.enable_feature(BNO_REPORT_GRAVITY)
                if BNO_REPORT_LINEAR_ACCELERATION is not None:
                    self.device.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            self.state.connected = True
            return True
        except Exception:
            self.device = None
            self.state.connected = False
            return False

    def _read_raw(self):
        """
        Returns dict with gyro (rad/s) and accel+gravity (m/s^2), or None.
        Uses .gyro and .gravity if available; falls back to .acceleration.
        """
        if not self.device:
            return None

        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                gyro = self.device.gyro
                # Prefer gravity vector; if not available, use acceleration
                gravity = getattr(self.device, "gravity", None)
                accel = getattr(self.device, "acceleration", None)

            if gyro and len(gyro) == 3 and (gravity or accel):
                if gravity and len(gravity) == 3:
                    ax, ay, az = gravity
                else:
                    ax, ay, az = accel  # includes gravity

                return {
                    "gx": gyro[0],
                    "gy": gyro[1],
                    "gz": gyro[2],
                    "ax": ax,
                    "ay": ay,
                    "az": az,
                }
        except Exception:
            return None

        return None

    def calibrate(self, duration_s=3.0, samples=150):
        print(f"\n[{self.name}] Calibrating... Keep robot STILL and level.")
        sample_delay = duration_s / samples

        gx_s, gy_s, gz_s = [], [], []
        ax_s, ay_s, az_s = [], [], []

        collected = 0
        for i in range(samples):
            data = self._read_raw()
            if data:
                gx_s.append(data["gx"])
                gy_s.append(data["gy"])
                gz_s.append(data["gz"])
                ax_s.append(data["ax"])
                ay_s.append(data["ay"])
                az_s.append(data["az"])
                collected += 1

            if (i + 1) % 30 == 0:
                print(f"[{self.name}] Progress: {i + 1}/{samples}")
            time.sleep(sample_delay)

        if collected > 0:
            s = self.state
            s.gx_off = sum(gx_s) / len(gx_s)
            s.gy_off = sum(gy_s) / len(gy_s)
            s.gz_off = sum(gz_s) / len(gz_s)
            s.ax_off = sum(ax_s) / len(ax_s)
            s.ay_off = sum(ay_s) / len(ay_s)
            s.az_off = sum(az_s) / len(az_s)

            print(
                f"[{self.name}] Gyro offsets: "
                f"X={s.gx_off:.4f}, Y={s.gy_off:.4f}, Z={s.gz_off:.4f}"
            )
            print(
                f"[{self.name}] Accel offsets: "
                f"X={s.ax_off:.4f}, Y={s.ay_off:.4f}, Z={s.az_off:.4f}"
            )
        else:
            print(f"[{self.name}] WARNING: Calibration failed, using zero offsets.")

    def update_filtered(self):
        """Read, apply offsets, filter, and apply gyro deadband."""
        data = self._read_raw()
        if not data:
            self.state.connected = False
            return False

        s = self.state

        # Subtract calibration offsets
        gx = data["gx"] - s.gx_off
        gy = data["gy"] - s.gy_off
        gz = data["gz"] - s.gz_off
        ax = data["ax"] - s.ax_off
        ay = data["ay"] - s.ay_off
        az = data["az"] - s.az_off

        # Filter
        gx_f = s.gx_f.update(gx)
        gy_f = s.gy_f.update(gy)
        gz_f = s.gz_f.update(gz)
        ax_f = s.ax_f.update(ax)
        ay_f = s.ay_f.update(ay)
        az_f = s.az_f.update(az)

        # Deadband on gyro
        s.gx = gx_f if abs(gx_f) > GYRO_THRESHOLD else 0.0
        s.gy = gy_f if abs(gy_f) > GYRO_THRESHOLD else 0.0
        s.gz = gz_f if abs(gz_f) > GYRO_THRESHOLD else 0.0

        s.ax = ax_f
        s.ay = ay_f
        s.az = az_f

        # Linear acceleration (gravity removed) for velocity integration; same frame as gyro/gravity
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                lin = self.device.linear_acceleration
            if lin and len(lin) == 3:
                s.lax, s.lay, s.laz = lin
        except Exception:
            s.lax = s.lay = s.laz = 0.0

        s.connected = True
        s.last_update = time.monotonic()
        return True

    def update_body_pitch(self):
        """Complementary filter for chassis pitch angle (IMU1)."""
        s = self.state
        now = time.monotonic()
        dt = now - s.last_angle_update
        if dt <= 0.0 or dt > 0.1:
            s.last_angle_update = now
            return

        # Integrate gyro y for short-term angle
        gyro_angle = s.body_pitch + s.gy * dt

        # Long-term reference from gravity: pitch from ax, az
        # Assuming +X points forward, +Z upward (adjust if mount differs)
        accel_angle = math.atan2(s.ax, s.az + 1e-9)

        alpha = BODY_PITCH_ALPHA
        s.body_pitch = alpha * gyro_angle + (1.0 - alpha) * accel_angle
        s.last_angle_update = now

    def update_pendulum_angle(self):
        """Complementary filter for pendulum angle (IMU2)."""
        s = self.state
        now = time.monotonic()
        dt = now - s.last_angle_update
        if dt <= 0.0 or dt > 0.1:
            s.last_angle_update = now
            return

        gyro_angle = s.pendulum_angle + s.gy * dt

        # Use gravity vector: assume X tangent to arc, Z toward base
        accel_angle = math.atan2(s.ax, s.az + 1e-9)

        stable = (abs(s.ax) < ACCEL_STABILITY_THRESHOLD and
                  abs(s.ay) < ACCEL_STABILITY_THRESHOLD)
        alpha = PENDULUM_ALPHA_STABLE if stable else PENDULUM_ALPHA_MOVING

        s.pendulum_angle = alpha * gyro_angle + (1.0 - alpha) * accel_angle
        s.last_angle_update = now

    def integrate_linear_velocity(self, reset_if_stopped=False,
                                  left_speed=0.0, right_speed=0.0):
        """
        Integrate linear acceleration (gravity removed) to get linear velocity (IMU1 only).
        Uses BNO085 linear_acceleration report so integration is physically correct.
        Optionally reset when encoders show near-zero speed.
        """
        s = self.state
        now = time.monotonic()
        dt = now - s.last_vel_update
        if dt <= 0.0 or dt > 0.1:
            s.last_vel_update = now
            return

        # Integrate linear acceleration (gravity already removed by sensor)
        s.vx += s.lax * dt
        s.vy += s.lay * dt
        s.vz += s.laz * dt

        if reset_if_stopped:
            if (abs(left_speed) < VELOCITY_RESET_THRESHOLD and
                    abs(right_speed) < VELOCITY_RESET_THRESHOLD):
                s.vx = 0.0
                s.vy = 0.0
                s.vz = 0.0

        s.last_vel_update = now


# ------------------------- ENCODER HANDLING ---------------------------


class Encoder:
    def __init__(self, name, pin_a, pin_b, pin_x):
        self.name = name
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_x = pin_x

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

        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self.last_encoded = (a << 1) | b

    def update(self):
        try:
            a = GPIO.input(self.pin_a)
            b = GPIO.input(self.pin_b)
            encoded = (a << 1) | b
            sum_val = (self.last_encoded << 2) | encoded

            # Gray-code decoding
            if sum_val in (0b1101, 0b0100, 0b0010, 0b1011):
                self.position += 1
            elif sum_val in (0b1110, 0b0111, 0b0001, 0b1000):
                self.position -= 1

            self.last_encoded = encoded
            self.connected = True
        except Exception:
            self.connected = False

    def compute_speed(self):
        now = time.monotonic()
        dt = now - self.last_time
        if dt <= 0.0:
            return

        delta_counts = self.position - self.last_position
        self.last_position = self.position
        self.last_time = now

        # Simple noise gate: ignore tiny count changes
        NOISE_COUNTS = 2
        if abs(delta_counts) <= NOISE_COUNTS:
            self.speed_rad_s = 0.0
            self.direction = "stopped"
            return

        counts_per_sec = delta_counts / dt
        self.speed_rad_s = (counts_per_sec / COUNTS_PER_REV) * (2.0 * math.pi)

        if delta_counts > 0:
            self.direction = "forward"
        else:
            self.direction = "backward"

    def wheel_linear_speed(self):
        """Return wheel tangential speed [m/s]."""
        return self.speed_rad_s * WHEEL_RADIUS_M


# ---------------------- COMBINED SENSOR READER -----------------------


class CombinedSensorPipeline:
    def __init__(self):
        # I2C
        self.i2c = None

        # IMUs
        self.imu1 = None
        self.imu2 = None

        # Encoders
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.left_encoder = Encoder("Left", LEFT_A, LEFT_B, LEFT_X)
        self.right_encoder = Encoder("Right", RIGHT_A, RIGHT_B, RIGHT_X)

        # Threads
        self.encoder_thread = None
        self.running = False

        # Database
        self.db = SensorDatabase(DB_FILE)

        # Optional text output for real-time consumption
        self.text_file = None
        try:
            self.text_file = open(TEXT_OUTPUT_FILE, "w")
        except Exception as e:
            print(f"[Warning] Could not open text output file: {e}")

        self._init_i2c()

    def _init_i2c(self):
        try:
            with suppress_all_output():
                self.i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQUENCY)
        except Exception as e:
            print(f"Failed to init I2C: {e}")
            sys.exit(1)

    def start(self):
        print("\n" + "=" * 60)
        print("IMU + ENCODER CALIBRATION AND INITIALIZATION")
        print("=" * 60)

        # Initialize IMUs
        self.imu1 = IMUManager(self.i2c, IMU1_ADDRESS, "IMU1 (Chassis)")
        self.imu2 = IMUManager(self.i2c, IMU2_ADDRESS, "IMU2 (Pendulum)")

        if not self.imu1.initialize():
            print("[IMU1] Failed to initialize!")
        else:
            self.imu1.calibrate()

        if not self.imu2.initialize():
            print("[IMU2] Failed to initialize!")
        else:
            self.imu2.calibrate()
            # Define initial pendulum angle as zero
            self.imu2.state.pendulum_angle = 0.0

        print("\nInitializing encoders...")
        self.left_encoder.setup()
        self.right_encoder.setup()
        print("Encoders initialized!")
        print("=" * 60)
        print(f"[Database] Total readings: {self.db.get_count()}")
        print("\nStarting data collection...\n")

        # Start encoder polling thread
        self.running = True
        self.encoder_thread = threading.Thread(
            target=self._encoder_poll_loop, daemon=True
        )
        self.encoder_thread.start()

        self._main_loop()

    def _encoder_poll_loop(self):
        while self.running:
            self.left_encoder.update()
            self.right_encoder.update()
            time.sleep(ENCODER_POLL_RATE)

    def _robot_kinematics(self, v_l, v_r):
        """
        From left/right wheel linear speeds [m/s], compute:
        - v (robot forward speed) [m/s]
        - w (yaw rate) [rad/s]
        """
        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / (ROBOT_TRACK_WIDTH_M + 1e-9)
        return v, w

    def _write_text_output(self, line):
        """Write to text file for real-time consumption (optional)."""
        if self.text_file:
            try:
                self.text_file.seek(0)
                self.text_file.write(line + "\n")
                self.text_file.truncate()
                self.text_file.flush()
            except Exception:
                pass

    def _main_loop(self):
        next_poll = time.monotonic()
        last_db_flush = time.monotonic()

        try:
            while True:
                now = time.monotonic()

                # IMU updates
                if self.imu1 and self.imu1.device:
                    self.imu1.update_filtered()
                    # body pitch angle / rad
                    self.imu1.update_body_pitch()

                if self.imu2 and self.imu2.device:
                    self.imu2.update_filtered()
                    self.imu2.update_pendulum_angle()

                # Encoder speeds (rad/s + m/s)
                self.left_encoder.compute_speed()
                self.right_encoder.compute_speed()
                v_l = self.left_encoder.wheel_linear_speed()
                v_r = self.right_encoder.wheel_linear_speed()
                v_robot, w_robot_enc = self._robot_kinematics(v_l, v_r)

                # Integrate IMU1 linear velocity with encoder-based reset
                if self.imu1 and self.imu1.device:
                    self.imu1.integrate_linear_velocity(
                        reset_if_stopped=True,
                        left_speed=self.left_encoder.speed_rad_s,
                        right_speed=self.right_encoder.speed_rad_s,
                    )

                # Build data dictionary for database
                s1 = self.imu1.state if self.imu1 else IMUState()
                s2 = self.imu2.state if self.imu2 else IMUState()

                imu1_rot_vel = math.sqrt(s1.gx**2 + s1.gy**2 + s1.gz**2)
                imu2_rot_vel = math.sqrt(s2.gx**2 + s2.gy**2 + s2.gz**2)

                pend_angle = s2.pendulum_angle
                pend_angle_deg = math.degrees(pend_angle)

                timestamp = now
                datetime_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

                data_dict = {
                    "timestamp": timestamp,
                    "datetime": datetime_str,
                    "imu1_gx": s1.gx,
                    "imu1_gy": s1.gy,
                    "imu1_gz": s1.gz,
                    "imu1_ax": s1.ax,
                    "imu1_ay": s1.ay,
                    "imu1_az": s1.az,
                    "imu1_body_pitch": s1.body_pitch,
                    "imu1_yaw_rate": s1.gz,
                    "imu1_rot_vel": imu1_rot_vel,
                    "imu1_vx": s1.vx,
                    "imu1_vy": s1.vy,
                    "imu1_vz": s1.vz,
                    "imu2_gx": s2.gx,
                    "imu2_gy": s2.gy,
                    "imu2_gz": s2.gz,
                    "imu2_ax": s2.ax,
                    "imu2_ay": s2.ay,
                    "imu2_az": s2.az,
                    "imu2_pendulum_angle": pend_angle,
                    "imu2_pendulum_angle_deg": pend_angle_deg,
                    "imu2_pendulum_ang_vel": s2.gy,
                    "imu2_rot_vel": imu2_rot_vel,
                    "encoder_left_rad_s": self.left_encoder.speed_rad_s,
                    "encoder_right_rad_s": self.right_encoder.speed_rad_s,
                    "encoder_left_dir": self.left_encoder.direction,
                    "encoder_right_dir": self.right_encoder.direction,
                    "robot_v": v_robot,
                    "robot_w_enc": w_robot_enc,
                    "imu1_connected": 1 if s1.connected else 0,
                    "imu2_connected": 1 if s2.connected else 0,
                    "encoder_left_connected": 1 if self.left_encoder.connected else 0,
                    "encoder_right_connected": 1 if self.right_encoder.connected else 0,
                }

                # Add to database (batched)
                self.db.add_reading(data_dict)

                # Build human-readable line for console/text output (format per user spec)
                ts = datetime.now().strftime("%H:%M:%S")
                imu1_linear_vel = math.sqrt(s1.vx**2 + s1.vy**2)
                line = (
                    f"{ts}, "
                    f"IMU1, Forward/backwards Tilt in rad/s, {s1.gy:.4f}, "
                    f"Side-to-Side tilt in rad/s, {s1.gx:.4f}, "
                    f"Yaw in rad/s, {s1.gz:.4f}, "
                    f"Pitch Rate in rad/s, {s1.gy:.4f}, "
                    f"Roll Rate in rad/s, {s1.gx:.4f}, "
                    f"Rotational Velocity in rad/s, {imu1_rot_vel:.4f}, "
                    f"IMU2, Tilt in rad/s, {s2.gy:.4f}, "
                    f"Side-to-Side tilt in rad/s, {s2.gx:.4f}, "
                    f"Yaw in rad/s, {s2.gz:.4f}, "
                    f"Pitch Rate in rad/s, {s2.gy:.4f}, "
                    f"Roll Rate in rad/s, {s2.gx:.4f}, "
                    f"Rotational Velocity in rad/s, {imu2_rot_vel:.4f}, "
                    f"IMU1 Linear Velocity, {imu1_linear_vel:.4f}, "
                    f"IMU1's X-velocity, {s1.vx:.4f}, "
                    f"IMU1's Y-velocity, {s1.vy:.4f}, "
                    f"Robot Yaw Rate, {w_robot_enc:.4f}, "
                    f"Pendulum Angular Velocity, {s2.gy:.4f}, "
                    f"Pendulum Angle, {pend_angle:.4f}, "
                    f"Pendulum Angle (deg), {pend_angle_deg:.2f}, "
                    f"EncoderL, {self.left_encoder.speed_rad_s:.4f}, Direction, {self.left_encoder.direction}, "
                    f"EncoderR, {self.right_encoder.speed_rad_s:.4f}, Direction, {self.right_encoder.direction}"
                )

                # Print single updating line to console (pad so \r overwrites full previous line)
                CONSOLE_LINE_WIDTH = 700
                print("\r" + line.ljust(CONSOLE_LINE_WIDTH), end="", flush=True)

                # Write to text file (optional, for real-time consumers) — no padding
                self._write_text_output(line)

                # Periodic database flush (every ~1.5 seconds)
                if now - last_db_flush > 1.5:
                    self.db.flush()
                    last_db_flush = now

                # Fixed rate timing
                next_poll += POLLING_RATE
                sleep_time = next_poll - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # if we fell behind, reset schedule
                    next_poll = time.monotonic()

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            # Final flush
            count = self.db.flush()
            print(f"\n[Database] Final flush: {count} readings")
            print(f"[Database] Total readings: {self.db.get_count()}")
            self.db.close()
            if self.text_file:
                self.text_file.close()
            GPIO.cleanup()


# ------------------------------ MAIN ----------------------------------


def main():
    pipeline = CombinedSensorPipeline()
    pipeline.start()


if __name__ == "__main__":
    main()



