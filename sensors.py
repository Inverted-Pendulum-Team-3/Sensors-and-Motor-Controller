#!/usr/bin/env python3
"""
Dual BNO085 IMU + AMT-103 Encoders + HC-SR04 Ultrasonics — Sensor pipeline (Cursor version).
For inverted pendulum robot (ECEN 403/404). ~33 Hz, SQLite, same output format/units as sensors.py.

- Calibration at start (both IMUs, robot level and still).
- Complementary filter for body pitch (IMU1) and pendulum angle (IMU2); level detection via raw 3D gravity.
- IMU1 pitch axis/sign configurable for 180° board mount.
- SQLite batch inserts, optional real-time text file.
"""

import time
import math
import os
import sys
import io
import contextlib
import threading
import warnings
import sqlite3
from collections import deque
from datetime import datetime

import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GRAVITY,
)
try:
    from adafruit_bno08x import BNO_REPORT_LINEAR_ACCELERATION
except ImportError:
    BNO_REPORT_LINEAR_ACCELERATION = None
from adafruit_bno08x.i2c import BNO08X_I2C

warnings.filterwarnings("ignore")

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# IMUs
IMU1_ADDRESS = 0x4A
IMU2_ADDRESS = 0x4B
I2C_FREQUENCY = 400_000
GYRO_THRESHOLD = 0.02
FILTER_WINDOW_SIZE = 10

# Complementary filter
BODY_PITCH_ALPHA = 0.98
BODY_PITCH_ALPHA_LEVEL = 0.82
PENDULUM_ALPHA_MOVING = 0.98
PENDULUM_ALPHA_STABLE = 0.90
PENDULUM_ALPHA_LEVEL = 0.82
ACCEL_STABILITY_THRESHOLD = 0.1
GRAVITY_MIN_FOR_ANGLE = 3.0  # raw 3D mag: below = level (correct to 0), above = use gravity angle
IMU1_PITCH_AXIS = "x"   # "x" or "y" — chip axis that sees forward/back tilt
IMU1_PITCH_SIGN = -1    # -1 or 1 — forward tilt should give positive angle

# Robot
PENDULUM_LENGTH_M = 0.65
CHASSIS_HEIGHT_M = 0.05
WHEEL_DIAMETER_IN = 8.0
WHEEL_DIAMETER_MM = WHEEL_DIAMETER_IN * 25.4
WHEEL_RADIUS_M = (WHEEL_DIAMETER_MM / 1000.0) / 2.0
ROBOT_TRACK_WIDTH_M = 0.25
PPR = 2048
COUNTS_PER_REV = PPR * 4
VELOCITY_RESET_THRESHOLD = 0.1

# GPIO (BCM) — encoders: Left A=18,B=17,X=27; Right A=14,B=4,X=15
LEFT_A, LEFT_B, LEFT_X = 18, 17, 27
RIGHT_A, RIGHT_B, RIGHT_X = 14, 4, 15
# Ultrasonics: Right Trig=8,Echo=11; Left Trig=25,Echo=9
TRIG_RIGHT, ECHO_RIGHT = 8, 11
TRIG_LEFT, ECHO_LEFT = 25, 9
ULTRASONIC_ECHO_TIMEOUT_S = 0.025
ULTRASONIC_INTER_SENSOR_DELAY_S = 0.012
CM_PER_US = 1.0 / 58.3

# Timing
POLLING_RATE = 0.03
ENCODER_POLL_RATE = 0.0002

# DB & output
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")
TEXT_OUTPUT_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.txt")
BATCH_INSERT_SIZE = 50

# ---------------------------------------------------------------------------
# UTILS
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def suppress_all_output():
    with open(os.devnull, "w") as devnull:
        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout = sys.stderr = devnull
        try:
            yield
        finally:
            sys.stdout, sys.stderr = old_stdout, old_stderr


class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.data = deque(maxlen=window_size)

    def update(self, value):
        self.data.append(value)
        return sum(self.data) / len(self.data) if self.data else 0.0


# ---------------------------------------------------------------------------
# DATABASE
# ---------------------------------------------------------------------------


class SensorDatabase:
    def __init__(self, db_path):
        self.db_path = db_path
        self.conn = None
        self.pending_inserts = []
        self._init_db()

    def _init_db(self):
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("""
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
        for col in ("ultrasonic_right_cm", "ultrasonic_left_cm"):
            try:
                self.conn.execute(f"ALTER TABLE sensor_readings ADD COLUMN {col} REAL")
                self.conn.commit()
            except sqlite3.OperationalError:
                pass
        self.conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON sensor_readings(timestamp)")
        self.conn.execute("CREATE INDEX IF NOT EXISTS idx_datetime ON sensor_readings(datetime)")
        self.conn.commit()
        print(f"[Database] Initialized: {self.db_path}")

    def add_reading(self, data_dict):
        self.pending_inserts.append(data_dict)
        if len(self.pending_inserts) >= BATCH_INSERT_SIZE:
            self.flush()

    def flush(self):
        if not self.pending_inserts:
            return 0
        try:
            self.conn.executemany("""
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
            """, self.pending_inserts)
            self.conn.commit()
            n = len(self.pending_inserts)
            self.pending_inserts.clear()
            return n
        except Exception as e:
            print(f"[Database] Error flushing: {e}")
            self.pending_inserts.clear()
            return 0

    def get_count(self):
        try:
            return self.conn.execute("SELECT COUNT(*) FROM sensor_readings").fetchone()[0]
        except Exception:
            return 0

    def close(self):
        self.flush()
        if self.conn:
            self.conn.close()


# ---------------------------------------------------------------------------
# IMU STATE & MANAGER
# ---------------------------------------------------------------------------


class IMUState:
    def __init__(self):
        self.gx = self.gy = self.gz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.ax_raw = self.ay_raw = self.az_raw = 0.0
        self.gx_off = self.gy_off = self.gz_off = 0.0
        self.ax_off = self.ay_off = self.az_off = 0.0
        self.gx_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.gy_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.gz_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.ax_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.ay_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.az_f = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.body_pitch = 0.0
        self.pendulum_angle = 0.0
        self.last_angle_update = time.monotonic()
        self.lax = self.lay = self.laz = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.last_vel_update = time.monotonic()
        self.connected = False


class IMUManager:
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
        if not self.device:
            return None
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                gyro = self.device.gyro
                gravity = getattr(self.device, "gravity", None)
                accel = getattr(self.device, "acceleration", None)
            if not gyro or len(gyro) != 3 or not (gravity or accel):
                return None
            ax, ay, az = gravity if gravity and len(gravity) == 3 else accel
            return {"gx": gyro[0], "gy": gyro[1], "gz": gyro[2], "ax": ax, "ay": ay, "az": az}
        except Exception:
            return None

    def calibrate(self, duration_s=3.0, samples=150):
        print(f"\n[{self.name}] Calibrating... Keep robot STILL and level.")
        delay = duration_s / samples
        gx_s, gy_s, gz_s, ax_s, ay_s, az_s = [], [], [], [], [], []
        for i in range(samples):
            data = self._read_raw()
            if data:
                gx_s.append(data["gx"]); gy_s.append(data["gy"]); gz_s.append(data["gz"])
                ax_s.append(data["ax"]); ay_s.append(data["ay"]); az_s.append(data["az"])
            if (i + 1) % 30 == 0:
                print(f"[{self.name}] Progress: {i + 1}/{samples}")
            time.sleep(delay)
        s = self.state
        if gx_s:
            s.gx_off = sum(gx_s) / len(gx_s)
            s.gy_off = sum(gy_s) / len(gy_s)
            s.gz_off = sum(gz_s) / len(gz_s)
            s.ax_off = sum(ax_s) / len(ax_s)
            s.ay_off = sum(ay_s) / len(ay_s)
            s.az_off = sum(az_s) / len(az_s)
            print(f"[{self.name}] Gyro offsets: X={s.gx_off:.4f}, Y={s.gy_off:.4f}, Z={s.gz_off:.4f}")
            print(f"[{self.name}] Accel offsets: X={s.ax_off:.4f}, Y={s.ay_off:.4f}, Z={s.az_off:.4f}")
        else:
            print(f"[{self.name}] WARNING: Calibration failed, using zero offsets.")

    def update_filtered(self):
        data = self._read_raw()
        if not data:
            self.state.connected = False
            return False
        s = self.state
        gx = data["gx"] - s.gx_off
        gy = data["gy"] - s.gy_off
        gz = data["gz"] - s.gz_off
        ax = data["ax"] - s.ax_off
        ay = data["ay"] - s.ay_off
        az = data["az"] - s.az_off
        gx_f = s.gx_f.update(gx)
        gy_f = s.gy_f.update(gy)
        gz_f = s.gz_f.update(gz)
        s.gx = gx_f if abs(gx_f) > GYRO_THRESHOLD else 0.0
        s.gy = gy_f if abs(gy_f) > GYRO_THRESHOLD else 0.0
        s.gz = gz_f if abs(gz_f) > GYRO_THRESHOLD else 0.0
        s.ax = s.ax_f.update(ax)
        s.ay = s.ay_f.update(ay)
        s.az = s.az_f.update(az)
        s.ax_raw, s.ay_raw, s.az_raw = ax, ay, az
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
        s = self.state
        now = time.monotonic()
        dt = now - s.last_angle_update
        if dt <= 0 or dt > 0.1:
            s.last_angle_update = now
            return
        gyro_angle = s.body_pitch + s.gy * dt
        mag = math.sqrt(s.ax_raw**2 + s.ay_raw**2 + s.az_raw**2)
        if mag < GRAVITY_MIN_FOR_ANGLE:
            accel_angle = 0.0
            alpha = BODY_PITCH_ALPHA_LEVEL
        else:
            h = s.ax if IMU1_PITCH_AXIS == "x" else s.ay
            accel_angle = math.atan2(IMU1_PITCH_SIGN * h, s.az + 1e-9)
            alpha = BODY_PITCH_ALPHA
        s.body_pitch = alpha * gyro_angle + (1.0 - alpha) * accel_angle
        s.last_angle_update = now

    def update_pendulum_angle(self):
        s = self.state
        now = time.monotonic()
        dt = now - s.last_angle_update
        if dt <= 0 or dt > 0.1:
            s.last_angle_update = now
            return
        gyro_angle = s.pendulum_angle + s.gy * dt
        mag = math.sqrt(s.ax_raw**2 + s.ay_raw**2 + s.az_raw**2)
        if mag < GRAVITY_MIN_FOR_ANGLE:
            accel_angle = 0.0
            alpha = PENDULUM_ALPHA_LEVEL
        else:
            h = s.ax if abs(s.ax) >= abs(s.ay) else s.ay
            accel_angle = math.atan2(h, s.az + 1e-9)
            stable = abs(s.ax) < ACCEL_STABILITY_THRESHOLD and abs(s.ay) < ACCEL_STABILITY_THRESHOLD
            alpha = PENDULUM_ALPHA_STABLE if stable else PENDULUM_ALPHA_MOVING
        s.pendulum_angle = alpha * gyro_angle + (1.0 - alpha) * accel_angle
        s.last_angle_update = now

    def integrate_linear_velocity(self, reset_if_stopped=False, left_speed=0.0, right_speed=0.0):
        s = self.state
        now = time.monotonic()
        dt = now - s.last_vel_update
        if dt <= 0 or dt > 0.1:
            s.last_vel_update = now
            return
        s.vx += s.lax * dt
        s.vy += s.lay * dt
        s.vz += s.laz * dt
        if reset_if_stopped and abs(left_speed) < VELOCITY_RESET_THRESHOLD and abs(right_speed) < VELOCITY_RESET_THRESHOLD:
            s.vx = s.vy = s.vz = 0.0
        s.last_vel_update = now


# ---------------------------------------------------------------------------
# ENCODER
# ---------------------------------------------------------------------------


class Encoder:
    NOISE_COUNTS = 2

    def __init__(self, name, pin_a, pin_b, pin_x):
        self.name = name
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
# ULTRASONIC
# ---------------------------------------------------------------------------


def _read_ultrasonic_cm(trig_pin, echo_pin):
    GPIO.output(trig_pin, True)
    time.sleep(10e-6)
    GPIO.output(trig_pin, False)
    t0 = time.monotonic()
    while GPIO.input(echo_pin) == 0:
        if time.monotonic() - t0 > ULTRASONIC_ECHO_TIMEOUT_S:
            return None
    start = time.monotonic()
    while GPIO.input(echo_pin) == 1:
        if time.monotonic() - start > ULTRASONIC_ECHO_TIMEOUT_S:
            return None
    return (time.monotonic() - start) * 1e6 * CM_PER_US


# ---------------------------------------------------------------------------
# PIPELINE
# ---------------------------------------------------------------------------


class CombinedSensorPipeline:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.i2c = None
        self.imu1 = self.imu2 = None
        self.left_encoder = Encoder("Left", LEFT_A, LEFT_B, LEFT_X)
        self.right_encoder = Encoder("Right", RIGHT_A, RIGHT_B, RIGHT_X)
        self.encoder_thread = None
        self.running = False
        self.db = SensorDatabase(DB_FILE)
        self.text_file = None
        try:
            self.text_file = open(TEXT_OUTPUT_FILE, "w")
        except Exception as e:
            print(f"[Warning] Could not open text output file: {e}")
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
            self.imu2.state.pendulum_angle = 0.0
        print("\nInitializing encoders...")
        self.left_encoder.setup()
        self.right_encoder.setup()
        for p in (TRIG_RIGHT, TRIG_LEFT):
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, False)
        for p in (ECHO_RIGHT, ECHO_LEFT):
            GPIO.setup(p, GPIO.IN)
        print("Encoders and ultrasonics initialized.")
        print("=" * 60)
        print(f"[Database] Total readings: {self.db.get_count()}")
        print("\nStarting data collection...\n")
        self.running = True
        self.encoder_thread = threading.Thread(target=self._encoder_loop, daemon=True)
        self.encoder_thread.start()
        self._main_loop()

    def _encoder_loop(self):
        while self.running:
            self.left_encoder.update()
            self.right_encoder.update()
            time.sleep(ENCODER_POLL_RATE)

    def _main_loop(self):
        next_poll = time.monotonic()
        last_db_flush = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if self.imu1 and self.imu1.device:
                    self.imu1.update_filtered()
                    self.imu1.update_body_pitch()
                if self.imu2 and self.imu2.device:
                    self.imu2.update_filtered()
                    self.imu2.update_pendulum_angle()
                self.left_encoder.compute_speed()
                self.right_encoder.compute_speed()
                v_l = self.left_encoder.wheel_linear_speed()
                v_r = self.right_encoder.wheel_linear_speed()
                v_robot = 0.5 * (v_r + v_l)
                w_robot_enc = (v_r - v_l) / (ROBOT_TRACK_WIDTH_M + 1e-9)
                if self.imu1 and self.imu1.device:
                    self.imu1.integrate_linear_velocity(
                        reset_if_stopped=True,
                        left_speed=self.left_encoder.speed_rad_s,
                        right_speed=self.right_encoder.speed_rad_s,
                    )
                d_right = _read_ultrasonic_cm(TRIG_RIGHT, ECHO_RIGHT)
                time.sleep(ULTRASONIC_INTER_SENSOR_DELAY_S)
                d_left = _read_ultrasonic_cm(TRIG_LEFT, ECHO_LEFT)

                s1 = self.imu1.state if self.imu1 else IMUState()
                s2 = self.imu2.state if self.imu2 else IMUState()
                imu1_rot_vel = math.sqrt(s1.gx**2 + s1.gy**2 + s1.gz**2)
                imu2_rot_vel = math.sqrt(s2.gx**2 + s2.gy**2 + s2.gz**2)
                pend_angle = s2.pendulum_angle
                pend_angle_deg = math.degrees(pend_angle)

                data_dict = {
                    "timestamp": now,
                    "datetime": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                    "imu1_gx": s1.gx, "imu1_gy": s1.gy, "imu1_gz": s1.gz,
                    "imu1_ax": s1.ax, "imu1_ay": s1.ay, "imu1_az": s1.az,
                    "imu1_body_pitch": s1.body_pitch, "imu1_yaw_rate": s1.gz, "imu1_rot_vel": imu1_rot_vel,
                    "imu1_vx": s1.vx, "imu1_vy": s1.vy, "imu1_vz": s1.vz,
                    "imu2_gx": s2.gx, "imu2_gy": s2.gy, "imu2_gz": s2.gz,
                    "imu2_ax": s2.ax, "imu2_ay": s2.ay, "imu2_az": s2.az,
                    "imu2_pendulum_angle": pend_angle, "imu2_pendulum_angle_deg": pend_angle_deg,
                    "imu2_pendulum_ang_vel": s2.gy, "imu2_rot_vel": imu2_rot_vel,
                    "encoder_left_rad_s": self.left_encoder.speed_rad_s,
                    "encoder_right_rad_s": self.right_encoder.speed_rad_s,
                    "encoder_left_dir": self.left_encoder.direction,
                    "encoder_right_dir": self.right_encoder.direction,
                    "robot_v": v_robot, "robot_w_enc": w_robot_enc,
                    "ultrasonic_right_cm": d_right, "ultrasonic_left_cm": d_left,
                    "imu1_connected": 1 if s1.connected else 0,
                    "imu2_connected": 1 if s2.connected else 0,
                    "encoder_left_connected": 1 if self.left_encoder.connected else 0,
                    "encoder_right_connected": 1 if self.right_encoder.connected else 0,
                }
                self.db.add_reading(data_dict)

                ts = datetime.now().strftime("%H:%M:%S")
                imu1_linear_vel = math.sqrt(s1.vx**2 + s1.vy**2)
                r_cm = f"{d_right:.1f}" if d_right is not None else "---"
                l_cm = f"{d_left:.1f}" if d_left is not None else "---"
                line = (
                    f"{ts}, "
                    f"IMU1, Forward/backwards, {s1.body_pitch:.4f}, Side-to-Side, {s1.ay:.4f}, "
                    f"Yaw, {s1.gz:.4f}, Pitch Rate, {s1.gy:.4f}, Roll Rate, {s1.gx:.4f}, "
                    f"Rotational Velocity, {imu1_rot_vel:.4f}, "
                    f"IMU2, Tilt, {pend_angle:.4f}, Side-to-Side tilt, {s2.ax:.4f}, "
                    f"Yaw, {s2.gz:.4f}, Pitch Rate, {s2.gy:.4f}, Roll Rate, {s2.gx:.4f}, "
                    f"Rotational Velocity, {imu2_rot_vel:.4f}, "
                    f"IMU1 Linear Velocity, {imu1_linear_vel:.4f}, "
                    f"IMU1's X-velocity, {s1.vx:.4f}, IMU1's Y-velocity, {s1.vy:.4f}, "
                    f"Robot Yaw Rate, {w_robot_enc:.4f}, "
                    f"Pendulum Angular Velocity, {s2.gy:.4f}, "
                    f"Pendulum Angle, {pend_angle:.4f}, Pendulum Angle (deg), {pend_angle_deg:.2f}, "
                    f"EncoderL, {self.left_encoder.speed_rad_s:.4f}, Direction, {self.left_encoder.direction}, "
                    f"EncoderR, {self.right_encoder.speed_rad_s:.4f}, Direction, {self.right_encoder.direction}, "
                    f"Ultrasonic Right, {r_cm} cm, Ultrasonic Left, {l_cm} cm"
                )
                print("\r" + line.ljust(900), end="", flush=True)
                if self.text_file:
                    try:
                        self.text_file.seek(0)
                        self.text_file.write(line + "\n")
                        self.text_file.truncate()
                        self.text_file.flush()
                    except Exception:
                        pass

                if now - last_db_flush > 1.5:
                    self.db.flush()
                    last_db_flush = now

                next_poll += POLLING_RATE
                sleep_time = next_poll - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_poll = time.monotonic()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            n = self.db.flush()
            print(f"\n[Database] Final flush: {n} readings")
            print(f"[Database] Total readings: {self.db.get_count()}")
            self.db.close()
            if self.text_file:
                self.text_file.close()
            GPIO.cleanup()


def main():
    CombinedSensorPipeline().start()


if __name__ == "__main__":
    main()


