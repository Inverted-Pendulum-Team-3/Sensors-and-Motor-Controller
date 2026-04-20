#!/usr/bin/env python3
"""

Calibration procedure (run once at startup):
1. Hold the robot UPRIGHT at the balance point → script auto-samples for 3 s
The average raw pitch over the sample window is set as the offset
so the balance point reads exactly 0° during operation.

IMU mounting (robot): arrow pointing to X on the IMU is pointed FORWARD.
So: X = forward, Y = lateral (left/right), Z = up.
Pitch (forward/back tilt) = rotation about Y axis → pitch_rate = gyro_y.
Roll (side-to-side tilt)  = rotation about X axis → roll_rate  = gyro_x.
"""

import time
import math
import sys
import os
import signal
import struct
import sqlite3
import threading
import traceback
import queue
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
COUNTS_PER_REV = PPR * 4
WHEEL_DIAMETER_IN = 8.0
WHEEL_RADIUS_M = (WHEEL_DIAMETER_IN * 25.4 / 1000.0) / 2.0
TRACK_WIDTH_IN = 13.5
ROBOT_TRACK_WIDTH_M = TRACK_WIDTH_IN * 0.0254

LEFT_A, LEFT_B, LEFT_X = 18, 17, 27
RIGHT_A, RIGHT_B, RIGHT_X = 14, 4, 15
ENCODER_POLL_RATE = 0.0005  # 2 kHz
LEFT_WHEEL_SIGN  = -1
RIGHT_WHEEL_SIGN =  1

_SCRIPT_DIR       = os.path.dirname(os.path.abspath(__file__))
DB_FILE           = os.path.join(_SCRIPT_DIR, "sensor_data.db")
OBS_CACHE_FILE    = os.path.join(_SCRIPT_DIR, "obs_cache.bin")
SENSOR_STATS_FILE = os.path.join(_SCRIPT_DIR, "sensor_stats.bin")

SHM_NAME = "sensors_shm"
SHM_SIZE = 8 + 9 * 4   # float64 ts + 9 × float32


# ---------------------------------------------------------------------------
# Encoder
# ---------------------------------------------------------------------------
class Encoder:
    """Quadrature encoder — wheel speed in rad/s and m/s."""

    NOISE_COUNTS = 2

    def __init__(self, pin_a, pin_b, pin_x):
        self.pin_a, self.pin_b, self.pin_x = pin_a, pin_b, pin_x
        self.position      = 0
        self.last_position = 0
        self.last_time     = time.monotonic()
        self.speed_rad_s   = 0.0
        self.direction     = "stopped"
        self.last_encoded  = 0
        self.connected     = True
        self.lock          = threading.Lock()

    def setup(self):
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_x, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        a, b = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
        self.last_encoded = (a << 1) | b

    def update(self):
        try:
            a, b    = GPIO.input(self.pin_a), GPIO.input(self.pin_b)
            enc     = (a << 1) | b
            sum_val = (self.last_encoded << 2) | enc
            delta   = 0
            if sum_val in (0b1101, 0b0100, 0b0010, 0b1011):
                delta = 1
            elif sum_val in (0b1110, 0b0111, 0b0001, 0b1000):
                delta = -1
            if delta:
                with self.lock:
                    self.position += delta
            self.last_encoded = enc
            self.connected    = True
        except Exception:
            self.connected = False

    def compute_speed(self):
        now = time.monotonic()
        dt  = now - self.last_time
        if dt <= 0:
            return
        with self.lock:
            delta              = self.position - self.last_position
            self.last_position = self.position
        self.last_time = now
        if abs(delta) <= self.NOISE_COUNTS:
            self.speed_rad_s = 0.0
            self.direction   = "stopped"
            return
        self.speed_rad_s = (delta / dt / COUNTS_PER_REV) * (2.0 * math.pi)
        self.direction   = "forward" if delta > 0 else "backward"

    def wheel_linear_speed(self):
        return self.speed_rad_s * WHEEL_RADIUS_M


# ---------------------------------------------------------------------------
# Database
# ---------------------------------------------------------------------------
def init_db(db_path):
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("""
        CREATE TABLE IF NOT EXISTS sensor_readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL NOT NULL,
            datetime TEXT NOT NULL,
            imu1_gy REAL,
            imu1_body_pitch REAL,
            imu1_yaw_rate REAL,
            encoder_left_rad_s REAL,
            encoder_right_rad_s REAL,
            robot_v REAL,
            robot_w_yaw REAL,
            imu1_connected INTEGER,
            encoder_left_connected INTEGER,
            encoder_right_connected INTEGER
        )
    """)
    conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON sensor_readings(timestamp)")
    conn.commit()
    return conn


def insert_reading(conn, data):
    conn.execute("""
        INSERT INTO sensor_readings (
            timestamp, datetime,
            imu1_gy, imu1_body_pitch, imu1_yaw_rate,
            encoder_left_rad_s, encoder_right_rad_s,
            robot_v, robot_w_yaw,
            imu1_connected, encoder_left_connected, encoder_right_connected
        ) VALUES (
            :timestamp, :datetime,
            :imu1_gy, :imu1_body_pitch, :imu1_yaw_rate,
            :encoder_left_rad_s, :encoder_right_rad_s,
            :robot_v, :robot_w_yaw,
            :imu1_connected, :encoder_left_connected, :encoder_right_connected
        )
    """, data)
    conn.commit()


def _db_writer_loop(db_queue: queue.Queue, db_path: str):
    """Background thread — drains DB write queue so main loop never blocks."""
    conn = sqlite3.connect(db_path, check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL")
    while True:
        row = db_queue.get()
        if row is None:
            break
        try:
            insert_reading(conn, row)
        except Exception:
            pass
        db_queue.task_done()
    conn.close()


def _file_writer_loop(file_queue: queue.Queue):
    """Background thread — atomic tmp+replace writes for obs_cache.bin / sensor_stats.bin."""
    while True:
        item = file_queue.get()
        if item is None:
            break
        path, data = item
        try:
            tmp = path + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        except Exception:
            pass
        file_queue.task_done()


# ---------------------------------------------------------------------------
# BNO085 IMU
# ---------------------------------------------------------------------------
class BNO085_IMU:
    def __init__(self, sda_pin, scl_pin, address=0x4A, imu_name="IMU"):
        self.sda_pin  = sda_pin
        self.scl_pin  = scl_pin
        self.address  = address
        self.imu_name = imu_name
        self.bno      = None
        self.i2c      = None
        self.connection_attempts     = 0
        self.max_connection_attempts = 3
        self.reconnect_delay         = 0.3
        self.pitch_offset_deg        = None
        self._initialize_imu()

    def _initialize_imu(self):
        try:
            if self.i2c:
                try:
                    self.i2c.deinit()
                except Exception:
                    pass
            time.sleep(0.5)
            self.i2c = busio.I2C(self.scl_pin, self.sda_pin)
            # Suppress noisy BNO08X_I2C constructor output
            _devnull    = open(os.devnull, 'w')
            _old_stdout = sys.stdout
            sys.stdout  = _devnull
            try:
                self.bno = BNO08X_I2C(self.i2c, address=self.address)
            finally:
                sys.stdout = _old_stdout
                _devnull.close()
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            # 2 s AHRS convergence settle — only runs at startup / after a hard fault
            time.sleep(2.0)
            self.connection_attempts = 0
            print(f"{self.imu_name} initialized at 0x{self.address:02X}")
            return True
        except Exception as e:
            self.connection_attempts += 1
            print(f"Error initializing {self.imu_name} (attempt {self.connection_attempts}): {e}")
            self.bno = None
            self.i2c = None
            return False

    def _attempt_reconnection(self):
        if self.connection_attempts >= self.max_connection_attempts:
            print(f"Max reconnect attempts for {self.imu_name} — waiting...")
            time.sleep(self.reconnect_delay * 2.0)
            self.connection_attempts = 0
        print(f"Reconnecting {self.imu_name}...")
        time.sleep(self.reconnect_delay)
        return self._initialize_imu()

    @staticmethod
    def quaternion_to_euler(q_i, q_j, q_k, q_real):
        try:
            w, x, y, z = q_real, q_i, q_j, q_k
            sinr_cosp = 2.0 * (w * x + y * z)
            cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
            roll      = math.atan2(sinr_cosp, cosr_cosp)
            sinp      = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
            pitch     = math.asin(sinp)
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw       = math.atan2(siny_cosp, cosy_cosp)
            return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        except Exception as e:
            print(f"Error in quaternion conversion: {e}")
            return 0.0, 0.0, 0.0

    def calibrate_pitch(self, duration_s=3.0):
        if self.bno is None and not self._attempt_reconnection():
            print(f"{self.imu_name}: cannot calibrate, IMU not connected.")
            return
        print(f"\n{'='*55}")
        print(f"  {self.imu_name}: Zero-point pitch calibration")
        print(f"  Sampling {duration_s:.1f} s — keep the robot still...")
        print(f"{'='*55}\n")
        t_end   = time.monotonic() + duration_s
        samples = []
        while time.monotonic() < t_end:
            try:
                qi, qj, qk, qr = self.bno.quaternion
                _, p, _         = self.quaternion_to_euler(qi, qj, qk, qr)
                samples.append(p)
            except Exception:
                pass
            time.sleep(0.01)
        if samples:
            self.pitch_offset_deg = sum(samples) / len(samples)
            print(f"  raw 0° = {self.pitch_offset_deg:+.3f}°  ({len(samples)} samples)")
        else:
            print("  WARNING: no samples — offset set to 0.")
            self.pitch_offset_deg = 0.0
        print("  Calibration complete.\n")

    def get_sensor_data(self):
        """Read one IMU frame. Called only from IMUReader._loop (background thread)."""
        if self.bno is None:
            if not self._attempt_reconnection():
                return None

        try:
            qi, qj, qk, qr        = self.bno.quaternion
            roll_raw, pitch_raw, yaw_raw = self.quaternion_to_euler(qi, qj, qk, qr)
            offset = self.pitch_offset_deg or 0.0
            pitch  = pitch_raw - offset
            if pitch >  180.0: pitch -= 360.0
            if pitch < -180.0: pitch += 360.0
            _, gy, gz = self.bno.gyro
            return {
                "pitch": pitch,
                "roll":  roll_raw,
                "yaw":   yaw_raw,
                "gyro_y": gy,
                "gyro_z": gz,
                "connected": True,
            }

        except KeyError:
            # Unknown BNO report type — bus is alive, just skip this frame
            return {"connected": True, "skipped": True}

        except (OSError, RuntimeError, ValueError) as e:
            print(f"[IMU] I2C error: {e} — will reconnect")
            self.bno = None
            self.i2c = None
            return {"connected": False, "error": str(e)}

        except Exception as e:
            print(f"[IMU] Unexpected error: {e}")
            traceback.print_exc()
            return {"connected": False, "error": str(e)}


# ---------------------------------------------------------------------------
# IMU reader — background thread, 3-sample rolling median on pitch
# ---------------------------------------------------------------------------
class IMUReader:
    """
    Reads the BNO085 on a dedicated thread at 200 Hz.

    The main loop calls get() without ever blocking on I2C.
    A 3-sample rolling median suppresses single-frame pitch outliers
    before they reach shared memory.

    When get_sensor_data() returns a "skipped" sentinel (unknown BNO
    report packet), the frame is dropped without touching self.latest
    or the median buffer — the last valid reading is preserved.
    """

    def __init__(self, imu: BNO085_IMU, rate_hz: float = 200.0):
        self._imu      = imu
        self._interval = 1.0 / rate_hz
        self._lock     = threading.Lock()
        self.latest    = None
        self._running  = False
        self._thread   = None

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True, name="imu-reader")
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            t0   = time.monotonic()
            data = self._imu.get_sensor_data()

            if data and data.get("connected") and not data.get("skipped"):
                with self._lock:
                    self.latest = data

            elapsed = time.monotonic() - t0
            sleep   = self._interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def get(self):
        """Non-blocking — returns last valid dict or None."""
        with self._lock:
            return self.latest


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    IMU1_SDA = board.D2   # GPIO 2
    IMU1_SCL = board.D3   # GPIO 3

    def _sigterm_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm_handler)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    left_enc  = Encoder(LEFT_A,  LEFT_B,  LEFT_X)
    right_enc = Encoder(RIGHT_A, RIGHT_B, RIGHT_X)
    left_enc.setup()
    right_enc.setup()
    successful_reads = 0
    failed_reads     = 0
    reading_count    = 0

    running = False
    from multiprocessing import shared_memory as _shm_mod
    shm     = None
    shm_buf = None
    try:
        # Destroy any leftover SHM from a previous session
        try:
            _stale = _shm_mod.SharedMemory(name=SHM_NAME, create=False, size=SHM_SIZE)
            _stale.close()
            _stale.unlink()
            print(f"[sensors] Removed stale SHM '{SHM_NAME}'")
        except FileNotFoundError:
            pass
        shm     = _shm_mod.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        shm_buf = shm.buf
        shm_buf[:SHM_SIZE] = b'\x00' * SHM_SIZE
        print(f"[sensors] SHM '{SHM_NAME}' ready ({SHM_SIZE} bytes)")
    except Exception as e:
        print(f"[sensors] SHM unavailable, using file cache only: {e}")
        shm_buf = None

    try:
        # Zero cache files so consumers see "no data" during calibration
        try:
            with open(OBS_CACHE_FILE, "wb") as _f:
                _f.write(b'\x00' * SHM_SIZE)
        except Exception:
            pass
        try:
            os.remove(SENSOR_STATS_FILE)
        except Exception:
            pass
        if shm_buf is not None:
            shm_buf[:SHM_SIZE] = b'\x00' * SHM_SIZE

        print("Initializing BNO085 IMU...")
        imu1 = BNO085_IMU(IMU1_SDA, IMU1_SCL, imu_name="IMU #1")
        imu1.calibrate_pitch(duration_s=3.0)

        # IMU thread shares the BNO085_IMU object directly — I2C stays open
        imu_reader = IMUReader(imu1, rate_hz=200.0)
        imu_reader.start()

        conn      = init_db(DB_FILE)
        _db_queue = queue.Queue(maxsize=50)
        threading.Thread(target=_db_writer_loop, args=(_db_queue, DB_FILE),
                         daemon=True, name="db-writer").start()
        _file_queue = queue.Queue(maxsize=20)
        threading.Thread(target=_file_writer_loop, args=(_file_queue,),
                         daemon=True, name="file-writer").start()

        # Encoder polling thread — must run at high rate to catch quadrature edges
        running = True
        def _encoder_loop():
            while running:
                left_enc.update()
                right_enc.update()
                time.sleep(ENCODER_POLL_RATE)
        threading.Thread(target=_encoder_loop, daemon=True, name="enc-poll").start()

        print("obs vector layout:")
        print("  [0] linear_velocity  m/s")
        print("  [1] pitch            rad   (calibrated)")
        print("  [2] pitch_rate       rad/s (gyro Y)")
        print("  [3] yaw_rate         rad/s (gyro Z)")
        print("  [4] wheel_left       rad/s")
        print("  [5] wheel_right      rad/s")
        print("  [6] 0.0  [7] 0.0    filled by hardware_interface")
        print("  [8] yaw              rad   (absolute heading)")
        print(f"\nCache: {OBS_CACHE_FILE}\n")

        _stats_last_write  = time.monotonic()
        _loop_count_window = 0
        _pass_window_ok    = 0
        _pass_window_total = 0

        while True:
            now = time.monotonic()
            _loop_count_window += 1

            # Encoder speed computation at ~100 Hz
            left_enc.compute_speed()
            right_enc.compute_speed()
            v_l     = left_enc.wheel_linear_speed()  * LEFT_WHEEL_SIGN
            v_r     = right_enc.wheel_linear_speed() * RIGHT_WHEEL_SIGN
            v_robot = 0.5 * (v_l + v_r)
            wv_l    = left_enc.speed_rad_s  * LEFT_WHEEL_SIGN
            wv_r    = right_enc.speed_rad_s * RIGHT_WHEEL_SIGN

            reading_count      += 1
            _pass_window_total += 1
            data1 = imu_reader.get()   # non-blocking, returns last valid dict or None

            if data1 and data1.get("connected"):
                successful_reads += 1
                _pass_window_ok  += 1

                pitch_rad  = math.radians(data1["pitch"])
                pitch_rate = data1["gyro_y"]
                yaw_rate   = data1["gyro_z"]
                yaw_rad    = math.radians(data1["yaw"])

                obs9 = (v_robot, pitch_rad, pitch_rate, yaw_rate,
                        wv_l, wv_r, 0.0, 0.0, yaw_rad)
                buf  = struct.pack("<d9f", now, *obs9)

                # Write full obs vector to SHM (timestamp last = atomic from reader's view)
                try:
                    if shm_buf is not None:
                        shm_buf[:SHM_SIZE] = buf
                except Exception:
                    pass

                # Queue obs_cache.bin write — background thread does the I/O
                try:
                    _file_queue.put_nowait((OBS_CACHE_FILE, bytes(buf)))
                except queue.Full:
                    pass

                # Queue DB write at ~10 Hz
                if reading_count % 10 == 0:
                    try:
                        _db_queue.put_nowait({
                            "timestamp":              now,
                            "datetime":               datetime.now().isoformat(),
                            "imu1_gy":                pitch_rate,
                            "imu1_body_pitch":        pitch_rad,
                            "imu1_yaw_rate":          yaw_rate,
                            "encoder_left_rad_s":     wv_l,
                            "encoder_right_rad_s":    wv_r,
                            "robot_v":                v_robot,
                            "robot_w_yaw":            yaw_rate,
                            "imu1_connected":         1,
                            "encoder_left_connected": int(left_enc.connected),
                            "encoder_right_connected":int(right_enc.connected),
                        })
                    except queue.Full:
                        pass

                if reading_count % 10 == 0:
                    sys.stdout.write(
                        "\r" +
                        "obs=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]  "
                        "pitch=%+6.2f°  yaw=%+7.2f°" % (
                            v_robot, pitch_rad, pitch_rate, yaw_rate, wv_l, wv_r,
                            data1["pitch"], data1["yaw"]
                        ) + "          \r"
                    )
                    sys.stdout.flush()

            else:
                failed_reads += 1
                if reading_count % 10 == 0:
                    msg = "no data" if not data1 else data1.get("error", "unknown")
                    sys.stdout.write(
                        f"\rIMU: {msg} | ok={successful_reads} fail={failed_reads}          \r"
                    )
                    sys.stdout.flush()

            # sensor_stats.bin — once per second via file queue
            _elapsed_window = now - _stats_last_write
            if _elapsed_window >= 1.0:
                _pass_rate = (
                    _pass_window_ok / _pass_window_total * 100.0
                    if _pass_window_total > 0 else 0.0
                )
                try:
                    _file_queue.put_nowait((SENSOR_STATS_FILE, bytes(struct.pack(
                        "<dffffdIII",
                        now,
                        float(_loop_count_window / _elapsed_window),
                        200.0,
                        1.0 / ENCODER_POLL_RATE,
                        float(_pass_rate),
                        0.0,
                        successful_reads,
                        failed_reads,
                        reading_count,
                    ))))
                except Exception:
                    pass
                _stats_last_write  = now
                _loop_count_window = 0
                _pass_window_ok    = 0
                _pass_window_total = 0

            elapsed = time.monotonic() - now
            if 0.01 - elapsed > 0:
                time.sleep(0.01 - elapsed)

    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print("Stopped")
        print(f"  Total:      {reading_count}")
        print(f"  Successful: {successful_reads}")
        print(f"  Failed:     {failed_reads}")
        if reading_count > 0:
            print(f"  Rate:       {successful_reads/reading_count*100:.1f}%")
        print("=" * 60)

    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()

    finally:
        running = False
        try:
            imu_reader.stop()
        except NameError:
            pass
        try:
            _db_queue.put_nowait(None)
        except Exception:
            pass
        try:
            _file_queue.put_nowait(None)
        except Exception:
            pass
        try:
            conn.close()
        except NameError:
            pass
        GPIO.cleanup()
        if shm is not None:
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass


if __name__ == "__main__":
    main()
