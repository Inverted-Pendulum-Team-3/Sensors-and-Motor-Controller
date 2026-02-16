#!/usr/bin/env python3
"""
Read the last N sensor readings from the SQLite database.
Prints each reading in the same full format as the main pipeline.
Usage:
  python3 read_last_readings.py           # last 50 (default)
  python3 read_last_readings.py 100      # last 100
  python3 read_last_readings.py --count 20
"""

import math
import os
import sqlite3
import argparse
from datetime import datetime

# Same folder as this script
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")


def get_last_readings(count=50, db_path=DB_FILE):
    """
    Return the last `count` readings (most recent first).
    Each row is a dict with column names as keys.
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM sensor_readings
        ORDER BY timestamp DESC
        LIMIT ?
    """, (count,))
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]


def format_row_like_pipeline(row):
    """Format one DB row as the same full line the main pipeline prints."""
    ts_raw = row.get("datetime") or row.get("timestamp")
    if isinstance(ts_raw, (int, float)):
        ts = datetime.fromtimestamp(ts_raw).strftime("%H:%M:%S")
    else:
        # "2026-02-04 18:23:22.529543" -> "18:23:22"
        ts = str(ts_raw)[:19].split(" ")[-1] if ts_raw else ""

    s1_gx = row.get("imu1_gx") or 0.0
    s1_gy = row.get("imu1_gy") or 0.0
    s1_gz = row.get("imu1_gz") or 0.0
    s1_vx = row.get("imu1_vx") or 0.0
    s1_vy = row.get("imu1_vy") or 0.0
    imu1_rot_vel = row.get("imu1_rot_vel") or math.sqrt(s1_gx**2 + s1_gy**2 + s1_gz**2)
    imu1_linear_vel = math.sqrt(s1_vx**2 + s1_vy**2)

    s2_gx = row.get("imu2_gx") or 0.0
    s2_gy = row.get("imu2_gy") or 0.0
    s2_gz = row.get("imu2_gz") or 0.0
    imu2_rot_vel = row.get("imu2_rot_vel") or math.sqrt(s2_gx**2 + s2_gy**2 + s2_gz**2)

    pend_angle = row.get("imu2_pendulum_angle") or 0.0
    pend_angle_deg = row.get("imu2_pendulum_angle_deg") or 0.0
    w_robot_enc = row.get("robot_w_enc") or 0.0
    enc_l = row.get("encoder_left_rad_s") or 0.0
    enc_r = row.get("encoder_right_rad_s") or 0.0
    enc_l_dir = row.get("encoder_left_dir") or "stopped"
    enc_r_dir = row.get("encoder_right_dir") or "stopped"

    return (
        f"{ts}, "
        f"IMU1, Forward/backwards Tilt in rad/s, {s1_gy:.4f}, "
        f"Side-to-Side tilt in rad/s, {s1_gx:.4f}, "
        f"Yaw in rad/s, {s1_gz:.4f}, "
        f"Pitch Rate in rad/s, {s1_gy:.4f}, "
        f"Roll Rate in rad/s, {s1_gx:.4f}, "
        f"Rotational Velocity in rad/s, {imu1_rot_vel:.4f}, "
        f"IMU2, Tilt in rad/s, {s2_gy:.4f}, "
        f"Side-to-Side tilt in rad/s, {s2_gx:.4f}, "
        f"Yaw in rad/s, {s2_gz:.4f}, "
        f"Pitch Rate in rad/s, {s2_gy:.4f}, "
        f"Roll Rate in rad/s, {s2_gx:.4f}, "
        f"Rotational Velocity in rad/s, {imu2_rot_vel:.4f}, "
        f"IMU1 Linear Velocity, {imu1_linear_vel:.4f}, "
        f"IMU1's X-velocity, {s1_vx:.4f}, "
        f"IMU1's Y-velocity, {s1_vy:.4f}, "
        f"Robot Yaw Rate, {w_robot_enc:.4f}, "
        f"Pendulum Angular Velocity, {s2_gy:.4f}, "
        f"Pendulum Angle, {pend_angle:.4f}, "
        f"Pendulum Angle (deg), {pend_angle_deg:.2f}, "
        f"EncoderL, {enc_l:.4f}, Direction, {enc_l_dir}, "
        f"EncoderR, {enc_r:.4f}, Direction, {enc_r_dir}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Read the last N sensor readings from sensor_data.db"
    )
    parser.add_argument(
        "count",
        nargs="?",
        type=int,
        default=50,
        help="Number of readings to fetch (default: 50)",
    )
    parser.add_argument(
        "--db",
        default=DB_FILE,
        help="Path to sensor_data.db",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Only print raw dict per row, no formatting",
    )
    args = parser.parse_args()

    rows = get_last_readings(count=args.count, db_path=args.db)

    if not rows:
        print("No readings in database.")
        return

    if args.quiet:
        for r in rows:
            print(r)
        return

    print(f"Last {len(rows)} readings (most recent first), full format:\n")
    print("-" * 120)

    for i, row in enumerate(rows, 1):
        line = format_row_like_pipeline(row)
        print(f"{i:3}. {line}")

    print("-" * 120)
    print(f"Total: {len(rows)} readings")


if __name__ == "__main__":
    main()


