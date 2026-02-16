#!/usr/bin/env python3
"""
Utility script to query sensor readings from SQLite database.
Useful for analysis, debugging, or feeding data to other applications.
"""

import os
import sqlite3
import sys
import argparse
from datetime import datetime, timedelta

# Same folder as this script (so it finds sensor_data.db next to the pipeline)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")


def get_recent_readings(seconds=10, limit=1000, db_path=DB_FILE):
    """Get recent readings within last N seconds."""
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    
    cutoff_time = datetime.now().timestamp() - seconds
    
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM sensor_readings
        WHERE timestamp >= ?
        ORDER BY timestamp DESC
        LIMIT ?
    """, (cutoff_time, limit))
    
    rows = cursor.fetchall()
    conn.close()
    return rows


def get_readings_by_time_range(start_time, end_time, db_path=DB_FILE):
    """Get readings between two timestamps (datetime strings or timestamps)."""
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    
    # Convert datetime strings to timestamps if needed
    if isinstance(start_time, str):
        start_ts = datetime.fromisoformat(start_time).timestamp()
    else:
        start_ts = start_time
    
    if isinstance(end_time, str):
        end_ts = datetime.fromisoformat(end_time).timestamp()
    else:
        end_ts = end_time
    
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM sensor_readings
        WHERE timestamp >= ? AND timestamp <= ?
        ORDER BY timestamp ASC
    """, (start_ts, end_ts))
    
    rows = cursor.fetchall()
    conn.close()
    return rows


def get_statistics(db_path=DB_FILE):
    """Get database statistics."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Total count
    cursor.execute("SELECT COUNT(*) FROM sensor_readings")
    total = cursor.fetchone()[0]
    
    # Time range
    cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM sensor_readings")
    min_ts, max_ts = cursor.fetchone()
    
    if min_ts:
        min_dt = datetime.fromtimestamp(min_ts)
        max_dt = datetime.fromtimestamp(max_ts)
        duration = max_dt - min_dt
    else:
        min_dt = max_dt = None
        duration = None
    
    conn.close()
    
    return {
        "total_readings": total,
        "first_reading": min_dt,
        "last_reading": max_dt,
        "duration": duration,
    }


def export_to_csv(output_file, start_time=None, end_time=None, db_path=DB_FILE):
    """Export readings to CSV file."""
    import csv
    
    if start_time and end_time:
        rows = get_readings_by_time_range(start_time, end_time, db_path)
    else:
        rows = get_recent_readings(seconds=3600*24, limit=1000000, db_path=db_path)
    
    if not rows:
        print("No data to export.")
        return
    
    # Get column names
    columns = rows[0].keys()
    
    with open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row))
    
    print(f"Exported {len(rows)} readings to {output_file}")


def print_recent_summary(seconds=10, limit=20, db_path=DB_FILE):
    """Print a summary of recent readings."""
    rows = get_recent_readings(seconds=seconds, limit=limit, db_path=db_path)
    
    if not rows:
        print("No recent readings found.")
        return
    
    print(f"\nRecent readings (last {seconds}s, showing {len(rows)}):")
    print("-" * 100)
    
    for row in rows[:limit]:
        dt = datetime.fromtimestamp(row["timestamp"])
        print(f"\n[{dt.strftime('%H:%M:%S.%f')[:-3]}]")
        print(f"  IMU1: pitch={row['imu1_body_pitch']:.4f} rad, "
              f"yaw_rate={row['imu1_yaw_rate']:.4f} rad/s, "
              f"vx={row['imu1_vx']:.4f} m/s")
        print(f"  IMU2: pendulum_angle={row['imu2_pendulum_angle']:.4f} rad "
              f"({row['imu2_pendulum_angle_deg']:.2f}°), "
              f"ang_vel={row['imu2_pendulum_ang_vel']:.4f} rad/s")
        print(f"  Encoders: L={row['encoder_left_rad_s']:.4f} rad/s, "
              f"R={row['encoder_right_rad_s']:.4f} rad/s")
        print(f"  Robot: v={row['robot_v']:.4f} m/s, w={row['robot_w_enc']:.4f} rad/s")


def main():
    parser = argparse.ArgumentParser(
        description="Query sensor readings from SQLite database"
    )
    parser.add_argument(
        "--db",
        default=DB_FILE,
        help=f"Path to database file (default: {DB_FILE})"
    )
    parser.add_argument(
        "--recent",
        type=int,
        metavar="SECONDS",
        help="Show recent readings from last N seconds"
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=20,
        help="Limit number of results (default: 20)"
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Show database statistics"
    )
    parser.add_argument(
        "--export",
        metavar="FILE",
        help="Export readings to CSV file"
    )
    parser.add_argument(
        "--start",
        help="Start time for export (ISO format: YYYY-MM-DD HH:MM:SS)"
    )
    parser.add_argument(
        "--end",
        help="End time for export (ISO format: YYYY-MM-DD HH:MM:SS)"
    )
    
    args = parser.parse_args()
    
    if args.stats:
        stats = get_statistics(args.db)
        print("\nDatabase Statistics:")
        print("-" * 60)
        print(f"Total readings: {stats['total_readings']}")
        if stats['first_reading']:
            print(f"First reading: {stats['first_reading']}")
            print(f"Last reading: {stats['last_reading']}")
            print(f"Duration: {stats['duration']}")
        else:
            print("Database is empty.")
    
    elif args.export:
        export_to_csv(args.export, args.start, args.end, args.db)
    
    elif args.recent:
        print_recent_summary(args.recent, args.limit, args.db)
    
    else:
        # Default: show recent 10 seconds
        print_recent_summary(10, args.limit, args.db)


if __name__ == "__main__":
    main()


