import numpy as np
import matplotlib
matplotlib.use('Agg')  # no display needed, saves to file directly
import matplotlib.pyplot as plt
import glob
import os

# automatically finds the most recent log file in Downloads
log_dir = os.path.dirname(os.path.abspath(__file__))
log_files = glob.glob(os.path.join(log_dir, "robot_pid_new_log_*.txt"))
latest_log = max(log_files, key=os.path.getmtime)
print(f"Using log file: {latest_log}")

data = np.loadtxt(latest_log, delimiter=",", skiprows=1)

time_s = data[:, 0] - data[0, 0]
pitch  = np.degrees(data[:, 2])
left   = data[:, 7]
right  = data[:, 8]
dt     = data[:, 9]

# ------------------------------------------------------------------
# Stats printout
# ------------------------------------------------------------------
actual_hz = 1000.0 / dt.mean()
stable_seconds = (np.abs(pitch) < 5).sum() / actual_hz
print(f"\n{'='*55}")
print(f"  PID Run Summary")
print(f"{'='*55}")
print(f"  Log file:         {os.path.basename(latest_log)}")
print(f"  Run duration:     {time_s[-1]:.1f} seconds")
print(f"  Total loops:      {len(data)}")
print(f"  Mean loop rate:   {1000/dt.mean():.1f} Hz")
print(f"  Mean dt:          {dt.mean():.2f} ms")
print(f"  Max dt:           {dt.max():.2f} ms")
print(f"  Pitch min/max:    {pitch.min():.2f}° / {pitch.max():.2f}°")
print(f"  Stable window:    {stable_seconds:.1f}s (|pitch| < 5°)")
print(f"{'='*55}\n")

# ------------------------------------------------------------------
# Plot
# ------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(time_s, pitch, color='blue', linewidth=0.8)
ax1.axhline(y=5,  color='red', linestyle='--', linewidth=1, label='+5° limit')
ax1.axhline(y=-5, color='red', linestyle='--', linewidth=1, label='-5° limit')
ax1.set_ylabel("Pitch (degrees)")
ax1.set_title("PID Balance Run — Pitch vs Time")
ax1.legend()
ax1.grid(True)

ax2.plot(time_s, left,  label='Left motor',  color='green',  linewidth=0.8)
ax2.plot(time_s, right, label='Right motor', color='orange', linewidth=0.8)
ax2.set_ylabel("Motor command")
ax2.set_xlabel("Time (seconds)")
ax2.set_title("Motor Commands vs Time")
ax2.legend()
ax2.grid(True)

plt.tight_layout()

save_path = os.path.join(log_dir, "pid_balance_run.png")
plt.savefig(save_path, dpi=150)
print(f"✅ Plot saved → {save_path}")
