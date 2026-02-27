import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import re

plt.style.use('seaborn-v0_8-whitegrid')

# --- Parse touch_times.txt ---
rows = []
pattern = re.compile(
    r"Session\s+(\d+),\s+Simulation\s+(\d+),\s+attempt\s+(\d+),\s+(\d+)\s+ns"
)

with open("/home/RVLuser/rvl-linux/data/Exp-correct_calibration-20250616/ExpRez/touch_times.log", "r") as f:
    for line_idx, line in enumerate(f):
        m = pattern.match(line.strip())
        if m:
            session = int(m.group(1))
            sim = int(m.group(2))
            attempt_in_sim = int(m.group(3))
            time_ns = int(m.group(4))

            rows.append({
                "Session": session,
                "Simulation": sim,
                "AttemptInSimulation": attempt_in_sim,
                "Time_ms": time_ns / 1e6,  # convert ns -> ms
                "LineIndex": line_idx      # to preserve log order
            })

df = pd.DataFrame(rows)

# --- Count corrections per session ---
corrections_per_session = (
    df.groupby("Session")
      .size()
      .rename("NumCorrections")
      .reset_index()
)

print("\n=== Corrections per session ===")
print(corrections_per_session)

# --- Rank sessions by number of corrections (descending) ---
ranked = corrections_per_session.sort_values("NumCorrections", ascending=False)

print("\n=== Sessions ranked by number of corrections (highest first) ===")
print(ranked)

# --- Order by session & appearance in log, then assign correction index within session ---
df = df.sort_values(["Session", "LineIndex"])
df["CorrectionIndex"] = df.groupby("Session").cumcount() + 1  # 1st correction, 2nd, ...

# --- Average over sessions: mean time for each correction index ---
agg = (
    df.groupby("CorrectionIndex")["Time_ms"]
      .agg(["mean", "std", "count"])
      .reset_index()
      .sort_values("CorrectionIndex")
)

print(agg)  # optional: see numbers in terminal



plt.figure(figsize=(8, 4))

# --- Plot each session's correction-time curve (faint lines) ---
for session_id, sdf in df.groupby("Session"):
    plt.plot(
        sdf["CorrectionIndex"],
        sdf["Time_ms"],
        alpha=0.2,
        linewidth=1.0,
        color="tab:orange"
    )

# --- Legend entries (dummy lines that don't appear in the plot) ---
plt.plot([], [], color="tab:orange", alpha=0.5, linewidth=2,
         label="Individual Sequences")

plt.plot([], [], color="tab:blue", marker='o', linewidth=2,
         label="Average Across Sequences")

# --- Plot the actual average curve ---
plt.plot(
    agg["CorrectionIndex"],
    agg["mean"],
    '-o',
    linewidth=1.5,
    markersize=3,
    color="tab:blue"
)

plt.xlabel("Unsuccessful Actions", fontsize=12)
plt.ylabel("Correction Time (ms)", fontsize=12)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)
plt.xticks(agg["CorrectionIndex"], fontsize=9)
plt.locator_params(axis="y", nbins=12)

plt.legend(frameon=False)
plt.tight_layout()

plt.savefig("/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/analysis/simulation_avg_time_per_action.pdf",
            bbox_inches='tight', dpi=300)

plt.show()