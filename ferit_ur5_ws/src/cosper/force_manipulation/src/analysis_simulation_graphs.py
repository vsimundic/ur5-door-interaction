import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import re
import os
from collections import defaultdict
import yaml
import numpy as np

# --- Load Configs ---
script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '../cfg/door_replanning_control.yaml'), 'r') as f:
    config = yaml.safe_load(f)

analysis_dir = config["analysis_dir"]
log_dir = config["rvl_log_dir"]

# --- Mapping of filenames to LaTeX-friendly labels ---
file_label_map = {
    "touch_success_baseline.log": r"Baseline",

    # "touch_success_stdgxyk_0.02.log": r"$\sigma_{g,xy} = 0.02$",
    # "touch_success_stdgxyk_0.1.log": r"$\sigma_{g,xy} = 0.1$",
    # "touch_success_stdgzk_0.01.log": r"$\sigma_{g,z} = 0.01$",
    # "touch_success_stdgzk_0.05.log": r"$\sigma_{g,z} = 0.05$",
    # "touch_success_stdphi_1.0.log": r"$\sigma_{\varphi} = 1.0$",
    # "touch_success_stdphi_3.0.log": r"$\sigma_{\varphi} = 3.0$",
    # "touch_success_vertexDistThr_0.05.log": r"$d_{vtx} = 0.05$",
    # "touch_success_vertexDistThr_0.06.log": r"$d_{vtx} = 0.06$",

    "touch_success_alpha_0.0001.log": r"$\alpha = 10^{-4}$",
    "touch_success_alpha_0.00001.log": r"$\alpha = 10^{-5}$",
    "touch_success_alpha_0.0000001.log": r"$\alpha = 10^{-7}$",
    "touch_success_nbest_0.log": r"$N_{best} = 0$",
    "touch_success_nbest_50.log": r"$N_{best} = 50$",
    "touch_success_nbest_100.log": r"$N_{best} = 100$",
    "touch_success_nbest_750.log": r"$N_{best} = 750$"
}

# --- Function to Parse Log Files ---
def parse_log_file(filepath):
    attempt_counts = []
    scene_attempts = defaultdict(list)
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            match = re.match(r"Simulation\s+(\d+),\s+attempts\s+(\d+),\s+(\w+)", line.strip())
            if match:
                scene_id = i % 20  # scenes 0–19
                attempts = int(match.group(2))
                attempt_counts.append(attempts)
                scene_attempts[scene_id].append(attempts)

                if scene_id == 0:
                    debug = 0

    return attempt_counts, scene_attempts

# --- Plot 1: Histogram of Attempt Frequencies ---
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(6.5, 4))
bins = list(range(1, 9))  # Attempt counts from 1 to 8
width = 0.07  # Width of each bar per group

# Assign each configuration a unique offset for bar grouping
labels = list(file_label_map.keys())
x = bins
n_configs = len(file_label_map)
offsets = [i * width - (n_configs / 2) * width for i in range(n_configs)]

all_attempts = []
all_freqs = []
for i, (filename, label) in enumerate(file_label_map.items()):
    filepath = os.path.join(log_dir, filename)
    if not os.path.isfile(filepath):
        continue
    attempts, _ = parse_log_file(filepath)
    all_attempts.append(attempts)
    freq = pd.Series(attempts).value_counts().reindex(bins, fill_value=0).sort_index()
    all_freqs.append(freq)
    plt.bar(
        [xi + offsets[i] for xi in x],
        freq.values,
        width=width,
        label=label,
        alpha=0.9
    )

# All freqs to 
df_all = pd.DataFrame(all_freqs, index=file_label_map.values(), columns=bins).T
df_all.to_csv(os.path.join(analysis_dir, "simulation_ablation_attempt_histogram.csv"), index_label="Attempts")

plt.xlabel('Number of Attempts', fontsize=12)
plt.ylabel('Count', fontsize=12)
plt.xticks(bins, fontsize=10)
plt.yticks(fontsize=10)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)

# Legend handling
plt.legend(fontsize=9, loc='upper right', frameon=False, handlelength=1.5, ncol=2)

plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, "simulation_ablation_attempt_histogram.pdf"),
            bbox_inches='tight', dpi=300)
plt.show()


# --- Plot 2: Average Number of Attempts per Scene ---
scene_plot_data = []
for filename, label in file_label_map.items():
    filepath = os.path.join(log_dir, filename)
    if not os.path.isfile(filepath):
        continue
    _, scene_attempts = parse_log_file(filepath)
    for scene_id in sorted(scene_attempts.keys()):
        avg = sum(scene_attempts[scene_id]) / len(scene_attempts[scene_id])
        scene_plot_data.append({
            "Scene": scene_id,
            "Average Attempts": avg,
            "Configuration": label
        })

df_scene = pd.DataFrame(scene_plot_data)

plt.figure(figsize=(6.5, 4))
sns.lineplot(data=df_scene, x="Scene", y="Average Attempts", hue="Configuration", marker="o", linewidth=2)

plt.xlabel("Scene", fontsize=12)
plt.ylabel("Average Number of Attempts", fontsize=12)
plt.xticks(range(0, 20), fontsize=10)
plt.yticks(fontsize=10)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)

# Legend setup: internal or external (choose one)
plt.legend(fontsize=9, loc='upper right', frameon=False, handlelength=2, borderaxespad=0.5, labelspacing=0.4, ncol=1)
# plt.legend(fontsize=9, bbox_to_anchor=(1.01, 1), loc='upper left', frameon=False)
# plt.tight_layout(rect=[0, 0, 0.85, 1])  # for external legend
plt.tight_layout()

plt.savefig(os.path.join(analysis_dir, "simulation_ablation_attempts_per_scene.pdf"),
            bbox_inches='tight', dpi=300)
plt.show()



# --- Plot 3: Per-Scene Attempt Distribution (1, 2, 3, 4+ attempts) ---
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(8, 4.5))

# Prepare binned data: [1 attempt, 2 attempts, 3 attempts, 4+]
binned_data = defaultdict(lambda: [0, 0, 0, 0])  # scene_id -> [1, 2, 3, 4+]

# Use the baseline file only
baseline_file = "touch_success_baseline.log"
filepath = os.path.join(log_dir, baseline_file)

_, scene_attempts = parse_log_file(filepath)
for scene_id in sorted(scene_attempts.keys()):
    for attempt in scene_attempts[scene_id]:
        if attempt == 1:
            binned_data[scene_id][0] += 1
        elif attempt == 2:
            binned_data[scene_id][1] += 1
        elif attempt == 3:
            binned_data[scene_id][2] += 1
        else:
            binned_data[scene_id][3] += 1

# Convert to DataFrame
scene_ids = sorted(binned_data.keys())
bins_labels = ['1 Attempt', '2 Attempts', '3 Attempts', '4+ Attempts']
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
bar_width = 0.28

# Increase spacing between groups
x = np.arange(len(scene_ids)) * 1.5

data = []

# Plot bars with offsets
for i, (label, color) in enumerate(zip(bins_labels, colors)):
    counts = [binned_data[scene_id][i] for scene_id in scene_ids]
    offset = (i - 1.5) * (bar_width + 0.02)
    data.append((x + offset, counts, bar_width, label, color))
    plt.bar(x + offset, counts, width=bar_width, label=label, color=color)

# Save binned data to CSV
binned_df = pd.DataFrame(
    {f"Scene {scene_id}": binned_data[scene_id] for scene_id in scene_ids},
    index=bins_labels
)
binned_df.to_csv(os.path.join(analysis_dir, "simulation_scene_attempts_distribution_baseline.csv"), index_label="Attempts")

# Axis settings
plt.xlabel("Scene", fontsize=12)
plt.ylabel("Count", fontsize=12)
plt.xticks(x, [str(s) for s in scene_ids], fontsize=10)
plt.yticks(fontsize=10)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)

# Legend below plot
plt.legend(
    title="Attempts",
    fontsize=9,
    title_fontsize=10,
    loc='upper center',
    bbox_to_anchor=(0.5, -0.18),
    ncol=4,
    frameon=False
)
plt.tight_layout()
plt.subplots_adjust(bottom=0.28)

# Save
plt.savefig(os.path.join(analysis_dir, "simulation_scene_attempts_distribution_grouped.pdf"),
            bbox_inches='tight', dpi=300)
plt.show()