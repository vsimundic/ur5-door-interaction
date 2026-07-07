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

# --- All parameter logs combined ---
all_param_logs = {
    "touch_success_baseline.log": r"Baseline",
    # "touch_success_stdgxyk_0.02.log": r"$\sigma_{g,xy} = 2 \%$",
    "touch_success_stdgxyk_0.1.log": r"$\sigma_{g,xy} = 10 \%$",
    # "touch_success_stdgzk_0.01.log": r"$\sigma_{g,z} = 1 \%$",
    # "touch_success_stdgzk_0.05.log": r"$\sigma_{g,z} = 5 \% $",
    # "touch_success_stdphi_1.0.log": r"$\sigma_{\varphi} = 1^\circ$",
    "touch_success_stdphi_3.0.log": r"$\sigma_{\varphi} = 3^\circ$",
    # "touch_success_vertexDistThr_0.05.log": r"$d_{vtx} = 5 \ cm$",
    "touch_success_vertexDistThr_0.06.log": r"$d_{vtx} = 6 \ cm$",
    "touch_success_alpha_0.0001.log": r"$\alpha = 10^{-4}$",
    # "touch_success_alpha_0.00001.log": r"$\alpha = 10^{-5}$",
    # "touch_success_alpha_0.0000001.log": r"$\alpha = 10^{-7}$",
    "touch_success_nbest_0.log": r"$N_{best} = 0$",
    # "touch_success_nbest_50.log": r"$N_{best} = 50$",
    # "touch_success_nbest_100.log": r"$N_{best} = 100$",
    # "touch_success_nbest_750.log": r"$N_{best} = 750$",
}

def parse_log_file(filepath):
    attempt_counts = []
    with open(filepath, "r") as f:
        for line in f:
            match = re.match(r"Simulation\s+\d+,\s+attempts\s+(\d+),\s+(\w+)", line.strip())
            if match:
                attempt = int(match.group(1))
                success = match.group(2).lower() == "success"
                if success:
                    attempt_counts.append(attempt)
    return attempt_counts

def parse_log_file_prev(filepath):
    attempt_counts = []
    scene_attempts = defaultdict(list)
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            match = re.match(r"Simulation\s+(\d+),\s+attempts\s+(\d+),\s+(\w+)", line.strip())
            if match:
                scene_id = i % 20
                attempts = int(match.group(2))
                attempt_counts.append(attempts)
                scene_attempts[scene_id].append(attempts)
    return attempt_counts, scene_attempts

def parse_scene_attempts(filepath):
    scene_attempts = defaultdict(list)
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            match = re.match(r"Simulation\s+(\d+),\s+attempts\s+(\d+),\s+(\w+)", line.strip())
            if match:
                scene_id = i % 20
                attempts = int(match.group(2))
                scene_attempts[scene_id].append(attempts)
    return scene_attempts

# --- Plot 1: Histogram of attempts ---
plt.style.use('seaborn-v0_8-whitegrid')
fig, ax = plt.subplots(figsize=(14, 6))
bins = list(range(1, 9))
width = 0.05

x = np.arange(len(bins))
offsets = np.linspace(-width * len(all_param_logs)/2, width * len(all_param_logs)/2, len(all_param_logs))

all_freqs = {}
for i, (filename, label) in enumerate(all_param_logs.items()):
    filepath = os.path.join(log_dir, filename)
    if not os.path.isfile(filepath):
        continue
    attempts = parse_log_file(filepath)
    freq = pd.Series(attempts).value_counts().reindex(bins, fill_value=0).sort_index()
    all_freqs[label] = freq
    ax.bar(x + offsets[i], freq.values, width=width, label=label, alpha=0.9)

all_freqs_df = pd.DataFrame(all_freqs)
all_freqs_df.to_csv(os.path.join(analysis_dir, "all_parameters_attempts.csv"), index_label="Number of Actions")

ax.set_xlabel("Number of Actions", fontsize=12)
ax.set_ylabel("Count", fontsize=12)
ax.set_title("All Parameter Configurations", fontsize=14)
ax.set_xticks(x)
ax.set_xticklabels(bins, fontsize=10)
ax.legend(fontsize=11, frameon=False, loc='upper right', ncol=2)
ax.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)

plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, "simulation_ablation_all_params_histogram_grouped.png"),
            bbox_inches='tight', dpi=300)
plt.show()

# --- Plot 2: Average attempts per scene ---
plt.style.use('seaborn-v0_8-whitegrid')
fig, ax = plt.subplots(figsize=(10, 6))

scene_data = []
for filename, label in all_param_logs.items():
    filepath = os.path.join(log_dir, filename)
    if not os.path.isfile(filepath):
        continue
    scene_attempts = parse_scene_attempts(filepath)
    for scene_id in sorted(scene_attempts.keys()):
        avg = np.mean(scene_attempts[scene_id])
        scene_data.append({
            "Scene": scene_id + 1,
            "Average Actions": avg,
            "Configuration": label
        })

df_all = pd.DataFrame(scene_data)
df_all.to_csv(os.path.join(analysis_dir, "all_parameters_average_attempts_per_scene.csv"), index=False)
sns.lineplot(data=df_all, x="Scene", y="Average Actions", hue="Configuration",
             marker="o", linewidth=2, ax=ax)
ax.set_xlabel("Scene", fontsize=12)
ax.set_ylabel("Average Number of Actions", fontsize=12)
ax.set_title("All Parameter Configurations", fontsize=14)
ax.set_xticks(range(1, 21))
ax.tick_params(axis='x', labelsize=10)
ax.tick_params(axis='y', labelsize=10)
ax.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)
ax.legend(fontsize=11, frameon=False, loc='upper right', ncol=2)

plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, "simulation_ablation_all_params_per_scene_grouped.png"),
            bbox_inches='tight', dpi=300)
plt.show()

# --- Plot 3: Per-Scene Attempt Distribution (Baseline) ---
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(8, 4.5))

binned_data = defaultdict(lambda: [0, 0, 0, 0])
baseline_file = "touch_success_baseline.log"
filepath = os.path.join(log_dir, baseline_file)

_, scene_attempts = parse_log_file_prev(filepath)
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

scene_ids = sorted(binned_data.keys())
bins_labels = ['1 Action', '2 Actions', '3 Actions', '4+ Actions']
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
bar_width = 0.28
x = np.arange(len(scene_ids)) * 1.5

for i, (label, color) in enumerate(zip(bins_labels, colors)):
    counts = [binned_data[scene_id][i] for scene_id in scene_ids]
    offset = (i - 1.5) * (bar_width + 0.02)
    plt.bar(x + offset, counts, width=bar_width, label=label, color=color)

binned_df = pd.DataFrame(
    {f"Scene {scene_id+1}": binned_data[scene_id] for scene_id in scene_ids},
    index=bins_labels
)
binned_df.to_csv(os.path.join(analysis_dir, "simulation_scene_attempts_distribution_baseline.csv"), index_label="Attempts")

plt.xlabel("Scene", fontsize=12)
plt.ylabel("Count", fontsize=12)
plt.xticks(x, [str(s+1) for s in scene_ids], fontsize=10)
plt.yticks(fontsize=10)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)
plt.legend(fontsize=11, title_fontsize=11, loc='upper center',
           bbox_to_anchor=(0.5, -0.18), ncol=4, frameon=False)
plt.tight_layout()
plt.subplots_adjust(bottom=0.28)
plt.savefig(os.path.join(analysis_dir, "simulation_scene_attempts_distribution_grouped.png"),
            bbox_inches='tight', dpi=300)
plt.show()