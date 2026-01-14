#!/usr/bin/python3

import os
import numpy as np
import matplotlib.pyplot as plt
import csv
import yaml

def get_matrix(base, size, row_, col_idx_):
    start = col_idx_[base]
    return np.array(row_[start:start + size], dtype=np.float64)

def compute_column_indices(header):
    col_idx = {}
    idx = 0
    for name in header:
        if name in {"R_A_C", "R_C_E", "R_E_0", 'R_A_S', 'R_Ek_E'}:
            col_idx[name] = idx
            idx += 9
        elif name in {"t_A_C", "t_C_E", "t_E_0", 't_A_S', 't_Ek_E', 'V'}:
            col_idx[name] = idx
            idx += 3
        else:
            col_idx[name] = idx
            idx += 1
    return col_idx

def load_final_Dgt_percent_per_scene(touches_path, analysis_path):
    # Load touches
    with open(touches_path, 'r') as f_touch:
        reader_touch = csv.reader(f_touch)
        header_touch = next(reader_touch)
        col_idx_touch = compute_column_indices(header_touch)
        touch_rows = list(reader_touch)

    # Load analysis
    with open(analysis_path, 'r') as f_analysis:
        reader_analysis = csv.reader(f_analysis)
        header_analysis = next(reader_analysis)
        col_idx_analysis = compute_column_indices(header_analysis)
        analysis_rows = list(reader_analysis)
        Dgt_percent = [float(row[col_idx_analysis['Dgt_percent']]) * 100 for row in analysis_rows]  # Convert to %

    # Get last attempt index per scene
    scene_to_last_attempt_idx = {}
    for i, row in enumerate(touch_rows):
        scene_idx = int(row[col_idx_touch['scene_idx']])
        scene_to_last_attempt_idx[scene_idx] = i  # overwritten each time to get last

    final_Dgt = [Dgt_percent[i] for i in scene_to_last_attempt_idx.values()]
    return final_Dgt


def load_final_D_dists_per_scene(touches_path, analysis_path):
    # Load touches
    with open(touches_path, 'r') as f_touch:
        reader_touch = csv.reader(f_touch)
        header_touch = next(reader_touch)
        col_idx_touch = compute_column_indices(header_touch)
        touch_rows = list(reader_touch)

    # Load analysis results
    with open(analysis_path, 'r') as f_analysis:
        reader_analysis = csv.reader(f_analysis)
        header_analysis = next(reader_analysis)
        col_idx_analysis = compute_column_indices(header_analysis)
        analysis_rows = list(reader_analysis)
        D_dists = [float(row[col_idx_analysis['D_dist']]) * 100 for row in analysis_rows]  # Convert to cm

    # Get the last D_dist per scene
    scene_to_last_attempt_idx = {}
    for i, row in enumerate(touch_rows):
        scene_idx = int(row[col_idx_touch['scene_idx']])
        scene_to_last_attempt_idx[scene_idx] = i  # overwritten each time to get last

    final_D_dists = [D_dists[i] for i in scene_to_last_attempt_idx.values()]
    return final_D_dists


# Load config
script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '../cfg/door_replanning_control.yaml'), 'r') as f:
    config = yaml.safe_load(f)

modes = ['offline', 'online']

offline_cabinet_filename = config["offline_cabinet_filename"]
online_cabinet_filename = config["online_cabinet_filename"]
offline_touches_filename = config["offline_touches_filename"]
online_touches_filename = config["online_touches_filename"]
analysis_dir = config["analysis_dir"]

# Count scenes
with open(offline_cabinet_filename, 'r') as f:
    n_offline = sum(1 for _ in f) - 1
with open(online_cabinet_filename, 'r') as f:
    n_online = sum(1 for _ in f) - 1

# Count attempts per scene
def count_attempts(filename, n_scenes):
    attempts_per_scene = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        col_idx = compute_column_indices(header)
        rows = list(reader)
        for i_scene in range(n_scenes):
            scene_attempts = [row for row in rows if int(row[col_idx['scene_idx']]) == i_scene]
            attempts_per_scene.append(len(scene_attempts))
    return attempts_per_scene

offline_num_attempts_per_scene = count_attempts(offline_touches_filename, n_offline)
online_num_attempts_per_scene = count_attempts(online_touches_filename, n_online)

# Pad for alignment
max_length = max(len(offline_num_attempts_per_scene), len(online_num_attempts_per_scene))
offline_num_attempts_per_scene = [None] * (max_length - len(offline_num_attempts_per_scene)) + offline_num_attempts_per_scene
online_num_attempts_per_scene = [None] * (max_length - len(online_num_attempts_per_scene)) + online_num_attempts_per_scene
offline_calib_attempts = sum(x for x in offline_num_attempts_per_scene[:3] if x is not None)

# ----------- Plot 1: Attempts per Scene ----------- #
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(6.5, 4))

plt.plot(offline_num_attempts_per_scene, label='W/ Offline Calibration', color='tab:blue', linewidth=2, marker='o', markersize=5)
plt.plot(online_num_attempts_per_scene, label='W/o Offline Calibration', color='tab:orange', linewidth=2, marker='s', markersize=5)
plt.xlabel('Scene', fontsize=12)
plt.ylabel('Number of Actions', fontsize=12)
plt.xticks(range(max_length), fontsize=10)
plt.yticks(fontsize=10)
plt.axvline(x=3 - 0.5, color='gray', linestyle='--', linewidth=1.2, label='Offline Calibration End')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(loc='upper right', fontsize=11, frameon=False)
plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, 'comparison_num_attempts_per_scene.pdf'), bbox_inches='tight', dpi=300)
plt.show()


# Plotting
offline_analysis_path = os.path.join(analysis_dir, 'offline_analysis_results_final.csv')
online_analysis_path = os.path.join(analysis_dir, 'online_analysis_results_final.csv')
offline_analysis_path_per_scene = os.path.join(analysis_dir, 'offline_analysis_results_per_scene_final.csv')
online_analysis_path_per_scene = os.path.join(analysis_dir, 'online_analysis_results_per_scene_final.csv')

with open(offline_analysis_path_per_scene, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    offline_D_dists_per_scene = [float(row[header.index('D_dist')]) * 100. for row in reader]
with open(online_analysis_path_per_scene, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    online_D_dists_per_scene = [float(row[header.index('D_dist')]) * 100. for row in reader]

offline_final_D_dists = offline_D_dists_per_scene
online_final_D_dists = online_D_dists_per_scene

# offline_final_D_dists = load_final_D_dists_per_scene(offline_touches_filename, offline_analysis_path)
# online_final_D_dists = load_final_D_dists_per_scene(online_touches_filename, online_analysis_path)
# Calculate offset
max_len = max(len(offline_final_D_dists), len(online_final_D_dists))
online_offset = max_len - len(online_final_D_dists)

# Create x-axis ranges
offline_x = list(range(max_len))
online_x = list(range(online_offset, max_len))

# Plot D distances
plt.style.use('seaborn-v0_8-whitegrid')
plt.figure(figsize=(6.5, 4))

plt.plot(offline_x, offline_final_D_dists,
         label='W/ Offline Calibration',
         marker='o', linewidth=2, color='tab:blue')

plt.plot(online_x, online_final_D_dists,
         label='W/o Offline Calibration',
         marker='s', linewidth=2, color='tab:orange')
plt.axvline(x=3 - 0.5, color='gray', linestyle='--', linewidth=1.2, label='Offline Calibration End')

plt.xlabel('Scene', fontsize=12)
plt.ylabel('Distance to Ground Truth Back Panel [cm]', fontsize=12)
plt.xticks(range(max_len), fontsize=10)
plt.yticks(fontsize=10)
plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)
plt.legend(fontsize=11, loc='upper right', frameon=False)

plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, 'comparison_final_D_dist_per_scene.pdf'),
            bbox_inches='tight', dpi=300)
plt.show()


# Load data
offline_final_Dgt = load_final_Dgt_percent_per_scene(offline_touches_filename, offline_analysis_path)
online_final_Dgt = load_final_Dgt_percent_per_scene(online_touches_filename, online_analysis_path)

# Align lengths
max_len = max(len(offline_final_Dgt), len(online_final_Dgt))
online_offset = max_len - len(online_final_Dgt)

offline_x = list(range(max_len))
online_x = list(range(online_offset, max_len))

# Plot Planning Points Match percentages
plt.style.use('seaborn-v0_8-whitegrid')  # Avoid deprecation warning
plt.figure(figsize=(6.5, 4))

plt.plot(offline_x, offline_final_Dgt,
         label='W/ Offline Calibration',
         marker='o', linewidth=2, color='tab:blue')

plt.plot(online_x, online_final_Dgt,
         label='W/o Offline Calibration',
         marker='s', linewidth=2, color='tab:orange')

plt.xlabel('Scene', fontsize=12)
plt.ylabel('Planning Points Match [%]', fontsize=12)
plt.xticks(range(max_len), fontsize=10)
plt.yticks(np.arange(0, 105, 20), fontsize=10)

plt.grid(True, linestyle='--', linewidth=0.6, alpha=0.7)
plt.legend(fontsize=11, loc='lower right', frameon=False)

plt.tight_layout()
plt.savefig(os.path.join(analysis_dir, 'comparison_final_planning_points_match_per_scene.pdf'),
            bbox_inches='tight', dpi=300)
plt.show()


# ----------- Plots 2–4: Per-Attempt Metrics ----------- #
for mode in modes:
    chamfer_dists, D_dists, Dgt_nums = [], [], []
    analysis_path = os.path.join(analysis_dir, f'{mode}_analysis_results_final.csv')

    with open(analysis_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        col_idx = compute_column_indices(header)
        for row in reader:
            chamfer_dists.append(float(row[col_idx['chamfer_dist']]) * 100)
            D_dists.append(float(row[col_idx['D_dist']]) * 100)
            Dgt_nums.append(float(row[col_idx['Dgt_percent']]) * 100)

    # Chamfer Distance Plot
    plt.figure(figsize=(6.5, 4))
    plt.plot(chamfer_dists, color='tab:blue', linewidth=2, marker='o', markersize=4, label='Chamfer Distance')
    if mode == 'offline':
        plt.axvline(x=offline_calib_attempts - 0.5, color='gray', linestyle='--', linewidth=1.5, label='Offline Calibration End')
    plt.xlabel('Attempt', fontsize=12)
    plt.ylabel('Chamfer Distance [cm]', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    if mode == 'offline':
        plt.legend(fontsize=11, frameon=False)
    plt.tight_layout()
    plt.savefig(os.path.join(analysis_dir, f'{mode}_chamfer_distances.pdf'), bbox_inches='tight', dpi=300)
    plt.show()

    # D Distance Plot
    plt.figure(figsize=(6.5, 4))
    plt.plot(D_dists, color='tab:green', linewidth=2, marker='s', markersize=4, label='Distance')
    if mode == 'offline':
        plt.axvline(x=offline_calib_attempts - 0.5, color='gray', linestyle='--', linewidth=1.5, label='Offline Calibration End')
    plt.xlabel('Attempt', fontsize=12)
    plt.ylabel('Distance to Ground Truth Back Panel [cm]', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    if mode == 'offline':
        plt.legend(fontsize=11, frameon=False)
    plt.tight_layout()
    plt.savefig(os.path.join(analysis_dir, f'{mode}_D_distances.pdf'), bbox_inches='tight', dpi=300)
    plt.show()

    # Dgt % Plot
    plt.figure(figsize=(6.5, 4))
    plt.plot(Dgt_nums, color='tab:red', linewidth=2, marker='^', markersize=4, label='Tool Points Behind Door')
    if mode == 'offline':
        plt.axvline(x=offline_calib_attempts - 0.5, color='gray', linestyle='--', linewidth=1.5, label='Offline Calibration End')
    plt.xlabel('Attempt', fontsize=12)
    plt.ylabel('Points Behind Door Surface [%]', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    if mode == 'offline':
        plt.legend(fontsize=11, frameon=False)
    plt.tight_layout()
    plt.savefig(os.path.join(analysis_dir, f'{mode}_back_door_points.pdf'), bbox_inches='tight', dpi=300)
    plt.show()