#!/usr/bin/env python
"""Print the top-left and top-right door corner positions of cabinets.

Only the cabinets that were fully successful in simulation (path found,
trajectory successful, contact free and door opened) are considered. Their
ground-truth poses are loaded from a pickle file, transformed into the world
frame, and reported in millimetres. All parameters are read from the
force-manipulation config YAML.
"""

import os
import pickle

import numpy as np

from core.util import read_config, read_csv_DataFrame

CFG_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'cfg', 'door_replanning_control_multi_contact.yaml'
)


def load_gt_poses(base_dir, is_offline):
    """Load all ground-truth cabinet poses from the pickle file."""
    filename = 'offline_cabinet_gt_poses.pkl' if is_offline else 'cabinet_gt_poses.pkl'
    poses_path = os.path.join(base_dir, 'gt_cabinets', filename)
    with open(poses_path, 'rb') as f:
        return pickle.load(f)


def successful_cabinet_indices(results_path):
    """Return the indices of cabinets that succeeded on every metric in sim."""
    data = read_csv_DataFrame(results_path)
    success_data = data.loc[((data['path_found'] == True) &
                             (data['traj_success'] == True) &
                             (data['contact_free'] == True) &
                             (data['door_opened'] == True))]
    return success_data['idx'].astype(int).tolist()


def door_corner_in_world(T_A_W, door_thickness, height, y_offset):
    """Return a top door corner in the world frame, in millimetres."""
    T_corner_A = np.eye(4)
    T_corner_A[:3, 3] = np.array([-door_thickness, y_offset, height * 0.5])
    T_corner_W = T_A_W @ T_corner_A
    T_corner_W[:3, 3] *= 1000.0  # m -> mm
    return T_corner_W[:3, 3]


def main():
    config = read_config(os.path.abspath(CFG_PATH))

    is_offline = config['is_offline']
    door_thickness = config['door_thickness']
    width = config['gt_width']
    height = config['gt_height']
    # The gt_cabinets folder lives next to the simulation directory.
    base_dir = os.path.dirname(config['simulation_dir'])

    gt_poses = load_gt_poses(base_dir, is_offline)
    cabinet_indices = successful_cabinet_indices(config['multi_contact_simulation_results_path'])

    for cabinet_idx in cabinet_indices:
        T_A_W = gt_poses[cabinet_idx][0]

        top_left = door_corner_in_world(T_A_W, door_thickness, height, y_offset=0.0)
        top_right = door_corner_in_world(T_A_W, door_thickness, height, y_offset=-width)

        print('Pose %d - Topleft corner: %.2f %.2f %.2f'
              % (cabinet_idx, -top_left[0], -top_left[1], top_left[2]))
        print('Pose %d - Topright corner: %.2f %.2f %.2f'
              % (cabinet_idx, -top_right[0], -top_right[1], top_right[2]))
        print('')


if __name__ == '__main__':
    main()
