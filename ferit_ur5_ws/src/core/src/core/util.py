import yaml
import rospkg
import os
import pandas as pd
from pandas.errors import EmptyDataError
import numpy as np

def read_config(cfg_filename: str):
    rp = rospkg.RosPack()
    # demo_pkg_path = rp.get_path('a_demo')
    # cfg_filepath = os.path.join(demo_pkg_path, 'config', cfg_filename)
    
    if not os.path.isfile(cfg_filename):
        raise Exception('The file does not exist.')
    if not cfg_filename.endswith('.yaml'):
        raise Exception('A config file must end with .yaml.')
    
    try:
        with open(cfg_filename, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except:
        raise Exception('Config must be a valid YAML file.')
    
    return config

def read_csv_DataFrame(path:str, separator:str=',') -> pd.DataFrame:
    try:
        df = pd.read_csv(filepath_or_buffer=path, sep=separator, header=0)
    except EmptyDataError as e:
        return None
    return df

def sample_unit_sphere_angles(n_samples):
    """
    Alternative method using spherical coordinates.
    """
    np.random.seed(0)  
    theta = np.random.uniform(0, 2*np.pi, n_samples)  # azimuth
    phi = np.arccos(np.random.uniform(-1, 1, n_samples))  # inclination
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=1)

def angular_difference(q1, q2):
    return np.arctan2(np.sin(q1 - q2), np.cos(q1 - q2))

def get_nearest_joints(candidates, current):
    diffs = angular_difference(candidates, current)
    dists = np.linalg.norm(diffs, axis=1)
    closest_index = np.argmin(dists)
    return candidates[closest_index], closest_index

def get_nearest_joints_with_dist(candidates, current):
    diffs = angular_difference(candidates, current)
    dists = np.linalg.norm(diffs, axis=1)
    closest_index = np.argmin(dists)
    return candidates[closest_index], closest_index, dists[closest_index]

def get_nearest_joints_pair_indices(joint_array1, joint_array2):
    """
    Find the indices of the closest pairs of joints between two arrays.
    
    Args:
        joint_array1 (np.ndarray): First array of joint angles.
        joint_array2 (np.ndarray): Second array of joint angles.
        
    Returns:
        np.ndarray: Indices of the closest pairs of joints.
    """
    indices = []
    dists = []
    for i, joint1 in enumerate(joint_array1):
        _, index, dist = get_nearest_joints_with_dist(joint_array2, joint1)
        indices.append((i, index))
        dists.append(dist)
    idx = np.argmin(dists)
    return indices[idx]


def furthest_point_sampling(points, num_samples):
    N = points.shape[0]
    sampled_indices = []
    distances = np.full(N, np.inf)

    idx = np.random.randint(0, N)
    sampled_indices.append(idx)

    for _ in range(1, num_samples):
        current_point = points[idx]
        dist = np.linalg.norm(points - current_point, axis=1)
        distances = np.minimum(distances, dist)

        idx = np.argmax(distances)
        sampled_indices.append(idx)

    return sampled_indices


def chamfer_distance(A: np.ndarray, B: np.ndarray, squared: bool = False) -> float:
    """
    Chamfer Distance between two point clouds A (n,3) and B (m,3).

    CD(A,B) = mean_{a in A} min_{b in B} d(a,b) + mean_{b in B} min_{a in A} d(b,a)
    where d is squared Euclidean if squared=True, else Euclidean.

    Args:
        A: (n, 3) float array
        B: (m, 3) float array
        squared: use squared L2 if True (common in literature), else plain L2

    Returns:
        float: symmetric Chamfer distance.
    """
    
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    if A.ndim != 2 or B.ndim != 2 or A.shape[1] != 3 or B.shape[1] != 3:
        raise ValueError("A and B must be shape (n,3) and (m,3).")
    if len(A) == 0 or len(B) == 0:
        raise ValueError("A and B must be non-empty.")

    # Fast path with SciPy KDTree if available
    try:
        from scipy.spatial import cKDTree
        kdt_B = cKDTree(B)
        d_AB, _ = kdt_B.query(A, k=1)  # distances from each A to nearest B

        kdt_A = cKDTree(A)
        d_BA, _ = kdt_A.query(B, k=1)  # distances from each B to nearest A

        if squared:
            d_AB = d_AB**2
            d_BA = d_BA**2

        return float(d_AB.mean() + d_BA.mean())

    except Exception:
        # Fallback (O(n*m) memory/time) – fine for small point sets
        # Pairwise squared distances: ||A||^2 + ||B||^2 - 2 A·B^T
        AA = np.sum(A**2, axis=1, keepdims=True)           # (n,1)
        BB = np.sum(B**2, axis=1, keepdims=True).T         # (1,m)
        D2 = AA + BB - 2.0 * (A @ B.T)                     # (n,m)
        # Numerical safety
        np.maximum(D2, 0.0, out=D2)

        if squared:
            d_AB_min = D2.min(axis=1)                      # (n,)
            d_BA_min = D2.min(axis=0)                      # (m,)
        else:
            D = np.sqrt(D2, dtype=np.float64)              # (n,m)
            d_AB_min = D.min(axis=1)
            d_BA_min = D.min(axis=0)

        return float(d_AB_min.mean() + d_BA_min.mean())
