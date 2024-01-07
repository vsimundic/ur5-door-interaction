import numpy as np

def rot_x(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    T = np.array([[1, 0, 0],
                  [0, c, -s],
                  [0, s, c]])
    

def rot_y(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    T = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    T = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])

