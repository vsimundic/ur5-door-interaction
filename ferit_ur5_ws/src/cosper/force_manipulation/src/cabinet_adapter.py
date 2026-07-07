#!/usr/bin/python
"""Adapter exposing the Cabinet2 API on top of the (TSMC24) Cabinet model.

The door-replanning FSM was written against `Cabinet2` (cabinet_model2.py): it
reads `T_A_W`, `T_D_Arot`, `sx/sy/sz`, `side`, `static_mesh`, etc. The TSMC24
`Cabinet` (cabinet_model.py) models the same cabinet but with different
attribute names and frame conventions.

This adapter subclasses `Cabinet` (so the actual geometry, URDF and meshes come
from Cabinet) and adds the `Cabinet2`-style members the FSM needs. The frame
mapping is exact: in `Cabinet` the world door point at door angle theta is

    T_A_S @ rot_z(theta) @ T_D_A_init

which is the same composition the FSM performs as

    T_A_W @ T_Arot_A @ T_D_Arot

so `T_A_W := T_A_S` and `T_D_Arot := T_D_A_init` keep planning consistent with
the spawned Cabinet model. It is used only by this FSM; cabinet_model.py is
left untouched.
"""

import numpy as np

from gazebo_push_open.cabinet_model import Cabinet


class CabinetAdapter(Cabinet):
    def __init__(self,
                 s: np.ndarray = np.array([0.018, 0.4, 0.5, 0.4]),  # [sx, sy, sz, static_d]
                 r: np.ndarray = np.array([0, 0]),
                 axis_pos: float = -1.0,
                 T_A_W: np.ndarray = np.eye(4),
                 save_path: str = None,
                 has_handle: bool = False,
                 static_side_width: float = 0.018,
                 axis_distance: float = 0.0,
                 moving_to_static_part_distance: float = None):
        # Cabinet2 's' is [sx, sy, sz, static_d] = [thickness, width, height, depth];
        # Cabinet 'door_params' is [w_door, h_door, d_door, static_d] = [width, height, thickness, depth].
        door_params = np.array([s[1], s[2], s[0], s[3]])
        axis_pos_int = int(np.sign(axis_pos)) if axis_pos != 0 else -1

        super().__init__(door_params=door_params,
                         r=r,
                         axis_pos=axis_pos_int,
                         T_A_S=T_A_W,
                         save_path=save_path,
                         has_handle=has_handle,
                         static_side_width=static_side_width,
                         axis_distance=axis_distance)

        # Cabinet hardcodes moving_to_static_part_distance; honor the FSM's value
        # by rebuilding the geometry that depends on it.
        if (moving_to_static_part_distance is not None
                and moving_to_static_part_distance != self.moving_to_static_part_distance):
            self.moving_to_static_part_distance = moving_to_static_part_distance
            self.setup_matrices()
            if self.save_path is not None:
                self.xml_model = self.generate_cabinet_urdf_from_door_panel()
            self.mesh = self.create_mesh()

        # Door-point transform in the axis frame at the closed door (Cabinet2 name).
        # Settable by the FSM (e.g. RVL correction via get_corrected_pose_D_Arot).
        self.T_D_Arot = self.T_D_A_init.copy()

    # --- Cabinet2-style frame aliases -------------------------------------
    @property
    def T_A_W(self):
        return self.T_A_S

    @T_A_W.setter
    def T_A_W(self, value):
        self.T_A_S = value

    @property
    def T_O_W(self):
        return self.T_O_S

    @T_O_W.setter
    def T_O_W(self, value):
        self.T_O_S = value

    # --- Cabinet2-style dimension/attribute aliases -----------------------
    @property
    def sx(self):  # door thickness
        return self.d_door

    @property
    def sy(self):  # door width
        return self.w_door

    @property
    def sz(self):  # door height
        return self.h_door

    @property
    def side(self):
        return self.static_side_width

    @side.setter
    def side(self, value):
        self.static_side_width = value

    @property
    def static_mesh(self):
        return self.dd_static_mesh

    @static_mesh.setter
    def static_mesh(self, value):
        self.dd_static_mesh = value
