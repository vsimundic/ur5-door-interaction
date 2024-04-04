import numpy as np
import open3d as open3d

def rgb(r, g, b):
    return [r/255., g/255., b/255.]

colors = [rgb(70, 25, 89),
          rgb(122, 49, 111),
          rgb(205, 102, 136),
          rgb(174, 216, 204),
          rgb(0, 121, 255),
          rgb(0, 223, 162),
          rgb(246, 250, 112),
          rgb(255, 0, 96),
          rgb(111, 56, 197),
          rgb(135, 162, 251),
          rgb(173, 221, 208),
          rgb(63, 167, 150),
          rgb(254, 194, 96),
          rgb(161, 0, 53)]


vis = open3d.visualization.Visualizer()
vis.create_window()
view_control = vis.get_view_control()
# Set the saved view matrix
parameters = open3d.io.read_pinhole_camera_parameters('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/o3d_camera_params.json')
view_control.convert_from_pinhole_camera_parameters(parameters, True)


# Load models
geometries = []
tool_mesh = open3d.io.read_triangle_mesh('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply')
tool_mesh.compute_vertex_normals()
# geometries.append(tool_mesh)

spheres = np.load('/home/RVLuser/rvl-linux/data/Robotiq3Finger/spheres.npy')
spheres /= 1000.
geometries = []
for i in range(spheres.shape[0]):
    sphere = open3d.geometry.TriangleMesh.create_sphere(radius=spheres[i, 3])
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color([0.5, 0.5, 0.5])
    vec = np.array(spheres[i, 0:3]).ravel()

    sphere.translate(vec)
    geometries.append(sphere)

# Add models to the visualizer
for geom in geometries:
    vis.add_geometry(geom)

for i_color, color in enumerate(colors):

    for sphere in geometries:
        sphere.paint_uniform_color(color)    
        vis.update_geometry(sphere)


    view_control.convert_from_pinhole_camera_parameters(parameters, True)
    vis.update_renderer()
    vis.poll_events()
    vis.capture_screen_image('/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/spheres_%d.png' % i_color, True)
    vis.run()


    # Destroy the visualizer window
    # vis.destroy_window()

    # del vis