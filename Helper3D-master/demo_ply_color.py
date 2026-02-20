import open3d as o3d
import numpy as np
import os

# Define the path to the PLY file
# using the one present in your workspace
ply_file = "robotiq-3f-gripper_articulated.stl"

if not os.path.exists(ply_file):
    print(f"File {ply_file} not found.")
    exit()

print(f"Loading {ply_file}...")

# Load the PLY file as a triangle mesh
mesh = o3d.io.read_triangle_mesh(ply_file)

# Check if mesh loaded correctly
if not mesh.has_vertices():
    print("Failed to load mesh or mesh has no vertices.")
    exit()

# Compute normals for better lighting if they are missing
# if not mesh.has_vertex_normals():
mesh.compute_vertex_normals()

# Paint the mesh uniformly black
# Color is RGB [0, 0, 0] for black
mesh.paint_uniform_color([0.1, 0.1, 0.1])

print("Visualizing mesh in black...")
print("Press 'S' to save a snapshot.")

# Create snapshots folder if it doesn't exist
SNAPSHOT_FOLDER = "snapshots"
if not os.path.exists(SNAPSHOT_FOLDER):
    os.makedirs(SNAPSHOT_FOLDER)

snapshot_counter = 0

def save_snapshot(vis):
    global snapshot_counter
    filename = os.path.join(SNAPSHOT_FOLDER, f"snapshot_{snapshot_counter}.png")
    vis.capture_screen_image(filename, do_render=True)
    print(f"Snapshot saved to {filename}")
    snapshot_counter += 1
    return False

# Initialize visualizer
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name="Black PLY Model", width=800, height=600)
vis.register_key_callback(ord("S"), save_snapshot)
vis.add_geometry(mesh)

# Set background to white so the black model is visible
opt = vis.get_render_option()
opt.background_color = np.asarray([1.0, 1.0, 1.0]) # White background

# Run the visualization
vis.poll_events()
vis.update_renderer()
vis.run()
vis.destroy_window()
