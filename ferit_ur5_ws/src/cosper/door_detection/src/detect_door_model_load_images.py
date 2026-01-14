#!/usr/bin/env python

import os
import shutil
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rosnode import kill_nodes
from PIL import Image as PilImage
import RVLPYRGBD2PLY as rvl
import RVLPYDDDetector
import json
from core.real_ur5_controller import UR5Controller

# ---------------- Globals ----------------
number_of_images = 0
key_counter = 0
last_save_time = 0
max_save_rate = 5.0  # Hz

base_dir = ""
rgb_dir = ""
depth_dir = ""
ply_dir = ""
model_output_path = ""
detector_config_path = ""
rgb_topic = ""
depth_topic = ""
node_name = "rgbAndDepth_image_subscriber"

# ------------- Helpers -------------------
def convert_numpy(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    else:
        return obj

def stop_node(node_name):
    kill_nodes([node_name])

def ensure_directories_exist(*dirs):
    for d in dirs:
        if not os.path.exists(d):
            os.makedirs(d)
            rospy.loginfo(f"Created directory: {d}")

def reset_directories(*dirs):
    for d in dirs:
        if os.path.exists(d):
            shutil.rmtree(d)
            rospy.loginfo(f"Cleared directory: {d}")
        os.makedirs(d, exist_ok=True)

def list_numbered_files(dir_path, ext):
    """
    Returns a list of (idx, fullpath) for files named like 0000.ext, 0001.ext, ...
    Sorted by idx ascending.
    """
    files = []
    if not os.path.isdir(dir_path):
        return files
    for name in os.listdir(dir_path):
        if not name.lower().endswith("." + ext.lower()):
            continue
        stem = os.path.splitext(name)[0]
        if stem.isdigit():
            files.append((int(stem), os.path.join(dir_path, name)))
    files.sort(key=lambda x: x[0])
    return files

# ------------- Capture Callbacks ----------
def image_callback(rgb_msg, depth_msg):
    global number_of_images, key_counter, last_save_time
    global base_dir, rgb_dir, depth_dir, max_save_rate

    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

    cv2.imshow("RGB", rgb)
    key = cv2.waitKey(1)

    if key == 32:  # Space bar
        key_counter += 1

    if key_counter == 1:
        now = rospy.get_time()
        if now - last_save_time < 1.0 / max_save_rate:
            return  # limit save rate

        last_save_time = now

        file_idx = str(number_of_images).zfill(4)
        rgb_path = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_path = os.path.join(base_dir, depth_dir, f"{file_idx}.png")

        cv2.imwrite(rgb_path, rgb)
        cv2.imwrite(depth_path, depth)
        number_of_images += 1
        rospy.loginfo(f"Saved image {file_idx}")

    if key_counter >= 2:
        cv2.destroyAllWindows()
        create_ply_images(number_of_images)
        stop_node(node_name)

# ------------- PLY Generation -------------
def create_ply_images(count):
    global base_dir, rgb_dir, depth_dir, ply_dir

    rospy.loginfo("Creating PLY files...")
    rgbd2ply = rvl.RGBD2PLY()

    for i in range(count):
        file_idx = str(i).zfill(4)
        rgb_file = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_file = os.path.join(base_dir, depth_dir, f"{file_idx}.png")
        ply_file = os.path.join(base_dir, ply_dir, f"{file_idx}.ply")

        if not (os.path.isfile(rgb_file) and os.path.isfile(depth_file)):
            rospy.logwarn(f"Missing RGB/DEPTH for index {file_idx}, skipping PLY.")
            continue

        rgb_img = PilImage.open(rgb_file)
        rgb_data = np.array(rgb_img.getdata()).astype(np.byte)
        bgr_array = np.stack((rgb_data[:, 2], rgb_data[:, 1], rgb_data[:, 0]), axis=1)

        depth_img = PilImage.open(depth_file)
        depth_array = np.array(depth_img.getdata()).astype(np.short)

        rospy.loginfo(f"Converting to PLY: {file_idx}")
        rgbd2ply.pixel_array_to_ply(
            bgr_array,
            depth_array,
            597.9033203125, 598.47998046875,   # fx, fy
            323.8436584472656, 236.32774353027344,  # cx, cy
            640, 480,  # width, height
            0.050,     # depth scale (example)
            ply_file
        )

    rospy.loginfo("PLY generation complete.")

# ------------- Detection ------------------
def build_model_from_paths(rgb_files, ply_files, rgb_detected_path, robot):
    """
    rgb_files: list of full RGB image paths (strings)
    ply_files: list of full PLY mesh paths (strings)
    """
    global base_dir, model_output_path, detector_config_path

    rospy.loginfo("Starting 3D detection...")
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(detector_config_path)

    rospy.loginfo("Loading meshes:")
    for mesh_file in ply_files:
        rospy.loginfo(f"  {mesh_file}")
        detector.add_mesh(mesh_file)

    rospy.loginfo("Loading RGB images:")
    for rgb_file in rgb_files:
        rospy.loginfo(f"  {rgb_file}")
        detector.add_rgb(rgb_file)

    detector.set_hyp_file_name(os.path.join(base_dir, "hyps.txt"))
    detector.set_rgb_detected_dir(rgb_detected_path)
    ao_result = detector.detect()

    # Robot state for logging/calibration
    current_joints = robot.get_current_joint_values()
    ao_result["joint_values"] = current_joints

    T_6_0 = robot.get_current_tool_pose()
    T_C_6 = np.load('/home/RVLuser/ferit_ur5_ws/data/camera_calibration_20250331/T_C_T.npy')

    R_A_C = np.array(ao_result['R']).reshape(3, 3)
    t_A_C = np.array(ao_result['t'])

    T_A_C = np.eye(4)
    T_A_C[:3, :3] = R_A_C
    T_A_C[:3, 3] = t_A_C

    T_A_0 = T_6_0 @ T_C_6 @ T_A_C
    print("T_A_0:\n", T_A_0)

    # Save as JSON
    ao_json = convert_numpy(ao_result)
    json_path = os.path.splitext(model_output_path)[0] + ".json"
    with open(json_path, "w") as f:
        json.dump(ao_json, f, indent=2)

    rospy.loginfo(f"Detection complete. Result saved to: {json_path}")
    return ao_result

# ------------- Main -----------------------
def main():
    global base_dir, rgb_dir, depth_dir, ply_dir
    global model_output_path, detector_config_path
    global rgb_topic, depth_topic, max_save_rate

    rospy.init_node(node_name)

    # Params
    IS_OFFLINE = rospy.get_param("~is_offline", True)
    scene_idx = rospy.get_param("~scene_idx", 0)
    mode = rospy.get_param("~mode", "load")  # "capture" or "load"

    base_dir = rospy.get_param("~base_dir",
        "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/{}_detection/cabinet_{}".format(
            'offline' if IS_OFFLINE else 'online', scene_idx))
    rgb_dir = rospy.get_param("~rgb_dir", "RGB_images")
    rgb_detected_dir = rospy.get_param("~rgb_detected_dir", "RGB_images_detected")
    depth_dir = rospy.get_param("~depth_dir", "DEPTH_images")
    ply_dir = rospy.get_param("~ply_dir", "PLY_seg")
    model_output_path = os.path.join(base_dir, rospy.get_param("~model_output_path", "models/doorModel.json"))
    detector_config_path = rospy.get_param("~detector_config_path", "/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg")
    rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
    depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
    max_save_rate = rospy.get_param("~save_fps", 5.0)

    # Absolute folders
    rgb_path = os.path.join(base_dir, rgb_dir)
    depth_path = os.path.join(base_dir, depth_dir)
    ply_path = os.path.join(base_dir, ply_dir)
    rgb_detected_path = os.path.join(base_dir, rgb_detected_dir)

    if mode == "capture":
        # Fresh capture session
        reset_directories(rgb_path, depth_path, ply_path, rgb_detected_path)
        ensure_directories_exist(rgb_path, depth_path, ply_path, os.path.dirname(model_output_path), rgb_detected_path)

        # Subscribers with synchronization
        rgb_sub = Subscriber(rgb_topic, Image)
        depth_sub = Subscriber(depth_topic, Image)
        sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        sync.registerCallback(image_callback)

        rospy.loginfo("Ready. Press SPACE to start image capture; press SPACE again to stop & process.")
        rospy.spin()

        # After capture completes, build model using captured count
        robot = UR5Controller()
        # If you generated PLYs, you can load by discovered files:
        rgb_files = [p for _, p in list_numbered_files(rgb_path, "png")]
        ply_files = [p for _, p in list_numbered_files(ply_path, "ply")]
        if not rgb_files:
            rospy.logwarn("No RGB files found—nothing to process.")
            return
        build_model_from_paths(rgb_files, ply_files, robot)

    else:  # mode == "load"
        # Don't reset; just ensure directories exist
        ensure_directories_exist(rgb_path, depth_path, ply_path, os.path.dirname(model_output_path), rgb_detected_path)

        # Discover existing files
        rgb_files = [p for _, p in list_numbered_files(rgb_path, "png")]
        ply_files = [p for _, p in list_numbered_files(ply_path, "ply")]

        if not rgb_files:
            rospy.logerr(f"No RGB images found in {rgb_path}.")
            return

        if not ply_files:
            rospy.logwarn(f"No PLY files found in {ply_path}. If you have RGB+DEPTH, call create_ply_images().")

        # (Optional) if you also have depth and want to (re)generate PLY:
        # depth_files = [p for _, p in list_numbered_files(depth_path, "png")]
        # if depth_files and (not ply_files or rospy.get_param("~regen_ply", False)):
        #     create_ply_images(len(rgb_files))
        #     ply_files = [p for _, p in list_numbered_files(ply_path, "ply")]

        robot = UR5Controller()
        build_model_from_paths(rgb_files, ply_files, rgb_detected_path, robot)

if __name__ == "__main__":
    main()
