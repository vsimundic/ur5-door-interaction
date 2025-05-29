#!/usr/bin/env python3.8

import os
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

# Global counters
number_of_images = 0
key_counter = 0

# Parameters (will be set in main)
base_dir = ""
rgb_dir = ""
depth_dir = ""
ply_dir = ""
model_output_path = ""
detector_config_path = ""
node_name = "rgbAndDepth_image_subscriber"

def stop_node(node_name):
    kill_nodes([node_name])

def ensure_directories_exist(*dirs):
    for d in dirs:
        if not os.path.exists(d):
            os.makedirs(d)
            rospy.loginfo(f"Created directory: {d}")

def image_callback(rgb_msg, depth_msg):
    global number_of_images, key_counter, base_dir, rgb_dir, depth_dir

    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

    cv2.imshow("RGB", rgb)
    key = cv2.waitKey(1)

    if key == 32:  # Space bar
        key_counter += 1

    if key_counter == 1:
        file_idx = str(number_of_images).zfill(4)
        rgb_path = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_path = os.path.join(base_dir, depth_dir, f"{file_idx}.png")

        cv2.imwrite(rgb_path, rgb)
        cv2.imwrite(depth_path, depth)
        number_of_images += 1

    if key_counter >= 2:
        cv2.destroyAllWindows()
        create_ply_images(number_of_images)

def create_ply_images(count):
    global base_dir, rgb_dir, depth_dir, ply_dir

    print("Creating PLY files...")
    rgbd2ply = rvl.RGBD2PLY()

    for i in range(count):
        file_idx = str(i).zfill(4)
        rgb_file = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_file = os.path.join(base_dir, depth_dir, f"{file_idx}.png")
        ply_file = os.path.join(base_dir, ply_dir, f"{file_idx}.ply")

        rgb_img = PilImage.open(rgb_file)
        rgb_data = np.array(rgb_img.getdata()).astype(np.byte)
        bgr_array = np.stack((rgb_data[:, 2], rgb_data[:, 1], rgb_data[:, 0]), axis=1)

        depth_img = PilImage.open(depth_file)
        depth_array = np.array(depth_img.getdata()).astype(np.short)

        rgbd2ply.pixel_array_to_ply(
            bgr_array, depth_array,
            fx=597.9033203125, fy=598.47998046875,
            cx=323.8436584472656, cy=236.32774353027344,
            width=640, height=480,
            scale=0.050,
            filename=ply_file
        )

    print("PLY generation complete.")
    stop_node(node_name)

def build_model(count):
    global base_dir, ply_dir, rgb_dir, model_output_path, detector_config_path

    print("Starting 3D detection...")
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(detector_config_path)

    first_id = 0
    last_id = count - 1

    print(f"Loading mesh sequence from {ply_dir}")
    for mesh_id in range(first_id, last_id):
        mesh_file = os.path.join(base_dir, ply_dir, f"{mesh_id:04d}.ply")
        print(f"  Adding mesh: {mesh_file}")
        detector.add_mesh(mesh_file)

    print("Loading RGB sequence")
    for img_id in range(first_id, last_id + 1):
        rgb_file = os.path.join(base_dir, rgb_dir, f"{img_id:04d}.png")
        print(f"  Adding RGB: {rgb_file}")
        detector.add_rgb(rgb_file)

    detector.set_hyp_file_name(os.path.join(base_dir, "hyps.txt"))

    ao_result = detector.detect()

    with open(model_output_path, "w") as f:
        f.write(str(ao_result))

    print("Detection complete.")
    return ao_result

def main():
    global base_dir, rgb_dir, depth_dir, ply_dir, model_output_path, detector_config_path

    rospy.init_node(node_name)

    # Load parameters
    base_dir = rospy.get_param("~base_dir", "/home/RVLuser/ur5_ws/src/ao_manipulation/scripts")
    rgb_dir = rospy.get_param("~rgb_dir", "RGB_images")
    depth_dir = rospy.get_param("~depth_dir", "DEPTH_images")
    ply_dir = rospy.get_param("~ply_dir", "PLY_seg")
    model_output_path = rospy.get_param("~model_output_path", base_dir + "/models/doorModel.txt")
    detector_config_path = rospy.get_param("~detector_config_path", "/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg")

    rgb_sub = Subscriber("/camera/color/image_raw", Image)
    depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
    sync.registerCallback(image_callback)

    rospy.spin()

    build_model(number_of_images)

if __name__ == "__main__":
    main()
