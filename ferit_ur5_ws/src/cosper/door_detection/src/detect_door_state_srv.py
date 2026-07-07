#!/usr/bin/env python

import os
import sys
import json
import yaml
import rospy
import numpy as np
import cv2
from PIL import Image as PilImage

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import RVLPYRGBD2PLY as rvl

from door_detection.srv import DetectDoorState, DetectDoorStateResponse
from core.real_ur5_controller import UR5Controller
from door_state_detector import DoorStateDetector

class DoorStateDetectionService:
    def __init__(self, config: dict):
        rospy.init_node("door_state_detection_service_node")

        # Load paths from config
        self.base_dir = ""
        self.rgb_dir = config.get("rgb_dir", "detect_state_images/rgb")
        self.depth_dir = config.get("depth_dir", "detect_state_images/depth")
        self.ply_dir = config.get("ply_dir", "detect_state_images/ply")
        self.detector_config_path = config.get("detector_config_path", "")
        self.best_hyp_relative_path = config.get("best_hyp_path", "DDT.txt")
        self.T_C_6_path = config.get("camera_calibration_path", "")

        # Camera topics
        self.cam_cfg = config.get("camera", {})
        self.rgb_topic = self.cam_cfg.get("rgb_topic", "/camera/color/image_raw")
        self.depth_topic = self.cam_cfg.get("depth_topic", "/camera/aligned_depth_to_color/image_raw")

        # Internal state
        self.current_rgb = None
        self.current_depth = None
        self.got_image = False
        self.cv_bridge = CvBridge()

        # Service
        self.srv = rospy.Service('detect_door_state', DetectDoorState, self.handle_detect_door_state)
        rospy.loginfo("Door State Detection Service Ready.")

    def image_callback(self, rgb_msg, depth_msg):
        if not self.got_image:
            self.current_rgb = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.current_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.got_image = True

    def capture_one_image_pair(self):
        # Subscribe and wait for one RGB+Depth pair
        self.got_image = False
        rgb_sub = Subscriber(self.rgb_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=1, slop=0.05)
        sync.registerCallback(self.image_callback)

        timeout = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown() and not self.got_image:
            if rospy.Time.now() > timeout:
                rospy.logerr("Camera frame timeout.")
                return False
            rospy.sleep(0.05)
        return self.got_image

    def create_ply_from_images(self, rgb_path, depth_path, ply_path):
        """Convert a single RGBD frame to a PLY mesh."""
        rospy.loginfo("Creating PLY file for door state detection...")
        rgbd2ply = rvl.RGBD2PLY()

        with PilImage.open(rgb_path) as rgb_img:
            rgb_data = np.array(rgb_img.getdata()).astype(np.byte)
            bgr_array = np.stack((rgb_data[:, 2], rgb_data[:, 1], rgb_data[:, 0]), axis=1)

        with PilImage.open(depth_path) as depth_img:
            depth_array = np.array(depth_img.getdata()).astype(np.short)

        rgbd2ply.pixel_array_to_ply(
            bgr_array, depth_array,
            self.cam_cfg.get("fu", 597.903),
            self.cam_cfg.get("fv", 598.480),
            self.cam_cfg.get("uc", 323.844),
            self.cam_cfg.get("vc", 236.328),
            self.cam_cfg.get("width", 640),
            self.cam_cfg.get("height", 480),
            self.cam_cfg.get("depth_scale", 0.050),
            ply_path
        )
        del rgbd2ply, rgb_data, bgr_array, depth_array
        rospy.loginfo(f"PLY file created: {ply_path}")

    def handle_detect_door_state(self, req):
        """
        Service callback to estimate the current door opening angle.
        - Captures one RGB and Depth image pair
        - Creates a PLY mesh from the images
        - Loads the door model and computes the state angle using DoorStateDetector
        """
        self.base_dir = req.base_dir
        self.state_detection_dir = req.state_detection_dir
        T_Cdet_Cstate = np.array(req.T_Cdetected_Cstate).reshape(4, 4)

        self.base_dir_state_detection = os.path.join(self.base_dir, self.state_detection_dir)

        try:
            # Paths
            rgb_save_path = os.path.join(self.base_dir_state_detection, self.rgb_dir, "captured_rgb.png")
            depth_save_path = os.path.join(self.base_dir_state_detection, self.depth_dir, "captured_depth.png")
            ply_save_path = os.path.join(self.base_dir_state_detection, self.ply_dir, "captured.ply")
            best_hyp_path = os.path.join(self.base_dir, self.best_hyp_relative_path)

            # Ensure directories exist
            os.makedirs(os.path.dirname(rgb_save_path), exist_ok=True)
            os.makedirs(os.path.dirname(depth_save_path), exist_ok=True)
            os.makedirs(os.path.dirname(ply_save_path), exist_ok=True)

            # Capture one image pair
            if not self.capture_one_image_pair():
                return DetectDoorStateResponse(
                    success=False, state_angle_deg=0.0, message="Failed to capture RGB/Depth image."
                )

            # Save images
            cv2.imwrite(rgb_save_path, self.current_rgb)
            cv2.imwrite(depth_save_path, self.current_depth)
            rospy.loginfo(f"Saved RGB image: {rgb_save_path}")
            rospy.loginfo(f"Saved Depth image: {depth_save_path}")

            # Create PLY mesh from the captured images
            self.create_ply_from_images(rgb_save_path, depth_save_path, ply_save_path)

            detector = DoorStateDetector(
                detector_config_path=self.detector_config_path,
                best_hyp_path=best_hyp_path
            )
            state_angles = detector.detect_state(rgb_save_path, ply_save_path, T_Cdet_Cstate)
            angle_deg = float(state_angles[0])
            rospy.loginfo(f"[DoorStateSrv] Detected door state: {angle_deg:.2f} deg")

            # Save detected state
            with open(os.path.join(self.base_dir_state_detection, "detected_state.json"), 'w') as f:
                json.dump({"state_angle_deg": angle_deg}, f)

            return DetectDoorStateResponse(
                success=True,
                state_angle_deg=angle_deg,
                message=f"Detected angle: {angle_deg:.2f} deg"
            )

        except Exception as e:
            rospy.logerr(f"[DoorStateSrv] Detection failed: {e}")
            return DetectDoorStateResponse(
                success=False,
                state_angle_deg=0.0,
                message=str(e)
            )

if __name__ == "__main__":
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
    if not os.path.exists(cfg_path):
        rospy.logfatal(f"Config file not found: {cfg_path}")
        sys.exit(1)

    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f)

    try:
        server = DoorStateDetectionService(config)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass