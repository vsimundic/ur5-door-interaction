#!/usr/bin/env python

import os
import sys

# Force VTK (used by RVLPYDDDetector) into fully offscreen/headless mode
# BEFORE it is imported.  VTK and OpenCV both initialise the GTK GDK type
# system; if either does so first the other triggers a fatal GLib assertion
# (GdkDisplayManager already registered → g_once_init_leave fails → segfault).
# With offscreen rendering VTK never touches GDK at all.
os.environ.setdefault("VTK_DEFAULT_RENDER_WINDOW_OFFSCREEN", "1")

import json
import yaml
import shutil
import cv2
import numpy as np
import gc
from PIL import Image as PilImage

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import RVLPYRGBD2PLY as rvl
import RVLPYDDDetector

from door_detection.srv import DetectDoor, DetectDoorResponse


def convert_numpy(obj):
    """Recursively converts numpy types to standard Python types for JSON serialization."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    else:
        return obj


def ensure_directories_exist(*dirs):
    """Creates given directories if they do not exist."""
    for d in dirs:
        if not os.path.exists(d):
            os.makedirs(d)
            rospy.loginfo(f"Created directory: {d}")


def reset_directories(*dirs):
    """Clears and recreates directories to ensure a clean state."""
    for d in dirs:
        if os.path.exists(d):
            shutil.rmtree(d)
            rospy.loginfo(f"Cleared directory: {d}")
        os.makedirs(d, exist_ok=True)


class DoorDetectionService:
    def __init__(self, config: dict):
        rospy.init_node("door_detection_service_node")
        
        # Load paths from config
        self.rgb_dir = config.get("rgb_dir", "RGB_images")
        self.depth_dir = config.get("depth_dir", "DEPTH_images")
        self.ply_dir = config.get("ply_dir", "PLY_seg")
        self.detector_config_path = config.get("detector_config_path", "")
        self.camera_calibration_path = config.get("camera_calibration_path", "")
        
        # Initialize internal state for paths
        self.base_dir = ""
        self.rgb_path = ""
        self.depth_path = ""
        self.ply_path = ""
        self.model_output_path = ""

        # Load camera config
        cam_cfg = config.get("camera", {})
        rgb_topic = cam_cfg.get("rgb_topic", "/camera/color/image_raw")
        depth_topic = cam_cfg.get("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.cam_fu = cam_cfg.get("fu", 597.903)
        self.cam_fv = cam_cfg.get("fv", 598.480)
        self.cam_uc = cam_cfg.get("uc", 323.844)
        self.cam_vc = cam_cfg.get("vc", 236.328)
        self.cam_w = cam_cfg.get("width", 640)
        self.cam_h = cam_cfg.get("height", 480)
        self.depth_scale = cam_cfg.get("depth_scale", 0.050)

        # Load capture config
        cap_cfg = config.get("capture", {})
        self.save_fps = cap_cfg.get("save_fps", 5.0)

        # State Variables
        self.current_rgb = None
        self.current_depth = None
        self.recording = False
        self.gui_active = False
        self.gui_done = False
        self.number_of_images = 0
        self.last_save_time = 0
        self.last_frame_time = 0        # <-- track when last frame arrived
        self.camera_timeout = 3.0       # <-- seconds before we consider camera lost

        # Subscriptions
        rgb_sub = Subscriber(rgb_topic, Image)
        depth_sub = Subscriber(depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.srv = rospy.Service('detect_door', DetectDoor, self.handle_detect_door)
        rospy.loginfo("Door Detection Service Ready.")

    def image_callback(self, rgb_msg, depth_msg):
        """Callback to handle synchronized RGB and Depth frames."""

        bridge = CvBridge()
        self.current_rgb = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.current_depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        self.last_frame_time = rospy.get_time()     # <-- update heartbeat

        if not self.recording:
            return

        now = rospy.get_time()
        if now - self.last_save_time < 1.0 / self.save_fps:
            return

        self.last_save_time = now
        
        file_idx = str(self.number_of_images).zfill(4)
        cv2.imwrite(os.path.join(self.rgb_path, f"{file_idx}.png"), self.current_rgb)
        cv2.imwrite(os.path.join(self.depth_path, f"{file_idx}.png"), self.current_depth)
        
        self.number_of_images += 1
        rospy.loginfo(f"Saved image {file_idx}...")

    def _camera_is_alive(self) -> bool:
        """Returns True if a frame arrived within the timeout window."""
        if self.last_frame_time == 0:
            return False
        return (rospy.get_time() - self.last_frame_time) < self.camera_timeout

    def run_gui_loop(self):
        """Main thread loop to handle OpenCV GUI events safely."""
        rate = rospy.Rate(30)
        window_name = "Door Capture"
        window_created = False

        while not rospy.is_shutdown():
            if self.gui_active:
                if not window_created:
                    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
                    window_created = True

                camera_alive = self._camera_is_alive()

                if not camera_alive:
                    # Show a warning frame instead of freezing
                    warning_img = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(warning_img, "WAITING FOR CAMERA...",
                                (80, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
                    cv2.putText(warning_img, "Check RealSense connection",
                                (100, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 255), 1)

                    # If we were recording and camera dropped, pause recording
                    if self.recording:
                        self.recording = False
                        rospy.logwarn("[GUI] Camera lost during recording! Recording paused.")
                        cv2.putText(warning_img, "Recording PAUSED - camera lost",
                                    (80, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    cv2.imshow(window_name, warning_img)

                elif self.current_rgb is not None:
                    display_img = self.current_rgb.copy()

                    if self.recording:
                        cv2.putText(display_img, f"RECORDING [{self.number_of_images}] - SPACE to stop",
                                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        cv2.putText(display_img, "Press SPACE to START",
                                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        if self.number_of_images > 0:
                            cv2.putText(display_img, f"Captured: {self.number_of_images} - SPACE again to confirm",
                                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)

                    cv2.imshow(window_name, display_img)

                key = cv2.waitKey(1) & 0xFF

                # SPACE bar logic - only allow start if camera is alive
                if key == 32:
                    if not self.recording:
                        if not camera_alive:
                            rospy.logwarn("[GUI] Cannot start recording: camera not available.")
                        else:
                            self.recording = True
                            rospy.loginfo("[GUI] Started recording images.")
                    else:
                        self.recording = False
                        self.gui_done = True
                        rospy.loginfo("[GUI] Stopped recording images.")

            else:
                if window_created:
                    cv2.destroyWindow(window_name)
                    window_created = False

                # Keep OpenCV events processing to avoid hanging windows
                cv2.waitKey(100)

            rate.sleep()

    def create_ply_images(self, count):
        """Converts synchronized RGBD frames to PLY meshes."""
        rospy.loginfo("Creating PLY files...")
        rgbd2ply = rvl.RGBD2PLY()
        for i in range(count):
            file_idx = str(i).zfill(4)
            rgb_file = os.path.join(self.rgb_path, f"{file_idx}.png")
            depth_file = os.path.join(self.depth_path, f"{file_idx}.png")
            ply_file = os.path.join(self.ply_path, f"{file_idx}.ply")

            with PilImage.open(rgb_file) as rgb_img:
                rgb_data = np.array(rgb_img.getdata()).astype(np.byte)
                # Reorder channels (RGB -> BGR)
                bgr_array = np.stack((rgb_data[:, 2], rgb_data[:, 1], rgb_data[:, 0]), axis=1)

            with PilImage.open(depth_file) as depth_img:
                depth_array = np.array(depth_img.getdata()).astype(np.short)

            rgbd2ply.pixel_array_to_ply(
                bgr_array, depth_array,
                self.cam_fu, self.cam_fv,
                self.cam_uc, self.cam_vc,
                self.cam_w, self.cam_h, self.depth_scale, ply_file
            )

            # Explicitly delete large arrays in the loop heavily loaded with RAM
            del rgb_data, bgr_array, depth_array

        del rgbd2ply
        rospy.loginfo("PLY files created.")

    def build_model(self, count, joint_values):
        """Runs the 3D detector on generated PLY meshes and generates model poses."""
        rospy.loginfo("Starting 3D detection...")
        detector = RVLPYDDDetector.PYDDDetector()
        detector.create(self.detector_config_path)

        for mesh_id in range(count):
            detector.add_mesh(os.path.join(self.ply_path, f"{mesh_id:04d}.ply"))
        for img_id in range(count):
            detector.add_rgb(os.path.join(self.rgb_path, f"{img_id:04d}.png"))

        detector.set_hyp_file_name(os.path.join(self.base_dir, "hyps.txt"))
        ao_result = detector.detect()

        # Use the passed kinematics instead of querying the robot
        ao_result["joint_values"] = list(joint_values)
        
        if os.path.exists(self.camera_calibration_path):
            T_C_6 = np.load(self.camera_calibration_path)
        else:
            rospy.logwarn("Camera calibration file not found! Using Identity matrix.")
            T_C_6 = np.eye(4)

        R_A_C = np.array(ao_result['R']).reshape(3, 3)
        t_A_C = np.array(ao_result['t'])

        T_A_C = np.eye(4)
        T_A_C[:3, :3] = R_A_C
        T_A_C[:3, 3] = t_A_C

        ao_result["T_A_C"] = T_A_C.tolist()

        detector.clear_mesh_sequence()
        detector.clear_rgb_sequence()
        detector.clear()

        del detector

        return ao_result

    def handle_detect_door(self, req):
        """Service callback to clear memory, capture new frames, and return full door model."""
        if not req.trigger:
            return DetectDoorResponse(success=False, result_json="{}")

        # Extract kinematics from request
        req_joint_values = req.joint_values
        req_T_6_0 = np.array(req.T_6_0).reshape(4, 4)

        # Set paths dynamically based on incoming request
        self.base_dir = req.base_dir
        self.rgb_path = os.path.join(self.base_dir, self.rgb_dir)
        self.depth_path = os.path.join(self.base_dir, self.depth_dir)
        self.ply_path = os.path.join(self.base_dir, self.ply_dir)
        self.model_output_path = os.path.join(self.base_dir, "models", "doorModel.json")

        reset_directories(self.rgb_path, self.depth_path, self.ply_path)
        ensure_directories_exist(self.rgb_path, self.depth_path, self.ply_path, os.path.dirname(self.model_output_path))
        
        # Reset and activate GUI variables
        self.number_of_images = 0
        self.recording = False
        self.gui_done = False
        self.gui_active = True
        
        rospy.loginfo("Look at the camera window and press SPACE to start/stop.")
        
        # Block this service thread until the user stops recording via SPACE
        while not self.gui_done and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        self.gui_active = False

        if self.number_of_images == 0:
            rospy.logwarn("No images were captured!")
            return DetectDoorResponse(success=False, result_json="{}")

        # Process and build model
        self.create_ply_images(self.number_of_images)
        
        # Pass the kinematics to the build method
        ao_result = self.build_model(self.number_of_images, req_joint_values)
        
        ao_json = convert_numpy(ao_result)
        
        # Save to disk
        with open(self.model_output_path, "w") as f:
            json.dump(ao_json, f, indent=2)

        # Clear the saved images from the node state to free RAM
        self.current_rgb = None
        self.current_depth = None

        gc.collect()

        return DetectDoorResponse(success=True, result_json=json.dumps(ao_json))


if __name__ == "__main__":
    cfg_path = os.path.join(os.path.dirname(__file__), '../cfg/config.yaml')
    if not os.path.exists(cfg_path):
        rospy.logfatal(f"Config file not found: {cfg_path}")
        sys.exit(1)

    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f)

    try:
        server = DoorDetectionService(config)
        server.run_gui_loop()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()