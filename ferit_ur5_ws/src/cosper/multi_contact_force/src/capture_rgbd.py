#!/usr/bin/env python
"""
Capture synchronized RGB-D frames plus camera info from the RealSense
camera and save them to disk. Press SPACE to save the current frame, move
the camera, press SPACE again to save the next one, and so on.

Usage:
    rosrun multi_contact_force capture_rgbd.py
"""

import os
import re
import yaml
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

RGB_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'

SAVE_DIR = os.path.expanduser('/home/RVLuser/data/Exp-cabinet-sam3-20260713')
RGB_DIR = os.path.join(SAVE_DIR, 'rgb')
DEPTH_DIR = os.path.join(SAVE_DIR, 'depth')


def next_capture_index(rgb_dir):
    pattern = re.compile(r'^(\d{4})\.png$')
    indices = [int(m.group(1)) for f in os.listdir(rgb_dir)
               for m in [pattern.match(f)] if m]
    return max(indices) + 1 if indices else 0


def camera_info_to_dict(msg):
    return {
        'width': msg.width,
        'height': msg.height,
        'distortion_model': msg.distortion_model,
        'D': list(msg.D),
        'K': list(msg.K),
        'R': list(msg.R),
        'P': list(msg.P),
        'binning_x': msg.binning_x,
        'binning_y': msg.binning_y,
    }


class RGBDCapture(object):
    def __init__(self):
        rospy.init_node('capture_rgbd', anonymous=True)

        for d in (RGB_DIR, DEPTH_DIR):
            if not os.path.exists(d):
                os.makedirs(d)

        self.bridge = CvBridge()
        self.capture_count = next_capture_index(RGB_DIR)
        self.camera_info_saved = os.path.exists(os.path.join(SAVE_DIR, 'camera_info.yaml'))
        if self.capture_count > 0:
            rospy.loginfo('Resuming: found existing captures, next index is %04d',
                           self.capture_count)

        rgb_sub = Subscriber(RGB_TOPIC, Image)
        depth_sub = Subscriber(DEPTH_TOPIC, Image)
        info_sub = Subscriber(CAMERA_INFO_TOPIC, CameraInfo)

        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        rospy.loginfo('Showing live preview. Press SPACE to save the current frame, '
                       'ESC/q to quit without saving.')
        rospy.loginfo('Topics:\n  %s\n  %s\n  %s',
                       RGB_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC)
        rospy.spin()
        cv2.destroyAllWindows()

    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        if depth_image.dtype != np.uint16:
            depth_image = np.round(depth_image.astype(np.float64) * 1000.0).astype(np.uint16)

        depth_preview = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow('RGB image', rgb_image)
        cv2.imshow('Depth image', depth_preview)
        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord('q'):
            rospy.signal_shutdown('Quit.')
            return

        if key != 32:  # SPACE
            return

        rgb_path = os.path.join(RGB_DIR, '%04d.png' % self.capture_count)
        depth_path = os.path.join(DEPTH_DIR, '%04d.png' % self.capture_count)
        cv2.imwrite(rgb_path, rgb_image)
        cv2.imwrite(depth_path, depth_image)
        rospy.loginfo('Saved RGB image to %s', rgb_path)
        rospy.loginfo('Saved depth image to %s', depth_path)
        self.capture_count += 1

        if not self.camera_info_saved:
            info_path = os.path.join(SAVE_DIR, 'camera_info.yaml')
            with open(info_path, 'w') as f:
                yaml.safe_dump(camera_info_to_dict(camera_info_msg), f)
            rospy.loginfo('Saved camera info to %s', info_path)
            self.camera_info_saved = True


if __name__ == '__main__':
    RGBDCapture()
