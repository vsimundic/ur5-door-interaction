#!/usr/bin/env python

from detectron2_tensormask import Detectron2Tensormask
from detectron2.engine import default_argument_parser

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from rospkg import RosPack
from cv_bridge import CvBridge
import os
import yaml
import subprocess
import warnings
import cv2
warnings.filterwarnings('ignore', category=UserWarning)

class SegmentVideosNode:
    def __init__(self, args, config):      
        self.config = config


        # TensorMask segmentation object
        self.d2t = Detectron2Tensormask(cfg_args=args, config=self.config)
        rospy.sleep(5)
        rospy.loginfo('Detectron2 setup over! You can start loading bags.')

        self.image_counter = 0
        self.fps_scaler = int(self.config['camera_fps'] / self.config['wanted_fps'])
        self.fps_scaler_counter = 0
        self.bag_name = ''
        self.bridge = CvBridge()

    def load_and_run_bags(self):
        """
        Loads rosbags one by one, runs them and lets callback do the segmentation.
        """
        load_path = self.config['load_path_root']

        with open(os.path.join(load_path, 'bags_sequence.txt'), 'r') as f:
            bags = []
            while True:
                line = f.readline().strip()
                if line.startswith('bags_sequence'):
                    continue
                elif line == 'end' or not line:
                    break
                else:
                    bags.append(os.path.join(load_path, line))
        
        for bag in bags:
            self.bag_name = bag.split('/')[-1].split('.')[0]

            self.save_path_root = os.path.join(self.config['save_path_root'], self.bag_name)

            self.set_save_paths(self.save_path_root)

            self.rgb_subscriber = message_filters.Subscriber(self.config['rgb_topic'], Image)
            self.depth_subscriber = message_filters.Subscriber(self.config['depth_topic'], Image)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_subscriber, self.depth_subscriber], queue_size=500, slop=2)

            self.camera_info_subscriber = rospy.Subscriber(self.config['camera_info_topic'], CameraInfo, self.camera_info_callback)
            self.image_counter = 0
            self.fps_scaler_counter = 0
            self.ts.registerCallback(self.images_callback)

            rospy.loginfo(bag)

            subprocess.call(['rosbag', 'play', bag, '-r', str(self.config['bag_reproduce_rate'])]) # Waits here until the rosbag stops playing
            rospy.sleep(2.0)
            self.save_scene_sequence_file()
            [sub.sub.unregister() for sub in [self.rgb_subscriber, self.depth_subscriber]]
        
        rospy.signal_shutdown('shutdown')

    def images_callback(self, rgb_msg, depth_msg):
        """
        Handles Image messages, segments on rgb and depth images, saves and shows the image. 
        """

        self.fps_scaler_counter +=1
        if self.fps_scaler_counter % self.fps_scaler == 0:
            self.fps_scaler_counter = 0
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            self.d2t.segment_rgb_and_depth_images(rgb_image, depth_image, self.image_counter, self.save_path_root)
            self.image_counter += 1

    def camera_info_callback(self, camera_info_msg):
        save_path = self.save_path_root

        if not os.path.exists(save_path):
            os.makedirs(save_path)
        with open(os.path.join(save_path, 'camera_info.yaml'), 'w') as f:
            f.write(camera_info_msg.__str__())
            rospy.loginfo('Saved camera information to file.')

        self.camera_info_subscriber.unregister()

    def save_scene_sequence_file(self):
        file_path = os.path.join(self.save_path_root, 'SceneSequence.txt')
        with open(file_path, 'w') as f:
            for i in range(self.image_counter):
                f.write(f"{i:04d}.png\n")
            f.write("end\n")
        rospy.loginfo(f"Saved SceneSequence.txt to {file_path}")

    def set_save_paths(self, root_path):
        # Save paths
        self.rgb_save_path = os.path.join(root_path, 'rgb')
        self.rgb_seg_save_path = os.path.join(root_path, 'rgb_seg')
        self.depth_seg_save_path = os.path.join(root_path, 'depth_seg')
        self.ply_seg_save_path = os.path.join(root_path, 'PLY_seg')
        if not os.path.exists(self.rgb_save_path):
            os.makedirs(self.rgb_save_path)
        if not os.path.exists(self.rgb_seg_save_path):
            os.makedirs(self.rgb_seg_save_path)
        if not os.path.exists(self.depth_seg_save_path):
            os.makedirs(self.depth_seg_save_path)
        if not os.path.exists(self.ply_seg_save_path):
            os.makedirs(self.ply_seg_save_path)


if __name__ == '__main__':

    rospy.init_node('segment_videos_node')
    rp = RosPack()
    package_path = rp.get_path('human_seg')
    config_path = os.path.join(package_path, 'config')

    with open(os.path.join(config_path, 'config.yaml'), 'r') as fcfg:
        config = yaml.load(fcfg, Loader=yaml.FullLoader)

    # Get passed arguments from terminal
    args = default_argument_parser().parse_args()

    segment_videos_obj = SegmentVideosNode(args, config)
    segment_videos_obj.load_and_run_bags()

    rospy.spin()