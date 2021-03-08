#!/usr/bin/env python

"""
Load yaml calibration into CameraInfo topic
"""
import sys
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class yaml2CameraInfoNode:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic to publish camera info
        self.info_pub = rospy.Publisher(
            "/camera_info",
            CameraInfo, queue_size=10)

        self.yaml_file = rospy.get_param(
            'camera_info_url',
            '/home/bknight/catkin_ws/src/i3dr_titania-ros/calibration/6465696d6f736i/left.yaml'
        )

        # Load data from file
        with open(self.yaml_file, "r") as file_handle:
            self.calib_data = yaml.load(file_handle)

        # Parse as camera info
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.calib_data["image_width"]
        self.camera_info_msg.height = self.calib_data["image_height"]
        self.camera_info_msg.K = self.calib_data["camera_matrix"]["data"]
        self.camera_info_msg.D = self.calib_data["distortion_coefficients"]["data"]
        self.camera_info_msg.R = self.calib_data["rectification_matrix"]["data"]
        self.camera_info_msg.P = self.calib_data["projection_matrix"]["data"]
        self.camera_info_msg.distortion_model = self.calib_data["distortion_model"]

        # subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/image_raw",
            Image, self.image_callback, queue_size=1)

    def image_callback(self, image_msg):
        self.camera_info_msg.header = image_msg.header
        self.info_pub.publish(self.camera_info_msg)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('yaml2CameraInfo', anonymous=True)
    y2cam = yaml2CameraInfoNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down yaml2CameraInfo module")


if __name__ == '__main__':
    main(sys.argv)
