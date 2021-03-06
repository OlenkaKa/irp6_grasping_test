#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera_track/image_color', Image, queue_size=10)
        self.image_rect_pub = rospy.Publisher('/camera_track/image_rect_color', Image, queue_size=10)

        self.image_dir = '/home/akarbarc/ci'
        self.image_rect_dir = '/home/akarbarc/ci-rect'

        self.image_files = os.listdir(self.image_dir)
        self.image_rect_files = os.listdir(self.image_rect_dir)

        self.image_file_idx = 0
        self.image_rect_file_idx = 0

        self.frame_id = '/t_c_optical_frame'

    def _create_message(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.get_rostime()
        return msg

    def publish(self):
        image_filename = self.image_files[self.image_file_idx]
        self.image_file_idx = (self.image_file_idx + 1) % len(self.image_files)
        cv_image = cv2.imread('%s/%s' % (self.image_dir, image_filename))

        image_rect_filename = self.image_rect_files[self.image_rect_file_idx]
        self.image_rect_file_idx = (self.image_rect_file_idx + 1) % len(self.image_rect_files)
        cv_image_rect = cv2.imread('%s/%s' % (self.image_rect_dir, image_rect_filename))

        try:
            self.image_pub.publish(self._create_message(cv_image))
            self.image_rect_pub.publish(self._create_message(cv_image_rect))
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    image_publisher = ImagePublisher()
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        image_publisher.publish()
        rate.sleep()
