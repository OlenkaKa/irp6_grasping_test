#!/usr/bin/env python
# -*- coding: utf-8 -*-


import datetime
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera_track/image_color', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/camera_track/image_rect_color', Image, self.image_rect_callback)

    def image_callback(self, data):
        self._callback('/home/akarbarc/RESULTS/cameraimages', data)

    def image_rect_callback(self, data):
        self._callback('/home/akarbarc/RESULTS/cameraimages-rect', data)

    def _callback(self, dir, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            filename = datetime.datetime.now().strftime('%H%M%S%f')
            print filename
            cv2.imwrite('%s/%s.jpg' % (dir, filename), cv_image)
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'
