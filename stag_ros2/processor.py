# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import numpy as np
import cv2
from cv_bridge import CvBridge

import stag

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image


class Processor(Node):

    def __init__(self):
        super().__init__('processor')
        self.bridge = CvBridge()

        t = 'tags'
        self.tags_pub = self.create_publisher(String, t, 10)

        t = 'color/image_labelled'
        self.image_pub = self.create_publisher(Image, t, 10)

        t = '/camera1/image_raw'
        self.image_sub = self.create_subscription(Image, t, self.image_cb, 10)

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # detect markers
        (corners, ids, rejected_corners) = stag.detectMarkers(image, 21)
        s = String()
        s.data = ""+str(ids)
        self.tags_pub.publish(s)

        # draw detected markers with ids
        stag.drawDetectedMarkers(image, corners, ids)
        stag.drawDetectedMarkers(image, rejected_corners, border_color=(255, 0, 0))

        # save resulting image
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_pub.publish(ros_image)
        #cv2.imshow("Converted Image", image)
        #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    Pr = Processor()
    rclpy.spin(Pr)

    Pr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
