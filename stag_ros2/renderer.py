# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Point

class Renderer(Node):

    def __init__(self):
        super().__init__('renderer')

        t = 'pose_array'
        self.pose_array_sub = self.create_subscription(PoseArray, t, self.pose_array_cb, 10)

        t = 'marker'
        self.marker_pub = self.create_publisher(Marker, t, 10)

    def default_marker(self):
        Ma = Marker()
        Ma.header.stamp = self.get_clock().now().to_msg()
        Ma.ns = 'stag_markers'
        Ma.id = 0
        Ma.action = Ma.ADD
        Ma.scale.x = 0.05
        Ma.scale.y = 0.05
        Ma.scale.z = 0.05
        Ma.color.r = 1.0
        Ma.color.g = 1.0
        Ma.color.b = 1.0
        Ma.color.a = 1.0
        return Ma

    def pose_array_cb(self, msg):
        Ma = self.default_marker()
        Ma.header.frame_id = msg.header.frame_id
        Ma.type = Ma.CUBE_LIST
        for pose in msg.poses:
            p = Point()
            p.x, p.y, p.z = pose.position.x, pose.position.y, pose.position.z
            Ma.points.append(p)
        self.marker_pub.publish(Ma)


def main(args=None):
    rclpy.init(args=args)

    Re = Renderer()
    rclpy.spin(Re)

    Re.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
