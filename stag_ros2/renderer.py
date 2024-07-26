# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseArray, Point

class Renderer(Node):

    def __init__(self):
        super().__init__('renderer')

        t = 'pose_array'
        #self.pose_array_sub = self.create_subscription(PoseArray, t, self.pose_array_cb, 10)
        self.pose_array_2_sub = self.create_subscription(PoseArray, t, self.pose_array_2_cb, 10)

        t = 'marker'
        self.marker_pub = self.create_publisher(Marker, t, 10)


    def default_marker(self):
        Ma = Marker()
        Ma.header.stamp = self.get_clock().now().to_msg()
        Ma.header.frame_id = 'stag_camera_link'
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
        # Doesnt work.
        Ma = self.default_marker()
        Ma.type = Ma.TEXT_VIEW_FACING

        self.image_path = '/home/james/Pictures/Markers/HD19/00000.png'
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().error(f"Image not found at {self.image_path}")
            return

        _, image_data = cv2.imencode('.jpg', image)
        compressed_image = CompressedImage()
        compressed_image.header = Header()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = 'jpeg'
        compressed_image.data = np.array(image_data).tobytes()

        Ma.text = ' '
        Ma.mesh_resource = "package://my_image_marker/meshes/plane.dae"
        Ma.mesh_use_embedded_materials = True
        Ma.mesh_texture = compressed_image

        self.publisher_.publish(Ma)
        self.get_logger().info("Published image marker")


    def pose_array_2_cb(self, msg):
        Ma = self.default_marker()
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
