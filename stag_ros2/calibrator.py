# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseArray, TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transforms3d


class Calibrator(Node):

    def __init__(self):
        super().__init__('calibrator')

        self.marker_0_absolute = Pose()
        self.marker_0_absolute.position.x = 0.0
        self.marker_0_absolute.position.y = 0.0
        self.marker_0_absolute.position.z = 0.0
        self.marker_0_absolute.orientation.w = 0.0 #math.sqrt(2)/2
        self.marker_0_absolute.orientation.x = 1.0 #math.sqrt(2)/2
        self.marker_0_absolute.orientation.y = 0.0
        self.marker_0_absolute.orientation.z = 0.0
        self.marker_1_absolute = Pose()
        self.marker_1_absolute.position.x = 0.11
        self.marker_1_absolute.position.y = 0.0
        self.marker_1_absolute.position.z = 0.0
        self.marker_1_absolute.orientation.w = 0.0 #math.sqrt(2)/2
        self.marker_1_absolute.orientation.x = 1.0 #math.sqrt(2)/2
        self.marker_1_absolute.orientation.y = 0.0
        self.marker_1_absolute.orientation.z = 0.0

        self.stag_camera_link = 'stag_camera_link'
        self.trigger_calibration_once = True

        self.tf_published = False
        self.broadcaster = StaticTransformBroadcaster(self)

        t = 'calibration_array'
        self.calibration_array_sub = self.create_subscription(PoseArray, t, self.calibration_array_cb, 10)
        t = 'pose_array'
        self.pose_array_pub = self.create_publisher(PoseArray, t, 10)

        t = 'trigger'
        self.trigger_sub = self.create_subscription(Empty, t, self.trigger_cb, 10)

        t = 'camera_tf'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.camera_tf_pub = self.create_publisher(TransformStamped, t, qos)

    def calibration_array_cb(self, msg):
        # Extract list of ids
        ids = [int(i) for i in msg.header.frame_id.split(',')]
        # If the calibration numbers are in frame, save their positions
        if 0 in ids and 1 in ids:
            for k, pose in enumerate(msg.poses):
                if ids[k] == 0:
                    self.marker_0_relative = pose
                if ids[k] == 1:
                    self.marker_1_relative = pose
            # If we have not calibrated yet, trigger it now
            if not self.tf_published:
                self.trigger_cb(Empty())
                if self.trigger_calibration_once:
                    self.tf_published = True
        # Republish stag poses under the camera frame
        msg.header.frame_id = self.stag_camera_link
        self.pose_array_pub.publish(msg)


    def pose_to_mat(self, pose):
        """Convert Pose message to 4x4 transformation matrix."""
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        scale = [1.0, 1.0, 1.0]
        shear = [0.0, 0.0, 0.0]
        angl = transforms3d.euler.quat2euler(rot)
        self.get_logger().info(f'{angl}')
        rotation_mat = transforms3d.euler.euler2mat(angl[0], angl[1], angl[2])
        mat = transforms3d.affines.compose(T=trans, R=rotation_mat, Z=scale, S=shear)
        return mat

    def mat_to_pose(self, matrix):
        """Convert 4x4 transformation matrix to Pose message."""
        pose = Pose()
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        rotation_matrix = matrix[:3, :3]
        quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        return pose


    def trigger_cb(self, msg):
        self.get_logger().info('trigger received')

        # Convert Pose to transformation matrices
        T_rel_0 = self.pose_to_mat(self.marker_0_relative)
        T_rel_1 = self.pose_to_mat(self.marker_1_relative)
        T_abs_0 = self.pose_to_mat(self.marker_0_absolute)
        T_abs_1 = self.pose_to_mat(self.marker_1_absolute)

        # Compute relative transformations in camera and map frames
        T_rel_cam = np.dot(np.linalg.inv(T_rel_0), T_rel_1)
        T_rel_map = np.dot(np.linalg.inv(T_abs_0), T_abs_1)

        # Assuming T_rel_cam ~= T_rel_map, solve for T_cam_map
        T_cam_map = np.dot(T_abs_0, np.dot(np.linalg.inv(T_rel_cam), np.linalg.inv(T_rel_0)))
        #self.get_logger().info(f'T_cam_map, {T_cam_map}')

        # Convert the matrix back to a Pose
        camera_pose = self.mat_to_pose(T_cam_map)

        # Publish the transformation using the existing function
        self.publish_tf(camera_pose)


    def publish_tf(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.stag_camera_link
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.w = pose.orientation.w
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        self.broadcaster.sendTransform(t)
        self.camera_tf_pub.publish(t)

def main(args=None):
    rclpy.init(args=args)

    Ca = Calibrator()
    rclpy.spin(Ca)

    Ca.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
