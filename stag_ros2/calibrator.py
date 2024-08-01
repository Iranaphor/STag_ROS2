# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
import yaml
import math
import numpy as np

from ament_index_python.packages import get_package_share_directory

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

        # Declare rosparams for access
        #self.declare_parameter('stag_camera_link_suffix', rclpy.Parameter.Type.STRING)
        self.declare_parameter('calibration_config_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('trigger_calibration_once', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('minimum_calibration_markers', rclpy.Parameter.Type.INTEGER)

        # Read config from yaml file
        config_file = self.get_parameter('calibration_config_file').value
        if config_file.startswith('package://'):
            config_file = config_file[len('package://'):]
            package_name, relative_path = config_file.split('/', 1)
            package_path = get_package_share_directory(package_name)
            config_file = os.path.join(package_path, relative_path)
        with open(config_file, 'r') as file:
            data = yaml.safe_load(file)

        # Convert marker list into id dict
        self.minimum_calibration_markers = self.get_parameter('minimum_calibration_markers').value
        self.absolute_markers = dict()
        for marker in data['markers']:
            # Format pose object
            pose = Pose()
            pose.position.x = marker['pose']['position']['x']
            pose.position.y = marker['pose']['position']['y']
            pose.position.z = marker['pose']['position']['z']
            pose.orientation.w = marker['pose']['orientation']['w']
            pose.orientation.x = marker['pose']['orientation']['x']
            pose.orientation.y = marker['pose']['orientation']['y']
            pose.orientation.z = marker['pose']['orientation']['z']
            # Save pose to dictionary
            if marker['id'] not in self.absolute_markers:
                self.absolute_markers[marker['id']] = dict()
            self.absolute_markers[marker['id']]['pose'] = pose
            self.absolute_markers[marker['id']] = pose

        self.relative_markers = dict()

        ns = self.get_namespace() if self.get_namespace() != '/' else 'stag_ros2'
        self.stag_camera_link = ns[1:].replace('/', '_')+'_camera_link'
        self.get_logger().info(f'{ns}')
        self.get_logger().info(f'{self.stag_camera_link}')

        self.trigger_calibration_once = self.get_parameter('trigger_calibration_once').value

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
        known = [i for i in ids if i in self.absolute_markers.keys()]
        if len(known) >= self.minimum_calibration_markers:

            # Save marker positions into reference dictionary
            for k, pose in enumerate(msg.poses):
                self.relative_markers[ids[k]] = pose

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

        # We will use these lists to store individual transformations
        T_cam_map_list = []

        # Process each marker independently
        keys = sorted(self.relative_markers.keys())
        for key in keys:
            if key in self.absolute_markers:
                T_rel = self.pose_to_mat(self.relative_markers[key])
                T_abs = self.pose_to_mat(self.absolute_markers[key])

                # Assuming the transformation from the camera frame to the map frame through this marker
                T_cam_map = np.dot(T_abs, np.linalg.inv(T_rel))
                T_cam_map_list.append(T_cam_map)

        # Combine all T_cam_map calculations. For example, average them if they are similar enough.
        if T_cam_map_list:
            # A simple way to combine is to average the transformations
            # More complex methods could involve optimizing to best fit all T_cam_map_list elements
            T_cam_map_average = np.mean(T_cam_map_list, axis=0)

            # Convert the matrix back to a Pose
            camera_pose = self.mat_to_pose(T_cam_map_average)

            # Publish the transformation using the existing function
            self.publish_tf(camera_pose)
        else:
            self.get_logger().info('No valid marker pairs found for transformation calculation.')


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
