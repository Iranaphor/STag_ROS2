# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
import json
import msgpack
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class Receiver(Node):

    def __init__(self):
        super().__init__('mqtt_receiver')

        self.declare_parameter('mqtt_encoding', rclpy.Parameter.Type.STRING)
        self.declare_parameter('mqtt_ns', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('pose_update_delay', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('markers_to_subscribe', rclpy.Parameter.Type.STRING_ARRAY)

        # Config
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT'))
        self.mqtt_ns = self.get_parameter('mqtt_ns').value
        self.mqtt_encoding = self.get_parameter('mqtt_encoding').value

        # Run
        self.robot_name = self.get_parameter('robot_name').value
        self.tracker_marker = os.getenv('STAG_TRACKER_NUMBER')
        self.markers = self.get_parameter('markers_to_subscribe').value
        self.marker_publishers = dict()
        for marker in self.markers:
            t = f'stag/markers/m{marker}'
            self.marker_publishers[marker] = self.create_publisher(PoseStamped, t, 10)
        t = 'stag/pose_array'
        self.pose_array_pub = self.create_publisher(PoseArray, t, 10)

        # Set new pose
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = self.get_clock().now().to_msg()
        self.pwcs = pwcs
        t = '/initialpose'
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, t, 10)

        # Setup
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads
        self.connect_to_mqtt()


    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT | Connected")
        self.mqtt_client.subscribe(f'{self.mqtt_ns}/camera_tf')
        self.mqtt_client.subscribe(f'{self.mqtt_ns}/pose_array')
        for marker in self.markers:
            self.mqtt_client.subscribe(f'{self.mqtt_ns}/markers/{marker}')


    def connect_to_mqtt(self):
        self.get_logger().warn(f'mqtt connection to {self.mqtt_ip}:{self.mqtt_port}')
        try:
            # Test with both pre and post paho 2.0.0
            try:
                self.get_logger().warn('running with paho version post-2.0.0')
                ver = mqtt.CallbackAPIVersion.VERSION1
                self.mqtt_client = mqtt.Client(ver, f'{self.robot_name}___stag_ros2_receiver')
            except:
                self.get_logger().warn('running with paho version pre-2.0.0')
                self.mqtt_client = mqtt.Client(f'{self.robot_name}___stag_ros2_receiver')
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().warn(str(e))
            self.get_logger().warn('Client connection failed. Not restarting.')

    def on_mqtt_message(self, client, userdata, msg):
        data = self.loads(msg.payload)
        if msg.topic.endswith('camera_tf'):
            self.camera_tf_cb(data)
        if msg.topic.endswith('pose_array'):
            self.pose_array_cb(data)
        if '/markers/' in msg.topic:
            self.marker_cb(data, msg.topic.split('/markers/')[-1])


    def camera_tf_cb(self, tf_data):
        # self.get_logger().warn(f'camera tf received')
        # Publish tf for camera
        msg = TransformStamped()
        msg.header.stamp.sec = tf_data['header']['stamp']['sec']
        msg.header.stamp.nanosec = tf_data['header']['stamp']['nanosec']
        msg.header.frame_id = tf_data['header']['frame_id']
        msg.child_frame_id = tf_data['child_frame_id']
        msg.transform.translation.x = tf_data['transform']['translation']['x']
        msg.transform.translation.y = tf_data['transform']['translation']['y']
        msg.transform.translation.z = tf_data['transform']['translation']['z']
        msg.transform.rotation.w = tf_data['transform']['rotation']['w']
        msg.transform.rotation.x = tf_data['transform']['rotation']['x']
        msg.transform.rotation.y = tf_data['transform']['rotation']['y']
        msg.transform.rotation.z = tf_data['transform']['rotation']['z']
        self.broadcaster.sendTransform(msg)
        self.stag_camera_tf = msg.child_frame_id


    def pose_array_cb(self, pa_data):
        # Publish full list of markers
        msg = PoseArray()
        msg.header.stamp.sec = pa_data['header']['stamp']['sec']
        msg.header.stamp.nanosec = pa_data['header']['stamp']['nanosec']
        msg.header.frame_id = pa_data['header']['frame_id']
        for p_data in pa_data['poses']:
            pose = Pose()
            pose.position.x = p_data['position']['x']
            pose.position.y = p_data['position']['y']
            pose.position.z = p_data['position']['z']
            pose.orientation.w = p_data['orientation']['w']
            pose.orientation.x = p_data['orientation']['x']
            pose.orientation.y = p_data['orientation']['y']
            pose.orientation.z = p_data['orientation']['z']
            msg.poses += [pose]
        # Publish full list of markers
        self.pose_array_pub.publish(msg)


    def marker_cb(self, p_data, marker_id):
        # Publish full list of markers
        msg = PoseStamped()
        msg.header.stamp.sec = p_data['header']['stamp']['sec']
        msg.header.stamp.nanosec = p_data['header']['stamp']['nanosec']
        msg.header.frame_id = p_data['header']['frame_id']
        msg.pose.position.x = p_data['pose']['position']['x']
        msg.pose.position.y = p_data['pose']['position']['y']
        msg.pose.position.z = p_data['pose']['position']['z']
        msg.pose.orientation.w = p_data['pose']['orientation']['w']
        msg.pose.orientation.x = p_data['pose']['orientation']['x']
        msg.pose.orientation.y = p_data['pose']['orientation']['y']
        msg.pose.orientation.z = p_data['pose']['orientation']['z']
        # Publish full list of markers
        self.marker_publishers[marker_id].publish(msg)
        if marker_id == self.tracker_marker:
            self.pose_update(msg)

    def rotate_180(self, w, x, y, z):
        w_rot, x_rot, y_rot, z_rot = 0,1,0,0

        # Quaternion multiplication (q * r)
        # q = w + xi + yj + zk
        # r = w_rot + x_rot*i + y_rot*j + z_rot*k
        new_w = w * w_rot - x * x_rot - y * y_rot - z * z_rot
        new_x = w * x_rot + x * w_rot + y * z_rot - z * y_rot
        new_y = w * y_rot - x * z_rot + y * w_rot + z * x_rot
        new_z = w * z_rot + x * y_rot - y * x_rot + z * w_rot

        return new_w, new_x, new_y, new_z

    def transform_pose(self, input_pose, from_frame, to_frame):
        # Ensure that the pose is a PoseStamped object
        pose_stamped = PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        try:
            # Wait for the necessary transformation to be available
            t = rclpy.duration.Duration(seconds=1)
            self.tf_buffer.can_transform(to_frame, from_frame, pose_stamped.header.stamp, timeout=t)
            # Perform the transformation
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame)
            return output_pose_stamped.pose
        except Exception as e:
            self.get_logger().error('Failed to transform pose: %s' % str(e))
            return None

    def pose_update(self, msg):
        d = self.get_parameter('pose_update_delay').value
        if msg.header.stamp.sec - self.pwcs.header.stamp.sec < d:
            return
        # Convert pose to upward facing marker
        ori = msg.pose.orientation
        w,x,y,z = self.rotate_180(ori.w, ori.x, ori.y, ori.z)
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        transformed_pose = self.transform_pose(msg.pose, msg.header.frame_id, 'map')
        #Format and publish new pose
        pwcs = PoseWithCovarianceStamped()
        pwcs.header = msg.header
        pwcs.header.frame_id = 'map'
        pwcs.pose.pose = transformed_pose
        self.initialpose_pub.publish(pwcs)
        self.pwcs = pwcs

def main(args=None):
    rclpy.init(args=args)

    Re = Receiver()
    rclpy.spin(Re)

    Re.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
