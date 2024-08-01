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

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Receiver(Node):

    def __init__(self):
        super().__init__('mqtt_receiver')

        self.declare_parameter('mqtt_encoding', rclpy.Parameter.Type.STRING)
        self.declare_parameter('mqtt_ns', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('markers_to_subscribe', rclpy.Parameter.Type.STRING_ARRAY)

        # Config
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT'))
        self.mqtt_ns = self.get_parameter('mqtt_ns').value
        self.mqtt_encoding = self.get_parameter('mqtt_encoding').value

        # Run
        self.robot_name = self.get_parameter('robot_name').value
        self.markers = self.get_parameter('markers_to_subscribe').value
        self.marker_publishers = dict()
        for marker in self.markers:
            t = f'markers/m{marker}'
            self.marker_publishers[marker] = self.create_publisher(PoseStamped, t, 10)

        # Setup
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
            self.camera_tf_cb(data)
        if '/markers/' in msg.topic:
            self.marker_cb(data, msg.topic.split('/markers/')[-1])


    def camera_tf_cb(self, tf_data):
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
        msg = PoseStamped()
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
        self.posearray_pub.publish(msg)


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

def main(args=None):
    rclpy.init(args=args)

    Re = Receiver()
    rclpy.spin(Re)

    Re.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
