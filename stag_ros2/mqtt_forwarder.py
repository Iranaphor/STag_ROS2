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

from geometry_msgs.msg import PoseArray


class Forwarder(Node):

    def __init__(self):
        super().__init__('forwarder')

        t = 'pose_array'
        self.pose_array_sub = self.create_subscription(PoseArray, t, self.pose_array_cb, 10)

        self.mqtt_ip = os.getenv('MQTT_BROKER_IP')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT'))
        self.mqtt_ns = 'stag_ros2'
        self.mqtt_encoding = 'json'
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads

        self.connect_to_mqtt()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT | Connected")

    #def on_mqtt_message(self, client, userdata, mqtt_msg):
    #    self.get_logger().info(t"MQTT | Topic: {mqtt_msg.topic}")

    def connect_to_mqtt(self):
        self.get_logger().warn(f'mqtt connection to {self.mqtt_ip}:{self.mqtt_port}')
        try:
            # Test with both pre and post paho 2.0.0
            try:
                self.get_logger().warn('running with paho version post-2.0.0')
                ver = mqtt.CallbackAPIVersion.VERSION1
                self.mqtt_client = mqtt.Client(ver, 'stag_ros2_publisher')
            except:
                self.get_logger().warn('running with paho version pre-2.0.0')
                self.mqtt_client = mqtt.Client('stag_ros2_publisher')
            self.mqtt_client.on_connect = self.on_connect
            #self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().warn(str(e))
            self.get_logger().warn('Client connection failed. Not restarting.')

    def pose_to_dict(self, pose):
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

    def pose_array_cb(self, msg):
        pa_dict = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'poses': [self.pose_to_dict(pose) for pose in msg.poses]
        }
        if not self.mqtt_client:
            return
        self.mqtt_client.publish(f'{self.mqtt_ns}/poses', self.dumps(pa_dict))



def main(args=None):
    rclpy.init(args=args)

    Fo = Forwarder()
    rclpy.spin(Fo)

    Fo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
