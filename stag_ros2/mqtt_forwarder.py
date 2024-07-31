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
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseArray, TransformStamped


class Forwarder(Node):

    def __init__(self):
        super().__init__('mqtt_forwarder')

        self.declare_parameter('mqtt_encoding', rclpy.Parameter.Type.STRING)
        self.declare_parameter('mqtt_ns', rclpy.Parameter.Type.STRING)

        self.stag_camera_link = None
        t = 'camera_tf'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.camera_tf_sub = self.create_subscription(TransformStamped, t, self.camera_tf_cb, qos)

        t = 'pose_array'
        self.pose_array_sub = self.create_subscription(PoseArray, t, self.pose_array_cb, 10)

        t = 'calibration_array'
        self.calibration_array_sub = self.create_subscription(PoseArray, t, self.calibration_array_cb, 10)

        self.mqtt_ip = os.getenv('MQTT_BROKER_IP') #rosparam?
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT')) #rosparam?
        self.mqtt_ns = self.get_parameter('mqtt_ns').value
        self.mqtt_encoding = self.get_parameter('mqtt_encoding').value
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads

        self.connect_to_mqtt()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT | Connected")

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


    def camera_tf_cb(self, msg):
        if not self.mqtt_client: return
        # Publish tf for camera
        tf_dict = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'child_frame_id': msg.child_frame_id,
            'transform': {
                'translation': {
                    'x': msg.transform.translation.x,
                    'y': msg.transform.translation.y,
                    'z': msg.transform.translation.z
                },
                'rotation': {
                    'w': msg.transform.rotation.w,
                    'x': msg.transform.rotation.x,
                    'y': msg.transform.rotation.y,
                    'z': msg.transform.rotation.z
                }
            }
        }
        self.mqtt_client.publish(f'{self.mqtt_ns}/camera', self.dumps(tf_dict), retain=True)
        self.stag_camera_link = msg.child_frame_id

    def pose_array_cb(self, msg):
        # Exit if tf is not published or client is not ready
        if not self.mqtt_client: return
        if not self.stag_camera_link: return
        # Publish full list of markers
        pa_dict = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'poses': [{
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'w': pose.orientation.w,
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z
                }
            } for pose in msg.poses]
        }
        self.mqtt_client.publish(f'{self.mqtt_ns}/poses', self.dumps(pa_dict))

    def calibration_array_cb(self, msg):
        # Exit if tf is not published or client is not ready
        if not self.mqtt_client: return
        if not self.stag_camera_link: return
        # Publish markers to individual topics (robots can subscribe to stag/markers/$personal_marker)
        ids = msg.header.frame_id.split(',')
        for i, pose in enumerate(msg.poses):
            ma_dict = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': self.stag_camera_link
                },
                'pose': {
                    'position': {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    },
                    'orientation': {
                        'w': pose.orientation.w,
                        'x': pose.orientation.x,
                        'y': pose.orientation.y,
                        'z': pose.orientation.z
                    }
                }
            }
            self.mqtt_client.publish(f'{self.mqtt_ns}/markers/{ids[i]}', self.dumps(ma_dict))



def main(args=None):
    rclpy.init(args=args)

    Fo = Forwarder()
    rclpy.spin(Fo)

    Fo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
