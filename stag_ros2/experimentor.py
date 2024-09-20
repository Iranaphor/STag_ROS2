# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
import cv2
import time
import shutil
import threading

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt64
from sensor_msgs.msg import Image

import stag_ros2.marker_generators.utils as utils
from stag_ros2.marker_generators.generate_occlusions import generate_masks, apply_mask


class Experimentor(Node):

    def __init__(self):
        super().__init__('experimentor')
        self.bridge = CvBridge()

        # Initialize the lock and condition variable
        self.lock = threading.Condition()
        self.lock_value = False  # Start with lock being False

        # Define subscriber and publisher
        t = '/cam1/image_raw'
        self.image_pub = self.create_publisher(Image, t, 10)

        t = '/cam1/ids'
        self.ids_sub = self.create_subscription(UInt64, t, self.ids_cb, 10)

        # Start the main loop in a separate thread
        threading.Thread(target=self.start).start()


    def start(self):

        # 0. Prepare Directories
        masks_dir = f"{os.getenv('HOME')}/STag-Markers/occlusions/setA/"
        generate_masks(masks_dir, total_masks=1000, min_size=150, max_size=150, total_occlusions=1)
        masks = utils.list_files(masks_dir)

        # 0. Prepare Directories
        directories_to_assess = [
            f"{os.getenv('HOME')}/STag-Markers/standard/HD23/",
            #f"{os.getenv('HOME')}/STag-Markers/high-capacity/HC23/",
            f"{os.getenv('HOME')}/STag-Markers/high-occlusion/HO23/",
        ]

        # 0. Loop through each experiment directory
        total_fails = dict()
        for i,input_dir in enumerate(directories_to_assess):
            images = utils.list_files(input_dir)

            total_fails[input_dir] = 0

            # 1. Loop through each marker image to apply masks
            for f_image in images:
                #self.get_logger().warn(f"Processing Marker {f_image}")

                # 2. Load the image
                fp_image = os.path.join(input_dir, f_image)
                image = cv2.imread(fp_image)
                if image is None:
                    self.get_logger().error(f"Failed to load image from {fp_image}")
                    continue

                # 3. Loop through each mask to apply
                for f_mask in masks:

                    # 4. Load the mask
                    fp_mask = os.path.join(masks_dir, f_mask)
                    mask = cv2.imread(fp_mask)
                    if mask is None:
                        self.get_logger().error(f"Failed to load mask from {fp_mask}")
                        continue

                    # 4. Apply the mask to the image
                    masked_image = apply_mask(image, mask, (255,128,255))

                    # 5. Prepare lock
                    with self.lock:
                        self.lock_value = True  # Set lock to True

                        # 4. Publish the image to the processor
                        image_msg = self.bridge.cv2_to_imgmsg(masked_image, encoding="bgr8")
                        self.image_pub.publish(image_msg)

                        # 5. Lock waiting for response
                        timeout = 0.5
                        end_time = time.monotonic() + timeout
                        while self.lock_value:
                            remaining = end_time - time.monotonic()
                            if remaining <= 0:
                                self.id = 'x'
                                total_fails[input_dir] += 1
                                self.lock_value = False  # Release the lock
                                break
                            self.lock.wait(timeout=remaining)
                        self.get_logger().info(f'Detected, {self.id}, with image, {f_image}, mask, {f_mask}')
            self.get_logger().info(f'Total Failures, {total_fails[input_dir]}')
        self.get_logger().info(f'Totals, {str(total_fails)}')


    def ids_cb(self, msg):
        with self.lock:
            self.id = msg.data
            self.lock_value = False  # Set lock to False
            self.lock.notify()  # Notify the main loop


def main(args=None):
    rclpy.init(args=args)

    Ex = Experimentor()
    rclpy.spin(Ex)

    Ex.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
