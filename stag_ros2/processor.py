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
from geometry_msgs.msg import PoseArray, Pose


class Processor(Node):

    def __init__(self):
        super().__init__('processor')
        self.bridge = CvBridge()

        self.hamming = 'HD19'
        self.marker_width = 0.0865
        self.usb_cam_fov = 68.5

        self.use_rolling_filter = True
        self.rolling_filter_len = 15
        self.buffer = dict()

        self.corners = None
        self.ids = None
        self.rejected_corners = None
        self.image_size = None

        t = 'tags'
        self.tags_pub = self.create_publisher(String, t, 10)

        t = 'pose_array'
        self.pose_array_pub = self.create_publisher(PoseArray, t, 10)

        t = 'calibration_array'
        self.pose_array2_pub = self.create_publisher(PoseArray, t, 10)

        t = 'image_labelled'
        self.image_pub = self.create_publisher(Image, t, 10)

        t = 'depth_labelled'
        self.depth_pub = self.create_publisher(Image, t, 10)

        t = 'image_raw'
        self.image_sub = self.create_subscription(Image, t, self.image_cb, 10)

        t = 'image_depth'
        self.depth_sub = self.create_subscription(Image, t, self.depth_cb, 10)


    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # detect markers
        hamming = int(self.hamming.split('HD')[-1])
        (corners, ids, rejected_corners) = stag.detectMarkers(image, hamming)
        self.corners = corners
        self.ids = ids
        self.rejected_corners = rejected_corners
        self.image_size = (msg.height, msg.width)

        if len(ids) > 0:
            marker_dict = dict()
            for i in range(len(ids)):
                # Save value for easy access
                marker_dict[ids[i][0]] = {'coordinates': corners[i][0].tolist()}
                # Begin buffering for new node
                if ids[i][0] not in self.buffer:
                    self.buffer[ids[i][0]] = {'poses':[], 'rots':[]}
            self.publish_cam_relative_pose(marker_dict, self.marker_width)

        # draw detected markers with ids
        stag.drawDetectedMarkers(image, corners, ids)
        stag.drawDetectedMarkers(image, rejected_corners, border_color=(255, 0, 0))

        # save resulting image
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_pub.publish(ros_image)
        #cv2.imshow("Converted Image", image)
        #cv2.waitKey(1)


######################################################################
######################################################################
######################################################################
######################################################################
######################################################################


    def publish_cam_relative_pose(self, marker_dict, marker_width):
        #self.get_logger().info(str(marker_dict))

        # This code only works assuming markers are placed perpendicular
        # to direction of camera (i.e. are viewed straight on)
        PA = PoseArray()
        #PA.header.frame_id = 'map'
        PA.header.frame_id = 'stag_camera_link'
        for id, values in marker_dict.items():
            corners = values['coordinates']

            # Calculate the differences
            dx = corners[1][0] - corners[0][0]
            dy = corners[1][1] - corners[0][1]
            angle_radians = np.arctan2(dy, dx)
            angle_degrees = round(np.degrees(angle_radians)+90,2)
            #angle_degrees = angle_degrees if angle_degrees >=0 else 360 + angle_degrees
            values['degrees'] = angle_degrees


            # Calculate the rolling filter
            if self.use_rolling_filter:
                # Add latest pose
                self.buffer[id]['rots'].append(angle_degrees)
                # Remove oldest rotation
                #self.get_logger().info(str(len(self.buffer[id]['rots'])))
                if len(self.buffer[id]['rots']) > self.rolling_filter_len:
                    del self.buffer[id]['rots'][0]
                # Determine iqr mean value
                angle_degrees = np.median(np.array(self.buffer[id]['rots']))


            # Determine orientation
            angle_radians = np.deg2rad(angle_degrees)
            p = Pose()
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = np.sin(angle_radians / 2)
            p.orientation.w = np.cos(angle_radians / 2)
            values['pose'] = p

            # Calculate centre of object as point between opposite corners
            center = [(corners[0][0]+corners[2][0])/2, (corners[0][1]+corners[2][1])/2]

            # Calculate the focal length
            fov_radians = np.deg2rad(self.usb_cam_fov)
            focal_length = self.image_size[1] / (2 * np.tan(fov_radians / 2))

            # Assuming we have the width in pixels of the object (need actual measurement)
            width_in_pixels = np.sqrt( dx**2 + dy**2 )

            # Calculate the distance to the object
            pZ = focal_length * (self.marker_width / width_in_pixels)
            p.position.z = round(pZ,3) #dont make this negative

            # Calculate the position of the object in the camera's frame
            pX = (center[0] - (self.image_size[1] / 2)) * (pZ / focal_length) #dont make this negative
            pY = (center[1] - (self.image_size[0] / 2)) * (pZ / focal_length)
            p.position.x = round(pX,3)
            p.position.y = round(pY,3)

            # Calculate the rolling filter
            if self.use_rolling_filter:
                # Add latest pose
                self.buffer[id]['poses'].append([p.position.x, p.position.y, p.position.z])
                # Remove oldest pose
                #self.get_logger().info(str(len(self.buffer[id]['poses'])))
                if len(self.buffer[id]['poses']) > self.rolling_filter_len:
                    del self.buffer[id]['poses'][0]
                # Determine iqr mean value
                filt = np.median(np.array(self.buffer[id]['poses']), axis=0)
                p.position.x, p.position.y, p.position.z = filt[0], filt[1], filt[2]

            # Debug the values
            self.get_logger().debug(f"ID:{id}, Deg:{angle_degrees}, X:{p.position.x}, Y:{p.position.y}, Z:{p.position.z}")

            # Add to pose array
            PA.poses.append(p)

        # Publish Pose Array
        self.pose_array_pub.publish(PA)
        PA.header.frame_id = ','.join([str(i) for i in marker_dict.keys()])
        self.pose_array2_pub.publish(PA)


######################################################################
######################################################################
######################################################################
######################################################################
######################################################################


    def depth_cb(self, msg):
        mn = str(np.max(np.frombuffer(msg.data, dtype=np.uint8)))
        mx = str(np.min(np.frombuffer(msg.data, dtype=np.uint8)))
        self.get_logger().info(mn+", "+mx)

        # Get latest marker info
        if self.ids is None:
            self.get_logger().info('exit')
            return
        corners = self.corners
        ids = self.ids
        rejected_corners = self.rejected_corners
        image_size = self.image_size


        def convert_depth_to_rgb(depth_image):
            # Normalize the depth image to max value of 255
            normalized_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # Convert to 8-bit image
            normalized_img = np.uint8(normalized_img)
            # Convert the normalized image to RGB
            rgb_image = cv2.cvtColor(normalized_img, cv2.COLOR_GRAY2RGB)
            return rgb_image

        def convert_rgb_to_depth(rgb_image):
            # Convert RGB to grayscale
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            # Scale back to original depth range, this scale factor depends on your specific use case
            scale_factor = 65535 / 255
            # Here, just an example scaling factor; adjust it as per your application's depth scale
            depth_image = (gray_image * scale_factor).astype(np.uint16)
            return depth_image

        def transform(rgb_x1, rgb_y1, depth_width, depth_height, rgb):
            # Camera resolutions
            rgb_width, rgb_height = rgb[0], rgb[1]
            depth_width, depth_height = depth_height, depth_width

            # Field of Views
            rgb_fov_h, rgb_fov_v = 69, 42
            depth_fov_h, depth_fov_v = 87, 58

            # Calculate angular resolutions (degrees per pixel)
            rgb_ar_h = rgb_fov_h / rgb_width
            rgb_ar_v = rgb_fov_v / rgb_height
            depth_ar_h = depth_fov_h / depth_width
            depth_ar_v = depth_fov_v / depth_height

            # Calculate angles for each corner in the RGB image
            angle_x1 = (rgb_x1 - rgb_width / 2) * rgb_ar_h
            angle_y1 = (rgb_y1 - rgb_height / 2) * rgb_ar_v

            # Map angles to depth image coordinates
            depth_x1 = (angle_x1 / depth_ar_h) + depth_width / 2
            depth_y1 = (angle_y1 / depth_ar_v) + depth_height / 2
            return depth_x1, depth_y1

        # Example usage
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        rgb_img = convert_depth_to_rgb(depth_img)

        # draw detected markers with ids
        #resize corners
        height_proportion = msg.height / self.image_size[0]
        width_proportion = msg.width / self.image_size[1]
        for rect in corners:
            for c in rect[0]:
                c[0], c[1] = transform(c[0], c[1], msg.width, msg.height, self.image_size)
        for rect in rejected_corners:
            for c in rect[0]:
                c[0], c[1] = transform(c[0], c[1], msg.width, msg.height, self.image_size)
        stag.drawDetectedMarkers(rgb_img, corners, ids, border_color=(0,0,0))
        stag.drawDetectedMarkers(rgb_img, rejected_corners, border_color=(0, 0, 0))

        # save resulting image
        img = convert_rgb_to_depth(rgb_img)
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.depth_pub.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)

    Pr = Processor()
    rclpy.spin(Pr)

    Pr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
