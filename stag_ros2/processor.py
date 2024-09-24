# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import math
import numpy as np
import sympy
import cv2
from cv_bridge import CvBridge
from statistics import mean

import stag

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

from stag_ros2.utils.rotate_region import rotate_channels
from stag_ros2.utils.process_polygon import process_polygon

class Processor(Node):

    def __init__(self):
        super().__init__('processor')
        self.bridge = CvBridge()

        self.declare_parameter('marker_set', rclpy.Parameter.Type.STRING)
        self.declare_parameter('marker_width', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('only_process_image_on_trigger', rclpy.Parameter.Type.BOOL)

        self.declare_parameter('overlay_proximity', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('merge_option_1', rclpy.Parameter.Type.STRING)
        self.declare_parameter('merge_option_2', rclpy.Parameter.Type.STRING)

        self.declare_parameter('dfov', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('hfov', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('use_rolling_filter', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('rolling_filter_len', rclpy.Parameter.Type.INTEGER)

        self.declare_parameter('label_color_image', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('label_depth_image', rclpy.Parameter.Type.BOOL)


        # For accurate position estimation
        self.marker_set = self.get_parameter('marker_set').value
        self.marker_width = self.get_parameter('marker_width').value
        self.dfov = self.get_parameter('dfov').value
        self.hfov = self.get_parameter('hfov').value

        # For pose filtering
        self.use_rolling_filter = self.get_parameter('use_rolling_filter').value
        self.rolling_filter_len = self.get_parameter('rolling_filter_len').value
        self.buffer = dict()

        # For handling detections
        self.corners = None
        self.ids = None
        self.rejected_corners = None
        self.image_size = None

        # For High-Capacity marker detection
        self.overlay_proximity = int(self.get_parameter('overlay_proximity').value)
        self.merge_option_1 = self.get_parameter('merge_option_1').value
        self.merge_option_2 = self.get_parameter('merge_option_2').value

        # For experimentation
        self.only_process_image_on_trigger = bool(self.get_parameter('only_process_image_on_trigger').value)
        self.process_image_trigger = True
        t = 'trigger_processor'
        self.trigger_sub = self.create_subscription(Empty, t, self.trigger_cb, 10)
        t = 'input_file'
        self.input_file_sub = self.create_subscription(String, t, self.input_file_cb, 10)

        # For rendering
        self.label_color_image = self.get_parameter('label_color_image').value
        self.label_depth_image = self.get_parameter('label_depth_image').value

        t = 'ids'
        self.ids_pub = self.create_publisher(String, t, 10)

        t = 'calibration_array'
        self.pose_array2_pub = self.create_publisher(PoseArray, t, 10)

        if self.label_color_image:
            t = 'image_labelled'
            self.image_pub = self.create_publisher(Image, t, 10)

        if self.label_depth_image:
            t = 'depth_labelled'
            self.depth_pub = self.create_publisher(Image, t, 10)

        t = 'image_raw'
        self.image_sub = self.create_subscription(Image, t, self.image_cb, 10)

        t = 'image_depth'
        self.depth_sub = self.create_subscription(Image, t, self.depth_cb, 10)


    def configure_fov(self):
        if self.hfov: return

        # Calculate the aspect ratio
        aspect_ratio = self.image_size[1] / self.image_size[0]

        # Convert DFOV from degrees to radians
        dfov_radians = math.radians(self.dfov)

        # Calculate the tangent of half the DFOV
        tan_half_dfov = math.tan(dfov_radians / 2)

        # Calculate the denominator of the argument of arctan
        denominator = math.sqrt(1 + aspect_ratio**-2)

        # Divide the tangent by the denominator
        arg = tan_half_dfov / denominator

        # Calculate the arctan of the result
        hfov_radians = 2 * math.atan(arg)

        # Convert the result back to degrees
        self.hfov = math.degrees(hfov_radians)
        return


    def merge_image_findings(self, image, hamming, save=True):
        # detect markers
        corners, ids, rejected_corners = stag.detectMarkers(image, hamming)
        if save:
            self.corners = self.corners + corners
            self.ids = np.concatenate((self.ids, ids), axis=0)
            self.rejected_corners = self.rejected_corners + rejected_corners

        # Return early if not allowing flipped images
        return corners, ids, rejected_corners


    def merge_overlays(self, data, include_ids=True):

        # Get centrpoints of each bounding box
        for slice in data.values():
            slice['m'] = []
            for c in slice['c']:
                if len(c) > 0:
                    slice['m'] += [[round(mean(c[0][:,0])), round(mean(c[0][:,1]))]]
                else:
                    slice['m'] += []

        # Detemine if there are overlapping boxes
        p = self.overlay_proximity #proximity in px
        markers = []
        for i,rm in enumerate(data['r']['m']):

            # Check if Green is overlapping with Red
            r_g = []
            for gm in data['g']['m']:
                r_g += [abs(gm[0] - rm[0]) <= p and abs(gm[1] - rm[1]) <= p]
                #TODO: instead, check proximity of all four corners

            # Check if Blue is overlapping with Red
            r_b = []
            for bm in data['b']['m']:
                r_b += [abs(bm[0] - rm[0]) <= p and abs(bm[1] - rm[1]) <= p]

            # Check if there were overlaps
            if any(r_g) and any(r_b):
                if include_ids:
                    ri = data['r']['i'][i]
                    gi = data['g']['i'][r_g.index(True)]
                    bi = data['b']['i'][r_b.index(True)]

                    # Add marker if they are not the same in all slices
                    markers += [{'r':ri, 'g':gi, 'b':bi, 'corners':data['r']['c'][i]}]
                else:
                    markers += [{'corners':data['r']['c'][i]}]
        return markers


    def decode_high_capacity(self, hamming, markers):

        # Convert triple marker ids to joined ones
        combined_markers = []
        for m in markers:
            mr = m['r'][0]
            mg = m['g'][0]
            mb = m['b'][0]

            file_lengths = {'23':6, '21':12, '19':38, '17':157, '15':766, '13':2884, '11':22335}

            # Scaled Bits
            sr = mr * (file_lengths[hamming[0]]**0)
            sg = mg * (file_lengths[hamming[1]]**1)
            sb = mb * (file_lengths[hamming[2]]**2)
            uuid = sr + sg + sb

            # Log Output
            s = f"UUID: MarkerID({mr},{mg},{mb}), UUID({uuid})"
            self.get_logger().error(s)

            # Save Output
            combined_markers += [{'uuid':uuid, 'corners':m['corners']}]

        # Add combined markers to the detection result
        corners = tuple()
        ids = np.array([[]], dtype=int).transpose()
        for m in combined_markers:
            corners = corners + (m['corners'],)
            u = np.array([[m['uuid']]], dtype=int).transpose()
            ids = np.concatenate((ids, u), axis=0)

        self.corners = self.corners + corners
        self.ids = np.concatenate((self.ids, ids), axis=0)


    def trigger_cb(self, msg):
        self.process_image_trigger = True

    def input_file_cb(self, msg):
        # Read the image using OpenCV and convert to Image
        image = cv2.imread(msg.data)
        if image is None:
            self.get_logger().error(f"Failed to load image from {self.image_file_path}")
            return
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_cb(image_msg)

    def image_cb(self, msg):

        # Only trigger processing when commanded
        if self.only_process_image_on_trigger:
            if not self.process_image_trigger:
                return
            self.process_image_trigger = False

        # Format image for processing
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_size = (msg.height, msg.width)

        # Default the values
        self.corners = tuple()
        self.ids = np.array([[]], dtype=int).transpose()
        self.rejected_corners = tuple()
        data = { 'r':dict(), 'g':dict(), 'b':dict() }

        # TODO: Define marker set "temporarilly"
        self.marker_set = self.get_parameter('marker_set').value
        #self.marker_set = 'HD19'
        #self.marker_set = 'HG19'
        #self.marker_set = 'HC19'
        #self.marker_set = 'HC192311'
        #self.marker_set = 'HO19'

        ####################################################################################
        # Detect Standard HD markers
        if self.marker_set.startswith('HD'):
            hamming = int(self.marker_set.replace('HD',''))
            self.merge_image_findings(image, hamming)

        ####################################################################################
        # Detect Greyscale markers
        elif self.marker_set.startswith('HG'):
            hamming = int(self.marker_set.replace('HG',''))
            self.merge_image_findings(image, hamming)
            self.merge_image_findings(255 - image, hamming)

        ####################################################################################
        # For High-Capacity detection, merge polygons if overlapping and decode detected ids
        elif self.marker_set.startswith('HC'):
            # Seperate Image
            b, g, r = image[:, :, 0], image[:, :, 1], image[:, :, 2]
            blue, green, red = cv2.merge([b, b, b]), cv2.merge([g, g, g]), cv2.merge([r, r, r])
            hamming = int(self.marker_set.replace('HC',''))

            # Detect R, G, and B markers
            data['r']['c'], data['r']['i'], _ = self.merge_image_findings(r, hamming, save=False)
            data['g']['c'], data['g']['i'], _ = self.merge_image_findings(g, hamming, save=False)
            data['b']['c'], data['b']['i'], _ = self.merge_image_findings(b, hamming, save=False)

            # Merge detections across all three channels
            markers = self.merge_overlays(data)

            # Determing hamming distance to use
            ids = self.marker_set.replace('HC','')
            hamming = [ids, ids, ids]
            if len(ids) == 6:
                hamming = [ids[0:2],ids[2:4],ids[4:6]]

            # Decode ids
            self.decode_high_capacity(hamming, markers)

        ####################################################################################
        # For High-Occlusion detection, merge polygons if overlapping, align the channels, detect ids
        elif self.marker_set.startswith('HO'):
            # Seperate Image
            b, g, r = image[:, :, 0], image[:, :, 1], image[:, :, 2]
            blue, green, red = cv2.merge([b, b, b]), cv2.merge([g, g, g]), cv2.merge([r, r, r])
            hamming = int(self.marker_set.replace('HO',''))

            # Detect R, G, and B markers
            r_corners, _, r_reject = self.merge_image_findings(r, hamming, save=False)
            g_corners, _, g_reject = self.merge_image_findings(g, hamming, save=False)
            b_corners, _, b_reject = self.merge_image_findings(b, hamming, save=False)
            #print(f"Rectangles in R={len(r_reject)}, G={len(g_reject)}, B={len(b_reject)}")

            # 1. Merge bounding boxes from corners and rejected_corners
            data['r']['c'] = r_corners + r_reject
            data['g']['c'] = g_corners + g_reject
            data['b']['c'] = b_corners + b_reject
            unlabelled_markers = self.merge_overlays(data, include_ids=False)
            #print(f"Unlabelled Markers: {len(unlabelled_markers)}")

            # 2. Extract the regions and rotate the channels
            #print('\n\n\n\n\n\n---------\n\n\n')
            for marker in unlabelled_markers:
                polygon = [(int(m[0]),int(m[1])) for m in marker['corners'][0]]
                print(polygon)
                image = rotate_channels(polygon, image)
                image = process_polygon(polygon, image, join_method=self.merge_option_1)
                image = process_polygon(polygon, image, join_method=self.merge_option_2)


            #   3. Pass corrected image back for detection again
            self.merge_image_findings(image, hamming)

            # OPTION 2:
            #   3. Tidy them joined up
            #   4. Pass each extracted marker for detection
            pass


        # Save output for local reference
        ids = self.ids
        corners = self.corners
        rejected_corners = self.rejected_corners

        # Publish simple list of detected ids
        if len(ids) > 0:
            S = String()
            S.data = f"i{ids[0][0]}/c{len(self.corners)}/rc{len(self.rejected_corners)}"
            if 'c' not in data['r']:
                S.data += "/r0/g0/b0"
            else:
                S.data += f"/r{len(data['r']['c'])}/g{len(data['g']['c'])}/b{len(data['b']['c'])}"
            self.ids_pub.publish(S)


        # Send marker dictionary to determine the marker poses relative to the camera
        if len(ids) > 0:
            marker_dict = dict()
            for i in range(len(ids)):
                marker_dict[ids[i][0]] = {'coordinates': corners[i][0].tolist()}
                # Begin buffering for new marker ids
                if ids[i][0] not in self.buffer:
                    self.buffer[ids[i][0]] = {'poses':[], 'rots':[]}
            self.publish_cam_relative_pose(marker_dict, self.marker_width)


        # Draw detected markers on the original image and publish to ros topic
        if self.label_color_image:
            stag.drawDetectedMarkers(image, corners, ids)
            stag.drawDetectedMarkers(image, rejected_corners, border_color=(255, 0, 0))
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.image_pub.publish(ros_image)


    def publish_cam_relative_pose(self, marker_dict, marker_width):
        # This code only works assuming markers are placed perpendicular
        # to direction of camera (i.e. are viewed straight on)
        PA = PoseArray()
        PA.header.stamp = self.get_clock().now().to_msg()
        PA.header.frame_id = 'map'
        for id, values in marker_dict.items():
            corners = values['coordinates']

            # Calculate the differences
            dx = corners[1][0] - corners[0][0]
            dy = corners[1][1] - corners[0][1]
            angle_radians = np.arctan2(dy, dx)
            angle_degrees = round(np.degrees(angle_radians)+90,2)
            #if id == 2:
            #    self.get_logger().info('m2='+str(angle_degrees))
            values['degrees'] = angle_degrees

            # Calculate the rolling filter
            self.use_rolling_filter = self.get_parameter('use_rolling_filter').value
            if self.use_rolling_filter:
                self.rolling_filter_len = self.get_parameter('rolling_filter_len').value
                # Add latest rotation
                self.buffer[id]['rots'].append(angle_degrees)
                # Remove oldest rotation
                if len(self.buffer[id]['rots']) > self.rolling_filter_len:
                    del self.buffer[id]['rots'][:-self.rolling_filter_len]
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
            if not self.hfov: self.configure_fov()
            fov_radians = np.deg2rad(self.hfov)
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
        PA.header.frame_id = ','.join([str(i) for i in marker_dict.keys()])
        self.pose_array2_pub.publish(PA)


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
