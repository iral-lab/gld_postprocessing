#!/usr/bin/env python3

import os
import argparse
import numpy as np
import struct
import ctypes
import rospy
import image_geometry
import math
import pcl
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from PIL import Image

MIN_DIST = 0.0
MAX_DIST = 4.5

def float_to_rgb(float_rgb):

    """ Converts a packed float RGB format to an RGB list

        Args:
            float_rgb: RGB value packed as a float

        Returns:
            color (list): 3-element list of integers [0-255,0-255,0-255]
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    color = [r,g,b]

    return color


class PointCloudToImages():
	def __init__(self):
		self.rgb_cam_info = CameraInfo()
		self.rgb_cam_info.header.frame_id = "rgb_camera_link"
		self.rgb_cam_info.height = 1080
		self.rgb_cam_info.width = 1920
		self.rgb_cam_info.distortion_model = "rational_polynomial"
		self.rgb_cam_info.D = [0.6736099123954773, -2.707728385925293, 0.001006936188787222, -0.0004615978104993701, 1.5371538400650024, 0.5529225468635559, -2.5477864742279053, 1.4728691577911377]
		self.rgb_cam_info.K = [900.3588256835938, 0.0, 958.7286987304688, 0.0, 900.0320434570312, 549.4136352539062, 0.0, 0.0, 1.0]
		self.rgb_cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.rgb_cam_info.P = [900.3588256835938, 0.0, 958.7286987304688, 0.0, 0.0, 900.0320434570312, 549.4136352539062, 0.0, 0.0, 0.0, 1.0, 0.0]
		self.rgb_cam_info.binning_x = 0
		self.rgb_cam_info.binning_y = 0
		self.rgb_cam_info.roi.x_offset = 0
		self.rgb_cam_info.roi.y_offset = 0
		self.rgb_cam_info.roi.height = 0
		self.rgb_cam_info.roi.width = 0
		self.rgb_cam_info.roi.do_rectify = False

		self.depth_cam_info = CameraInfo()
		self.depth_cam_info.header.frame_id = "depth_camera_link"
		self.depth_cam_info.height = 576
		self.depth_cam_info.width = 640
		self.depth_cam_info.distortion_model = "rational_polynomial"
		self.depth_cam_info.D = [1.085646390914917, 0.7031514644622803, -8.193295798264444e-05, 2.8774375095963478e-05, 0.04076514020562172, 1.4187815189361572, 1.0156532526016235, 0.20575040578842163]
		self.depth_cam_info.K = [504.8343811035156, 0.0, 326.8834228515625, 0.0, 504.8576965332031, 348.2257080078125, 0.0, 0.0, 1.0]
		self.depth_cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.depth_cam_info.P = [504.8343811035156, 0.0, 326.8834228515625, 0.0, 0.0, 504.8576965332031, 348.2257080078125, 0.0, 0.0, 0.0, 1.0, 0.0]
		self.depth_cam_info.binning_x = 0
		self.depth_cam_info.binning_y = 0
		self.depth_cam_info.roi.x_offset = 0
		self.depth_cam_info.roi.y_offset = 0
		self.depth_cam_info.roi.height = 0
		self.depth_cam_info.roi.width = 0
		self.depth_cam_info.roi.do_rectify = False

		self.cam_model = image_geometry.StereoCameraModel()
		self.cam_model.fromCameraInfo(self.depth_cam_info, self.rgb_cam_info)

	def convert(self, pcd_file, data_dir):
		instance, ext = os.path.splitext(pcd_file)

		cloud = pcl.load_XYZRGB(data_dir+"/"+pcd_file)

		color_path = data_dir+"/../color/"+instance+"_color.png"
		depth_path = data_dir+"/../depth/"+instance+"_depth.png"

		w_depth = self.depth_cam_info.width
		h_depth = self.depth_cam_info.height

		w_rgb = self.rgb_cam_info.width
		h_rgb = self.rgb_cam_info.height
		
		color_arr = np.zeros( (h_rgb, w_rgb, 3), np.int8)
		depth_arr = np.zeros( (h_depth, w_depth), np.int8)

		for p in cloud:
			d = math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])
			r,g,b = float_to_rgb(p[3])

			left,right = self.cam_model.project3dToPixel( [ p[0], p[1], p[2] ] )

			u_depth = int(left[0])
			v_depth = int(left[1])

			u_rgb = int(right[0])
			v_rgb = int(right[1])

			color_arr[v_rgb][u_rgb][0] = r
			color_arr[v_rgb][u_rgb][1] = g
			color_arr[v_rgb][u_rgb][2] = b
			
			depth_arr[v_depth][u_depth] = ((d- MIN_DIST) / (MAX_DIST - MIN_DIST)) * 255

		depth_img = Image.fromarray(depth_arr, 'L')
		depth_img.save(depth_path)

		color_img = Image.fromarray(color_arr, 'RGB')
		color_img.save(color_path)
		print("Wrote depth and color to ",depth_path," and ",color_path)
		
def parse_args():
	"""Extract a folder of images from a rosbag.
	"""
	parser = argparse.ArgumentParser(description="Extract color and depth from a PCD file.")
	parser.add_argument('data_dir', help='Top level data dir')
	parser.add_argument('frame_rate', type=int, help='Capture every n frames')

	return parser.parse_known_args()

def main():
	ARGS, unused = parse_args()

	print('Extracting images from pcds in top level dir: %s' % ARGS.data_dir)
	
	color_path = os.path.join(ARGS.data_dir, '..', 'color')
	os.makedirs(color_path)

	depth_path = os.path.join(ARGS.data_dir, '..', 'depth')
	os.makedirs(depth_path)

	pcd_to_images = PointCloudToImages()

	for pcd_file in os.listdir(ARGS.data_dir):
		pcd_to_images.convert(pcd_file, ARGS.data_dir)

if __name__ == '__main__':
	main()



