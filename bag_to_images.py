#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def parse_args():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument('data_dir', help='Top level data dir')
    parser.add_argument('topic', help='Image topic.')
    parser.add_argument('frame_rate', type=int, help='Capter every n frames from bag')
    #parser.add_argument("bag_file", help="Input ROS bag.")
    #parser.add_argument("output_dir", help="Output directory.")
    #parser.add_argument("image_topic", help="Image topic.")

    return parser.parse_known_args()

def main():

    ARGS, unused = parse_args()

    print('Extracting images in top level dir: %s' % ARGS.data_dir)

    #print "Extract images from %s on topic %s into %s" % (args.bag_file,
    #                                                      args.image_topic, args.output_dir)

    bridge = CvBridge()

    for directory in os.listdir(ARGS.data_dir):
        for bag_file in os.listdir(os.path.join(ARGS.data_dir, directory)):
            # create necessary path to store image data
            instance, ext = os.path.splitext(bag_file)
            instance_path = os.path.join(ARGS.data_dir, 'images', directory, instance)
            os.makedirs(instance_path)
           
            bag = rosbag.Bag(os.path.join(ARGS.data_dir, directory, bag_file), 'r')

            frame = 0
            for topic, msg, t in bag.read_messages(topics=[ARGS.topic]):
                frame += 1

                # Get every nth
                if frame % ARGS.frame_rate:
                    continue

                image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                y = len(image)
                x = len(image[0])
                y_crop = y / 6
                x_crop = x / 3
                cropped_image = image[y_crop / 2:y - y_crop, x_crop:x - x_crop]
                
                #cv2.imshow('original', image)
                #cv2.imshow('cropoped', cropped_image)
                #cv2.waitKey(0)

                cv2.imwrite(os.path.join(instance_path, '%s_%d.png' % (instance, frame / ARGS.frame_rate)), cropped_image)
                
            bag.close()
    return

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        print "Wrote image %i" % count

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
