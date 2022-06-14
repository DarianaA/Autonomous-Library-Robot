#!/usr/bin/python

from __future__ import print_function
import random
import roslib
import time
import sys
import os
import argparse
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import rospkg

class yolo_detector:

    def __init__(self):
        print('capture image server running. publish dataset name to /cap_images please')
        # initialise taken to 20 (prevent instant capture) and time since last image
        self.images_taken = 200
        self.last_time = time.time()
        self.object_counter = 0
        # set up bridge to opencv + publisher/subscriber
        self.bridge = CvBridge()
        self.started = False
        rospy.Subscriber('xtion/rgb/image_raw', Image, self.capture_callback)
        rospy.Subscriber("cap_images", String, self.start_callback)

    def start_callback(self, data):
        # validate input
        self.image_name = data.data.split(' ')
        if len(self.image_name) != 2:
            print('Invalid input. Please publish a String <dataset> <image brief>')
            return
        
        if not os.path.exists(os.path.dirname(os.path.realpath(__file__)) + '/datasets/' + self.image_name[0]):
            os.makedirs(os.path.dirname(os.path.realpath(__file__)) + '/datasets/' + self.image_name[0])
        
        # get the current index of images in dataset
        self.index_file_path = os.path.dirname(os.path.realpath(__file__)) + '/datasets/' + self.image_name[0] + '/.current_index'
        if os.path.exists(self.index_file_path):
            with open(self.index_file_path, 'r') as f:
                self.imageNo = int(f.readline().splitlines()[0])
        else:
            self.imageNo = 0

        self.images_taken = 0
        print('current index for ' + self.image_name[0] + ' dataset is: ' + str(self.imageNo))
        print('taking 20 images from topic: ' + args['topic'])
        # self.spawn_three_objects()
        self.started = True

    def capture_callback(self, data):
        # get Image if 1 second has passed and current
        if(time.time() - self.last_time >= 1) and self.images_taken <= 200:
            # try to convert xtion image to opencv
            try:
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            
            #writeImage
            location = os.path.dirname(os.path.realpath(__file__)) + \
                '/datasets/' + self.image_name[0] + '/' + self.image_name[0] + str(self.imageNo) + '_' + self.image_name[1] + ".jpg"
            if(cv2.imwrite(location,frame)):
                print (location + " saved")
            else:
                print (location + ' FAILED!!!!!!!!!')

            self.last_time = time.time()
            self.imageNo+=1
            self.images_taken +=1

def main():
    rospy.init_node('yolo_opencv4', anonymous=False)
    yd = yolo_detector()
    cv2.destroyAllWindows()
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", default="/xtion/rgb/image_raw", help="ros topic to subscribe to for image feed")
    args = vars(parser.parse_args())
    main()
