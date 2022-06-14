#!/usr/bin/env python3
import rospy
from yolo_object_detection.srv import YoloDetection, YoloDetectionResponse
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

if __name__ == "__main__":
    # initialise node
    rospy.init_node('detect')
    # service as a function.
    detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
    while True:
        # wait for an image message.
        image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        # wait for pcl message.
        pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        # call the service.
        result = detect_objects(image_msg, pcl_msg, 'books', 0.7, 0.3)

        print(result.detected_objects)
        for obj in result.detected_objects:
            print(obj.name, obj.confidence)

        # construct OpenCV image from sensor_msgs/Image.
        im = np.frombuffer(result.image_bb.data, dtype=np.uint8).reshape(result.image_bb.height, result.image_bb.width,
                                                                         -1)
        # show the results.
        cv2.imshow('frame', im)
        cv2.waitKey(0)
    rospy.spin()