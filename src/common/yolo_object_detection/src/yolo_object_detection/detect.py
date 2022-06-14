#!/usr/bin/env python3
import rospy
from yolo_object_detection.srv import YoloDetection, YoloDetectionResponse
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float32
from object_detection_server import object_detection_server



def object_is_attached(scene):
    attached_objects = scene.get_attached_objects(['object'])
    return len(attached_objects.keys()) > 0


def remove_object(scene):
    scene.remove_world_object('object')


def detach_object(move_group, scene):
    eef_link = move_group.get_end_effector_link()
    scene.remove_attached_object(eef_link, name='object')


server = object_detection_server()

LOCATIONS = []
def detect(msg):
    """
        Arguments:
            msg {DetectObjectsRequest} -- msg is empty
        Returns:
            detected_objects {YoloDetectionResponse} -- full_pcl, object_clouds, scores, labels_text
        """

    pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
    model_path = '/LASR/src/common/yolo_object_detection/models/books/'

    frame, pcl, boxes, clouds, scores, labels, labels_text, masks, locations = server.detect_objects(pclmsg, image_msg, model_path, 0.76, 0.3)

    labels_text = [String(label) for label in labels_text]
    print('labels: {}, scores: {}'.format(labels_text, scores))
    print('location for {}: {}'.format(labels_text, locations))
    global LOCATIONS
    LOCATIONS = locations
    detected_objects = YoloDetectionResponse(pclmsg, clouds, scores, labels_text)

    return detected_objects


def getLocations():
    global LOCATIONS
    return LOCATIONS


if __name__ == "__main__":
    # initialise node
    rospy.init_node('detect')
    s = rospy.Service('yolo_detection', YoloDetection, detect)
    print('Ready to detect objects')

    rospy.spin()
