#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from tiago_sensors import TiagoSensors
from colours import k_means_colour
from std_msgs.msg import String, Float32
#from graspnet_detection import get_boxes
# https://github.com/spookycouch/lasr_robocup_receptionist/tree/master/src
from pick_up_object.srv import DetectBoxes, DetectObjects
import math

def detect_box():
    """
        Calls detection server (Mask-RCNN)

        Return:
            detection {DetectObjects}
    """

    print('waiting for detect_objects')
    rospy.wait_for_service('detect_boxes', timeout=10)
    try:
        get_boxes = rospy.ServiceProxy('detect_boxes', DetectBoxes)
        resp = get_boxes()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def detect_objs():
    """
        Calls detection server (Mask-RCNN)

        Return:
            detection {DetectObjects}
    """

    print('waiting for detect_objects')
    rospy.wait_for_service('detect_objects', timeout=10)
    try:
        detect = rospy.ServiceProxy('detect_objects', DetectObjects)
        resp = detect()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("blob_detector_node", log_level=rospy.DEBUG)
    #obj = detect_objs()
    #print(type(obj.scores))
    object_box = detect_box()
    print(type(object_box.boxes))
    box = object_box.boxes
    print(box)
    tiago_sensors_obj = TiagoSensors()
    cv_image = tiago_sensors_obj.get_image()

    rows = cv_image.shape[0]
    cols = cv_image.shape[1]
    size = min([rows, cols])

    # center_x = int(cols / 2.0)
    # center_y = int(rows / 2.0)
    # print("here", rows, cols, center_x, center_y)

    #box = get_boxes()
    #y1 = 311
   # y2 = 365
    #x1 = 318
   # x2 = 384

    y1 = int(box[1])
    y2 = int(box[3])
    x1 = int(box[0])
    x2 = int(box[2])
    print(x1, y1, x2, y2)
    # crop image
    img_colours = cv_image[y1:y2, x1:x2]
    img_colourss = np.float32(img_colours.reshape(img_colours.size / 3, 3))

    # get dominant colour in LAB for colour name
    dominant_colour, object_colour, centres = k_means_colour(3, img_colourss)
    print(object_colour)

    frame_height, frame_width = cv_image.shape[:2]
    # print(frame_height, frame_width)
    # what for???
    if dominant_colour is not None:
        # pallette
        colours_bgr = []
        for i in range(len(centres)):
            start_i = i * 40
            colour = [int(i) for i in centres[i]]
            colours_bgr.append(colour)
            # print("colour: ", colours_bgr)
            cv2.rectangle(cv_image, (start_i, frame_height - 40), (start_i + 40, frame_height), colour, 1)

    cv2.imshow("image colours", img_colours)
    cv2.waitKey(0)
    rospy.logwarn("Shutting down")
    cv2.destroyAllWindows()

