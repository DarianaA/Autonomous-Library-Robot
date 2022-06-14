#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class TiagoSensors(object):
    def __init__(self, show_raw_image=False):
        self._show_raw_image = show_raw_image
        self.bridge_object = CvBridge()
        self.camera_topic = "/xtion/rgb/image_raw"
        self._check_cv_image_ready()
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.camera_callback)

    def _check_cv_image_ready(self):
        self.cv_image = None
        while self.cv_image is None and not rospy.is_shutdown():
            try:
                raw_cv_image = rospy.wait_for_message("/xtion/rgb/image_raw", Image, timeout=1.0)
                self.cv_image = self.bridge_object.imgmsg_to_cv2(raw_cv_image, desired_encoding="bgr8")
                rospy.logdebug("Current " + self.camera_topic + " READY=>")
            except:
                rospy.logerr(
                    "Current " + self.camera_topic + " not ready yet, retrying for getting " + self.camera_topic + "")
        return self.cv_image

    def camera_callback(self, data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        if self._show_raw_image:
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(1)

    def get_image(self):
        return self.cv_image


def main():
    tiago_sensors_object = TiagoSensors()
    rospy.init_node('tiago_sensors_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()