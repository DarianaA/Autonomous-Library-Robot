#!/usr/bin/env python3
from re import I
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from PIL import Image as PIL_Image
import time

from yolo_object_detection.msg import Detection
from yolo_object_detection.srv import YoloDetection, YoloDetectionResponse
from pcl_manipulation.srv import (
    SegmentPlane, SegmentPlaneResponse,
    RemoveOutliers, RemoveOutliersResponse,
    Downsample, DownsampleResponse,
    Crop, CropResponse,
    Euclidian, EuclidianResponse,
    CartesianFilter, CartesianFilterResponse,
    MinMax, MinMaxResponse,
    Centroid, CentroidResponse,
    CombineClouds, CombineCloudsResponse
)
import torch
import torchvision.transforms as transforms

if torch.cuda.is_available():
    DEVICE = 'cuda'
else:
    DEVICE = 'cpu'

import rospkg
import sys
import cv2

from darknet_pytorch.darknet import Darknet
from darknet_pytorch.utils import post_processing

MODEL_ROOT = rospkg.RosPack().get_path('yolo_object_detection') + '/models/'
from os.path import isdir


def yolo_transform():
    return transforms.Compose([
        transforms.Resize((416, 416)),
        transforms.ToTensor(),
    ])


class object_detection_server():
    def __init__(self):
        self.segment_plane = rospy.ServiceProxy('/pcl_manipulation/segment_plane', SegmentPlane)
        self.remove_outliers = rospy.ServiceProxy('/pcl_manipulation/remove_outliers', RemoveOutliers)
        self.downsample = rospy.ServiceProxy('/pcl_manipulation/downsample', Downsample)
        self.crop = rospy.ServiceProxy('/pcl_manipulation/crop', Crop)
        self.euclid = rospy.ServiceProxy('/pcl_manipulation/cluster', Euclidian)
        self.cartesian_filter = rospy.ServiceProxy('/pcl_manipulation/cartesian_filter', CartesianFilter)
        self.min_max = rospy.ServiceProxy('/pcl_manipulation/min_max', MinMax)
        self.centroid = rospy.ServiceProxy('/pcl_manipulation/centroid', Centroid)
        self.combine_clouds = rospy.ServiceProxy('/pcl_manipulation/combine_clouds', CombineClouds)

        self.transform = yolo_transform()
        self.model_name = None
        self.load_model('books')

    def load_model(self, model_name):
        model_path = MODEL_ROOT + model_name + '/'
        print(model_path)
        if isdir(model_path):
            self.model_name = model_name

            start_time = time.time()

            self.labels = open(model_path + 'classes.txt').read().strip().split('\n')
            self.yolov4 = Darknet(model_path + 'yolov4.cfg')
            self.yolov4.load_weights(model_path + 'yolov4_last.weights')
            self.yolov4.eval()
            self.yolov4.to(DEVICE)

            rospy.loginfo('Time to load {} model: {:.2f} seconds'.format(model_name, time.time() - start_time))
        else:
            self.model_name = None
            self.yolov4 = None

    def detect_objects(self, req):
        # load model if it is not already up
        if not self.model_name == req.dataset:
            self.load_model(req.dataset)

        assert self.yolov4 is not None

        np.random.seed(42)
        COLORS = np.random.randint(0, 255, size=(len(self.labels), 3), dtype="uint8")

        # preprocess
        frame = np.frombuffer(req.image_raw.data, dtype=np.uint8).reshape(req.image_raw.height, req.image_raw.width, -1)
        image = PIL_Image.fromarray(frame)
        image = torch.stack([self.transform(image)]).to(DEVICE)

        # net forward and nms
        outputs = self.yolov4(image)
        outputs = post_processing(image, req.confidence, req.nms, outputs)
        print(outputs)

        # build response
        # see: YoloDetection.srv, Detection.msg
        mask = frame.copy()
        response = YoloDetectionResponse()
        if not outputs[0] is None:
            for detection in outputs[0]:
                classID = np.argmax(detection[5:])
                bbox = detection[:4]
                bbox[0] *= req.image_raw.width
                bbox[1] *= req.image_raw.height
                bbox[2] *= req.image_raw.width
                bbox[3] *= req.image_raw.height
                x1, y1, x2, y2 = [int(i) for i in bbox]

                obj_conf, class_score = detection[4:6]
                class_pred = int(detection[6])

                name = self.labels[class_pred]
                confidence = class_score
                x, y, w, h = x1, y1, x2 - x1, y2 - y1
                color = [int(c) for c in COLORS[classID]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(name, confidence)
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                mask = np.zeros(shape=frame.shape[:2])
                mask[y1:y2, x1:x2] = 255
                mask_ds = mask[::2, ::2]
                indices = np.argwhere(mask_ds.flatten()).flatten()

                pcl = np.fromstring(req.cloud.data, dtype=np.uint8)

                # Downsample
                pcl = pcl.reshape(req.cloud.height, req.cloud.width, -1)
                pcl_ds = pcl[::2, ::2, :]
                pcl_out = np.take(pcl_ds.reshape(pcl_ds.shape[0] * pcl_ds.shape[1], -1), indices, axis=0)

                pclmsg = PointCloud2()
                pclmsg.header = req.cloud.header
                pclmsg.height = pcl_out.shape[0]
                pclmsg.width = 1
                pclmsg.fields = req.cloud.fields
                pclmsg.is_bigendian = req.cloud.is_bigendian
                pclmsg.point_step = req.cloud.point_step
                pclmsg.row_step = pcl_out.shape[1]
                pclmsg.is_dense = req.cloud.is_dense
                pclmsg.data = pcl_out.flatten().tostring()


        clouds = [obj.points for obj in response.detected_objects]
        response.cloud = self.combine_clouds(clouds).points

        rows, cols = frame.shape[:2]
        image = Image()
        image.header.stamp = rospy.Time.now()
        image.header.frame_id = 'xtion_bgr_optical_frame'
        image.height = rows
        image.width = cols
        image.encoding = 'bgr8'
        image.is_bigendian = 0
        image.step = cols * 3
        image.data = list(frame.reshape(rows * cols * 3))
        response.image_bb = image
        return response


if __name__ == '__main__':
    rospy.init_node('yolo_detection')
    server = object_detection_server()
    serv = rospy.Service('yolo_detection', YoloDetection, server.detect_objects)
    rospy.loginfo('YOLOv4 object detection service initialised')
    rospy.spin()