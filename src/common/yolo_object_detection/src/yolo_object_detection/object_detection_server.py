#!/usr/bin/env python3
from re import I
import rospy
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from PIL import Image as PIL_Image
import time

from pcl_manipulation.srv import SegmentPlane, SegmentPlaneResponse, RemoveOutliers, RemoveOutliersResponse, Downsample, \
    DownsampleResponse, Crop, CropResponse, Euclidian, EuclidianResponse, CartesianFilter, CartesianFilterResponse, \
    MinMax, MinMaxResponse, Centroid, CentroidResponse, CombineClouds, CombineCloudsResponse

import torch
import torchvision.transforms as transforms

DEVICE = 'cpu'

import rospkg
import sys
import cv2
from cv_bridge import CvBridge
from darknet_pytorch.darknet import Darknet
from darknet_pytorch.utils import post_processing

MODEL_ROOT = rospkg.RosPack().get_path('yolo_object_detection') + '/models/'
from os.path import isdir


def pclmsg_to_pcl_cv2_imgmsg(pclmsg):
    # extract the xyz values from 32FC1
    pcl = np.fromstring(pclmsg.data, dtype=np.uint8)
    pcl = pcl.reshape(pclmsg.height, pclmsg.width, -1)

    # extract the rgb values from 32FC1
    # frame = np.fromstring(pclmsg.data, dtype=np.uint8)
    # frame = frame.reshape(pclmsg.height, pclmsg.width, 32)
    # frame = frame[:, :, 16:19].copy()

    return pcl# , frame


def yolo_transform():
    return transforms.Compose([
        transforms.Resize((416, 416)),
        transforms.ToTensor(),
    ])


DATASET_INSTANCE_CATEGORY_NAMES = ['got_book', 'oldman_book',  'oz_book']
DATASET_INSTANCE_LOCATIONS = [[3.551, 3.22, 0.0, 0.0, 0.0, 0.75709, 0.6533],
                              [-4.087, 6.121, 0.0, 0.0, 0.0, 0.75709, 0.6533],
                              [-1.783, 4.564, 0.0, 0.0, 0.0, 0.75709, 0.6533]]


class object_detection_server():
    def __init__(self):
        self.segment_plane = rospy.ServiceProxy("/pcl_manipulation/segment_plane", SegmentPlane)
        self.remove_outliers = rospy.ServiceProxy('/pcl_manipulation/remove_outliers', RemoveOutliers)
        self.downsample = rospy.ServiceProxy('/pcl_manipulation/downsample', Downsample)
        self.crop = rospy.ServiceProxy('/pcl_manipulation/crop', Crop)
        self.euclid = rospy.ServiceProxy('/pcl_manipulation/cluster', Euclidian)
        self.cartesian_filter = rospy.ServiceProxy('/pcl_manipulation/cartesian_filter', CartesianFilter)
        self.min_max = rospy.ServiceProxy('/pcl_manipulation/min_max', MinMax)
        self.centroid = rospy.ServiceProxy('/pcl_manipulation/centroid', Centroid)
        self.combine_clouds = rospy.ServiceProxy('/pcl_manipulation/combine_clouds', CombineClouds)

        self.transform = yolo_transform()
        self.model_path = None
        self.labels = DATASET_INSTANCE_CATEGORY_NAMES
        self.locations = DATASET_INSTANCE_LOCATIONS
        self.load_model(MODEL_ROOT + 'books' + '/')

    def load_model(self, model_path):
        # model_path = MODEL_ROOT + model_name + '/'
        print(model_path)
        if isdir(model_path):
            # self.model_name = model_name
            start_time = time.time()

            self.labels = open(model_path + 'classes.txt').read().strip().split('\n')
            self.locations = open(model_path + 'locations.txt').read().strip().split('\n')
            self.yolov4 = Darknet(model_path + 'yolov4.cfg')
            self.yolov4.load_weights(model_path + 'yolov4_last.weights')
            self.yolov4.eval()
            self.yolov4.to(DEVICE)

            rospy.loginfo('Time to load {} model: {:.2f} seconds'.format(model_path, time.time() - start_time))
        else:
            self.model_path = None
            self.yolov4 = None

    def forward(self, model_path, image_raw):
        # load model if it is not already up
        if not self.model_path == model_path:
            self.load_model(model_path)

        assert self.yolov4 is not None
        # print(model_path)
        # preprocess
        frame = np.frombuffer(image_raw.data, dtype=np.uint8).reshape(image_raw.height, image_raw.width, -1)
        image = PIL_Image.fromarray(frame)
        image = torch.stack([self.transform(image)]).to(DEVICE)

        # net forward and nms
        outputs = self.yolov4(image)
        #print(outputs[0])
        outputs = post_processing(image, 0.3, 0.7, outputs)
        #print(outputs)

        # outputs = [[x1, y1, x2, y2, confidence, prediction (class)]]
        boxes = []
        labels = []
        labels_text = []
        scores = []
        locations = []
        if not outputs[0] is None:
            for detection in outputs[0]:
                classID = np.argmax(detection[5:])
                bbox = detection[:4]
                bbox[0] *= image_raw.width
                bbox[1] *= image_raw.height
                bbox[2] *= image_raw.width
                bbox[3] *= image_raw.height
                x1, y1, x2, y2 = [int(i) for i in bbox]
                obj_conf, class_score = detection[4:6]
                class_pred = int(detection[6])

                name = self.labels[class_pred]
                location = self.locations[class_pred]
                confidence = class_score

                boxes.append((x1, y1, x2, y2))
                scores.append(confidence)
                labels.append(class_pred)
                labels_text.append(name)
                locations.append(location)

        #print("First : ", boxes, labels, labels_text, scores)

        return boxes, labels, labels_text, scores, locations, frame


    def detect_objects(self, pclmsg, imgmsg, model_path, confidence=0.76, nms=0.3):
        pcl = pclmsg_to_pcl_cv2_imgmsg(pclmsg)

        pred_boxes, pred_labels, pred_labels_text, pred_scores, pred_locations, frame = self.forward(model_path, imgmsg)

        # results
        labels = []
        labels_text = []
        scores = []
        boxes = []
        clouds = []
        masks = []
        locations = []

        mask = frame.copy()
        for i, label in enumerate(pred_labels):
            if pred_scores[i] > confidence:
                x1, y1, x2, y2 = pred_boxes[i]
                mask = np.zeros(shape=frame.shape[:2])
                mask[y1:y2, x1:x2] = 255
                mask_ds = mask[::2, ::2]
                indices = np.argwhere(mask_ds.flatten()).flatten()

                # Downsample
                pcl_ds = pcl[::2, ::2, :]
                pcl_out = np.take(pcl_ds.reshape(pcl_ds.shape[0] * pcl_ds.shape[1], -1), indices, axis=0)

                frame_ds = frame[::2, ::2, :]

                # create pcl
                pclmsg_out = PointCloud2()
                pclmsg_out.header = pclmsg.header
                pclmsg_out.height = pcl_out.shape[0]
                pclmsg_out.width = 1
                pclmsg_out.fields = pclmsg.fields
                pclmsg_out.is_bigendian = pclmsg.is_bigendian
                pclmsg_out.point_step = pclmsg.point_step
                pclmsg_out.row_step = pcl_out.shape[1]
                pclmsg_out.is_dense = pclmsg.is_dense
                pclmsg_out.data = pcl_out.flatten().tostring()

                # append results
                masks.append(mask)
                boxes.append(pred_boxes[i])
                clouds.append(pclmsg_out)
                scores.append(pred_scores[i])
                labels.append(pred_labels[i])
                labels_text.append(pred_labels_text[i])
                locations.append(pred_locations[i])

        #locations = pred_locations

        return frame, pcl, boxes, clouds, scores, labels, labels_text, masks, locations


def get_location():
    global LOCATION
    return LOCATION

if __name__ == '__main__':
    rospy.init_node('yolo_detection_server')
    model_path = MODEL_ROOT + 'books' + '/'
    server = object_detection_server()

    # colour stuff
    np.random.seed(69)
    COLOURS = np.random.randint(0, 256, (128, 3))
    alpha = 0.5

    while not rospy.is_shutdown():
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks, locations = server.detect_objects(pclmsg, image_msg, model_path)

        print(len(labels))
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)

        for i, box in enumerate(boxes):
            label = labels[i]
            color = color = [int(c) for c in COLOURS[label]]

            print("locations: ", locations[i])
            # segmentation masks
            #binary_mask = mask > 0.5
           # frame_coloured = np.array((frame * (1 - alpha) + color * alpha), dtype=np.uint8)
            #frame = np.where(binary_mask, frame_coloured, frame)

            # bboxes + info
            x1, y1, x2, y2 = [int(v) for v in boxes[i]]

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = "{}: {:.4f}".format(labels_text[i], scores[i])
            cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('test', frame)
        cv2.waitKey(1)

    # serv = rospy.Service('yolo_detection', YoloDetection, server.detect_objects)
    # rospy.loginfo('YOLOv4 object detection service initialised')

    # rospy.spin()
