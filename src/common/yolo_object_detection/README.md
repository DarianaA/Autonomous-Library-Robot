## YOLOv4 ROS object detection service

Originally created by Joe Jeffcock, modified for YOLOv4, Python3 and ROS Melodic by Dariana Agape, using the model from the LASR respository for the RoboCup competition.

**object_detection_server.py** spins a rosservice that performs object detection on a single Image using YOLOv4 and PYTORCH.

Datasets can be specified at service call, such that multiple sets of classes can be detected using one server.


## Loading new datasets
To add a dataset for object detection, create a directory with the intended name of the dataset in the **models** directory.

For each dataset folder, the following files are required:
* classes.txt
* yolov4.weights
* yolov4.cfg

To generate the **classes.txt**, **yolov4.cfg** files and train **yolov4.weights** for a new custom dataset, refer to src/train_yolo


## Usage
Ensure the **object_detection_server.py** server node is running

    rosrun yolo_object_detection object_detection_server.py


In the client, create a handle to the service

    detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
    
Then call the service with the following arguments:

    result = detect_objects(image_msg, dataset_name, confidence, nms)

| Type | Name | Description |
| ------ | ------ | ------ |
| sensor_msgs.msg/Image | image_msg | message to perform object detection on |
| string | dataset_name | name of dataset to use |
| float | confidence | confidence threshold for detection |
| float | nms | threshold for non-maximum suppression on bounding boxes |

## Output

| Type | Name | Description |
| ------ | ------ | ------ |
| sensor_msgs/Image | image_bb | Image with bounding boxes for detected objects |
| Detection[] | detected_objects | Array of Detection messages |

Each Detection is composed of name, confidence and bounding box dimensions:
* string name
* float32 confidence
* int32[] xywh

## Credits
### PyTorch-YOLOv4
https://github.com/Tianxiaomo/pytorch-YOLOv4
### Original Package
https://gitlab.com/sensible-robots/lasr_object_detection_yolo
