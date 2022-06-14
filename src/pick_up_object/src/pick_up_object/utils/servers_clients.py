#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import actionlib

from pick_up_object.srv import GenerateGrasps, TfTransform, TfTransformRequest, YoloDetection
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion


DATASET_INSTANCE_LOCATIONS = {'oldman_book': [3.551, 3.22, 0.0, 0.0, 0.0, 0.75709, 0.6533],
                              'got_book': [-4.087, 6.121, 0.0, 0.0, 0.0, 0.75709, 0.6533],
                              'oz_book': [-1.783, 4.564, 0.0, 0.0, 0.0, 0.75709, 0.6533]}

LOCATIONS = []
def detect_objs():
    """
        Calls detection server YoloDetection

        Return:
            detection {YoloDetection}
    """

    print('waiting for yolo_detection')
    rospy.wait_for_service('yolo_detection', timeout=60)
    try:
        detect = rospy.ServiceProxy('yolo_detection', YoloDetection)
        resp = detect()
        print('detection done!')

        if len(resp.labels_text) > 0:
            global LOCATIONS
            for i, loc in enumerate(resp.labels_text):
                LOCATIONS.append(DATASET_INSTANCE_LOCATIONS[resp.labels_text[i].data])
                print("locations in detection service: ", DATASET_INSTANCE_LOCATIONS[resp.labels_text[i].data])
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def move_to(obj_index):
    """
        Arguments:
            obj_index {Int} -- the index of the current object to be moved

        Return:
            bool -- navigation status
        """
    global LOCATIONS
    location = LOCATIONS[obj_index]
    movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    movebase_client.wait_for_server()
    header = Header(frame_id="map", stamp=rospy.Time.now())
    position = Point(location[0], location[1], location[2])
    orientation = Quaternion(location[3], location[4], location[5], location[6])

    goal = MoveBaseGoal()
    goal.target_pose.header = header
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation = orientation

    # sending goal
    movebase_client.send_goal(goal)

    # logging information about goal sent
    action_ok = movebase_client.wait_for_result()
    print("Waiting for result...")
    state = movebase_client.get_state()
    if action_ok:
        print("Action finished succesfully with state: " + str(actionlib.GoalStatus.to_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(actionlib.GoalStatus.to_string(state)))

    # print(action_ok and state == actionlib.GoalStatus.SUCCEEDED)
    return action_ok and state == actionlib.GoalStatus.SUCCEEDED


def generate_grasps(full_pcl, objs_pcl):
    """
    Arguments:
        full_pcl {PointCloud} -- full point cloud of the scene
        objs_pcl {dict of PointClouds} -- dictionary with each segmented object pointcloud
    
    Return:
        grasps {GenerateGrasps} -- array of grasps
    """

    print('waiting for generate_grasps_server')
    rospy.wait_for_service('generate_grasps_server', timeout=20)
    try:
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
        resp = grasp_poses(full_pcl, objs_pcl)
        print('generates grasps done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def clear_octomap():
    """
        Clears octomap using clear_octomap server
    """

    print('waiting for clear_octomap')
    rospy.wait_for_service('clear_octomap', timeout=10)
    try:
        clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
        response = clear_octo()
        print('clearing octomap done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def tf_transform(target_frame, pose_array=None, pointcloud=None):
    """
    Arguments:
        target_frame {frame_id} -- frame to transform to
        pose_array {PoseArray} -- array of poses
        pointcloud {PointCloud2}
    
    Returns:
        response {TfTransformResponse} -- target_pose_array {PoseArray}, target_pointcloud {PointCloud2}
    """

    assert pose_array is not None or pointcloud is not None

    # print('about to transform to ' + target_frame)
    rospy.wait_for_service('tf_transform', timeout=10)
    try:
        tf_transform_srv = rospy.ServiceProxy('tf_transform', TfTransform)
        request = TfTransformRequest()
        if pose_array is not None:
            request.pose_array = pose_array
        if pointcloud is not None:
            request.pointcloud = pointcloud
        request.target_frame.data = target_frame
        response = tf_transform_srv(request)
        # print('transform done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def to_frame_pose(self, pose, source_frame='xtion_depth_optical_frame', target_frame='base_footprint'):
    """
    Arguments:
        pose {Pose} -- pose to convert
        source_frame {frame id} -- original coordinate frame
        target_frame {frame id} -- target coordinate frame
    Return:
        pose {Pose} -- target pose
    """
    
    # remove '/' from source_frame and target frame to avoid tf2.InvalidArgumentException
    source_frame = source_frame.replace('/', '')
    target_frame = target_frame.replace('/', '')
    try:
        transformation = self.tfBuffer.lookup_transform(target_frame, source_frame,
        rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    
    pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
    return pose

def add_collision_object(object_cloud, planning_scene, num_primitives = 200):
    """
    adds collision object to the planning scene created by moveit

    Arguments:
        object_cloud {PointCloud2} -- pointcloud of the object
        num_primitives -- number of primitives (squares) to create the collision object

    Returns:
        co {CollisionObject} -- collision object
    """

    pcl = np.fromstring(object_cloud.data, np.float32)
    pcl = pcl.reshape(object_cloud.height, object_cloud.width, -1)
    
    cloud_obj = np.zeros(pcl.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
    ])
    cloud_obj['x'] = pcl[:,:,0].flatten()
    cloud_obj['y'] = pcl[:,:,1].flatten()
    cloud_obj['z'] = pcl[:,:,2].flatten()

    # add collision object to planning scene
    co = CollisionObject()
    co.id = 'object'

    # create collision object
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.02, 0.02, 0.02]

    indices = np.random.choice(range(pcl.shape[0]), size=np.min([num_primitives, pcl.shape[0]]), replace=False)
    pose_array = PoseArray()

    for i in indices:
        x,y,z = cloud_obj[i]
        primitive_pose = Pose()
        primitive_pose.position.x = x
        primitive_pose.position.y = y
        primitive_pose.position.z = z
        primitive_pose.orientation.x = 0
        primitive_pose.orientation.y = 0
        primitive_pose.orientation.z = 0
        primitive_pose.orientation.w = 1

        co.primitives.append(primitive)
        pose_array.poses.append(primitive_pose)

    # pose_array = tf_transform(pose_array)
    co.header = object_cloud.header
    co.primitive_poses = pose_array.poses

    planning_scene.add_object(co)
    rospy.sleep(2.0)
    return co

def play_motion_action(action='home'):
    """
    Arguments:
        action {String} -- predefined actions

    Return:
        bool -- action statusf 
    """


    pm_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    pm_client.wait_for_server()
    goal = PlayMotionGoal()
    goal.motion_name = action
    goal.skip_planning = False
    goal.priority = 0  # Optional

    print("Sending goal with motion: " + action)
    pm_client.send_goal(goal)

    print("Waiting for result...")
    action_ok = pm_client.wait_for_result(rospy.Duration(30.0))

    state = pm_client.get_state()
    if action_ok:
        print("Action finished succesfully with state: " + str(actionlib.GoalStatus.to_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(actionlib.GoalStatus.to_string(state)))
    
    print(action_ok and state == actionlib.GoalStatus.SUCCEEDED)
    return action_ok and state == actionlib.GoalStatus.SUCCEEDED
