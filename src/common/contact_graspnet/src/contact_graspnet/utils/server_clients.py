import rospy
import numpy as np
from contact_graspnet.srv import GenerateGrasps, GenerateGraspsResponse, TfTransform, TfTransformRequest
# from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Pose, PoseArray
from contact_graspnet.srv import DetectObjects
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
import actionlib

def detect_objs():
    print('waiting for detect_objects')
    rospy.wait_for_service('detect_objects', timeout=10)
    try:
        detect = rospy.ServiceProxy('detect_objects', DetectObjects)
        resp = detect()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def generate_grasps(full_pcl, objs_pcl):
    print('waiting for generate_grasps_server')
    rospy.wait_for_service('generate_grasps_server', timeout=10)
    try:
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
        resp = grasp_poses(full_pcl, objs_pcl)
        print('generates grasps done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def clear_octomap():
    print('waiting for clear_octomap')
    rospy.wait_for_service('clear_octomap', timeout=10)
    try:
        clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
        resp1 = clear_octo()
        print('clearing octomap done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def tf_transform(target_frame, pose_array=None, pointcloud=None):
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


def add_collision_object(object_cloud, planning_scene, num_primitives=200):
    """
    adds collision object to the planning scene created by moveit

    Arguments:
        object_cloud {PointCloud2} -- pointcloud of the object
        num_primitives -- number of primitives (squares) to create the collision object

    Returns:
        co {CollisionObject} -- collision object
    """

    print(type(object_cloud))
    print(type(planning_scene))

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
    # co.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.02, 0.02, 0.02]

    indices = np.random.choice(range(pcl.shape[0]), size=np.min([num_primitives, pcl.shape[0]]), replace=False)

    pose_array = PoseArray(header=object_cloud.header)

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
    
    pose_array = tf_transform(pose_array)
    co.header = pose_array.header
    co.primitive_poses = pose_array.poses

    # self.co_pub.publish(co)
    planning_scene.add_object(co)
    rospy.sleep(5.0)
    return co

# def play_motion_action(action='home'):
#     pm_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
#     pm_client.wait_for_server()
#     goal = PlayMotionGoal()
#     goal.motion_name = action
#     goal.skip_planning = False
#     goal.priority = 0  # Optional

#     print("Sending goal with motion: " + action)
#     pm_client.send_goal(goal)

#     print("Waiting for result...")
#     action_ok = pm_client.wait_for_result(rospy.Duration(30.0))

#     state = pm_client.get_state()

#     return action_ok and state == actionlib.GoalStatus.SUCCEEDED
