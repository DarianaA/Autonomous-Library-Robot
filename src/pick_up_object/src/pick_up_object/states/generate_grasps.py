#!/usr/bin/python

import rospy
import smach
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from pick_up_object.utils import generate_grasps

class GenerateGrasps(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             input_keys=['objs_resp', 'prev'],
                             output_keys=['grasps_resp', 'prev'])
        self.try_num = 0
        self.retry_attempts = 3

    def execute(self, userdata):

        pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        full_pcl = ros_numpy.numpify(pcl_msg)
        full_pcl = np.concatenate( (full_pcl['x'].reshape(-1,1), full_pcl['y'].reshape(-1,1), full_pcl['z'].reshape(-1,1)), axis=1)

        userdata.prev = 'GenerateGrasps'
        objs_resp = userdata.objs_resp

        grasps_resp = generate_grasps(objs_resp.full_pcl, objs_resp.object_clouds)
        self.try_num += 1

        pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.all_grasp_poses])
        if pose_count > 0:
            userdata.grasps_resp = grasps_resp
            return 'succeeded'
        elif self.try_num < self.retry_attempts:
            return 'looping'
        else:
            return 'failed'
