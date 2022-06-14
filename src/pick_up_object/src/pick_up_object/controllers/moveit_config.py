#!/usr/bin/env python

import rospy

from moveit_msgs.msg import PlanningSceneComponents, PlanningScene, PlanningSceneWorld, CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class MoveitConfig():

    def __init__(self):
        rospy.wait_for_service('/apply_planning_scene', 10.0)
        self.apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        self.sv_srv.wait_for_service()
        self.rs = RobotState()
        

    def getStateValidity(self, group_name='arm_torso', constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        result = self.sv_srv.call(gsvr)
        return result

if __name__ == '__main__':
    rospy.init_node('moveit_config')
    config = MoveitConfig()

    request_collision_matrix = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    request_robot_state = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)

    response_collision_m = config.get_planning_scene(request_collision_matrix)
    response_robot_s = config.get_planning_scene(request_robot_state)

    if config.getStateValidity().valid:
        rospy.loginfo('robot not in collision, all ok!')
    else:
        rospy.logwarn('robot in collision')


        

