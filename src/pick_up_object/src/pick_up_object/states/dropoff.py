#!/usr/bin/python
import smach
import rospy
import moveit_commander

from moveit_msgs.msg import AttachedCollisionObject
from geometry_msgs.msg import PoseArray
from pick_up_object.utils import play_motion_action, add_collision_object, clear_octomap
from pick_up_object.states import pickup


class DropOff(smach.State):

    def __init__(self, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             output_keys=['prev']
                             )
        self.arm_torso_controller = arm_torso_controller
        self.planning_scene = arm_torso_controller._scene
        self.target_poses_pub = rospy.Publisher('/current_target_poses', PoseArray, queue_size=200, latch=True)

    def execute(self, userdata):
        userdata.prev = 'Dropoff'

        result = self.drop_object()
        return result

    def drop_object(self):

        eef_link = self.arm_torso_controller.move_group.get_end_effector_link()
        config = dict(planning_time=15., allow_replanning=True)
        self.arm_torso_controller.configure_planner(config)

        '''self.target_poses_pub.publish(grasp_poses)
        result = self.arm_torso_controller.sync_reach_ee_poses(grasp_poses)
        self.arm_torso_controller.update_planning_scene(add=True)

        # remove any previous objects added to the planning scene
        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')'''

        print('About to Open gripper to drop off object')
        result = play_motion_action('open')

        rospy.sleep(5.)

        if not result:
            print('Open Failed')
            result = 'failed'
            self.arm_torso_controller.sync_reach_safe_joint_space()
        else:
            print('Open Succeeded')
            result = 'succeeded'
            self.arm_torso_controller.sync_reach_safe_joint_space()

        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')

        moveit_commander.roscpp_shutdown()
        return result
