#!/usr/bin/env python

from __future__ import print_function
import rospy
# imported to create move base goals
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
import actionlib

class Move:
    def __init__(self):
        rospy.loginfo("Move Utility Initialised")

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()
        rospy.loginfo("The move_base action server is up")

    def move_base(self, location):
        # Create Target Pose Header, Position and Orientation:
        header = Header(frame_id="map", stamp=rospy.Time.now())
        position = Point(location[0], location[1], location[2])
        orientation = Quaternion(location[3], location[4], location[5], location[6])

        goal = MoveBaseGoal()
        goal.target_pose.header = header
        goal.target_pose.pose.position = position
        goal.target_pose.pose.orientation = orientation

        # sending goal
        self.movebase_client.send_goal(goal)

        # logging information about goal sent
        rospy.loginfo("GOAL SENT!")
        wait = self.movebase_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal location achieved!")
            return self.movebase_client.get_result()


if __name__ == '__main__':
    rospy.init_node('Moving_around', anonymous=True)
    '''target = [['map', -3.172.517242, 3.993902, 0.0, 0.0, 0.0, -0.00546, 0.99998]]
        ['map', 0.500, -1.3661, 0.0, 0.0, 0.0, -0.7075, 0.70663],
        ['map', -3.4413, -0.8260, 0.0, 0.0, 0.0, 0.75709, 0.6533]]'''
    location = [-3.17, -3.58, 0.0, 0.0, 0.0, -0.00546, 0.99998]
    #location = get_location()
    reacher = Move()

    result = reacher.move_base(location)
