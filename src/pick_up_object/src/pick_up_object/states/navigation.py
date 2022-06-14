#!/usr/bin/python
import smach
from pick_up_object.utils import move_to

class Navigation(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['failed', 'succeeded'],
                             input_keys=['prev'])

        self.completeNavigation = False

    def execute(self, userdata):
        userdata.prev = 'Navigation'
        self.completeNavigation = move_to(0)

        if self.completeNavigation:
            return 'succeeded'
        else:
            return 'failed'
