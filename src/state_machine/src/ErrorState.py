#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

class ErrorState(smach.State):
    def __init__(self, previous_state_name):
        smach.State.__init__(self, outcomes=['exit'])
        self.previous_state_name = previous_state_name

    def execute(self, userdata):
        rospy.logerr(f"Etat Error : Une erreur est survenue dans l'état '{self.previous_state_name}'.")
        rospy.loginfo("Etat Error : Terminaison de la machine d'états.")
        return 'exit'
