#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String
from utils.TTS import TextToSpeech


class ErrorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'],input_keys=['sm_previous_state'],output_keys=['sm_previous_state'])
        self.TTS = TextToSpeech()

    def execute(self, userdata):
        rospy.logerr(f"Etat Error : Une erreur est survenue dans l'état {userdata.sm_previous_state}.")
        self.TTS.say(f"Etat Error : Une erreur est survenue dans l'état {userdata.sm_previous_state}.")
        rospy.loginfo("Etat Error : Terminaison de la machine d'états.")
        return 'idle'
