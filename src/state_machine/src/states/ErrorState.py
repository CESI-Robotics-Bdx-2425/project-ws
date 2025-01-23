#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String
from utils.TTS import TextToSpeech


class ErrorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'],input_keys=['sm_previous_state', 'error'],output_keys=['sm_previous_state'])
        self.TTS = TextToSpeech()

    def execute(self, userdata):
        rospy.logerr(f"Etat Error : Une erreur est survenue dans l'état {userdata.sm_previous_state}.")
        self.TTS.say(f"ERROR: {userdata.sm_previous_state}. {userdata.error.error}")
        if self.error.need_shut == True:
            rospy.signal_shutdown('ERROR')
        rospy.loginfo("Etat Error : Terminaison de la machine d'états.")
        return 'idle'
