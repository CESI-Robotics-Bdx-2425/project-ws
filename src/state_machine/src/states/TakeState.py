#!/usr/bin/env python
import rospy
import smach
from utils.TTS import TextToSpeech
from pick_and_give.srv import PickAndGive

class TakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle','error'],input_keys=['sm_previous_state','flyer_id','flyer_pos'],output_keys=['sm_previous_state', 'error'])
        self.service_name = 'pick_and_place'
        self.tts = TextToSpeech()


    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'pick_and_place'.")
        userdata.sm_previous_state = 'pick_and_place'

        rospy.loginfo(f"Waiting service: {self.service_name}...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo(f"Service {self.service_name} ready")

        try:
            pick = rospy.ServiceProxy(self.service_name, PickAndGive)
            # Créer un client de service et appeler le service
            r = pick(userdata.flyer_pos, 0)
            if r.result != 0:
                userdata.error = {
                    "error": "Erreur: mouvement non atteignable",
                    "need_shut": False
                }
                return "error"
            rospy.loginfo(f"Pick : {r}")
            rospy.sleep(3)
            self.tts.say("Bonne journée")
            return "idle"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "error"
