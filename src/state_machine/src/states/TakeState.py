#!/usr/bin/env python
import rospy
import smach
from utils.TTS import TextToSpeech
from pick_and_give.srv import PickAndGive

class TakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle','error','refill'],input_keys=['sm_previous_state','flyer_id','flyer_pos','book_count'],output_keys=['sm_previous_state', 'error', 'book_count'])
        self.service_name = 'pick_and_give'
        self.tts = TextToSpeech()


    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'pick_and_give'.")
        userdata.sm_previous_state = 'pick_and_give'

        rospy.loginfo(f"Waiting service: {self.service_name}...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo(f"Service {self.service_name} ready")

        try:
            pick = rospy.ServiceProxy(self.service_name, PickAndGive)
            # Créer un client de service et appeler le service
            r = pick(userdata.flyer_pos, 3 - userdata.book_count[userdata.flyer_id])
            if r.result != 0:
                userdata.error = {
                    "error": "Erreur: mouvement non atteignable",
                    "need_shut": False
                }
                return "error"
            else:
                userdata.book_count[userdata.flyer_id] -= 1             
                rospy.loginfo(f"Pick : {r}")
                rospy.sleep(1)
                self.tts.say("Bonne journée. Je vous souhaite une bonne continuation pour votre visite au sein du Campus CESI de Bordeaux.")
                if userdata.book_count[userdata.flyer_id] == 0:
                    return 'refill'
                rospy.sleep(30)
                return "idle"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "error"
