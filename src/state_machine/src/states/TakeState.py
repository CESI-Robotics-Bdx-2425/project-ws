#!/usr/bin/env python
import rospy
import smach
from utils.TTS import TextToSpeech
from pick_and_give.srv import PickAndGive, PickAndGiveRequest

class TakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle','error'],input_keys=['sm_previous_state','flyer_id'],output_keys=['sm_previous_state'])
        self.service_name = 'pick_and_place'
        self.tts = TextToSpeech()


    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'pick_and_place'.")
        userdata.sm_previous_state = 'pick_and_place'

        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
                # Création du proxy pour le service
        pick_and_place = rospy.ServiceProxy("pick_and_place", PickAndGive)

        try:
            Pick = rospy.ServiceProxy(self.service_name)
            for flyer, aruco_id in self.flyers.items():
                # Créer un client de service et appeler le service
                r = Pick()
                rospy.sleep(3)
                rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Listen.")
            self.tts.say("Les 3 flyers sont bien présents dans mon environnement")
            return "idle"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "error"
