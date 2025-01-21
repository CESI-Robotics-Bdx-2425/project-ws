import rospy
import smach
from std_msgs.msg import String
from utils.TTS import TextToSpeech
from tiago_asker.srv import TiagoAskerAnswer, TiagoAskerService

class TalkState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle','error'],input_keys=['sm_previous_state'],output_keys=['sm_previous_state'])
        self.service_name = '/tiago_asker'
        self.error_count = 0  # Initialiser le compteur d'erreurs
        self.tts = TextToSpeech()

    def execute(self, userdata):
        rospy.loginfo("Etat Talk : Appel au service 'talk_state'.")
        self.tts.say("Etat Talk : Appel au service 'talk_state'.")
        userdata.sm_previous_state = 'Talk_State'
        # Attendre que le service soit disp
        
        rospy.wait_for_service(self.service_name)
        self.tts.say("balise 1")

        try:
            talk_state = rospy.ServiceProxy(self.service_name, TiagoAskerService)
            # Créer un client de service et appeler le service
            r = talk_state()
            self.tts.say("balise 2")
            rospy.sleep(5)
            return "idle"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            # Vérifier si le nombre maximum d'erreurs a été atteint
            if self.error_count >= 3:
                rospy.logerr("Nombre maximum d'erreurs atteint. Passage à l'état Error.")
                return "error"
            else:
                return "idle"