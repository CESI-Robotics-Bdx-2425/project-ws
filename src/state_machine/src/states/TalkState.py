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
        self.tts.say("balise 0")
        rospy.wait_for_service(self.service_name)
        self.tts.say("balise 1")
            
        try:
            # Créer un client pour le service
            talk_state = rospy.ServiceProxy(self.service_name, TiagoAskerService)
            # Appeler le service (ici sans paramètres si la requête est vide)
            r = talk_state()
            # Récupérer la réponse du service
            flyer_id = r.flyer_id
        
            rospy.loginfo(f"Le flyer_id retourné est : {flyer_id}")
            self.tts.say(f"Le flyer ID est {flyer_id}")
            rospy.sleep(5)
            return "idle"
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service '{self.service_name}' : {e}")
            if self.error_count >= 3:
                rospy.logerr("Nombre maximum d'erreurs atteint. Passage à l'état Error.")
                return "error"
            else:
                return "idle"
