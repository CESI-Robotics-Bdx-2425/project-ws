import rospy
import smach
from std_msgs.msg import String
from utils.TTS import TextToSpeech
from tiago_asker.srv import TiagoAskerAnswer, TiagoAskerService

class TalkState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanBook2','idle','error'],input_keys=['sm_previous_state','flyer_id'],output_keys=['sm_previous_state','flyer_id'])
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
            userdata.flyer_id = r.flyer_id
            if r.flyer_id == 3: 
                return "idle"
            rospy.loginfo(f"Le flyer_id retourné est : {r}")
            return "scanBook2"
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service '{self.service_name}' : {e}")
            return "error"
