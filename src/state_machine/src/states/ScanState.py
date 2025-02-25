import rospy
import smach
from table_detector.srv import TableDetector
from utils.TTS import TextToSpeech

class ScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle', 'scan'])
        self.service_name = 'table_detector'
        self.TTS = TextToSpeech()
        
    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'table_detector'.")
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
        
        try:
            # Créer un client de service et appeler le service
            scan_table = rospy.ServiceProxy(self.service_name, TableDetector)
            r = scan_table()
            
            if r.result == -1:
                self.TTS.say("Table non détéctée. Veuillez vérifier mon environnement. Pour plus d'informations, référez-vous à la documentation d'utilisation.")
            else:
                self.TTS.say("Table détéctée. Environnement mis à jour !")
                
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Idle.")
            return "idle"
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "scan"