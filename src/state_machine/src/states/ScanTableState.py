#!/usr/bin/env python
import rospy
import smach
from table_detector.srv import TableDetector
from utils.TTS import TextToSpeech

class ScanTableState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanBook', 'error'],input_keys=['sm_previous_state'],output_keys=['sm_previous_state'])
        self.service_name = 'table_detector'
        self.TTS = TextToSpeech()

    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'table_detector'.")
        userdata.sm_previous_state = 'Scan Table'
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
        
        self.TTS.say('Je démarre la calibration. Merci de ne plus modifier mon environnement')
        
        try:
            # Créer un client de service et appeler le service
            scan_table = rospy.ServiceProxy(self.service_name, TableDetector)
            r = scan_table()
            
            if r.result == -1:
                self.TTS.say("Table non détéctée. Veuillez vérifier mon environnement. Pour plus d'informations, référez-vous à la documentation d'utilisation.")
                rospy.sleep(5)
                return "error"
            else:
                self.TTS.say("Table détéctée. Environnement mis à jour !")
                
            rospy.sleep(3)
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Idle.")
            return "scanBook"
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "error"
