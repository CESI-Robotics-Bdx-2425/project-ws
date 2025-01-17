#!/usr/bin/env python
import rospy
import smach
from book_detector.srv import FindAruco
from utils.TTS import TextToSpeech

class BookScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])
        self.service_name = 'book_detector'
        self.error_count = 0  # Initialiser le compteur d'erreurs
        self.tts = TextToSpeech()
        
        self.flyers = {
            0: "0",
            1: "3",
            2: "2"
        }


    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'book_detector'.")
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)


        try:
            scan_book = rospy.ServiceProxy(self.service_name, FindAruco)
            for flyer, aruco_id in self.flyers.items():
                # Créer un client de service et appeler le service
                self.tts.say(f"Je lance la recherche du flyer {flyer + 1}")
                r = scan_book(int(aruco_id))
                rospy.sleep(3)
                rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Listen.")
            return "idle"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            # Vérifier si le nombre maximum d'erreurs a été atteint
            if self.error_count >= 3:
                rospy.logerr("Nombre maximum d'erreurs atteint. Passage à l'état Error.")
                return "idle"
            else:
                return "idle"