#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

class BookScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['listen'])
        self.service_name = 'book_detector'
        self.error_count = 0  # Initialiser le compteur d'erreurs


    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'book_detector'.")
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)


        try:
            # Créer un client de service et appeler le service
            scan_book = rospy.ServiceProxy(self.service_name, Empty)
            scan_book()
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Listen.")
            return "listen"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            # Vérifier si le nombre maximum d'erreurs a été atteint
            if self.error_count >= 3:
                rospy.logerr("Nombre maximum d'erreurs atteint. Passage à l'état Error.")
                return "Error"
            else:
                return "scanBook"