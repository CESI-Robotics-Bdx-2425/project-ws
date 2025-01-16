#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

class TableScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanBook', 'Error'])
        self.service_name = 'table_detector'
        self.error_count = 0  # Initialiser le compteur d'erreurs

    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'table_detector'.")
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
        
        try:
            # Créer un client de service et appeler le service
            scan_table = rospy.ServiceProxy(self.service_name, Empty)
            scan_table()
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Scan Book.")
            
            # Réinitialiser le compteur d'erreurs en cas de succès
            self.error_count = 0
            return "scanBook"
        except rospy.ServiceException as e:
            self.error_count += 1  # Incrémenter le compteur d'erreurs
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' (tentative {self.error_count}/3) : {e}")
            
            # Vérifier si le nombre maximum d'erreurs a été atteint
            if self.error_count >= 3:
                rospy.logerr("Nombre maximum d'erreurs atteint. Passage à l'état Error.")
                return "Error"
            else:
                return "scanTable"
