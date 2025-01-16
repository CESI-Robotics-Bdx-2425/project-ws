#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

class TableScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanBook'])
        self.service_name = 'table_detector'

    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'table_detector'.")
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
        
        try:
            # Créer un client de service et appeler le service
            scan_table = rospy.ServiceProxy(self.service_name, Empty)
            scan_table()
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Idle.")
            return "scanBook"
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "scanTable "