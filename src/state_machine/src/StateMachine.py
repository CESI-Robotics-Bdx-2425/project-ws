#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scan'])
    
    def execute(self, userdata):
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        return 'scan'

class ScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])
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
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "scan"
        
        self.service_name = 'book_detector'
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)
        
        try:
            # Créer un client de service et appeler le service
            scan_book = rospy.ServiceProxy(self.service_name, Empty)
            scan_book()
            rospy.loginfo("Etat Scan : Réponse du service reçue, passage à l'état Idle.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            "return scan"

        

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move'])
        # Abonnement pour attendre un message sur /idle_topic
        self.subscriber = rospy.Subscriber('/idle_topic', String, self.idle_callback)

    def execute(self, userdata):
        self.message_received = False
        rospy.loginfo("Etat Idle : En attente de message sur /idle_topic.")
        
        # Rester dans cet état tant qu'on n'a pas reçu de message
        while not self.message_received:
            rospy.sleep(0.1)
        
        rospy.loginfo("Etat Idle : Message reçu, passage à l'état Move.")
        return 'move'  # Passage à l'état Move une fois le message reçu

    def idle_callback(self, msg):
        # Callback appelé lorsque le message est reçu
        rospy.loginfo(f"Message reçu sur /idle_topic : {msg.data}")
        self.message_received = True

class MoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])
        # Vous pouvez ajouter une logique ici pour gérer le mouvement.
    
    def execute(self, userdata):
        rospy.loginfo("Etat Move : Exécution d'un mouvement ou d'une action.")
        rospy.sleep(2)  # Simule une action ou un mouvement
        rospy.loginfo("Etat Move : Mouvement terminé, retour à l'état Idle.")
        return 'idle'  # Retour à l'état Idle après Move

def main():
    rospy.init_node("state_machine_example")

    # Créer la machine d'état
    sm = smach.StateMachine(outcomes=['DONE'])
    
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'scan': 'SCAN'})
        smach.StateMachine.add('SCAN', ScanState(), transitions={'idle': 'IDLE','scan': 'SCAN'})
        smach.StateMachine.add('IDLE', IdleState(), transitions={'move': 'MOVE'})
        smach.StateMachine.add('MOVE', MoveState(), transitions={'idle': 'IDLE'})

    # Activer le serveur d'introspection SMACH
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    
    # Exécution de la machine d'état
    sm.execute()
    
    # Garder le nœud actif pour écouter les topics
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
