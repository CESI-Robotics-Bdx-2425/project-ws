#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String
from StateScanTable import TableScanState
from StateScanBook import BookScanState


class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanTable'])
    
    def execute(self, userdata):
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        return 'scanTable'
 

class ListenState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move'])
        # Abonnement pour attendre un message sur /idle_topic
        self.subscriber = rospy.Subscriber('/idle_topic', String, self.listen_callback)

    def execute(self, userdata):
        self.message_received = False
        rospy.loginfo("Etat Idle : En attente de message sur /idle_topic.")
        
        # Rester dans cet état tant qu'on n'a pas reçu de message
        while not self.message_received:
            rospy.sleep(0.1)
        
        rospy.loginfo("Etat Idle : Message reçu, passage à l'état Move.")
        return 'move'  # Passage à l'état Move une fois le message reçu

    def listen_callback(self, msg):
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
        smach.StateMachine.add('START', StartState(), transitions={'scanTable': 'SCANTABLE'})
        smach.StateMachine.add('SCANTABLE', TableScanState(), transitions={'scanBook': 'SCANBOOK','scanTable': 'SCANTABLE'})
        smach.StateMachine.add('SCANBOOK', TableScanState(), transitions={'listen': 'LISTEN','scanBook': 'SCANBOOK'})
        smach.StateMachine.add('LISTEN', ListenState(), transitions={'move': 'MOVE'})
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
