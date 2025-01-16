#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String
from StateScanTable import TableScanState
from StateScanBook import BookScanState
from StateListen import ListenState
from ErrorState import ErrorState


class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanTable'])
    
    def execute(self, userdata):
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        return 'scanTable'

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
    sm = smach.StateMachine(outcomes=['DONE','EXIT'])
    
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'scanTable': 'SCANTABLE'})
        smach.StateMachine.add('SCANTABLE', TableScanState(), transitions={'scanBook': 'SCANBOOK','scanTable': 'SCANTABLE', 'error':'ERROR'})
        smach.StateMachine.add('SCANBOOK', BookScanState(), transitions={'listen': 'LISTEN','scanBook': 'SCANBOOK', 'error':'ERROR'})
        smach.StateMachine.add('LISTEN', ListenState(), transitions={'move': 'MOVE'})
        smach.StateMachine.add('MOVE', MoveState(), transitions={'listen': 'LISTEN'})
        smach.StateMachine.add('ERROR', ErrorState(previous_state_name='UNKNOWN'), transitions={'exit': 'EXIT'})

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
