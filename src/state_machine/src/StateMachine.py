#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

from states.IdleState import IdleState
from states.ScanTableState import ScanTableState
from states.StartState import StartState
from states.StateScanBook import BookScanState

def main():
    rospy.init_node("state_machine_example")

    # Créer la machine d'état
    sm = smach.StateMachine(outcomes=['DONE','EXIT'])
    
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'scanTable': 'SCANTABLE'})
        smach.StateMachine.add('SCANTABLE', ScanTableState(), transitions={'scanBook': 'SCANBOOK', 'error':'IDLE'})
        smach.StateMachine.add('SCANBOOK', BookScanState(), transitions={'idle': 'IDLE'})
        smach.StateMachine.add('IDLE', IdleState(), transitions={})

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
