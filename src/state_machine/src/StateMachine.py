#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String

from states.IdleState import IdleState
from states.ScanTableState import TableScanState
from states.StartState import StartState

def main():
    rospy.init_node("state_machine_example")

    # Créer la machine d'état
    sm = smach.StateMachine(outcomes=['DONE','EXIT'])
    
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'scanTable': 'SCANTABLE'})
        smach.StateMachine.add('SCANTABLE', TableScanState(), transitions={'scanBook': 'IDLE', 'error':'IDLE'})
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
