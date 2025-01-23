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
from states.StateScanBook2 import BookScanState2
from states.ErrorState import ErrorState
from states.TalkState import TalkState
from states.TakeState import TakeState
from states.StateRefill import RefillState

def main():
    rospy.init_node("state_machine")
    rospy.loginfo('Node state_machine started !')

    rospy.sleep(1)

    # Créer la machine d'état
    sm = smach.StateMachine(outcomes=['DONE','EXIT'])
    sm.userdata.sm_previous_state = 'Start'
    sm.userdata.flyer_id = 0
    sm.userdata.flyer_pos = None
    sm.userdata.error = None
    sm.userdata.book_count = {0: 3, 3: 3, 2: 3}
    
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'scanTable': 'IDLE'})
        smach.StateMachine.add('SCANTABLE', ScanTableState(), transitions={'scanBook': 'SCANBOOK', 'error':'ERROR'})
        smach.StateMachine.add('SCANBOOK', BookScanState(), transitions={'idle': 'IDLE','error':'ERROR'})
        
        smach.StateMachine.add('TALK', TalkState(), transitions={'scanBook2': 'SCANBOOK2','idle':'IDLE','error':'ERROR'})
        smach.StateMachine.add('SCANBOOK2', BookScanState2(), transitions={'take': 'TAKE','error':'ERROR'})
        
        smach.StateMachine.add('TAKE', TakeState(), transitions={'idle': 'IDLE','error':'ERROR', 'refill': 'REFILL'})
        smach.StateMachine.add('REFILL', RefillState(), transitions={'idle': 'IDLE','error':'ERROR'})

        smach.StateMachine.add('IDLE', IdleState(), transitions={'talk':'TALK','idle':'IDLE'})
        smach.StateMachine.add('ERROR', ErrorState(), transitions={'idle': 'IDLE'})

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
