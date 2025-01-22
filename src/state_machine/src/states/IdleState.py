import rospy
import smach
from std_msgs.msg import String

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['talk'])
        # Abonnement pour attendre un message sur /idle_topic

    def execute(self, userdata):

        rospy.sleep(20)
        return "talk"
        
    def idle_callback(self, msg):
        # Callback appelé lorsque le message est reçu
        rospy.loginfo(f"Message reçu sur /idle_topic : {msg.data}")
        self.message_received = True