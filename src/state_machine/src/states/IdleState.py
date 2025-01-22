import rospy
import smach
from std_msgs.msg import String

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[])
        # Abonnement pour attendre un message sur /idle_topic
        self.subscriber = rospy.Subscriber('/idle_topic', String, self.idle_callback)

    def execute(self, userdata):
        self.message_received = False
        rospy.loginfo("Etat Idle : En attente de message sur /idle_topic.")
        
        # Rester dans cet état tant qu'on n'a pas reçu de message
        while not self.message_received:
            rospy.sleep(0.1)
        
        rospy.loginfo("Etat Idle : Message reçu, passage à l'état Move.")
        
    def idle_callback(self, msg):
        # Callback appelé lorsque le message est reçu
        rospy.loginfo(f"Message reçu sur /idle_topic : {msg.data}")
        self.message_received = True