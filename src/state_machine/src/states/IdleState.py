import rospy
import smach
from std_msgs.msg import String
from pal_detection_msgs.msg import FaceDetections
import time

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['talk', 'idle'])
        
        self.face_detected = False
        self.face_detected_start_time = None
        self.is_finish = False

    def face_detect(self, ros_msg):
        # Si un visage est détecté
        if len(ros_msg.faces) > 0:
            if not self.face_detected:  # Début de la détection
                self.face_detected = True
                self.face_detected_start_time = time.time()
                rospy.loginfo('Visage détécté')
            else:  # Déjà en cours de détection
                elapsed_time = time.time() - self.face_detected_start_time
                if elapsed_time >= 3.0:
                    rospy.loginfo("Visage détecté pendant plus de 3 secondes.")
        else:
            # Réinitialisation si aucun visage détecté
            if self.face_detected:
                rospy.logwarn('Visage Perdu')
            self.face_detected = False
            self.face_detected_start_time = None

    def execute(self, userdata):
        # Abonnement au TOPIC de détection des visages
        self.face_topic = rospy.Subscriber('/pal_face/faces', FaceDetections, self.face_detect)
        rospy.sleep(2)
        
        rospy.loginfo("IdleState en cours d'exécution...")
        rate = rospy.Rate(10)  # Fréquence de vérification (10 Hz)
        while not rospy.is_shutdown() and not self.is_finish:
            if self.face_detected and self.face_detected_start_time:
                elapsed_time = time.time() - self.face_detected_start_time
                if elapsed_time >= 3.0:
                    self.face_topic.unregister()
                    return 'talk'
            rate.sleep()
        return 'idle'

    def idle_callback(self, msg):
        # Callback appelé lorsque le message est reçu
        rospy.loginfo(f"Message reçu sur /idle_topic : {msg.data}")
