#!/usr/bin/env python
import smach
import rospy
from std_srvs.srv import Empty

class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanTable'])
        self.services=['/book_detector','/tiago_interact/tts','/tiago_asker', '/homing']
    
    def execute(self, userdata):
        for s in self.services:
            rospy.wait_for_service(s)
            rospy.loginfo(f"le service {s} est demarré")
            
        self.go_home = rospy.ServiceProxy('/homing', Empty)
        self.go_home()
            
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        return 'scanTable'