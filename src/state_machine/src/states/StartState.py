#!/usr/bin/env python
import smach
import rospy
from utils.TTS import TextToSpeech

class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanBook'])
        self.services=['/book_detector','/tiago_interact/tts','/tiago_asker']
        
        
        self.TTS = TextToSpeech()    
    def execute(self, userdata):
        for s in self.services:
            rospy.wait_for_service(s)
            rospy.loginfo(f"le service {s} est demarré")
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        self.TTS.say('Je démarre la calibration. Merci de ne plus modifier mon environnement')
        return 'scanBook'