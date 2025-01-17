#!/usr/bin/env python
import smach
import rospy
from utils.TTS import TextToSpeech

class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanTable'])
        
        self.TTS = TextToSpeech()
    
    def execute(self, userdata):
        rospy.loginfo("Etat de départ : Passage à l'état Scan.")
        self.TTS.say('Je démarre la calibration. Merci de ne plus modifier mon environnement')
        return 'scanTable'