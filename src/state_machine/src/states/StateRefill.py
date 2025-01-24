#!/usr/bin/env python
import rospy
import smach
from utils.TTS import TextToSpeech
from std_srvs.srv import Empty, EmptyResponse
from pick_and_give.srv import PickAndGive

class RefillState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle','error'],input_keys=['sm_previous_state', 'book_count'],output_keys=['sm_previous_state', 'error', 'book_count'])
        self.tts = TextToSpeech()
        self.is_refill = False
        self.s = rospy.Service('/refill', Empty, self.handle_refill)
        
    def handle_refill(self, data):
        self.is_refill = True
        rospy.loginfo('Refill done')
        return EmptyResponse()


    def execute(self, userdata):
        self.is_refill = False
        while not self.is_refill:
            self.tts.say('Merci de recharger mes supports avec 3 flyers')
            rospy.sleep(30)
        userdata.book_count = {0: 3, 3: 3, 2: 3}
        self.tts.say('Rechargement effectué')
        return 'idle'
