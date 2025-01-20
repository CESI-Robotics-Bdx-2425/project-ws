#!/usr/bin/env python3
import rospy
from tiago_interact.srv import TiagoInteractTTS, TiagoInteractSTT, TiagoInteractTTSResponse
from pal_interaction_msgs.msg import TtsActionGoal, TtsActionResult
import std_msgs.msg as msg
import std_srvs.srv as srv

class TiagoInteract:
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('tiago_interact', anonymous=True)
        
        # Initialisation des variables
        self.is_using_stt = True
        self.is_speaking = False
        
        # Initialisation des TOPICS
        self.tts_topic = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        self.tts_result = rospy.Subscriber('/tts/result', TtsActionResult, self.tts_result)
        self.stt_state_topic = rospy.Publisher('/tiago_interact/stt/status', msg.Bool, queue_size=10)
        self.tts_status_topic = rospy.Publisher('/tiago_interact/tts/status', msg.Bool, queue_size=10)
        
        # Initialisation des services d'interaction
        self.tts_service = rospy.Service('/tiago_interact/tts', TiagoInteractTTS, self.speak)
        self.ask_service = rospy.Service('/tiago_interact/ask', TiagoInteractSTT, self.ask)
        self.gui_service = rospy.Service('/tiago_interact/gui/answer', srv.Empty, self.process_gui_answer)
        self.trigger_stt_service = rospy.Service('/tiago_interact/stt/toggle', srv.Empty, self.toggle_stt)
        
        self.spin()
    
    def tts_result(self, ros_msg):
        self.is_speaking = False
    
    def spin(self):
        self.rate = rospy.Rate(1/10)
        while not rospy.is_shutdown():
            self.stt_state_topic.publish(self.is_using_stt)
            self.rate.sleep()
        
    def speak(self, ros_req):
        try:
            self.say(ros_req.msg)
        except Exception as e:
            self.error(e)
            return -1
        return 0

    def process_gui_answer(self, ros_req):
        print(ros_req)
    
    def say(self, msg):
        # Prepare message to say
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.text = msg
        tts_msg.goal.rawtext.lang_id = 'fr_FR'
        
        # Lock TTS
        self.is_speaking = True
        self.tts_status_topic.publish(self.is_speaking)
        self.tts_topic.publish(tts_msg)
        
        while self.is_speaking:
            rospy.sleep(1)
        
        # Unlock TTS     
        self.tts_status_topic.publish(False)
    
    def stt_process(self, ros_msg):
        try:
            self.stt_msg = ros_msg.data
            if self.stt_msg in self.q_answers:
                self.q_answer = self.stt_msg
            else:
                self.say("Je n'ai pas compris")
        except Exception as e:
            self.error(e)
    
    def reset(self):
        self.q_answer = None
        self.stt_msg = None
    
    def ask(self, ros_req):
        self.reset()
        
        rospy.loginfo('Ask Call')
        self.q_answers = ros_req.answers
        print(self.q_answers)
        
        if self.is_using_stt:
            self.stt_topic = rospy.Subscriber('/stt/full', msg.String, callback=self.stt_process, queue_size=10)
            while self.q_answer is None:
                rospy.sleep(1)
            return self.q_answer
        else:
            pass
    
    def toggle_stt(self, ros_req):
        self.is_using_stt = not self.is_using_stt
        rospy.loginfo(f'STT toggle to {self.is_using_stt}')
    
    def error(self, e):
        rospy.logerr(f"[TIAGO_INTERACT]: {e}")
        
if __name__ == "__main__":
    interact = TiagoInteract()
