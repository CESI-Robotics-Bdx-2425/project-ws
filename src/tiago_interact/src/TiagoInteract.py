#!/usr/bin/env python3
import rospy
from tiago_interact.srv import TiagoInteractTTS, TiagoInteractSTT, TiagoInteractGUI, TiagoInteractGUIResponse
from pal_interaction_msgs.msg import TtsActionGoal, TtsActionResult
import std_msgs.msg as msg
import std_srvs.srv as srv

class TiagoInteract:
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('tiago_interact', anonymous=True)
        rospy.loginfo('Node tiago_interact started !')

        # Initialisation des variables
        self.is_using_stt = True
        self.is_speaking = False
        
        # Initialisation des TOPICS
        self.tts_topic = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        self.tts_result = rospy.Subscriber('/tts/result', TtsActionResult, self.tts_finish)
        self.tts_status_topic = rospy.Publisher('/tiago_interact/tts/status', msg.Bool, queue_size=10)
        self.stt_state_topic = rospy.Publisher('/tiago_interact/stt/status', msg.Bool, queue_size=10)
        
        # Initialisation des services d'interaction
        self.tts_service = rospy.Service('/tiago_interact/tts', TiagoInteractTTS, self.speak)
        self.ask_service = rospy.Service('/tiago_interact/ask', TiagoInteractSTT, self.ask)
        self.gui_service = rospy.Service('/tiago_interact/gui/answer', TiagoInteractGUI, self.process_gui_answer)
        self.trigger_stt_service = rospy.Service('/tiago_interact/stt/toggle', srv.Empty, self.toggle_stt)
        
        self.spin()
    
    def tts_finish(self, ros_msg):
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
        self.q_answer = ros_req.answer
        return TiagoInteractGUIResponse()
    
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
        if not self.is_using_stt:
            return
        try:
            self.stt_msg = ros_msg.data.lower()
            found_answers = [answer for answer in self.q_answers if answer in self.stt_msg]
            
            if len(found_answers) == 1:
                self.q_answer = found_answers[0]
            else:
                self.say(f"Je n'ai pas compris. Merci de répondre par {' ou '.join(self.q_answers)}")
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
        
        self.stt_topic = rospy.Subscriber('/stt/full', msg.String, callback=self.stt_process, queue_size=10)
        while self.q_answer is None:        
            rospy.sleep(1)
        self.stt_topic.unregister()
        return self.q_answer
    
    def toggle_stt(self, ros_req):
        self.is_using_stt = not self.is_using_stt
        rospy.loginfo(f'STT toggle to {self.is_using_stt}')
        return srv.EmptyResponse()
    
    def error(self, e):
        rospy.logerr(f"[TIAGO_INTERACT]: {e}")
        
if __name__ == "__main__":
    interact = TiagoInteract()
