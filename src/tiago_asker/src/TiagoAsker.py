#!/usr/bin/env python
import json
import random
import rospy
from std_msgs.msg import MultiArrayLayout, String
from std_srvs.srv import Empty
from pal_interaction_msgs.msg import TtsActionGoal
from tiago_asker.srv import TiagoAskerAnswer, TiagoAskerService

class TiagoAsker():
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('tiago_asker', anonymous=True)
        
        # Load params
        self.questions_file_path = rospy.get_param('~questions_file_path', default='questions.json') 
        self.finish = False
        self.waiting_answer = False
        
        # Load the questions file
        with open(self.questions_file_path, 'r') as file:
            self.questions = json.load(file)
            
        # Init the topics
        self.question_topic = rospy.Publisher('/tiago_asker/question', String, queue_size=10)
        self.answers_topic = rospy.Publisher('/tiago_asker/answers', MultiArrayLayout, queue_size=10)
        self.tts = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        
        # Publish the service to ask question
        self.question_service = rospy.Service('/tiago_asker', TiagoAskerService, self.start)
        self.process_answer_service = rospy.Service('/tiago_asker/answer', TiagoAskerAnswer, self.process_answer)
        rospy.spin()
        
    def say(self, msg):
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.text = msg
        tts_msg.goal.rawtext.lang_id = 'fr_FR'
        self.tts.publish(tts_msg)
        rospy.sleep(10)
        
    def process_answer(self, ros_req):
        self.answer = ros_req.answer
        return self.answer
    
    def reset(self):
        self.finish = False
        self.waiting_answer = False
        self.answer = None
        self.selected_question = None
    
    def start(self, ros_req):
        self.reset()
        # Start the conversation with a random question
        random_intro = random.choice(self.questions['intro'])
        self.say(random_intro)
        
        # Start with the first question
        q_type = 'bac'
        while not self.finish:
            # Select a question
            self.selected_question = [q for q in self.questions['questions'] if q_type == q['type']][0]
            
            if not self.waiting_answer:            
                self.say(self.selected_question['question'])
                self.waiting_answer = True
                
            if self.answer != None:
                q_type = self.selected_question['responses'][self.answer]
                print(q_type)
                if 'result_' in q_type:
                    self.finish = True
                    break
                self.waiting_answer = False
        
        # Get a result sentence
        print(q_type, self.selected_question)
        selected_result = [r for r in self.questions['results'] if r['type'] == q_type][0]
        self.say(selected_result['sentence'])
        return
                
        
if __name__ == "__main__":
    asker = TiagoAsker()