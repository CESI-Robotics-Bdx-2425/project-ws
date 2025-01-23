#!/usr/bin/env python
import json
import random
import rospy
from std_msgs.msg import MultiArrayLayout, String
from std_srvs.srv import Empty
from pal_interaction_msgs.msg import TtsActionGoal
from tiago_asker.srv import TiagoAskerService
from tiago_interact.srv import TiagoInteractTTS, TiagoInteractSTT

class TiagoAsker():
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('tiago_asker', anonymous=True)
        rospy.loginfo('Node tiago_asker started !')

        # Load params
        self.questions_file_path = rospy.get_param('~questions_file_path', default='questions.json') 
        self.finish = False
        self.waiting_answer = False
        
        # Load the questions file
        with open(self.questions_file_path, 'r') as file:
            self.questions = json.load(file)
            
        # Init the topics
        rospy.wait_for_service('/tiago_interact/tts')
        rospy.wait_for_service('/tiago_interact/ask')
        self.tts_service = rospy.ServiceProxy('/tiago_interact/tts', TiagoInteractTTS)
        self.ask_service = rospy.ServiceProxy('/tiago_interact/ask', TiagoInteractSTT)
        
        self.a_topic = rospy.Publisher('/tiago_asker/question/possible_answers', String, queue_size=10)
        
        # Publish the service to ask question
        self.question_service = rospy.Service('/tiago_asker', TiagoAskerService, self.start)
        rospy.spin()
    
    def reset(self):
        self.finish = False
        self.waiting_answer = False
        self.answer = None
        self.selected_question = None
    
    def start(self, ros_req):
        self.reset()
        # Start the conversation with a random question
        random_intro = random.choice(self.questions['intro'])
        _ = self.tts_service(random_intro)
        
        # Start with the first question
        q_type = 'question_bac'
        while not self.finish:
            # Select a question
            self.selected_question = [q for q in self.questions['questions'] if q_type == q['type']][0]
            
            if not self.waiting_answer:            
                _ = self.tts_service(self.selected_question['question'])
                self.waiting_answer = True
                
                # Ask TiagoInteract for answer
                self.possible_answers = self.selected_question['responses'].keys()
                
                # Publish possible answers
                self.a_topic.publish(",".join(self.possible_answers))
                
                self.answer = self.ask_service(self.possible_answers).answer
                print(self.answer)
                
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
        _ = self.tts_service(selected_result['sentence'])
        return selected_result['flyer_id']
                
        
if __name__ == "__main__":
    asker = TiagoAsker()