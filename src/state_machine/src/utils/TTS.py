import rospy
from std_msgs.msg import MultiArrayLayout, String
from std_srvs.srv import Empty
from pal_interaction_msgs.msg import TtsActionGoal
from tiago_asker.srv import TiagoAskerService
from tiago_interact.srv import TiagoInteractTTS, TiagoInteractSTT

class TextToSpeech():
    def __init__(self, language="fr_FR"):
        self.tts_service = rospy.ServiceProxy('/tiago_interact/tts', TiagoInteractTTS)


        rospy.loginfo("Initialisation synthèse vocale...")

        # Publishers
        self.pub_tts = rospy.Publisher("/tts/goal", TtsActionGoal, queue_size=10)
        rospy.sleep(1) # Attendre 1 seconde pour laisser le temps au publisher de s'initialiser

    def stop(self):
        """Arrête le thread de synthèse vocale"""
        self.pub_tts.unregister()

    def say(self, text):
        self.tts_service(text)
        """
        print(f"Synthétisation du texte : {text}")
        Synthétise un texte en parole
        self.is_tts_active = True
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = self.language
        tts_msg.goal.rawtext.text = text
        self.pub_tts.publish(tts_msg)
        rospy.sleep(1)  # Attendre 1 seconde pour laisser le temps à la synthèse vocale de démarrer
        """