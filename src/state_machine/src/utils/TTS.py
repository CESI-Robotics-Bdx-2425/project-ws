import rospy
from pal_interaction_msgs.msg import TtsActionGoal

class TextToSpeech():
    def __init__(self, language="fr_FR"):
        self.running = True
        self.language = language

        rospy.loginfo("Initialisation synthèse vocale...")

        # Publishers
        self.pub_tts = rospy.Publisher("/tts/goal", TtsActionGoal, queue_size=10)
        rospy.sleep(1) # Attendre 1 seconde pour laisser le temps au publisher de s'initialiser

    def stop(self):
        """Arrête le thread de synthèse vocale"""
        self.pub_tts.unregister()

    def say(self, text):
        print(f"Synthétisation du texte : {text}")
        """Synthétise un texte en parole"""
        self.is_tts_active = True
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = self.language
        tts_msg.goal.rawtext.text = text
        self.pub_tts.publish(tts_msg)
        rospy.sleep(1)  # Attendre 1 seconde pour laisser le temps à la synthèse vocale de démarrer
       