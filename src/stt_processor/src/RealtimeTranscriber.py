#!/usr/bin/env python3
import json
import queue
import threading
from vosk import Model, KaldiRecognizer
import rospy, rospkg
from std_msgs.msg import UInt8MultiArray, String, Bool

class RealtimeTranscriber(threading.Thread):
    def __init__(self, callback=None):
        super().__init__()
        self.running = True
        
        # Initialisation du noeud ROS
        rospy.init_node('audio_transcriber', anonymous=True)
        rospy.sleep(1)

        # Init du paquet
        r = rospkg.RosPack()
        path = r.get_path('stt_processor')

        # Initialisation du modèle Vosk
        self.model = Model(f"{path}/src/models/vosk-model-fr-0.22")
        self.sample_rate = 44100
        self.audio_queue = queue.Queue()
        self.is_speaking = False
        
        # Créer le recognizer
        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        self.rec.SetWords(True)  # Activer la détection mot par mot
        
        # Callback pour envoyer les résultats au serveur web
        self.callback = callback
        
        # Connection au topic ROS du micro TIAGO
        rospy.Subscriber("/sound", UInt8MultiArray, self.audio_callback)
        self.partial_pub = rospy.Publisher('/stt/partial', String, queue_size=10)
        self.full_pub = rospy.Publisher('/stt/full', String, queue_size=10)
        self.is_speaking_topic = rospy.Subscriber('/tiago_interact/tts/status', Bool, self.toggle_speaking)
        rospy.sleep(2)

        rospy.loginfo("En attente des données audio sur le topic /sound...")
    
    def toggle_speaking(self, ros_msg):
        self.is_speaking = ros_msg.data
    
    def audio_callback(self, data):
        """Callback appelé quand des données audio arrivent du topic ROS"""
        self.audio_queue.put(bytes(data.data))
    
    def stop(self):
        """Arrête le thread de transcription"""
        self.running = False
    
    def run(self):
        """Méthode principale du thread"""
        try:
            while self.running:
                try:
                    # Obtenir les données audio de la queue
                    data = self.audio_queue.get(timeout=1)  # timeout pour pouvoir vérifier self.running
                    
                    if not self.is_speaking:
                        if self.rec.AcceptWaveform(data):
                            result = json.loads(self.rec.Result())
                            if result.get("text", "").strip():
                                message = {"type": "final", "text": result["text"]}
                                self.full_pub.publish(result['text'])
                        else:
                            result = json.loads(self.rec.PartialResult())
                            if result.get("partial", "").strip():
                                message = {"type": "partial", "text": result["partial"]}
                                self.partial_pub.publish(result['partial'])
                except queue.Empty:
                    continue  # Continuer la boucle si pas de données audio
        except Exception as e:
            print(f"Erreur dans le thread de transcription: {e}")
        finally:
            rospy.signal_shutdown("Thread arrêté")

if __name__ == "__main__":
    try:
        stt = RealtimeTranscriber()
        stt.run()
    except rospy.ROSInterruptException:
        exit()