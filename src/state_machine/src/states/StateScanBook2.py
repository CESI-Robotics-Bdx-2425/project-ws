#!/usr/bin/env python
import rospy
import smach
from book_detector.srv import FindAruco
from utils.TTS import TextToSpeech

class BookScanState2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['take','error'],input_keys=['sm_previous_state','flyer_id','flyer_pos'],output_keys=['sm_previous_state','flyer_pos'])
        self.service_name = 'book_detector'
        self.tts = TextToSpeech()

    def execute(self, userdata):
        rospy.loginfo("Etat Scan : Appel au service 'book_detector'.")
        userdata.sm_previous_state = 'Scan Book'
        
        # Attendre que le service soit disponible
        rospy.wait_for_service(self.service_name)

        try:
            scan_book = rospy.ServiceProxy(self.service_name, FindAruco)
            # Créer un client de service et appeler le service
            self.tts.say(f"Laissez moi rechercher la brochure")
            r = scan_book(int(userdata.flyer_id))
            if r.coordinates.pose.position.x == 0 and r.coordinates.pose.position.y == 0 and r.coordinates.pose.position.z ==0:
                return 'error'
            userdata.flyer_pos = r.coordinates
            self.tts.say("La brochure est trouvée")
            return "take"

        except rospy.ServiceException as e:
            rospy.logerr(f"Erreur lors de l'appel au service 'table_detector' : {e}")
            return "error"
