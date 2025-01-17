import rospy
import smach

class MoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle'])
        # Vous pouvez ajouter une logique ici pour gérer le mouvement.
    
    def execute(self, userdata):
        rospy.loginfo("Etat Move : Exécution d'un mouvement ou d'une action.")
        rospy.sleep(2)  # Simule une action ou un mouvement
        rospy.loginfo("Etat Move : Mouvement terminé, retour à l'état Idle.")
        return 'idle'  # Retour à l'état Idle après Move