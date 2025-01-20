#!/usr/bin/env python3
from std_srvs.srv import Empty, EmptyResponse
from pick_and_give.srv import PickAndGive
import rospy
import moveit_commander
import sys
import copy
from pprint import pprint

class PickAndPlaceService:
    def __init__(self):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place_service", anonymous=True)

        rospy.wait_for_service('/homing')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("both_arms_torso")
        self.s = rospy.Service("pick_and_place", PickAndGive, self.pick_and_place_handler)

        # Ensure that the robot is initialized properly
        rospy.sleep(2)

    def monter_buste(self, hauteur):
        move_group = moveit_commander.MoveGroupCommander("torso")
        torso_joint_values = move_group.get_current_joint_values()
        pprint(torso_joint_values)
        move_group.set_joint_value_target(hauteur)
        move_group.go(wait=True)

    def arm_move_to(self, number):
        move_group = moveit_commander.MoveGroupCommander("arm_left")
        move_group.stop()
        move_group.clear_pose_targets()

        joint_start_1 = [0.011985545972480017, -0.03806922637254751, 1.5246317207337126, 1.621692406596665, -1.4274444343167114, 1.3826300810742314, 0.006182632455332989]
        joint_start_2 = [0.9421578142999586, -0.21311409590575092, 1.4010525892851553, 2.2242633924991946, -1.2781702963473467, -1.3937755123985682, 0.04869687921137075]
        joint_start_3 = [1.2499551420699813, -0.7040097991685714, 2.3231406716303367, 0.8809999584842535, -0.37348831951268313, 0.11445203552907525, -0.19233266659685944]

        if number == 1:
            move_group.go(joint_start_1, wait=True)
        elif number == 2:
            move_group.go(joint_start_2, wait=True)
        elif number == 3:
            move_group.go(joint_start_3, wait=True)

        move_group.clear_pose_targets()
        move_group.stop()

    def go_to_height(self, n):
        move_group = moveit_commander.MoveGroupCommander("torso")
        torso_joint_values = move_group.get_current_joint_values()
        new = [torso_joint_values[0] - 0.06 * n]
        move_group.set_joint_value_target(new)
        move_group.go(wait=True)

    def mouvement(self, flyerNb, take, px, py, pz, ox, oy, oz, ow):
        move_group = moveit_commander.MoveGroupCommander("arm_left")
        move_group.stop()
        move_group.clear_pose_targets()

        waypoints = []

        wpose = move_group.get_current_pose().pose
        if take:
            wpose.position.x = px
            wpose.position.z = pz - flyerNb * 0.06
        else:
            wpose.position.x = px - 0.18
            wpose.position.z = pz - flyerNb * 0.06 + 0.04
        wpose.position.y = py
        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0)
        move_group.execute(plan, wait=True)

        move_group.clear_pose_targets()
        move_group.stop()

    def mouvementPince(self, state):
        gripper_group = moveit_commander.MoveGroupCommander("gripper_left")
        gripper_group.clear_pose_targets()

        joint_open_positions = [0.035, 0.035]
        joint_close_positions = [0.001, 0.001]

        if state == 'open':
            gripper_group.set_joint_value_target(joint_open_positions)
            gripper_group.go(wait=True)
        elif state == 'close':
            gripper_group.set_joint_value_target(joint_close_positions)
            gripper_group.go(wait=True)

        gripper_group.clear_pose_targets()
        gripper_group.stop()

    def pick_and_place_handler(self, req):
        # Perform pick and place task
        self.monter_buste([0.20])
        self.arm_move_to(1)
        self.arm_move_to(2)

        flyerNb = 0  # You can set this based on your service input

        self.go_to_height(req.flyerNb)
        self.mouvementPince('open')

        print("Moving to pick position")
        self.mouvement(req.flyerNb, True, 0.54453056471187056, 0.12492931499231739, 0.9489589262033916, 0.9927617100741939, -0.11512265866514476, 0.022540725197273525, 0.025746381881962536)
        self.mouvementPince('close')
        self.mouvement(req.flyerNb, False, 0.54453056471187056, 0.12492931499231739, 0.9489589262033916, 0.9927617100741939, -0.11512265866514476, 0.022540725197273525, 0.025746381881962536)

        self.monter_buste([0.35])
        self.mouvement(req.flyerNb, True, 0.66375268727851, 0.20147028471550366, 1.2491498772354426, 0.9807443671508588, -0.19422556540011543, 0.014899242368626761, 0.013961684128638907)

        self.mouvementPince('open')

        self.arm_move_to(1)

        home = rospy.ServiceProxy('homing', Empty)
        r = home()
        rospy.sleep(3)

        print('Descendre torse')
        self.monter_buste([0.20])

        print("Pick and place completed.")

        return EmptyResponse()

if __name__ == "__main__":
    try:
        pick_and_place_service = PickAndPlaceService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# Shutdown MoveIt when done
moveit_commander.roscpp_shutdown()