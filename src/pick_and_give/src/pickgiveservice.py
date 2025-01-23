#!/usr/bin/env python3
from std_srvs.srv import Empty, EmptyResponse
from pick_and_give.srv import PickAndGive
import rospy
import moveit_commander
import sys
import copy
from pprint import pprint
from scipy.spatial.transform import Rotation as R

class PickAndPlaceService:
    def __init__(self):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place_service", anonymous=True)
        rospy.loginfo('Node pick_and_place_service started !')

        rospy.wait_for_service('/homing')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group_torso = moveit_commander.MoveGroupCommander("torso")

        self.move_group_arm_left = moveit_commander.MoveGroupCommander("arm_left")
        self.move_group_arm_right = moveit_commander.MoveGroupCommander("arm_right")

        self.gripper_group_left = moveit_commander.MoveGroupCommander("gripper_left")
        self.gripper_group_right = moveit_commander.MoveGroupCommander("gripper_right")

        self.s = rospy.Service("pick_and_place", PickAndGive, self.pick_and_place_handler)
        rospy.sleep(2) # Ensure that the robot is initialized properly
        rospy.loginfo('Service pick_and_place started')


    def monter_buste(self, hauteur):
        self.move_group_torso.set_joint_value_target(hauteur)
        self.move_group_torso.go(wait=True)

    def arm_move_to(self, number, arm):
        self.move_group_arm_left.stop()
        self.move_group_arm_left.clear_pose_targets()

        self.move_group_arm_right.stop()
        self.move_group_arm_right.clear_pose_targets()

        joint_start_1_l = [0.011985545972480017, -0.03806922637254751, 1.5246317207337126, 1.621692406596665, -1.4274444343167114, 1.3826300810742314, 0.006182632455332989]
        joint_start_1_r = [0.011985545972480017, 0.03806922637254751, 1.5246317207337126, 1.621692406596665, -1.4274444343167114, 1.3826300810742314, -0.006182632455332989]
        
        joint_start_2_l = [0.9421578142999586, -0.21311409590575092, 1.4010525892851553, 2.2242633924991946, -1.2781702963473467, -1.3937755123985682, 0.04869687921137075]
        joint_start_2_r = [0.9421578142999586, 0.21311409590575092, 1.4010525892851553, 2.2242633924991946, -1.4781702963473467, -1.3937755123985682, -0.04869687921137075]
        
        if arm == "left":
            if number == 1:
                self.move_group_arm_left.go(joint_start_1_l, wait=True)
            elif number == 2:
                self.move_group_arm_left.go(joint_start_2_l, wait=True)
        elif arm == "right":
            if number == 1:
                self.move_group_arm_right.go(joint_start_1_r, wait=True)
            elif number == 2:
                self.move_group_arm_right.go(joint_start_2_r, wait=True)

        self.move_group_arm_left.clear_pose_targets()
        self.move_group_arm_left.stop()

    def go_to_height(self, n):
        torso_joint_values = self.move_group_torso.get_current_joint_values()
        new = [torso_joint_values[0] - 0.06 * n]
        self.move_group_torso.set_joint_value_target(new)
        self.move_group_torso.go(wait=True)

    def take(self, flyerNb, take, arm, px, py, pz, ox, oy, oz, ow):
        if arm == "left":
            move_group = self.move_group_arm_left
        elif arm == "right":
            move_group = self.move_group_arm_right
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
        if arm == "right":
            # Define quaternion
            quaternion = R.from_quat([ox, oy, oz, ow]) # Quaternion to rotate
            q_rotation = R.from_quat([1, 0, 0, 0]) # Quaternion for a rotation of 180° on X

            q_rotated = (q_rotation * quaternion.inv()).as_quat()
            ox, oy, oz, ow = q_rotated

            wpose.orientation.x = ox
            wpose.orientation.x = oy
            wpose.orientation.z = oz
            wpose.orientation.w = ow
            wpose.position.z += 0.02
        if arm == "left":
            wpose.orientation.x = ox
            wpose.orientation.y = oy
            wpose.orientation.z = oz
            wpose.orientation.w = ow
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0)
        print(fraction)
        if fraction == 1.0:
            move_group.execute(plan, wait=True)
        else:
            raise RuntimeError(f"Failed to plan the entire path. Fraction: {fraction}")
        move_group.clear_pose_targets()
        move_group.stop()
    
    def give(self, arm, give):
        if arm == "left":
            move_group = self.move_group_arm_left
        elif arm == "right":
            move_group = self.move_group_arm_right

        move_group.stop()
        move_group.clear_pose_targets()

        waypoints = []

        wpose = move_group.get_current_pose().pose

        if give == True:
            wpose.position.x = 0.66375268727851
            if arm == "left":
                wpose.position.y = 0.20147028471550366
            elif arm == "right":
                wpose.position.y = - 0.20147028471550366
            wpose.position.z = 1.2491498772354426
        if give == False:
            wpose.position.x = 0.36375268727851
            if arm == "left":
                wpose.position.y = 0.40147028471550366
            elif arm == "right":
                wpose.position.y = - 0.40147028471550366
            wpose.position.z = 1.2491498772354426

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0)
        print(fraction)
        move_group.execute(plan, wait=True)
        move_group.clear_pose_targets()
        move_group.stop()

    def mouvementPince(self, state, arm):
        if arm == "left":
            move_gripper = self.gripper_group_left
        elif arm == "right":
            move_gripper = self.gripper_group_right

        move_gripper.clear_pose_targets()

        joint_open_positions = [0.030, 0.030]
        joint_close_positions = [0.001, 0.001]

        if state == 'open':
            move_gripper.set_joint_value_target(joint_open_positions)
            move_gripper.go(wait=True)
        elif state == 'close':
            move_gripper.set_joint_value_target(joint_close_positions)
            move_gripper.go(wait=True)

        move_gripper.clear_pose_targets()
        move_gripper.stop()

    def pick_and_place_handler(self, req):
        # Perform pick and give task
        try:
            coordinates = req.coordinates
            if coordinates.pose.position.y > 0:
                arm = "left"
            else:
                arm = "right"
            self.monter_buste([0.20])
            self.arm_move_to(1, arm)
            self.go_to_height(req.flyerNb)

            self.mouvementPince('open', arm)

            # bonnes coordonnées
            # self.take(req.flyerNb, True, arm, 0.54453056471187056, 0.12492931499231739, 0.9489589262033916, 0.9927617100741939, -0.11512265866514476, 0.022540725197273525, 0.025746381881962536)
            # self.mouvementPince('close')
            # self.take(req.flyerNb, False, arm, 0.54453056471187056, 0.12492931499231739, 0.9489589262033916, 0.9927617100741939, -0.11512265866514476, 0.022540725197273525, 0.025746381881962536)

            print("take")
            self.take(req.flyerNb, True, arm, coordinates.pose.position.x, coordinates.pose.position.y, 0.95, coordinates.pose.orientation.x, coordinates.pose.orientation.y, coordinates.pose.orientation.z, coordinates.pose.orientation.w)
            self.mouvementPince('close', arm)
            print("take return")
            self.take(req.flyerNb, False, arm, coordinates.pose.position.x, coordinates.pose.position.y, 0.95, coordinates.pose.orientation.x, coordinates.pose.orientation.y, coordinates.pose.orientation.z, coordinates.pose.orientation.w)

            self.monter_buste([0.35])
            print("give")
            self.give(arm, True)
            rospy.sleep(2)
            self.mouvementPince('open', arm)
            self.give(arm, False)

            self.arm_move_to(1, arm)

            home = rospy.ServiceProxy('homing', Empty)
            r = home()

            print("Pick and place completed.")

            return 0
        except Exception as e:
            return -1

if __name__ == "__main__":
    try:
        pick_and_place_service = PickAndPlaceService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# Shutdown MoveIt when done
moveit_commander.roscpp_shutdown()