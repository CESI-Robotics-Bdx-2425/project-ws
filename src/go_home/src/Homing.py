#!/usr/bin/env python3
from std_srvs.srv import Empty, EmptyResponse
import rospy
import moveit_commander
import sys


class Homing:
    def __init__(self):
        # Initialize the moveit_commander and rospy nodes

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("simple_moveit", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander("both_arms_torso")

        #### Get the robot Homing joint state
        # Get the current joint values
        # Ensure the robot state is updated

        robot_state = robot.get_current_state()
        self.move_group.set_start_state(robot_state)

        # Get the current joint values

        joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo("Current joint values: %s", joint_values)

        #### Get the robot Homing joint state
        #### Send the robot to Homing position

        self.joint_home = [
            0.2, # Torse
            -1.1001652770191288,
            1.4679210480602556,
            2.7139581408352638,
            1.7092685314029974,
            -1.5709013012901742,
            1.36994880082998,
            0.00016342434524205425,
            -1.099986792914449,
            1.4679260452329381,
            2.714045725773213,
            1.7096175853518232,
            -1.5707758171968906,
            1.3699403874228377,
            -0.0001064575413322888,
        ]

        self.s = rospy.Service("homing", Empty, self.home)

        rospy.spin()

    def home(self, _):
        self.move_group.go(self.joint_home, wait=True)
        self.move_group.clear_pose_targets()
        self.move_group.stop()

        return EmptyResponse()

if __name__ == "__main__":
    homing = Homing()