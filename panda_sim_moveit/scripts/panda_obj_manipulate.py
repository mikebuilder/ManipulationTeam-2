#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class PandaObjManipulate(object):
    def __init__(self):
        super(PandaObjManipulate, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                         moveit_msgs.msg.DisplayTrajectory,
        #                                         queue_size=20)

        # planning_frame = move_group.get_planning_frame()
        # eef_link = move_group.get_end_effector_link()
        # group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        # self.scene = scene
        self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        # self.planning_frame = planning_frame
        # self.eef_link = eef_link
        # self.group_names = group_names

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()



def main():
    try:
        robot = PandaObjManipulate()
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()