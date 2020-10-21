#!/usr/bin/env python
import moveit_commander as mv
import rospy
import sys

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from manipulation_final_project.srv import GetGraspPoints

service_name = "get_grasp_points"
camera_topic = "camera/depth/points"
move_group_name = "panda_arm"

getGraspPoints = rospy.ServiceProxy(service_name, GetGraspPoints)

pi = 3.14159265358979

def get_pose_from_cloud(cloud):
    rospy.loginfo("Received point cloud of size " + str(cloud.width) + " x " + str(cloud.height))

    graspPoints = getGraspPoints(cloud)

    pose = Pose()
    pose.position.x = graspPoints.com.x + 0.35
    pose.position.y = graspPoints.com.y
    pose.position.z = 0.755

    # manually picked quaternion values based on initial orientation state of robot
    pose.orientation.x = -0.5
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

    return pose

if __name__ == '__main__':
    mv.roscpp_initialize(["joint_states:=/panda/joint_states"])
    rospy.wait_for_service(service_name)
    rospy.init_node("main_py", log_level = rospy.DEBUG);

    robot = mv.RobotCommander()
    rospy.loginfo(robot.get_group_names())
    rospy.loginfo(robot.get_link_names()) # gets all links in the model regardless of group membership

    scene = mv.PlanningSceneInterface()
    moveGroup = mv.MoveGroupCommander(move_group_name)

    #joint_goal = moveGroup.get_current_joint_values()
    #joint_goal[0] = 0
    #joint_goal[1] = -pi/4
    #joint_goal[2] = 0
    #joint_goal[3] = -pi/2
    #joint_goal[4] = 0
    #joint_goal[5] = pi/3
    #joint_goal[6] = 0

    #moveGroup.go(joint_goal, wait = True)
    #moveGroup.stop()

    while not rospy.is_shutdown():
        # =============================
        # NOTE: Grippers can be moved by publishing to /panda/panda_finger1_controller/command
        # and /panda/panda_finger2_controller/command. A value of 0.0 is fully closed and
        # a value of ~0.04 is fully open for both fingers.
        # =============================

        cloud = rospy.wait_for_message(camera_topic, PointCloud2, timeout = None)
        pose = get_pose_from_cloud(cloud)

        moveGroup.set_pose_target(pose)
        joint_goal = moveGroup.get_joint_value_target() # convert target pose into joint values - seems to move better

        # manually adjust wrist and joint right before wrist
        joint_goal[5] = pi
        joint_goal[6] = pi/4 # wrist

        moveGroup.go(joint_goal, wait = True) # execute joint goals (to use pose, remove joint_goal parameter)
        moveGroup.stop()
        moveGroup.clear_pose_targets()

        break
