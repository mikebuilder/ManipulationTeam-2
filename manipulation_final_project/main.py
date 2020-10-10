#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from manipulation_final_project.srv import GetGraspPoints


service_name = "get_grasp_points"

def handle_points(point_cloud):
    print("Received point cloud of size " + str(point_cloud.width) + " x " + str(point_cloud.height))
    getGraspPoints = rospy.ServiceProxy("get_grasp_points",GetGraspPoints)
    graspPoints = getGraspPoints(point_cloud)
    print("grasp point 1:")
    print(graspPoints.grasp_point_1)
    print("grasp point 2:")
    print(graspPoints.grasp_point_2)


if __name__ == '__main__':
    rospy.init_node("main_py");
    print("waiting for grasp points service...")
    rospy.wait_for_service("get_grasp_points")
    print("get grasp point service found...")

    points_subscriber = rospy.Subscriber("camera/depth/points", PointCloud2, handle_points)

    rospy.spin()
