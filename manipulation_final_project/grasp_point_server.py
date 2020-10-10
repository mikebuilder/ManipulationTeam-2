#!/usr/bin/env python
import rospy
from numpy import *
from manipulation_final_project.srv import GetGraspPoints,GetGraspPointsResponse
from sensor_msgs import *

service_name = "get_grasp_points"

def get_grasp_points(get_grasp_points_request):
    cloud = get_grasp_points_request.point_cloud
    # print(cloud)
    print("processing point cloud")
    pointData = cloud.data
    print(type(cloud))
    print(type(pointData))
    print(type(pointData[1]))

    print("got grasp points, returning and exiting")
    return GetGraspPointsResponse(None, None)

if __name__ == '__main__':
    rospy.init_node(service_name + "_server")
    # svc = rospy.Service(service_name, me.srv.GetGraspPoints, get_grasp_points)
    svc = rospy.Service(service_name, GetGraspPoints, get_grasp_points)

    print("Started " + service_name + " service.")
    rospy.spin();
