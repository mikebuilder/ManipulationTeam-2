#!/usr/bin/env python
import rospy
import manipulation_final_project as me

service_name = "get_grasp_points"

def get_grasp_points(get_grasp_points_request):
    print(get_grasp_points_request.point_cloud)
    return manipulation_final_project.srv.GetGraspPointsResponse(grasp_point_1 = None, grasp_point_2 = None)

if __name__ == '__main__':
    rospy.init_node(service_name + "_server");
    svc = rospy.Service(service_name, me.srv.GetGraspPoints, get_grasp_points)

    print("Started " + service_name + " service.")
    rospy.spin();
