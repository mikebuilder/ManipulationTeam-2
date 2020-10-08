#!/usr/bin/env python
import rospy
import sensor_msgs

service_name = "get_grasp_points"

def handle_points(point_cloud):
    print("Received point cloud of size " + point_cloud.width + " x " + point_cloud.height)

if __name__ == '__main__':
    rospy.init_node("main_py");

    points_subscriber = rospy.Subscriber("camera/depth/points", sensor_msgs.msg.PointCloud2, handle_points)

    rospy.spin()
