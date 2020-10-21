#!/usr/bin/env python
import rospy
from numpy import *
from manipulation_final_project.srv import GetGraspPoints,GetGraspPointsResponse
from sensor_msgs import *
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import *

service_name = "get_grasp_points"

def get_grasp_points(get_grasp_points_request):
    cloud = get_grasp_points_request.point_cloud
    # print(cloud)
    print("processing point cloud")

    # find minimum Z heigh of point cloud (point on object closest to camera)
    minZ = 10000
    for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        # use this to find minimum z point (closest to camera) for filtering out all other points later
        if(p[2]<minZ):
            minZ = p[2]

    # find all points within a certain z height and calculate COM
    usefulPoints = []
    count = 0
    xSum = 0
    ySum = 0
    zSum = 0
    for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        # only add points within z height, calculate COM while you're at it
        if(p[2]>minZ+.028 and p[2]<minZ+.032):
            usefulPoints.append((p[0], p[1], p[2]))
            # add values for COM calculation
            xSum += p[0]
            ySum += p[1]
            zSum += p[2]
            count += 1

    print("minimum z height: " + str(minZ))
    print("total points for this plane" + str(len(usefulPoints)))

    # calculate COM
    COM = (xSum/count, ySum/count, zSum/count)
    print("center of mass: " + str(COM[0]) + ", "+ str(COM[1]) + ", " + str(COM[2]))

    # find closest point to COM
    closest = (0,0,0)
    dist = 1000
    for p in usefulPoints:
        newDist = sqrt((COM[0]-p[0])**2 + (COM[1]-p[1])**2 + (COM[2]-p[2])**2)
        if newDist < dist:
            dist = newDist
            closest = (p[0], p[1], p[2])

    print("closest point is: " + str(closest[0]) + ", "+ str(closest[1]) + ", " + str(closest[2]))
    graspPoint1 = Point()
    graspPoint1.x = closest[0]
    graspPoint1.y = closest[1]
    graspPoint1.z = closest[2]

    # find diametrically opposite point
    # first find the true opposite, which can be found using midpoint formula, then find the closest point cloud point to that point
    opposite = (2*COM[0]-closest[0],2*COM[1]-closest[1],closest[2])

    # opposite is now the true diametrically opposite point, we will find the closest point in the point cloud to that point
    trueOpposite = (0, 0, 0)
    dist = 1000
    for p in usefulPoints:
        newDist = sqrt((opposite[0] - p[0]) ** 2 + (opposite[1] - p[1]) ** 2 + (opposite[2] - p[2]) ** 2)
        if newDist < dist:
            dist = newDist
            trueOpposite = (p[0], p[1], p[2])

    graspPoint2 = Point()
    graspPoint2.x = trueOpposite[0]
    graspPoint2.y = trueOpposite[1]
    graspPoint2.z = trueOpposite[2]


    print("got grasp points, returning and exiting")

    comPoint = Point()
    comPoint.x = COM[0]
    comPoint.y = COM[1]
    comPoint.z = COM[2]

    return GetGraspPointsResponse(comPoint, graspPoint1, graspPoint2)

if __name__ == '__main__':
    rospy.init_node(service_name + "_server")
    # svc = rospy.Service(service_name, me.srv.GetGraspPoints, get_grasp_points)
    svc = rospy.Service(service_name, GetGraspPoints, get_grasp_points)

    print("Started " + service_name + " service.")
    rospy.spin();
