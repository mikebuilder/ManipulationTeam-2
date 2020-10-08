cmake_minimum_required(VERSION 3.0.2)

project(manipulation_final_project)

find_package(catkin
  REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs
)

add_service_files(
  FILES GetGraspPoints.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs
)
