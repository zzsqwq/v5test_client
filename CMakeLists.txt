cmake_minimum_required(VERSION 3.0.2)
project(v5test_client)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        cv_bridge
        image_transport
        message_generation

)
find_package(OpenCV 3 REQUIRED)

add_service_files(
        FILES
        Polygon_outerpoints.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package()

include_directories(utils)

add_subdirectory(utils)
add_subdirectory(image_publisher)
