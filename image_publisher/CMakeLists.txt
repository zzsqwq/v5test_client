project(image_publisher)

add_executable(image_publisher image_publisher.cpp)

target_link_libraries(image_publisher
        PUBLIC
        publisher::tool
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

target_include_directories(image_publisher
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        $ENV{HOME}/v5test_ws/devel/include)