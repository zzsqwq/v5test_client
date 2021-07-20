project(publisher_toolbox)

add_library(publisher_toolbox
        SHARED
        publisher_toolbox.cpp)

#target_sources(publisher_toolbox
#        SHARED
#        "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(publisher_toolbox
        PUBLIC
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES})

target_include_directories(publisher_toolbox
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

add_library(publisher::tool ALIAS publisher_toolbox)
