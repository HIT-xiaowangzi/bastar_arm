cmake_minimum_required(VERSION 3.0.2)
project(wrc_hit_lib)
## Compile as C++11, supported in ROS Kinetic and newer
if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        baxter_core_msgs
        kdl_parser
        pcl_ros
        pcl_conversions
        cv_bridge
        image_transport
        trac_ik_lib
        )

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})

find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

# find required opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(
        LIBRARIES
        CATKIN_DEPENDS
        trac_ik_lib
)

include_directories(
        /home/hit/wrc_hit/src/wrc_hit_lib/image
        /home/hit/wrc_hit/src/wrc_hit_lib/include
        /home/hit/wrc_hit/src/wrc_hit_lib/include/wrc_hit_lib
        ${catkin_INCLUDE_DIRS})

add_executable(wrc_hit_lib src/main.cpp
        src/baxtercontroller.cpp include/wrc_hit_lib/baxtercontroller.h
        src/first_task.cpp include/wrc_hit_lib/first_task.h
        include/wrc_hit_lib/third_task.h src/third_task.cpp
        src/camera_info.cpp include/wrc_hit_lib/camera_info.h src/forth_task.cpp include/wrc_hit_lib/forth_task.h
        src/Realsense_info.cpp src/Realsense_info.h 
        src/elbowcontroller.cpp src/elbowcontroller.h)


target_link_libraries(wrc_hit_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(test1 src/test.cpp
        src/baxtercontroller.cpp include/wrc_hit_lib/baxtercontroller.h
        src/camera_info.cpp include/wrc_hit_lib/camera_info.h )
target_link_libraries(test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

