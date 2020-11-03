cmake_minimum_required(VERSION 2.8.3)
project(dvs_test)

set(CMAKE_CXX_STANDARD 14)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11) 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dvs_msgs
  cv_bridge
  image_transport
  pcl_ros
  sensor_msgs
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(CUDA REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories ("/home/user/catkin_ws/opencv-3.4.0/include")
include_directories ("/home/huang/catkin_ws/opencv/include")
include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Eigen_LIBRARY_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include/")

#include_directories ("/home/huang/catkin_ws/src/rpg_dvs_ros-update-use_libcaer3_with_davis346/rpg_corner_events/corner_event_detector/include")

message("Opencv lib : ${OpenCV_INCLUDE_DIRS}")
set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dvs_msgs roscpp rospy sensor_msgs std_msgs cv_bridge image_transport pcl_ros DEPENDS OpenCV boost

)



message("Opencv lib : ${OpenCV_INCLUDE_DIRS}")
set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})


add_executable(dvs_test src/meanshift-mat/dvs_test_node.cpp src/meanshift-mat/meanShift-huang.cpp src/meanshift-mat/meanShift-Gaussian.cpp src/meanshift-mat/Point.cpp src/meanshift-mat/pca.cpp src/meanshift-mat/FindFarestPoints.cpp)
target_link_libraries(dvs_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(dvs_test ${LIBS} ${dvs_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


