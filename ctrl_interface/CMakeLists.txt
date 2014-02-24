cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/ros.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
  set(CMAKE_INSTALL_LIBDIR lib)
endif()
include(cmake/GNUInstallDirs.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("roscpp")
ADD_REQUIRED_DEPENDENCY("std_msgs")
ADD_REQUIRED_DEPENDENCY("sensor_msgs")
add_required_dependency(sot-core)
ADD_ROSPACK_DEPENDENCY("dynamic_graph_bridge")
# add_required_dependency(dynamic-graph)
ADD_REQUIRED_DEPENDENCY("urdfdom")

# Handle rpath necessary to handle ROS multiplace packages
# libraries inclusion
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(SOURCES
   src/vel_bridge_main.cpp
   src/vel_bridge.cpp
   )
#    src/pr2_sot_controller.cpp
#    src/pr2_device.cpp)

set(HEADERS
    src/vel_bridge.hpp)
#    include/sot_pr2/pr2_sot_controller.h
#    include/sot_pr2/pr2_device.h)

#rosbuild_add_library(sot_pr2 ${HEADERS} ${SOURCES})

rosbuild_add_executable(vel_bridge ${SOURCES})
target_link_libraries(vel_bridge urdfdom_model)

rosbuild_add_executable(fb_bridge src/fb_bridge.cpp src/fb_bridge_main.cpp)
rosbuild_add_executable(fake_sot_test src/fake_sot.cpp)

ROSPACK_USE_DEPENDENCY(vel_bridge    "dynamic_graph_bridge")
ROSPACK_USE_DEPENDENCY(fb_bridge     "dynamic_graph_bridge")
ROSPACK_USE_DEPENDENCY(fake_sot_test "dynamic_graph_bridge")
#TODO: CORRECT THE ISSUE in the manifest.xml dg_bridge !!!!
TARGET_LINK_LIBRARIES(vel_bridge "-L${SOT_CORE_LIBDIR}")
TARGET_LINK_LIBRARIES(fb_bridge "-L${SOT_CORE_LIBDIR}")
TARGET_LINK_LIBRARIES(fake_sot_test "-L${SOT_CORE_LIBDIR}")

# install(TARGETS ctrl_interface DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ADD_SUBDIRECTORY(src)
