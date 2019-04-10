# - Config file for the FooBar package
# It defines the following variables
#  POINTCLOUD_INCLUDE_DIRS - include directories for FooBar
#  POINTCLOUD_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(POINTCLOUD_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(POINTCLOUD_INCLUDE_DIRS ${POINTCLOUD_CMAKE_DIR}/../../../include/pointcloud-1.0.0 ${POINTCLOUD_CMAKE_DIR}/Util/)
#set(POINTCLOUD_INCLUDE_DIRS ${POINTCLOUD_CMAKE_DIR}/../../../include/pointcloud-1.0.0 COMMON_INCLUDE)

set(POINTCLOUD_ABI_VERSION 0)
 
# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET PointCloud::pointcloud AND NOT PointCloud_BINARY_DIR)
  include("${POINTCLOUD_CMAKE_DIR}/PointCloudTargets.cmake")
endif()
 
# These are IMPORTED targets created by FooBarTargets.cmake
set(POINTCLOUD_LIBRARIES PointCloud::pointcloud)
