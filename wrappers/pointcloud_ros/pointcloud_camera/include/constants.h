// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 PointCloud Corporation. All Rights Reserved

#pragma once

#include <string>

#define POINTCLOUD_ROS_MAJOR_VERSION    1
#define POINTCLOUD_ROS_MINOR_VERSION    0
#define POINTCLOUD_ROS_PATCH_VERSION    0

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#define POINTCLOUD_ROS_VERSION_STR (VAR_ARG_STRING(POINTCLOUD_ROS_MAJOR_VERSION.POINTCLOUD_ROS_MINOR_VERSION.POINTCLOUD_ROS_PATCH_VERSION))

namespace pointcloud_camera
{
    const bool POINTCLOUD     = false;
    
    const int IMAGE_WIDTH     = 640;
    const int IMAGE_HEIGHT    = 480;
    const int IMAGE_FPS       = 30;

    const bool ENABLE_DEPTH   = true;
    const bool ENABLE_PHASE  = true;
    const bool ENABLE_AMPLITUDE  = true;


    const std::string POINTCLOUD_PUBLISH_ID            = "depth/points";


    const std::string DEFAULT_BASE_FRAME_ID            = "camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID           = "camera_depth_frame";
    const std::string DEFAULT_INFRA1_FRAME_ID          = "camera_phase_frame";
    const std::string DEFAULT_INFRA2_FRAME_ID          = "camera_amplitude_frame";
    const std::string DEFAULT_COLOR_FRAME_ID           = "camera_color_frame";
    const std::string DEFAULT_UNITE_IMU_METHOD         = "";
    const std::string DEFAULT_FILTERS                  = "";
}  
