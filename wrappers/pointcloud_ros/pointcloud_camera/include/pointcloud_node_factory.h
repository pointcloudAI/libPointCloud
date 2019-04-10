// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 PointCloud Corporation. All Rights Reserved

#pragma once

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <constants.h>
#include <pointcloud_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <csignal>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include "CameraSystem.h"

using namespace PointCloud;

namespace pointcloud_camera
{
    
    enum STREAM_TYPE
    {
        UNKOWN_STREAM_TYPE,
        DEPTH,
        COLOR
    };

    const std::vector<STREAM_TYPE> IMAGE_STREAMS = {DEPTH};
    class InterfacePointCloudNode
    {
    public:
        virtual void publishTopics() = 0;
        virtual ~InterfacePointCloudNode() = default;
    };

    class PointCloudNodeFactory : public nodelet::Nodelet
    {
    public:
        PointCloudNodeFactory();
        virtual ~PointCloudNodeFactory() {}

    private:
        static void signalHandler(int signum);
        virtual void onInit() override;
        std::unique_ptr<InterfacePointCloudNode> _pointCloudNode;
    };
}
