// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 PointCloud Corporation. All Rights Reserved

#include "../include/pointcloud_node_factory.h"
#include "../include/base_pointcloud_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>

using namespace pointcloud_camera;

#define POINTCLOUD_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: POINTCLOUD_ROS_MAJOR_VERSION.POINTCLOUD_ROS_MINOR_VERSION.POINTCLOUD_ROS_PATCH_VERSION))
constexpr auto pointcloud_ros_camera_version = POINTCLOUD_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(pointcloud_camera::PointCloudNodeFactory, nodelet::Nodelet)

static PointCloud::CameraSystem    _cameraSys;
static PointCloud::DepthCameraPtr  _depthCamera;
static PointCloud::DevicePtr       _device;

PointCloudNodeFactory::PointCloudNodeFactory()
{
	ROS_INFO("PointCloud ROS v%s", POINTCLOUD_ROS_VERSION_STR);

	_device = nullptr;
	signal(SIGINT, signalHandler);
	PointCloud::LogLevel level = PointCloud::LOG_DEBUG;
	if (PointCloud::LOG_DEBUG == level)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	PointCloud::logger.setDefaultLogLevel(level);
}

void PointCloudNodeFactory::signalHandler(int signum)
{
	ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating PointCloud Node...");
  
	_depthCamera->stop();
    _cameraSys.disconnect(_depthCamera,true);
	ros::shutdown();
	exit(signum);
}


void PointCloudNodeFactory::onInit()
{
	try{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif

		auto nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		std::string serial_no("");
		privateNh.param("serial_no", serial_no, std::string(""));

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));

		if (!rosbag_filename.empty())
		{
			ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
			PointCloud::FrameStreamReaderPtr  frameStream = new PointCloud::FrameStreamReader(rosbag_filename.c_str(), _cameraSys);
        	if(!frameStream->isStreamGood())
            {
				ros::shutdown();
				exit(1);
			}
			_pointCloudNode = std::unique_ptr<BasePointCloudNode>(new BasePointCloudNode(nh, privateNh,_cameraSys, nullptr,frameStream ));
		}
		else
		{

    		logger.setDefaultLogLevel(LOG_ERROR);
			const PointCloud::Vector<PointCloud::DevicePtr> &devices = _cameraSys.scan();
		    bool found = false;
		    for (auto &d: devices){
		        PointCloud::logger(PointCloud::LOG_INFO) <<  "Detected devices: "  << d->id() << std::endl;
		        _device = d;
		        found = true;
		    }

		    if(!found)
		    {
		    	ROS_FATAL("The device has been found! Terminating PointCloud Node...");
				ros::shutdown();
				exit(1);
		    }
			_pointCloudNode = std::unique_ptr<BasePointCloudNode>(new BasePointCloudNode(nh, privateNh, _cameraSys,_device, nullptr));
				
		}

		assert(_pointCloudNode);
		_pointCloudNode->publishTopics();
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		throw;
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		throw;
	}
}

