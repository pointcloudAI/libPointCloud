// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 PointCloud Corporation. All Rights Reserved

#pragma once

#include "../include/pointcloud_node_factory.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <queue>
#include <mutex>
#include <atomic>

namespace pointcloud_camera
{
    struct FrequencyDiagnostics
    {
      FrequencyDiagnostics(double expected_frequency, std::string name, std::string hardware_id) :
        expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName() + "_" + name)
      {
        ROS_INFO("Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
        diagnostic_updater_.setHardwareID(hardware_id);
        diagnostic_updater_.add(frequency_status_);
      }

      void update()
      {
        frequency_status_.tick();
        diagnostic_updater_.update();
      }

      double expected_frequency_;
      diagnostic_updater::FrequencyStatus frequency_status_;
      diagnostic_updater::Updater diagnostic_updater_;
    };
    typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;


    class BasePointCloudNode : public InterfacePointCloudNode
    {
        class float3
        {
            public:
                float x, y, z;

            public:
                float3& operator*=(const float& factor)
                {
                    x*=factor;
                    y*=factor;
                    z*=factor;
                    return (*this);
                }
                float3& operator+=(const float3& other)
                {
                    x+=other.x;
                    y+=other.y;
                    z+=other.z;
                    return (*this);
                }
        };

        struct quaternion
        {
            double x, y, z, w;
        };
    public:
        BasePointCloudNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          PointCloud::CameraSystem& cameraSys,
                          PointCloud::DevicePtr devicePtr,
                          PointCloud::FrameStreamReaderPtr  streamPtr);

        void toggleSensors(bool enabled);
        virtual void publishTopics() override;
        virtual ~BasePointCloudNode() {}
    private:
        static BasePointCloudNode* _thisPtr;
        static void frameCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c);
        void    fileThreadFunc();
                static std::string getNamespaceStr();
        tf::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const ros::Time& t,
                               const float3& trans,
                               const quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void getParameters();
        void setupPublishers();
        void setupStreams();
        void publishStaticTransforms();
        void calcAndPublishStaticTransform();
        void setBaseTime(double frame_time, bool warn_no_metadata);
        void publishPointCloud(const Frame &frame, const ros::Time& t);
        void publishFrame(const Frame &frame, const ros::Time& t);
        void updateStreamCalibData(STREAM_TYPE streamType);
    private:
        Ptr<Thread> _fileThread;
        std::string _base_frame_id;
        std::atomic_bool _is_initialized_time_base;
        const std::string _namespace;

        PointCloud::CameraSystem    _cameraSys;
        PointCloud::DevicePtr       _devicePtr;
        PointCloud::DepthCameraPtr  _depthCameraPtr;
        PointCloud::FrameStreamReaderPtr  _streamPtr;
        ros::NodeHandle& _node_handle, _pnh;
    private:
        std::string _serial_no;
        bool _pointcloud;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        std::map<STREAM_TYPE, std::string> _stream_name;
        std::map<STREAM_TYPE, int> _image_format;
        std::map<STREAM_TYPE, int> _width;
        std::map<STREAM_TYPE, int> _height;
        std::map<STREAM_TYPE, int> _fps;
        std::map<STREAM_TYPE, bool> _enable;
        std::map<STREAM_TYPE, std::string> _encoding;

        std::map<STREAM_TYPE, std::string> _frame_id;
        std::map<STREAM_TYPE, cv::Mat> _images;
        std::map<STREAM_TYPE, sensor_msgs::CameraInfo> _camera_info;

        std::map<STREAM_TYPE, int> _seq;
        std::map<STREAM_TYPE, ImagePublisherWithFrequencyDiagnostics> _image_publisher;
        std::map<STREAM_TYPE, ros::Publisher> _info_publisher;    
        ros::Publisher _pointcloud_publisher;

        ros::Time _ros_time_base;
        double _camera_time_base;
        
    };//end class

}

