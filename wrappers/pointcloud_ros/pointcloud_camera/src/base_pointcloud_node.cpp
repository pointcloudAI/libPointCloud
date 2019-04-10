#include "../include/base_pointcloud_node.h"
#include "assert.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <mutex>
#include <tf/transform_broadcaster.h>

using namespace pointcloud_camera;
#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _stream_name[sip] )).str()
#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_frame")).str()
std::mutex mtx;
bool ready = false;
std::condition_variable wait_con;
BasePointCloudNode* BasePointCloudNode::_thisPtr = 0;

std::string BasePointCloudNode::getNamespaceStr()
{
    auto ns = ros::this_node::getNamespace();
    ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
    return ns;
}

BasePointCloudNode::BasePointCloudNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     PointCloud::CameraSystem& cameraSys,
                                     PointCloud::DevicePtr devicePtr,
                                     PointCloud::FrameStreamReaderPtr  streamPtr) :
    _node_handle(nodeHandle),_pnh(privateNodeHandle),
    _cameraSys(cameraSys),_devicePtr(devicePtr),_streamPtr(streamPtr),_base_frame_id(""),
    _is_initialized_time_base(false),
    _namespace(getNamespaceStr())
{

    _depthCameraPtr = nullptr;
    _stream_name[DEPTH] = "depth";
    _image_format[DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _thisPtr = this;

    ROS_INFO_STREAM("ROS Node Namespace: " << _namespace);
}

void BasePointCloudNode::toggleSensors(bool enabled)
{
    ROS_DEBUG_STREAM("toggleSensors: " << enabled);
    if (_depthCameraPtr && enabled && _depthCameraPtr->isInitialized())
        _depthCameraPtr->start();
    else
        _depthCameraPtr->stop();
}

void BasePointCloudNode::getParameters()
{
    _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
    ROS_DEBUG_STREAM("getParameters... enable_pointcloud:" << _pointcloud);
    _pointcloud = true;

    for (auto& stream : IMAGE_STREAMS)
    {
        std::string param_name(_stream_name[stream] + "_width");
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _width[stream], IMAGE_WIDTH);
        param_name = _stream_name[stream] + "_height";
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _height[stream], IMAGE_HEIGHT);
        param_name = _stream_name[stream] + "_fps";
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _fps[stream], IMAGE_FPS);
        param_name = "enable_" + STREAM_NAME(stream);
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _enable[stream], true);
        _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
        param_name = static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_frame_id").str();
        _pnh.param(param_name, _frame_id[stream], FRAME_ID(stream));
        ROS_DEBUG_STREAM("frame_id: reading parameter:" << param_name << " : " << _frame_id[stream]);
    }
}

void BasePointCloudNode::fileThreadFunc()
{
    std::unique_lock <std::mutex> lck(mtx);
    while (!ready) 
        wait_con.wait(lck); 

    uint numFrames = _streamPtr->size();
    for(int i=0;i<numFrames;i++)
    {
        if(!_streamPtr->readNext())
        {
            printf("Failed to read frame %d",i);
            break;
        }
        FramePtr rawFrame = _streamPtr->frames[DepthCamera::FRAME_RAW_FRAME_PROCESSED];//DepthCamera::FRAME_RAW_FRAME_UNPROCESSED]);
        FramePtr ptFrame = _streamPtr->frames[DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME];//DepthCamera::FRAME_RAW_FRAME_UNPROCESSED]);
        ROS_INFO_STREAM(" fileThreadFunc:" << i << " frame:" << rawFrame);

        try{
            double frame_time = rawFrame->timestamp;
            bool placeholder_false(false);
            if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
            {
                setBaseTime(frame_time, true);
            }

            ros::Time t = ros::Time(_ros_time_base.toSec()+ (frame_time -  _camera_time_base) / 1000);
           
            if (0 != _pointcloud_publisher.getNumSubscribers())
            {
                publishPointCloud(*ptFrame.get(), t);
            }

            publishFrame(*rawFrame.get(), t);
        }
        catch(const std::exception& ex)
        {
            ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
        }
    }
}

void BasePointCloudNode::setupStreams()
{
    
    ROS_INFO("setupStreams initialized Depth Camera ");
    if(_devicePtr!= nullptr)
    {
        _serial_no = _devicePtr->id();
        _depthCameraPtr = _cameraSys.connect(_devicePtr);
        if (!_depthCameraPtr) {
            logger(LOG_ERROR) << "Could not load depth camera for device "<< _devicePtr->id() << std::endl;
            ros::shutdown();
            exit(1);
        }
        
        if (!_depthCameraPtr->isInitialized()) {
            logger(LOG_ERROR) << "Depth camera not initialized for device "<< _devicePtr->id() << std::endl;
            ros::shutdown();
            exit(1);
        }
        
        _depthCameraPtr->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,BasePointCloudNode::frameCallback);
        _depthCameraPtr->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,BasePointCloudNode::frameCallback);
       
        _depthCameraPtr->start();

    }

    if(_depthCameraPtr!=nullptr)
    {

        for (auto& elem : IMAGE_STREAMS)
        {
            if (_enable[elem])
            {       
                FrameSize size;
                _depthCameraPtr->getFrameSize(size);
                _width[elem] = size.width;
                _height[elem] = size.height;
                 FrameRate r;
                _depthCameraPtr->getFrameRate(r);
                _fps[elem] = r.getFrameRate();
                _images[elem] = cv::Mat(_height[elem], _width[elem], _image_format[elem], cv::Scalar(0, 0, 0));

                updateStreamCalibData(elem);
                ROS_INFO_STREAM(STREAM_NAME(elem) << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem]);    
            }
        }
    }else if(_streamPtr != nullptr)
    {
        if(!_streamPtr->isStreamGood())
            return;

        updateStreamCalibData(DEPTH);
        _images[DEPTH] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH], cv::Scalar(0, 0, 0));
        _fileThread = Ptr<Thread> (new Thread(&BasePointCloudNode::fileThreadFunc, this));
        if (_fileThread )
        {
            _fileThread->detach();
        }
        
    }else
    {
        ROS_ERROR_STREAM("When setup Streams, An error has occurred ! \n" );
    }

}

void BasePointCloudNode::setupPublishers()
{
    ROS_INFO_STREAM("setupPublishers..." << _pointcloud);
    image_transport::ImageTransport image_transport(_node_handle);

    for (auto& stream : IMAGE_STREAMS)
    {
        if (_enable[stream] )
        {
            std::stringstream image_raw, camera_info;
            std::string stream_name(STREAM_NAME(stream));
            image_raw << stream_name << "/image_"  << "raw";
            camera_info << stream_name << "/camera_info";
            uint fps = 0;
            if(_depthCameraPtr)
            {
                
                FrameRate r;
                _depthCameraPtr->getFrameRate(r);
                fps = r.getFrameRate();
            }else
                fps = _fps[stream];

            std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(fps, stream_name, _serial_no));
            _image_publisher[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

            if (stream == DEPTH && _pointcloud)
            {
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_PUBLISH_ID, 1);
            }
        }
    }
}


void BasePointCloudNode::publishStaticTransforms()
{
    for (auto& stream : IMAGE_STREAMS)
    {
        if (_enable[stream])
        {
            calcAndPublishStaticTransform();
        }
    }
}

void BasePointCloudNode::publishTopics()
{
    getParameters();
    setupPublishers();
    setupStreams();
    publishStaticTransforms();
    
    ready = true; 
    wait_con.notify_all(); 

    ROS_INFO_STREAM("PointCloud Node Is Up!");
}

tf::Quaternion BasePointCloudNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as Pointcloud rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BasePointCloudNode::publish_static_tf(const ros::Time& t,
                                          const float3& trans,
                                          const quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    msg.transform.rotation.w = q.w;
    _static_tf_broadcaster.sendTransform(msg);
}

void BasePointCloudNode::calcAndPublishStaticTransform()
{
    // Transform base to stream
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};

    ros::Time transform_ts_ = ros::Time::now(); 
    float rotation[9] = {0};
    auto Q = rotationMatrixToQuaternion(rotation);

    Q = quaternion_optical * Q * quaternion_optical.inverse();

    float3 trans{0,0,0};
    quaternion q1{0,0,0,1};//{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
    publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[DEPTH]);
    ROS_DEBUG_STREAM("calcAndPublishStaticTransform _base_frame_id:" << _base_frame_id << " _frame_id:" <<  _frame_id[DEPTH] );
}

std::string create_graph_resource_name(const std::string &original_name)
{
  std::string fixed_name = original_name;
  std::replace(fixed_name.begin(), fixed_name.end(), '-', '_');
  std::replace(fixed_name.begin(), fixed_name.end(), ' ', '_');
  return fixed_name;
}


void BasePointCloudNode::setBaseTime(double frame_time, bool warn_no_metadata)
{
    //ROS_WARN_COND(warn_no_metadata, "Frame metadata isn't available! ()");
    _ros_time_base = ros::Time::now();
    _camera_time_base = frame_time;
}

void BasePointCloudNode::frameCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    try{
        double frame_time = frame.timestamp;

        // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
        // and the incremental timestamp from the camera.
        // In sync mode the timestamp is based on ROS time
        bool placeholder_false(false);
        if (_thisPtr->_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
        {
            _thisPtr->setBaseTime(frame_time, true);
        }

        ros::Time t = ros::Time(_thisPtr->_ros_time_base.toSec()+ (frame_time -  _thisPtr->_camera_time_base) / 1000);
       
        if(c == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
        {
            if (0 != _thisPtr->_pointcloud_publisher.getNumSubscribers())
            {
                _thisPtr->publishPointCloud(frame, t);
            }
        }else if(c == DepthCamera::FRAME_RAW_FRAME_PROCESSED)
        {
            
            _thisPtr->publishFrame(frame, t);
        }
        else
        {
            ROS_DEBUG("Data not correct!");
        }
       
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
    }
}


void BasePointCloudNode::publishFrame(const Frame &frame, const ros::Time& t)
{
    unsigned int width = 0;
    unsigned int height = 0;
    auto bpp = 2;

    const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);

    uint16_t* ph = (uint16_t *)d->phase();
    width = d->size.width;
    height = d->size.height;
    
    STREAM_TYPE stream = DEPTH;

    auto& image = _images[stream];

    if (_images[stream].size() != cv::Size(width, height))
    {
        image.create(height, width, image.type());
    }

    image.data = (uint8_t*)ph;

    ++(_seq[stream]);
    
    auto& info_publisher = _info_publisher.at(stream);
    auto& image_publisher = _image_publisher.at(stream);
    
    if(0 != info_publisher.getNumSubscribers() ||
       0 != image_publisher.first.getNumSubscribers())
    {

        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), _encoding.at(stream), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = _frame_id.at(stream);
        img->header.stamp = t;
        img->header.seq = _seq[stream];
        
        auto& cam_info = _camera_info.at(stream);
    
        if (cam_info.width != width)
        {
            updateStreamCalibData(DEPTH);
        }

        cam_info.header.stamp = t;
        cam_info.header.seq = _seq[stream];
        
        info_publisher.publish(cam_info);
        
        image_publisher.first.publish(img);
        image_publisher.second->update();
        //ROS_INFO_STREAM("fid: " <<  img->header.seq << ", time: " << std::setprecision (20) << t.toSec());
    }
}

void BasePointCloudNode::publishPointCloud(const Frame &frame, const ros::Time& t)
{
    const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
    
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _frame_id[DEPTH];
    msg_pointcloud.width = d->size();
    msg_pointcloud.height = 1;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");    

    sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
    for (size_t point_idx=0; point_idx < d->points.size(); point_idx++)
    {
        d->points[point_idx];
        
        *iter_x = d->points[point_idx].x;
        *iter_y =  d->points[point_idx].y;
        *iter_z =  d->points[point_idx].z;
        ++iter_x; ++iter_y; ++iter_z;
    }
    _pointcloud_publisher.publish(msg_pointcloud);
}

void BasePointCloudNode::updateStreamCalibData(STREAM_TYPE streamType)
{
    ROS_INFO_STREAM(STREAM_NAME(streamType) << "  width: " << _width[streamType] << ", height: " << _height[streamType] << " frame_id:" << _frame_id[streamType]);   
    _camera_info[streamType].width = _width[streamType];
    _camera_info[streamType].height = _height[streamType];
    _camera_info[streamType].header.frame_id = _frame_id[streamType];
    if(_depthCameraPtr)
    {
        float fx,fy,cx ,cy ,k1 ,k2 ,k3 ,p1 ,p2 ;

        _depthCameraPtr->get("fx",fx);
        _depthCameraPtr->get("fy",fy);
        _depthCameraPtr->get("cx",cx);
        _depthCameraPtr->get("cy",cy);
        _depthCameraPtr->get("k1",k1);
        _depthCameraPtr->get("k2",k2);
        _depthCameraPtr->get("k3",k3);
        _depthCameraPtr->get("p1",p1);
        _depthCameraPtr->get("p2",p2);

        _camera_info[streamType].K.at(0) = fx;
        _camera_info[streamType].K.at(2) = cx;
        _camera_info[streamType].K.at(4) = fy;
        _camera_info[streamType].K.at(5) = cy;
        _camera_info[streamType].K.at(8) = 1;

        _camera_info[streamType].distortion_model = "plumb_bob";

        _camera_info[streamType].D.push_back(k1);
        _camera_info[streamType].D.push_back(k2);
        _camera_info[streamType].D.push_back(k3);
        _camera_info[streamType].D.push_back(p1);
        _camera_info[streamType].D.push_back(p2);
    }

}

