/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#include "PCLGrabber.h"

namespace PointCloud
{
  
PCLGrabber::PCLGrabber(DepthCamera &depthCamera): _depthCamera(depthCamera), _pointCloudBuffer(2)
{
  _pointCloudSignal = createSignal<PointCloudCallBack>();
  _pointCloudFrameSignal = createSignal<PointCloudFrameCallBack>();
  _depthImageSignal = createSignal<DepthImageCallBack>();
  _rawImageSignal = createSignal<RawImageCallBack>();
  
  _depthCamera.registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, std::bind(&PCLGrabber::_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  _depthCamera.registerCallback(DepthCamera::FRAME_DEPTH_FRAME, std::bind(&PCLGrabber::_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  _depthCamera.registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED, std::bind(&PCLGrabber::_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  _depthCamera.registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED, std::bind(&PCLGrabber::_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void PCLGrabber::_callback(DepthCamera &depthCamera, const Frame &frame, DepthCamera::FrameType type)
{
  if(type == DepthCamera::FRAME_RAW_FRAME_UNPROCESSED || type == DepthCamera::FRAME_RAW_FRAME_PROCESSED)
  {
    if(_rawImageSignal->num_slots() > 0)
    {
      const RawFrame *f = dynamic_cast<const RawFrame *>(&frame);
      
      if(f)
        (*_rawImageSignal)(*f, type);
      else
        logger(LOG_ERROR) << "PCLGrabber: Callback type claims a raw frame, but obtained frame not raw?" << std::endl;
    }
  }
  else if(type == DepthCamera::FRAME_DEPTH_FRAME)
  {
    if(_depthImageSignal->num_slots() > 0)
    {
      const DepthFrame *f = dynamic_cast<const DepthFrame *>(&frame);
      
      if(f)
        (*_depthImageSignal)(*f);
      else
        logger(LOG_ERROR) << "PCLGrabber: Callback type claims a depth frame, but obtained frame not a depth frame?" << std::endl;
    }
  }
  else if(type == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
  {
    if(_pointCloudFrameSignal->num_slots() > 0)
    {
      const PointCloudFrame *f = dynamic_cast<const PointCloudFrame *>(&frame);
      
      if(f)
      {
        (*_pointCloudFrameSignal)(*f);
      }
    }
    
    if(_pointCloudSignal->num_slots() > 0)
    {
     // const XYZIPointCloudFrame *f = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
        const XYZIPointCloudFrame *f = dynamic_cast<const XYZIPointCloudFrame *>(&frame);

      if(f)
      {
        auto pointCloud = _pointCloudBuffer.get();
        
        if(!*pointCloud)
        {
         // (*pointCloud) = Ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>());
            (*pointCloud) = Ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
          (*pointCloud)->sensor_origin_.setZero ();
          (*pointCloud)->sensor_orientation_.w () = 1.0f;
          (*pointCloud)->sensor_orientation_.x () = 0.0f;
          (*pointCloud)->sensor_orientation_.y () = 0.0f;
       //   (*pointCloud)->sensor_orientation_.z () = 0.0f;
        }
        
        (*pointCloud)->points.resize(f->points.size());
        
        auto index = 0;
        for(auto &p: f->points)
        {
          auto &q = (*pointCloud)->points[index];
          q.x = p.x*1000;
          q.y = p.y*1000;
          q.z = p.z*1000;
          //q.intensity = p.i*256;
          index++;
        }
        
        (*_pointCloudSignal)(**pointCloud);
      }
      else
        logger(LOG_ERROR) << "PCLGrabber: Callback type claims a point cloud frame, but obtained frame is not?" << std::endl;
    }
  }
  else
    logger(LOG_ERROR) << "PCLGrabber: Do not know how to handle callback frame type = " << type << std::endl;
}

  
}
