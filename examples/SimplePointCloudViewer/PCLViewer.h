/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_PCLVIEWER_H
#define POINTCLOUD_PCLVIEWER_H

#include <DepthCamera.h>
#include <boost/shared_ptr.hpp>

/// Forward declaration of PCL related classes
namespace pcl
{
namespace visualization
{
class PCLVisualizer;

template <typename T>
class PointCloudColorHandlerGenericField;
}

template <typename T>
class PointCloud;

struct PointXYZI;
struct PointXYZ;

class Grabber;
}

namespace PointCloud
{

class PCLViewer
{
protected:
  DepthCameraPtr _depthCamera;
  
  Mutex _cloudUpdateMutex;
  
  Ptr<pcl::visualization::PCLVisualizer> _viewer;
  //Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> _handler;
  Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>> _handler;
    
  bool _stopLoop = false;
  
  Ptr<pcl::Grabber> _grabber; // This will link to our PointCloud::PCLGrabber
  
  //boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> _cloud;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> _cloud;
    
  //void _cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZI> &cloud);
  void _cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    
  void _renderLoop();
  
  std::thread _renderThread;
  
public:
  PCLViewer();
  
  void setDepthCamera(PointCloud::DepthCameraPtr depthCamera);
  void removeDepthCamera();
  
  inline Ptr<pcl::Grabber> getGrabber() { return _grabber; }
  
  void start();
  bool isRunning();
  void stop();
  void loop(){ _renderLoop();};
  bool viewerStopped();
  
  virtual ~PCLViewer() 
  {
    if(isRunning()) stop();
    
    if(_renderThread.joinable()) _renderThread.join();
  }
};


}
#endif // PCLVIEWER_H
