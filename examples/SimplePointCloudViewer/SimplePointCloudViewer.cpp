/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#include <CameraSystem.h>
#include <Common.h>

#include <thread>

#include "PCLViewer.h"
 
int main ()
{
  PointCloud::logger.setDefaultLogLevel(PointCloud::LOG_INFO);
  
  PointCloud::CameraSystem sys;
  PointCloud::DepthCameraPtr depthCamera;
  
  const PointCloud::Vector<PointCloud::DevicePtr> &devices = sys.scan();
  
  if(devices.size() > 0)
    depthCamera = sys.connect(devices[0]); // Connect to first available device
  else
  {
    std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
    return -1;
  }
  
  if(!depthCamera)
  {
    std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
    return -1;
  }
    
   PointCloud::PCLViewer v;
  
   v.setDepthCamera(depthCamera);
  
   v.start();
  
   v.loop();
  while(v.isRunning())
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  return 0;
}
