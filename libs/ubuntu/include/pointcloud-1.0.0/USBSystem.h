/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_USBSYSTEM_H
#define POINTCLOUD_USBSYSTEM_H

#include "Common.h"
#include "Device.h"

namespace PointCloud
{
  
/**
 * \addtogroup IO
 * @{
 */

  
class POINTCLOUD_NO_EXPORT USBSystemPrivate;
  
// This facilitates getting device handle corresponding a particular vid:pid:serial
class POINTCLOUD_EXPORT USBSystem
{
protected:
  Ptr<USBSystemPrivate> _usbPrivate;
  
public:
  USBSystem();
  
  Vector<DevicePtr> getDevices();
  
  bool isInitialized();
  
  inline USBSystemPrivate &getUSBSystemPrivate() { return *_usbPrivate; }
  
  String getDeviceNode(const USBDevice &usbd);
  
  virtual ~USBSystem() {}
};
/**
 * @}
 */

}
#endif // POINTCLOUD_USBSYSTEM_H
