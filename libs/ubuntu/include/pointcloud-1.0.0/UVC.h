/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_UVC_H
#define POINTCLOUD_UVC_H

#include "Device.h"

namespace PointCloud
{
  
/**
 * \addtogroup IO
 * @{
 */


class POINTCLOUD_EXPORT UVCRawData
{
public:
  Ptr<ByteType> data;
  std::size_t size;
};

  
class POINTCLOUD_NO_EXPORT UVCPrivate;
  
class POINTCLOUD_EXPORT UVC
{
protected:
  DevicePtr _usb;
  
  Ptr<UVCPrivate> _uvcPrivate;
  
public:
  UVC(DevicePtr usb);
  
  virtual bool isInitialized();
  
  inline UVCPrivate &getUVCPrivate() { return *_uvcPrivate; }
  
  bool read(uint8_t *buffer, std::size_t size);

  virtual ~UVC();
};
/**
 * @}
 */

}
#endif // UVC_H
