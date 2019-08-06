/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_UVCXU_H
#define POINTCLOUD_UVCXU_H

#include "UVC.h"

namespace PointCloud
{
/**
 * \addtogroup IO
 * @{
 */

  
class POINTCLOUD_EXPORT UVCXU : public UVC
{
protected:
  int _xuID;

  class POINTCLOUD_NO_EXPORT UVCXUPrivate;
  Ptr<UVCXUPrivate> _uvcXUPrivate;

public:
  UVCXU(DevicePtr usb, int xuID);

  bool isInitialized();
  
  virtual ~UVCXU();
  
  bool setControl(int controlnumber, int size, uint8_t *value);
  bool getControl(int controlnumber, int size, uint8_t *value);
};

typedef Ptr<UVCXU> UVCXUPtr;
/**
 * @}
 */

}
#endif // UVCXU_H
