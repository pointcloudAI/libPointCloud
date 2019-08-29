/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_TI_POINTCLOUD_USB_PROGRAMMER_H
#define POINTCLOUD_TI_POINTCLOUD_USB_PROGRAMMER_H

#include "USBIO.h"
#include "GeneralProgrammerBase.h"

namespace PointCloud
{
  
namespace TOF
{

class TOFCAM_EXPORT GeneralUSBProgrammer: public GeneralProgrammerBase
{
public:
  struct RequestParams
  {
    uint8_t readRequestCode, writeRequestCode;
    uint8_t leftShiftBits;
  };
  
  typedef Map<uint, RequestParams> SlaveAddressToRequestParamsMap;
protected:
  USBIOPtr _usbIO;
  bool _isBigEnd;
  SlaveAddressToRequestParamsMap _slaveAddressToRequestParamsMap;

  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length);
  
public:
  GeneralUSBProgrammer(const SlaveAddressToByteMap &map, const SlaveAddressToRequestParamsMap &slaveAddressToRequestParamsMap, USBIOPtr &usbIO, DevicePtr device,bool isBigEnd = false);
  
  virtual bool isInitialized() const;
  
  virtual bool reset();
  
  inline USBIOPtr &getUSBIO() { return _usbIO; }
  
  virtual ~GeneralUSBProgrammer() {}
};

}
}

#endif // POINTCLOUD_TI_VOXELXU_PROGRAMMER_H
