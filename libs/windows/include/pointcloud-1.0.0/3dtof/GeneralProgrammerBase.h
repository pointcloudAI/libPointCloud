/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_TI_POINTCLOUD_PROGRAMMER_BASE_H
#define POINTCLOUD_TI_POINTCLOUD_PROGRAMMER_BASE_H

#include "RegisterProgrammer.h"
#include "Device.h"

#include "3DToFExports.h"

namespace PointCloud
{
  
namespace TOF
{

class TOFCAM_EXPORT GeneralProgrammerBase: public RegisterProgrammer
{
public:
  typedef Map<uint, uint> SlaveAddressToByteMap;
  
protected:
  DevicePtr _device;
  
  mutable Mutex _mutex;
  
  SlaveAddressToByteMap _slaveAddressToByteMap;
  
  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const = 0;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length) = 0;
    //pointcloud.ai dynamic data length tranaction feature
  uint16_t getByteLength(uint16_t slave_address) const;
public:
  GeneralProgrammerBase(const SlaveAddressToByteMap &map, DevicePtr device);
  
  virtual bool readRegister(uint32_t address, uint32_t &value) const;
  virtual bool writeRegister(uint32_t address, uint32_t value);
  //pointcloud.ai for 16bit register_address support
 virtual bool readRegister(uint16_t slave_address,uint16_t register_address, uint32_t &value,uint8_t length) const;
 virtual bool writeRegister(uint16_t slave_address, uint16_t register_address, uint32_t &value,uint8_t length);
  // getValue() and setValue() are internally called readRegister() and writeRegister() which are thread-safe. So, no mutex locking in getValue() and setValue()
  virtual bool getValue(const Parameter &param, uint32_t &value) const;
  virtual bool setValue(const Parameter &param, uint32_t value, bool writeOnly = false);
  
  virtual ~GeneralProgrammerBase() {}
};

}
}

#endif // POINTCLOUD_TI_POINTCLOUD_PROGRAMMER_BASE_H
