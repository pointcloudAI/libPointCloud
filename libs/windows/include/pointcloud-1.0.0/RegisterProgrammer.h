/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */


#ifndef POINTCLOUD_REGISTER_PROGRAMMER_H
#define POINTCLOUD_REGISTER_PROGRAMMER_H

#include "Common.h"
#include <stdint.h>

namespace PointCloud
{
/**
 * \addtogroup IO
 * @{
 */

  
class POINTCLOUD_EXPORT Parameter;
  
class POINTCLOUD_EXPORT RegisterProgrammer
{
public:
  virtual bool isInitialized() const = 0;
  
  // NOTE: Make these thread-safe when implementing
  virtual bool setValue(const Parameter &param, uint32_t value, bool writeOnly = false) = 0;
  virtual bool getValue(const Parameter &param, uint32_t &value) const = 0;
  
  virtual bool readRegister(uint32_t address, uint32_t &value) const = 0;
  virtual bool writeRegister(uint32_t address, uint32_t value) = 0;
    //pointcloud.ai support 16bit register address
  virtual bool readRegister(uint16_t slave_address,uint16_t register_address, uint32_t &value,uint8_t length) const = 0;
  virtual bool writeRegister(uint16_t slave_address, uint16_t register_address, uint32_t &value,uint8_t length) = 0;
  virtual bool reset() = 0;
  
  virtual ~RegisterProgrammer() {}
};
  
/**
 * @}
 */


}

#endif
