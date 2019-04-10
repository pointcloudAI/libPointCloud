/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_IIR_H
#define POINTCLOUD_IIR_H

#include "Filter.h"

#include <string.h>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */
  
class POINTCLOUD_EXPORT IIRFilter: public Filter
{
protected:
  float _gain;
  Vector<ByteType> _current;
  
  FrameSize _size;
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
  virtual void _onSet(const FilterParameterPtr &f);
  
public:
  IIRFilter(float gain = 0.5);
  
  virtual void reset();
  
  virtual ~IIRFilter() {}
};
/**
 * @}
 */

}
#endif // POINTCLOUD_IIR_H
