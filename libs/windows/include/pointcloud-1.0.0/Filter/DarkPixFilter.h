/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_DARKPIX_H
#define POINTCLOUD_DARKPIX_H

#include "Filter.h"

#include <string.h>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */
  
class POINTCLOUD_EXPORT DarkPixFilter: public Filter
{
protected:
  float _aThrNear, _phThrNear;
  float _aThrFar, _phThrFar;
  float _ambThresh;

  Vector<ByteType> _current;
  
  FrameSize _size;
  
  template <typename T>
  bool _filter(const T *in, T *out);

  template <typename T, typename T2>
  bool _filter2(const T *in, T2 *amp, uint8_t *amb, T *out);

  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
  virtual void _onSet(const FilterParameterPtr &f);
  
public:
  DarkPixFilter(float aThrNear = 1, float phThrNear = 4095, float aThrFar = 20, float phThrFar = 2000, float ambThresh = 1);
  
  virtual void reset();
  
  virtual ~DarkPixFilter() {}
};
/**
 * @}
 */

}
#endif // POINTCLOUD_DARKPIX_H
