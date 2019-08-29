/*
 * PointCloud Lib component.
 *
 * Copyright (c)  PointCloud.AI Inc.
 */

#ifndef POINTCLOUD_FLYPIX_H
#define POINTCLOUD_FLYPIX_H

#include "Filter.h"

#define _MATH_DEFINES
#include <math.h>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */

class POINTCLOUD_EXPORT FlypixFilter: public Filter 
{
protected:
  float _thr;
  
  FrameSize _size;
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  FlypixFilter(float thr = 500);
  virtual ~FlypixFilter() {}
  
  virtual void reset();
};


/**
 * @}
 */

}
#endif // POINTCLOUD_FLYPIX_H
