/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_SMOOTH_H
#define POINTCLOUD_SMOOTH_H

#include "Filter.h"
#include "DiscreteGaussian.h"

#define _MATH_DEFINES
#include <math.h>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */

class SmoothFilter: public Filter 
{
protected:
  DiscreteGaussian _discreteGaussian;
  
  FrameSize _size;
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  SmoothFilter(float sigma = 0.5);
  virtual ~SmoothFilter() {}
  
  virtual void reset();
};


/**
 * @}
 */

}
#endif // POINTCLOUD_SMOOTH_H
