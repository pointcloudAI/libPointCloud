/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_BILATERAL_H
#define POINTCLOUD_BILATERAL_H

#include "Filter.h"

#include "DiscreteGaussian.h"


namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */

class BilateralFilter: public Filter
{
protected:
  DiscreteGaussian _discreteGuassian;
  
  FrameSize _size;
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T, typename T2>
  bool _filter(const T *in, const T2 *ref, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  BilateralFilter(float sigma = 0.5);
  virtual ~BilateralFilter() {}
  
  virtual void reset();


};

/**
 * @}
 */

}

#endif // POINTCLOUD_BILATERAL_H
