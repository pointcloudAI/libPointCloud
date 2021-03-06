/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_TEMPORAL_MEDIAN_FILTER_H
#define POINTCLOUD_TEMPORAL_MEDIAN_FILTER_H

#include "Filter.h"

#include <string.h>

namespace PointCloud
{
/**
 * \addtogroup Flt
 * @{
 */
class TemporalMedianFilter: public Filter
{
protected:
  float _deadband;
  uint _order;
  
  FrameSize _size;
  List<Vector<ByteType>> _history;
  Vector<ByteType> _current;
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T>
  void _getMedian(IndexType offset, T &value);
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  TemporalMedianFilter(uint order = 3, float deadband = 0.05);
  virtual ~TemporalMedianFilter() {}
  
  virtual void reset();
};

/**
 * @}
 */

}
#endif // POINTCLOUD_TEMPORAL_MEDIAN_FILTER_H
