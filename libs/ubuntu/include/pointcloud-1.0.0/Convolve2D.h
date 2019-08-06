/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */


#include <Common.h>

namespace PointCloud
{
  
class POINTCLOUD_EXPORT Convolve2D
{
protected:
  Vector<float> _coefficients;
  
  SizeType _rows, _columns;
  
public:
  Convolve2D(const Vector<float> &coefficients, SizeType rows, SizeType columns);
  
  bool convolve(const Vector<float> &in, SizeType rows, SizeType columns, Vector<float> &out);
  
};
  
  
}