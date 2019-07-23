/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_POINT_CLOUD_FRAME_GENERATOR_H
#define POINTCLOUD_POINT_CLOUD_FRAME_GENERATOR_H

#include <FrameGenerator.h>
#include <PointCloudTransform.h>

namespace PointCloud
{
  
class POINTCLOUD_EXPORT PointCloudFrameGenerator: public FrameGenerator
{
protected:
  PointCloudTransformPtr _pointCloudTransform;
  
  virtual bool _onReadConfiguration();
  virtual bool _onWriteConfiguration();
public:
  PointCloudFrameGenerator();
  
  bool setParameters(uint32_t left, uint32_t top, uint32_t width, uint32_t height, 
                     uint32_t rowsToMerge, uint32_t columnsToMerge,uint32_t amp_threhold,float max_range,
                     float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2);
  
  bool generate(const FramePtr &in, FramePtr &out);
  
  virtual ~PointCloudFrameGenerator() {}
};
  
  
}

#endif
