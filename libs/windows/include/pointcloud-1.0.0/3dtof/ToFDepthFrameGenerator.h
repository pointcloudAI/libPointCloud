/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_TOF_DEPTH_FRAME_GENERATOR_H
#define POINTCLOUD_TOF_DEPTH_FRAME_GENERATOR_H

#include <FrameGenerator.h>
#include <3DToFExports.h>
#include "ToFFrameGenerator.h"

namespace PointCloud
{
  
namespace TOF
{
  
class TOFCAM_EXPORT ToFDepthFrameGenerator: public DepthFrameGenerator
{
  float _amplitudeScalingFactor, _depthScalingFactor;
  
  FramePtr _intermediate;
  
  ToFFrameGeneratorPtr _tofFrameGenerator;

  virtual bool _onReadConfiguration();
  virtual bool _onWriteConfiguration();
  
public:
  ToFDepthFrameGenerator();
  
  virtual bool generate(const FramePtr &in, FramePtr &out);
  
  bool setParameters(float amplitudeScalingFactor, float depthScalingFactor);
  
  virtual bool setProcessedFrameGenerator(FrameGeneratorPtr &p);
  
  virtual ~ToFDepthFrameGenerator() {}
};
  
}
}

#endif
