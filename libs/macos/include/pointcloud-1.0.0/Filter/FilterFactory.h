/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_FILTER_FACTORY_H
#define POINTCLOUD_FILTER_FACTORY_H

#include <Common.h>
#include <DepthCamera.h>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */

class FilterDescription
{
  typedef Function<FilterPtr()> _CreateFilter;
protected:
  _CreateFilter _createFilter;
public:
  String name;
  uint frameTypes; // supported frame types (see #DepthCamera::FrameType)
  
  FilterDescription() {}
  
#ifndef SWIG
  FilterDescription(const String &n, uint f, _CreateFilter c): name(n), frameTypes(f), _createFilter(c) {}
#endif
  
  inline bool supports(DepthCamera::FrameType type)
  {
    return frameTypes & (1 << type);
  }
  
  friend class FilterFactory;
};

class POINTCLOUD_EXPORT FilterFactory
{
  
protected:
  Map<String, FilterDescription> _supportedFilters;
  bool _addSupportedFilters(const Vector<FilterDescription> &f);
  
  String _name;
  
public:
  FilterFactory(const String &name): _name(name) {}
  
  inline const String &name() const { return _name; }
  
  inline const Map<String, FilterDescription> &getSupportedFilters() const { return _supportedFilters; }
  
  virtual FilterPtr createFilter(const String &name, DepthCamera::FrameType type);
};

typedef Ptr<FilterFactory> FilterFactoryPtr;

#ifndef SWIG
extern "C" void getFilterFactory(FilterFactoryPtr &filterFactory);
#endif

typedef void (*GetFilterFactory)(FilterFactoryPtr &filterFactory); // Function type to return DepthCameraFactory

/**
 * @}
 */

}


#endif
