/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_DEPTHCAMERALIBRARY_H
#define POINTCLOUD_DEPTHCAMERALIBRARY_H

#include <Common.h>
#include <DepthCameraFactory.h>
#include <Filter/FilterFactory.h>
#include "DownloaderFactory.h"

namespace PointCloud
{

/**
  * \addtogroup CamSys
  * @{
  */

class POINTCLOUD_NO_EXPORT DepthCameraLibraryPrivate;

class POINTCLOUD_EXPORT DepthCameraLibrary
{
protected:
  String _libName;

  Ptr<DepthCameraLibraryPrivate> _libraryPrivate;
  
public:
  DepthCameraLibrary(const String &libName);

  bool load();
  
  inline bool isLoaded();
  
  DepthCameraFactoryPtr getDepthCameraFactory();
  
  FilterFactoryPtr getFilterFactory();
  
  DownloaderFactoryPtr getDownloaderFactory();

  int getABIVersion(); // Get DepthCameraLibrary's ABI version
  
  virtual ~DepthCameraLibrary();
};

typedef Ptr<DepthCameraLibrary> DepthCameraLibraryPtr;

/**
 * @}
 */
}

#endif // POINTCLOUD_DEPTHCAMERALIBRARY_H
