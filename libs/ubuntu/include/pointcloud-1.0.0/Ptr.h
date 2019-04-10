/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_PTR_H
#define POINTCLOUD_PTR_H

#include <memory>

namespace PointCloud
{
  
/**
 * \addtogroup Util
 * @{
 */


template <typename T>
void deleter(T *data)
{
  delete data;
}

  
/**
 * NOTE: This is a simple way to ensure that matching new/delete are used.  
 * WARNING: 
 * 1. DO NOT CREATE the pointer "data" in some other library (so/dll) context and pass it to Ptr<> created in another library (so/dll). 
 *    Always. create Ptr<> and allocate "data" in the same so/dll.
 * 2. DO NOT USE THIS CLASS FOR array data, that is, allocated with new[]
 */
template <typename T>
class Ptr: public std::shared_ptr<T>
{
public:
  Ptr(T *data): std::shared_ptr<T>(data, deleter<T>) {}
  
  template <typename _Deleter>
  Ptr(T *data, _Deleter d): std::shared_ptr<T>(data, d) {}
  
  Ptr(): std::shared_ptr<T>() {}
  
  Ptr(const std::shared_ptr<T> &p): std::shared_ptr<T>(p) {}
  
#ifdef SWIG
  T *operator ->() { return this->std::shared_ptr<T>::operator->(); }
  
  const T *operator ->() const { return this->std::shared_ptr<T>::operator->(); }
#endif
  
  virtual ~Ptr() {}
};

/**
 * @}
 */


}


#endif // POINTCLOUD_PTR_H