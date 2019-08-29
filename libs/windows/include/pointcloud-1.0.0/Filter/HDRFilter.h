/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_HDR_H
#define POINTCLOUD_HDR_H

#include "Filter.h"

#include <string.h>
#include <deque>

namespace PointCloud
{
  
/**
 * \addtogroup Flt
 * @{
 */
  
class POINTCLOUD_EXPORT HDRFilter: public Filter
{
protected:
   uint _order;

   std::deque<Vector<ByteType>> _ampHistory;
   std::deque<Vector<ByteType>> _phaseHistory;
   std::deque<Vector<ByteType>> _ambHistory;
   std::deque<Vector<ByteType>> _flagsHistory;

   FrameSize _size;
  
   template <typename T>
   bool _filter(const T *in, T *out);
  
   template <typename PhaseT, typename AmpT>
   bool _filter2(const FramePtr &in_p, FramePtr &out_p);

   virtual bool _filter(const FramePtr &in, FramePtr &out);
  
   virtual void _onSet(const FilterParameterPtr &f);
  
public:
   HDRFilter(uint order = 2);
    char *buffer;
    char *processedbuffer;
    
   int newId;
   int getId()
    {
        return newId++;
    }
    
   virtual void reset();
  
   virtual ~HDRFilter() {}
};
/**
 * @}
 */

}
#endif // POINTCLOUD_HDR_H
