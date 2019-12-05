/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_DATA_PACKET_H
#define POINTCLOUD_DATA_PACKET_H

#include "Common.h"
#include "SerializedObject.h"

#include "Logger.h"

namespace PointCloud
{
  
struct POINTCLOUD_EXPORT DataPacket
{
  char magic[6]; // 
  
  uint8_t type; // PacketType
  
  uint32_t size;
  
  SerializedObject object;
  
  DataPacket() { strcpy(magic, "PT.AI"); }

  bool readHeader(SerializedObject &in);
  bool readHeader(InputStream &in);
  
  bool read(SerializedObject &in);
  bool read(InputStream &in);
  
  bool write(SerializedObject &out);
  bool write(OutputStream &out);
  
  inline bool verifyMagic() { return magic[0] == 'P' && magic[1] == 'T' && magic[2] == '.' && magic[3] == 'A' && magic[4] == 'I'; }
};

  
  
}

#endif
