/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef USB_BULK_STREAMER_H
#define USB_BULK_STREAMER_H

#include <Streamer.h>
#include "USBIO.h"

namespace PointCloud
{
/**
 * \addtogroup IO
 * @{
 */


class POINTCLOUD_EXPORT USBBulkStreamer : public Streamer
{
protected:
  class USBBulkStreamerPrivate;
  Ptr<USBBulkStreamerPrivate> _usbBulkStreamerPrivate;
  
  virtual bool _start();
  virtual bool _capture(RawDataFramePtr &p);
  virtual bool _stop();
  
public:
  USBBulkStreamer(USBIOPtr &usbIO, DevicePtr device, uint8_t endpoint);
  
  virtual ~USBBulkStreamer();
  
  virtual bool isInitialized();
  
  virtual bool setBufferSize(size_t bufferSize);
  
  // All these three are dummy
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes);
  virtual bool getCurrentVideoMode(VideoMode &videoMode);
  virtual bool setVideoMode(const VideoMode &videoMode);

};
/**
 * @}
 */

}

#endif // USB_BULK_STREAMER_H
