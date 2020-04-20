/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */


#include "CameraSystem.h"
#include "Timer.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>
//#include "ToFSonyCamera.h"
//#include "USBBulkPhasesStreamer.h"
#include "framestream.h"
//#include <ToFSonyDepthFrameGenerator.h>
//#include <ToFSonyFrameGenerator.h>
// Also include GLFW to allow for graphical display
//#define ONE_PHASE_TEST 1

using namespace PointCloud;

#define MAX_FRAME_WITDH 640
#define MAX_FRAME_HEIGHT 480
#define SPEED_OF_LIGHT 299792458.0
int smin = 0;
int smax = 0xfff;//0x7ff;
uint8_t Rainbow[3][65536] = {0};

Mutex _dataAccessMutex;
ConditionVariable _dataAvailableCondition;
FrameSize frameSize;
RegionOfInterest roi;
uint binning_mod = 0;
uint scale_binning = 1;
std::vector<uint8_t> color_depth;
std::vector<uint16_t> amplitude_data,phase_data;
std::vector<IntensityPoint> xyz_data;
Map<uint,std::vector<uint16_t> > map_temp_phase;
char rawFrameQueue[42496 * 2];
char XYZpointCloutQueue[4800 * sizeof(float) * 4];
PointCloud::PointCloudFramePtr ptFramePtr ;



PointCloud::CameraSystem      cameraSys;
PointCloud::DepthCameraPtr depthCamera;

PointCloud::DevicePtr       device;
#define TMP_FILE "../examples/ParseVXL/test.vxl"
ThreadPtr _captureThread = 0;
uint mode = 0;

int main(int argc, char *argv[])
{
   
    
    logger.setDefaultLogLevel(LOG_DEBUG);
  
    const Vector<DevicePtr> &devices = cameraSys.scan();
    
    logger(LOG_ERROR) <<  "@@@ Load  VXL File:" << TMP_FILE << std::endl;
    PointCloud::FrameStreamReaderPtr  frameStream = new PointCloud::FrameStreamReader(TMP_FILE, cameraSys);
    if(!frameStream->isStreamGood())
        return -1;
    
    uint numFrames = frameStream->size();
    printf("Input VXL is a valid file. Total frames size is %d. \r\n" , numFrames);
    for(int i=0;i<numFrames;i++)
    {
        if(!frameStream->readNext())
        {
            printf("Failed to read frame %d. \r\n",i);
            break;
        }
        RawDataFramePtr rawFrame = std::dynamic_pointer_cast<RawDataFrame>(frameStream->frames[DepthCamera::FRAME_RAW_FRAME_UNPROCESSED]);
        
        uint index = 240*640+320;
        
        size_t raw_len = rawFrame->data.size();
        uint16_t* rawdata = (uint16_t*) rawFrame->data.data();
        std::cout << " index:"<< i << " phase:" << rawdata[2*index+1] << " amplitude:" << rawdata[2*index] << " tsensor:" <<  (int)rawFrame->data[raw_len - 2] << " tillum:" << (int)rawFrame->data[raw_len - 1] << std::endl;
        Ptr<XYZIPointCloudFrame> xyzFrame = std::dynamic_pointer_cast<XYZIPointCloudFrame>(frameStream->frames[DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME]);
        IntensityPoint point = xyzFrame->points[index];
        std::cout << " index:" << i   << " x:" <<point.x << " y:" <<point.y << " z:" <<point.z << " i:" <<point.i  << std::endl ;
    }
    // return true;
    
   

  return 0;
}
