//
//  DepthAscii.cpp
//  DepthAscii
//
//  Created by Lucas on 2018/10/31.
//  Copyright © 2018 PointCloud.AI. All rights reserved.
//
 
#include <iomanip>
#include <fstream>
#include <ostream>
#include "CameraSystem.h"
#include <thread>
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable
#include <queue>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
using namespace PointCloud;
using namespace std;

CameraSystem depthEyeSys;
int col_factor = 1;
int row_factor = 2;

int printOutFrameInfo(const ToFRawFrame *tofRawFrame,int col_factor,int row_factor){
	 
    FrameSize _size = tofRawFrame->size;

    char buffer[(_size.width/2)*(_size.height/(row_factor-1)+1)];
    char * out = buffer;
	  const short one_meter = static_cast<short>(1.0f);
    short coverage[_size.width/(col_factor-1)+1] ;

  
   	int k=0;
    for(int y=0; y<_size.height; ++y)
    {
        for(int x=0; x<_size.width; ++x)
        {
            // short depth = phaseFrame[y*80+x]/100;
            // std::cout << "x%col_factor : " << x%col_factor  << std::endl;
            if(x%col_factor == 1){
                 ushort depth=((ushort *)tofRawFrame->phase())[k]/100;
                if(depth > 0 && depth < 12)
                {
                	 coverage[x/col_factor] += static_cast<short>(depth);
                }else
                	 coverage[x/col_factor] = 0;
            }
            k++;
        }
         
        if(y%row_factor == 1){
	        for(short & c : coverage)
	        {
              
               // std::cout << "coverage index: " << c << std::endl;
	            *out++ =  " M@#&$%*o!;."[c/row_factor];
	            c = 0;
	        }
        	*out++ = '\n';
    	  }
    }
    *out++ = 0;
    printf("\n%s", buffer);
	return 0;
}


void rawdataCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) 
{

	const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);

	if(!d)
	{
		std::cout << "Null frame captured? or not of type ToFRawFrame"<< col_factor<< " ,"<<row_factor << std::endl;
		return;
	}
	std::cout << " factor"<< col_factor<< " ,"<<row_factor << std::endl;
    
	printOutFrameInfo(d,col_factor,row_factor);

}


int main(int argc, char const *argv[])
{
	
	DevicePtr     device;
  const Vector<DevicePtr> &devices = depthEyeSys.scan();
  bool found = false;
  for (auto &d: devices){
      std::cout <<"||| Detected devices: "  << d->id() << std::endl;
      device = d;
      found = true;
  }
  if (!found){
      std::cout <<"||| No device found "  << std::endl;
      return 1;
  }
  DepthCameraPtr depthCamera;
  depthCamera = depthEyeSys.connect(device);
  if (!depthCamera) {
  	  std::cout <<"||| Could not load depth camera for device "<< device->id() << std::endl;
      return 1;
  }

	if (!depthCamera->isInitialized()) {
        std::cout <<"||| Depth camera not initialized for device " << std::endl;
        return 1;
  }
  FrameSize s;
  if(depthCamera->getFrameSize(s))
  logger(LOG_INFO) << " ||| Frame size :  " << s.width << " * "<< s.height << std::endl;
  if (s.width >= 320){
    col_factor = (s.width/128) + 1;
    row_factor = s.height/48;
  }
  std::cout << " ||| Successfully loaded depth camera for device " << std::endl;

	depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
	
	if(depthCamera->start()){
          logger(LOG_INFO) <<  " ||| start camera pass" << std::endl;
          
    }else{
        logger(LOG_INFO) <<  " ||| start camera fail" << std::endl;  
    }
	
	std::cout << "Press any key to quit" << std::endl;
	getchar();
  std::cout << "Stoping System" << std::endl;
  depthCamera->stop();
	depthEyeSys.disconnect(depthCamera,true);
	return 0;
}

