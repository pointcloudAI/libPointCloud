//
//  DepthGesture.cpp
//  Demo_Gesture
//
//  Created by Lucas on 2018/10/31.
//  Copyright © 2018 PointCloud.AI. All rights reserved.
//
#include "Common.h"
#include "Logger.h"
using namespace PointCloud;
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "CameraSystem.h"
#include <sys/wait.h>
#include <iostream>
#include <fstream>
 
using namespace std;

void rawdataCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) 
{

}

int main(int argc,char *argv[]) {
    
    std::cout << "Hello, World!\n";
    
    // logger.setDefaultLogLevel(LOG_DEBUG);
    CameraSystem sys;
    fstream fs("/Users/hanyu/Downloads/i_q.bin",ios::in);
    if(!fs)
      cout<<"open error"<<endl;
    else
      cout<<"open ok"<<endl;
    

    FrameStreamReader r("/Users/hanyu/Desktop/dump_file2.vxl", sys);
    
    if(!r.isStreamGood())
    {
        logger(LOG_ERROR) << "File could not be opened for reading" << std::endl;
        return 1;
    }
    
    std::cout << "Number of data frames in stream = " << r.size() << std::endl;
    return 0;
    

    // DevicePtr     device;
    // const Vector<DevicePtr> &devices = sys.scan();
    // bool found = false;
    // for (auto &d: devices){
    //     logger(LOG_INFO) <<  " ||| Detected devices: "  << d->id() << std::endl;
    //     device = d;
    //     found = true;
    // }
    // if (!found){
    //   logger(LOG_INFO) <<  " ||| No device found "  << std::endl;
    //   return 0;
    // }
    // DepthCameraPtr depthCamera;
    // depthCamera = sys.connect(device);
    // if (!depthCamera) {
    //     logger(LOG_ERROR) << " ||| Could not load depth camera for device "<< device->id() << std::endl;
    //     return 1;
    // }

    //   FrameRate r;
    //   if(depthCamera->getFrameRate(r))
    //     logger(LOG_INFO) << " ||| Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
    //   r.numerator = 15;
    //   depthCamera->setFrameRate(r);
    //   // if(depthCamera->getFrameRate(r))
    //   logger(LOG_INFO) << " ||| Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
      
    //   // FrameSize s;
    //   // if(depthCamera->getFrameSize(s))
    //   // logger(LOG_INFO) << " ||| Frame size :  " << s.width << " * "<< s.height << std::endl;
    //   // ParameterPtr p;
    //   // float angle ;
    //   // depthCamera->getFieldOfView(angle);
    //   // logger(LOG_INFO) << " ||| FOV :  " << angle  << std::endl;
    //   // unsigned int  intg_time ;
    //   // depthCamera->get("intg_time",intg_time);
    //   // logger(LOG_INFO) << " ||| INTG_TIME :  " << intg_time  << std::endl;
     
    //   // intg_time = 6;
    //   // depthCamera->set("intg_time",intg_time);
    //   // depthCamera->get("intg_time",intg_time);
    //   // logger(LOG_INFO) << " ||| INTG_TIME :  " << intg_time  << std::endl;
     

    //   // float  mod_freq1 ;
    //   // depthCamera->get("mod_freq1",mod_freq1);
    //   // logger(LOG_INFO) << " ||| Modulation frequency :  " << mod_freq1  << std::endl;
    //   // mod_freq1 = 12;
    //   // depthCamera->set("mod_freq1",mod_freq1);
    //   // depthCamera->get("mod_freq1",mod_freq1);
    //   // logger(LOG_INFO) << " ||| Modulation frequency :  " << mod_freq1  << std::endl;

    //   // Vector<SupportedVideoMode> videoModes;

    //   // if(depthCamera->getSupportedVideoModes(videoModes))
    //   // {
    //   //   std::cout << "Supported video modes" << std::endl;
        
    //   //   for(auto i = 0; i < videoModes.size(); i++)
    //   //   {
    //   //     std::cout << videoModes[i].frameSize.width << "x" << videoModes[i].frameSize.height << "@" << videoModes[i].getFrameRate() << "fps" << std::endl;
    //   //   } 
    //   // }
    
    // if (!depthCamera->isInitialized()) {
    //     logger(LOG_ERROR) << " ||| Depth camera not initialized for device "<< device->id() << std::endl;
    //     return 1;
    // }
    // std::cout << " ||| Successfully loaded depth camera for device " << std::endl;
    
    // // FilterPtr fp = sys.createFilter("PointCloud::IIRFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  
    // // fp->set("gain", 0.2f);

    // // if(!fp)
    // // {
    // //  logger(LOG_ERROR) << "Failed to get IIRFilter" << std::endl;
    // //  return -1;
    // // }else{
    // //     std::cout << " ||| Successfully createFilter IIRFilter " << std::endl;
    
    // // }
   
    // // int position = depthCamera->addFilter(fp, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    
    // // std::cout << " ||| Successfully add  IIRFilter at  "<< position << std::endl;
    

    // // Must register one callback before starting capture 
    // //depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
    // depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
    //     logger(LOG_INFO) <<  " ||| on registerCallback" << std::endl;
    //     const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);
    //     if(!d)
    //     {
    //       std::cout << "Null frame captured? or not of type ToFRawFrame" << std::endl;
    //       return;
    //     }
        
    //     std::cout << "Capture frame " << d->id << "@" << d->timestamp << std::endl;

    //     if(d->phase()){   
    //         ushort phase_t=((ushort *)d->phase())[2369];
    //       std::cout << " ||| Center Point's phase :  " << phase_t << std::endl;
    //     }

        
    //     // if(d->amplitude())
    //     //   f.write((char *)d->amplitude(), d->amplitudeWordWidth()*d->size.width*d->size.height);
        
    //     // if(d->ambient())
    //     //   f.write((char *)d->ambient(), d->ambientWordWidth()*d->size.width*d->size.height);
        
    //     // if(d->flags())
    //     //   f.write((char *)d->flags(), d->flagsWordWidth()*d->size.width*d->size.height);
        

    // });
    //  depthCamera->registerCallback(DepthCamera::FRAME_DEPTH_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
    //     const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);
        
    //     if(!d)
    //     {
    //       std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
    //       return;
    //     }
    //     float x = d->depth[2430];
    //     //memcpy((char *)depthFrame, (char *)d->depth.data(), sizeof(short) * 80 * 60);
    
    //     // uint16_t depth_t=((uint16_t *)d->depth.data())[2430];
    //     std::cout << " ||| Center Point's depth : " << x << std::endl;
      
    //   });
  
    // if(depthCamera->start()){
    //       logger(LOG_INFO) <<  " ||| start camera pass" << std::endl;
          
    // }else{
    //     logger(LOG_INFO) <<  " ||| start camera fail" << std::endl;  
    // }
       
    // std::cout << "Press any key to quit" << std::endl;
    // getchar();
    // depthCamera->stop();
    // sys.disconnect(depthCamera,true);
    
}


