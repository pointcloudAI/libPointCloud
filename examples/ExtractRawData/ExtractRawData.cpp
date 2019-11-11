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
#if defined(LINUX) 
#include <sys/wait.h>
#endif
#include <iostream>
#include <fstream>
#include <complex>

//#include "opencv2/opencv.hpp"
//#include "opencv/cv.h"
 
using namespace std;

Vector<double> _sineTable, _cosineTable;
int phase_count = 4;
int width = 640 ;
int height = 480;
uint32_t phase_length = 460800; // => 640*480*1.5 = 460800
#define M_PI 3.14159265358979323846

void rawABdataCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType frameType) 
{
    // <-----  Importance notice   ------>
    // Register FRAME_RAW_FRAME_UNPROCESSED only on registerCallback function to get the correct raw A-B data.
    
    const RawDataFrame *rawDataFrame = dynamic_cast<const RawDataFrame *>(&frame);
    if(!rawDataFrame)
    {
      std::cout << "Null frame captured? or not of type RawDataFrame" << std::endl;
      return;
    }
    
    std::cout << " frame : " <<  rawDataFrame->id  << " size: " << rawDataFrame->data.size() << std::endl;
    if ( rawDataFrame->data.size() != 1843596 ){
       std::cout << " Data size not correct ! Register FRAME_RAW_FRAME_UNPROCESSED only on registerCallback function to get the correct raw A-B data."<< std::endl;
    }
    
    // For id usage:
    // If you using two modulation frequency ， 
    // below id field will show the modulation frequency number for reference
    // For data size ：
    // default value should be 1843596 for 640*480 and 4 phases , every pixel using 1.5 byte
    // 640*480 * 1.5 * 4 = 1843200 , another 396 byte additional information at the end


    uint8_t *in = (uint8_t *)rawDataFrame->data.data();
   
    auto index = 0,idx2=0;
    uint16_t amp,pha;
    float phase;
    int k = 0;
   
    Complex c;
    float factor = 255.0/1024;
    // std::cout << " factor : " << factor << std::endl;
   // cv::Mat tof_amplitude=cv::Mat::zeros(height,width,CV_8UC1);
    //unsigned char *pamp = tof_amplitude.data; 

    char filename[50] = { 0 };
    sprintf(filename, "RawDataAMP_%llu.bmp", rawDataFrame->timestamp);
                 /*
    for(int j = 0;j< width*height;j+=2)
    {
        // calculate two pixel at the same time  
        double ii = 0, iq = 0;
        double ii1 = 0, iq1 = 0;
        
        // if(j== width*240 + 320)
        //    std::cout << " pixel[320,240] a-b : ";
        
        for(int i=0;i<phase_count;i++)
        {
            int16_t a_b = in[i*phase_length + index];
            int16_t a_b1 = in[i*phase_length + index+1];
            a_b = (a_b << 4) | (0xf & in[i*phase_length + index+2]);
            a_b1 = (a_b1 << 4) | ((0xf0 & in[i*phase_length + index+2])>>4);
            
            a_b = (((int16_t)(a_b << 4) )>>4);
            a_b1 = (((int16_t)(a_b1 << 4) )>>4);
            
            // if(j== width*240 + 320)
            //    std::cout << "\t" << a_b  ;
        
            ii += a_b*_cosineTable[i];
            iq += a_b*_sineTable[i];
            ii1 += a_b1*_cosineTable[i];
            iq1 += a_b1*_sineTable[i];
            
        }
        
        c.real(ii);
        c.imag(iq);
        amp  = std::abs(c);
        phase = std::arg(c) ;

        *(pamp + k) = int(amp * factor) ;
        k++;
   
        
        c.real(ii1);
        c.imag(iq1);
        amp  = std::abs(c);
        phase = std::arg(c) ;
        *(pamp + k) = int(amp * factor) ;
        k++;
     
        
        idx2 +=4;
        index += 3;
    }
    cv::imwrite(filename,tof_amplitude);

	*/
    std::cout << "Write amplitude to file with name " << filename << std::endl;
	

}

int main(int argc,char *argv[]) {
    
    std::cout << "Hello, World!\n";

    _sineTable.resize(phase_count);
    _cosineTable.resize(phase_count);
	//logger.setDefaultLogLevel(LOG_DEBUG);

     for(auto i = 0; i < phase_count; i++)
    {
        _sineTable[i] = sin(2*M_PI/phase_count*i);
        _cosineTable[i] = cos(2*M_PI/phase_count*i);
        std::cout << "table @" << i << " = " << _sineTable[i] << ", " << _cosineTable[i] << std::endl;
    }
    
    
    CameraSystem sys;
   
    DevicePtr     device;
    const Vector<DevicePtr> &devices = sys.scan();
    bool found = false;
    for (auto &d: devices){
		std::cout <<  " ||| Detected devices: "  << d->id() << std::endl;
        device = d;
        found = true;
    }
    if (!found){
		std::cout <<  " ||| No device found "  << std::endl;
      return 0;
    }
    DepthCameraPtr depthCamera;
    depthCamera = sys.connect(device);
    if (!depthCamera) {
		std::cout << " ||| Could not load depth camera for device "<< device->id() << std::endl;
        return 1;
    }

    FrameRate r;
    if(depthCamera->getFrameRate(r))
		std::cout << " ||| Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
    r.numerator = 3;
    depthCamera->setFrameRate(r);
    if(depthCamera->getFrameRate(r))
		std::cout << " ||| Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
      
    FrameSize s;
    if(depthCamera->getFrameSize(s))
		std::cout << " ||| Frame size :  " << s.width << " * "<< s.height << std::endl;
    int centerPointIndex = (s.height/2 ) * s.width + s.width/2;
    
    if (!depthCamera->isInitialized()) {
		std::cout << " ||| Depth camera not initialized for device "<< device->id() << std::endl;
        return 1;
    }
    std::cout << " ||| Successfully loaded depth camera for device " << std::endl;
    
    
    // Must register one callback before starting capture 
    depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED,rawABdataCallback);
    // depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
    //     logger(LOG_INFO) <<  " ||| on registerCallback" << std::endl;
    
    //     // This part is as a comparison of results for checking ,
    //     // You can get correct gray scale image on this function

    //     const ToFRawFrame *tofRawFrame = dynamic_cast<const ToFRawFrame *>(&frame);
    //     if(!tofRawFrame)
    //     {
    //       std::cout << "Null frame captured? or not of type ToFRawFrame" << std::endl;
    //       return;
    //     }
        
    //     std::cout << "Capture frame " << tofRawFrame->id << "@" << tofRawFrame->timestamp << std::endl;

    //     if(tofRawFrame->phase()){   
    //         ushort phase_t=((ushort *)tofRawFrame->phase())[centerPointIndex];
    //       std::cout << " ||| Center Point's phase :  " << phase_t << std::endl;
    //     }

    //     int width= s.width;
    //     int height = s.height;
    //     int k = 0;
    //     int maxAmp = 0;
    //     float factor = 255.0/1024;
    //     std::cout << " factor : " << factor << std::endl;
    //     cv::Mat tof_amplitude=cv::Mat::zeros(height,width,CV_8UC1);
    //     char filename[50] = { 0 };
    //     sprintf(filename, "RawData_%llu.bmp", tofRawFrame->timestamp);

    //     if(tofRawFrame->amplitude()){
          
    //       unsigned char *pamp = tof_amplitude.data;
    //       for(int i=0;i<height;i++)
    //       {
    //           for(int j=0;j<width;j++)
    //           {
    //               // ushort phase_t=((ushort *)tofRawFrame->phase())[k];
    //               ushort amp_t=((ushort *)tofRawFrame->amplitude())[k];
    //               *(pamp + k) = int(amp_t * factor) ;
    //               if (amp_t > maxAmp){
    //                 maxAmp = amp_t;
    //               }
                  
    //               k++;
    //           }
    //       }
          
    //     }
    //     // double thre=cv::threshold(tof_amplitude, tof_amplitude, 0, 255, cv::THRESH_OTSU);
    //     // if(thre>10)
    //     // {
    //     //     thre-=5;
    //     // }
    //     std::cout << " maxAmp : " << maxAmp << std::endl;

    //     cv::imwrite(filename,tof_amplitude);
        
       

    // });
  
    if(depthCamera->start()){
          logger(LOG_INFO) <<  " ||| start camera pass" << std::endl;
          
    }else{
        logger(LOG_INFO) <<  " ||| start camera fail" << std::endl;  
    }
       
    std::cout << "Press any key to quit" << std::endl;
    getchar();
    depthCamera->stop();
    sys.disconnect(depthCamera,true);
    
}


