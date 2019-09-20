# libPointCloud SDK User Guide - C++ version

## 1. Connect to DepthCamera

All classes in the PointCoud library are available by doing:

```
#include "CameraSystem.h"
using namespace PointCloud;
```
Now,, in main(), do the following to list all available devices:
```
CameraSystem sys;
DevicePtr     device;
const Vector<DevicePtr> &devices = sys.scan();
bool found = false;
for (auto &d: devices){
    logger(LOG_INFO) <<  " ||| Detected devices: "  << d->id() << std::endl;
    device = d;
    found = true;
}
```
To connect to the first detected depth camera,
```
DepthCameraPtr depthCamera;
depthCamera = sys.connect(device);
 if (!depthCamera) {
    logger(LOG_ERROR) << " ||| Could not load depth camera for device "<< device->id() << std::endl;
    return 1;
}
```
Then , we can start the camera now:
```
if (!depthCamera->isInitialized()) {
    logger(LOG_ERROR) << " ||| Depth camera not initialized for device "<< device->id() << std::endl;
    return 1;
}
// Must register one callback before starting capture 
//depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,rawdataCallback);
depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
    logger(LOG_INFO) <<  " ||| on registerCallback" << std::endl;
});

if(depthCamera->start()){
    logger(LOG_INFO) <<  " ||| start camera pass" << std::endl;
}else{
    logger(LOG_INFO) <<  " ||| start camera fail" << std::endl;  
}
std::cout << "Press any key to quit" << std::endl;
getchar();

// Stop Camera 
depthCamera->stop();
sys.disconnect(depthCamera,true);
return 0;
```

## 2. Getter and setter of Main Parameters

1. Modulation frequency
2. Integration time 
3. FOV
4. FPS 
5. Resolution 

### Modulation frequency getter & setter

```
#Set Modulation frequency
float  mod_freq1 ;
mod_freq1 = 12;
depthCamera->set("mod_freq1",mod_freq1);

#Get Modulation frequency
depthCamera->get("mod_freq1",mod_freq1);
logger(LOG_INFO) << " ||| Modulation frequency :  " << mod_freq1  << std::endl;
```
### Integration time getter & setter
```
#Set integration time
unsigned int  intg_time ;
intg_time = 6;
depthCamera->set("intg_time",intg_time);

#Get integration time
depthCamera->get("intg_time",intg_time);
logger(LOG_INFO) << " ||| INTG_TIME :  " << intg_time  << std::endl;
```
### FOV Getter
```
#get FOV
float angle ;
depthCamera->getFieldOfView(angle);
logger(LOG_INFO) << " ||| FOV :  " << angle  << std::endl;
```
### FPS Getter & Setter
```
#Set FPS
FrameRate r;
r.numerator = 15;
depthCamera->setFrameRate(r);
#Get FPS
if(depthCamera->getFrameRate(r))
    logger(LOG_INFO) << " ||| Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
```
### Resolution Getter & Setter
```
#get framesize/resolution
FrameSize s;
depthCamera->getFrameSize(s)
s.height = 240
s.width = 320
depthCamera->setFrameSize(s)
```
## 3.  Frame call back 
Valid frame types are:
```
DepthCamera::FRAME_RAW_FRAME_UNPROCESSED
DepthCamera::FRAME_RAW_FRAME_PROCESSED
DepthCamera::FRAME_DEPTH_FRAME
DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME
```
Callback function needs of the following signature:
```
void callbackfunction(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c);
```
## 3. Add filter 

```
CameraSystem sys;
FilterPtr fp = sys.createFilter("PointCloud::IIRFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  
fp->set("gain", 0.2f);

if(!fp)
{
logger(LOG_ERROR) << "Failed to get IIRFilter" << std::endl;
return -1;
}else{
  std::cout << " ||| Successfully createFilter IIRFilter " << std::endl;

}

int position = depthCamera->addFilter(fp, DepthCamera::FRAME_RAW_FRAME_PROCESSED);

std::cout << " ||| Successfully add  IIRFilter at  "<< position << std::endl;
    
```
More filter usage please refer to Filter usage documents
