# libPointCloud SDK User Guide - Python version

## 1. Connect to DepthCamera

We use SWIG to generate Python bindings for the PointCloud library.  All classes in the PointCoud library are available by doing:

```
import PointCloud
```
Now, we can list all available devices:
```
import PointCloud
cameraSystem = PointCloud.CameraSystem()
devices = cameraSystem.scan()

for d in devices:
  print d.id()
```

To connect to the first detected depth camera,
```
if len(devices) == 0:
  print "No availale device"
  sys.exit(1)

depthCamera = cameraSystem.connect(devices[0])

```
Then , we can start the camera now:
```
if depthCamera:
    depthCamera.clearAllCallbacks()
    depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_DEPTH_FRAME, processDepthFrame)
    print 'DepthCamera ID = ', depthCamera.id()
    if not depthCamera.start():
        print("start fail")
    else: 
        print("\n start ok")
        print("chipset : %s " %depthCamera.chipset() )

depthCamera.stop()
cameraSystem.disconnect(depthCamera, True)
del cameraSystem
sys.exit(1)
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
if depthCamera.chipset() == CHIPSET_CALCULUS:
    r0 = depthCamera.setf("mod_freq1",24)
elif (depthCamera.chipset() == CHIPSET_SONY556):
    r0 = depthCamera.seti("mod_freq1",20)

print(" ||| Set frequency result: %s " %r0)


#Get Modulation frequency
if depthCamera.chipset() == CHIPSET_CALCULUS:
    r0,mod_freq = depthCamera.getf("mod_freq1")
elif (depthCamera.chipset() == CHIPSET_SONY556):
    r0,mod_freq = depthCamera.geti("mod_freq1")

print(" ||| Modulation frequency : %s  MHz" %mod_freq) 
```
### Integration time getter & setter
```
#Set integration time
r0,intg_time = depthCamera.setu("intg_time",6)  
print(" ||| Set integration time result: %s" %r0) 

#Get integration time
r0,intg_time = depthCamera.getu("intg_time")  
print(" ||| integration time : %s %% " %intg_time) 
```
### FOV Getter
```
#get FOV
r0,fov = depthCamera.getFieldOfView()
print(" ||| FOV : %f ° " %float(fov*100)) 
```
### FPS Getter & Setter
```
#get FPS
r0,rate = depthCamera.getFrameRate()
print(" ||| FPS : %s @FPS "%int(rate.getFrameRate()) ) 

#Set FPS
rate.numerator = 15;
r0 = depthCamera.setFrameRate(rate)
print(" ||| set FPS result : %s "%r0) 

#get FPS once more 
r0,rate = depthCamera.getFrameRate()
print(" ||| New FPS : %s @FPS "%int(rate.getFrameRate()) ) 
```
### Resolution Getter 
```
#get framesize/resolution
r0,frameSize = depthCamera.getFrameSize()
print(" ||| resolution: %d x %d" %(frameSize.width,frameSize.height))
```
## 3. Add filter 


```
## Init filter first
sys = PointCloud.CameraSystem()
filter = sys.createFilter("PointCloud::IIRFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
if filter is None:
   return None

## Create filter for usring depth camera:
    
filterIndex = depthCamera.addFilter(filter, DepthCamera::FRAME_RAW_FRAME_PROCESSED);

print(" ||| Successfully add  IIRFilter at  %d " %filterIndex)

depthCamera.removeFilter(filterIndex, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    
```
More filter usage please refer to Filter usage documents
