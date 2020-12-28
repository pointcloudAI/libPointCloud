###
# PointCloud Python Sample : ExtractRawData.
#
# Copyright (c) 2018 PointCloud.AI Inc.
#
# Author : Adam.Han
#
# Functional description:
#    Show simple usage with python to get Raw data（4 phase data） information from DepthEye Camera
# 
# Exit shortcut keys:
#     Input [Enter] Key
###

import sys
import os
import math

def getDefaultSdkPath():
    path = ''
        
    if sys.platform == 'win32':
        import win32api
        path = win32api.GetLongPathName(os.path.dirname(os.path.realpath(sys.argv[0]))+"\\..\\..\\..\\..")
        print('path',path)
        path = path + os.sep + "libs"+os.sep +"windows"
      
    elif sys.platform == 'darwin':
        path = sys.path[0]+ "/../../../.." + os.sep + "libs"+os.sep +"macos"
    else :
        path = sys.path[0]+ "/../../../.." + os.sep + "libs"+os.sep +"ubuntu"
    
    print('path',path)
    if os.path.isdir(path):
        return  path   
    else:
        print('Failed to get default PointCloud SDK path')
        return None
sdkPath = getDefaultSdkPath()

libPath = sdkPath + os.sep + "lib"
pythonPath = libPath + os.sep + "python3"
print("pythonPath ",pythonPath)
sys.path.append(libPath) 
sys.path.append(pythonPath) 

print("***** SDK path:",sdkPath," libPath:",libPath)

import PointCloud
import numpy as np
import sys

def createWindow():
    global window
    if window == None:
        window = MainWindow(cameraSystem)
    return

class MainWindow():
   
    def __init__(self, cameraSystem):
        print("MainWindow init")
        PointCloud.cvar.logger.setDefaultLogLevel(4)
        self.depthCamera = cameraSystem.connect(devices[0])
        self.data = {}
        self.width = 640
        self.height = 480
        self.phaseCount = 4
        self.pixelCount = self.width*self.height
        self.factor = 255.0/1024;
        self.phase_length = 460800; # => 640*480*1.5 = 460800
        self.M_PI = 3.14159265358979323846

        self._sineTable = []
        self._cosineTable = []
   
        for  i in range (0,self.phaseCount):
            self._sineTable.append(math.sin(2*self.M_PI/self.phaseCount*i))
            self._cosineTable.append(math.cos(2*self.M_PI/self.phaseCount*i))
            print("table @", i,self._sineTable[i] , self._cosineTable[i] )
        

        if self.depthCamera:
            self.depthCamera.clearAllCallbacks()
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_RAW_FRAME_UNPROCESSED, self.processRawFrame)
            if not self.depthCamera.start():
                print(" start fail")
            else:
                print(" start ok")

    def processRawFrame(self, depthCamera, frame, type):
        # <-----  Importance notice   ------>
        #Register FRAME_RAW_FRAME_UNPROCESSED only on registerCallback function to get the correct raw A-B data.
    
   
    
        if frame is None:
          return
        print("on new processRawFrame ")

        
        rawFrame = PointCloud.RawDataFrame.typeCast(frame)
        if not rawFrame:
            print("not a rawFrame  ")
            return
        # For id usage:
        # If you using two modulation frequency ， 
        # below id field will show the modulation frequency number for reference
        # For data size ：
        # default value should be 1843596 for 640*480 and 4 phases , every pixel using 1.5 byte
        # 640*480 * 1.5 * 4 = 1843200 , another 396 byte additional information at the end

        if ( rawFrame.data.size() != 1843596 ):
            print(" Data size not correct ! Register FRAME_RAW_FRAME_UNPROCESSED only on registerCallback function to get the correct raw A-B data.")
            return 
        
        print("is a raw frame and frame size is  %d." %rawFrame.data.size())
        rawData=np.array(rawFrame.data,dtype='uint8')
        print("rawData",len(rawData))
        index = 0
        # for j in range(0,self.pixelCount,2):
        #     # print("i",i)
        #     # calculate two pixel at the same time  
        #     for i in range(0,self.phaseCount):
        #         a_b = rawData[i*self.phase_length + index];
        #         a_b1 = rawData[i*self.phase_length + index+1];
        #         a_b = (a_b << 4) | (0xf & rawData[i*self.phase_length + index+2]);
        #         a_b1 = (a_b1 << 4) | ((0xf0 & rawData[i*self.phase_length + index+2])>>4);
                
        #         # a_b = ((a_b << 4) >>4);
        #         # a_b1 = ((a_b1 << 4) >>4);
                
        #         if(j== self.width*240 + 320):
        #             print("a_b",a_b  );
        #     index += 3;
        # 
        # Process raw data on python will cause error " Dropping a frame because of slow forward pipeline"
            
        print("========process one raw frame end=========")
        
      
    
    def stop(self):
        if self.depthCamera:
            self.depthCamera.stop()
            self.depthCamera.clearAllCallbacks()
            cameraSystem.disconnect(self.depthCamera, True)
            del self.depthCamera
            self.depthCamera = None
            
cameraSystem = PointCloud.CameraSystem()

devices = cameraSystem.scan()

if len(devices) == 1:
    print(" Find one device.")
    window = MainWindow(cameraSystem)
    key = input("Input enter key to quit.")
    print(" Quit now.")
    window.stop()
else:
    print(" No device found.")

del cameraSystem
cameraSystem = None


