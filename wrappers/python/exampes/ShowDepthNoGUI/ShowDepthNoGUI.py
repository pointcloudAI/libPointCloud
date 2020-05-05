###
# PointCloud Python Sample : ShowDepthNoGUI.
#
# Copyright (c) 2018 PointCloud.AI Inc.
#
# Author : Adam.Han
#
# Functional description:
#    Show simple usage with python to get depth information from DepthEye Camera
# 
# Exit shortcut keys:
#     Input [Enter] Key
###

import sys
import os

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
        self.depthCamera = cameraSystem.connect(devices[0])
        self.data = {}
        if self.depthCamera:
            self.depthCamera.clearAllCallbacks()
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_DEPTH_FRAME, self.processDepthFrame)
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_XYZI_POINT_CLOUD_FRAME, self.processPointCloudFrame)
            if not self.depthCamera.start():
                print(" start fail")
            else:
                print(" start ok")

    def processDepthFrame(self, depthCamera, frame, type):
        #frame = self.depthQueue.get(timeout=0.25)

        if frame is None:
            return

        depthFrame = PointCloud.DepthFrame.typeCast(frame)

        if not depthFrame:
            return

        d = np.array(depthFrame.depth)
        d1 = np.transpose(d.reshape((depthFrame.size.height, depthFrame.size.width)))
        print("point(x:40,y:30)'s distance is %s  meter." %d1[40][30])

    def processPointCloudFrame(self, depthCamera, frame, type):

        if frame is None:
            return

        pointCloudFrame = PointCloud.XYZIPointCloudFrame.typeCast(frame)

        if not pointCloudFrame:
            return

        pcf = np.array(pointCloudFrame, copy=True)

        print("pointCloudFrame.size:  %s" %pointCloudFrame.size())

        ###  point.i is intensity, which come from amplitude
        ###  point.z is distance, which come from depth

        # for index in range(pointCloudFrame.size()):
        #     point = pointCloudFrame.points[index]
        #     print("current point : index %s  [ x : %s , y: %s ,z : %s ,i : %s]" %(index, point.x,point.y,point.z,point.i))
    
    def stop(self):
        if self.depthCamera:
            print("depthCamera getrefcount 1:  " ,sys.getrefcount(self.depthCamera))
            self.depthCamera.stop()
            self.depthCamera.clearAllCallbacks()
            print("depthCamera getrefcount 2: " ,sys.getrefcount(self.depthCamera))
            print(" before   cameraSystem. disconnect")
            cameraSystem.disconnect(self.depthCamera, True)
            print(" after   cameraSystem. disconnect")
            print("depthCamera getrefcount 3: " ,sys.getrefcount(self.depthCamera))
            del self.depthCamera
            print(" after   del self.depthCamera")
            # print("depthCamera getrefcount 4: " ,sys.getrefcount(self.depthCamera))
            
            self.depthCamera = None
            print(" after   self.depthCamera is None")
            # print("depthCamera getrefcount 5: " ,sys.getrefcount(self.depthCamera))
            

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

print(" before del  cameraSystem.")
del cameraSystem
print(" after del  cameraSystem.")
cameraSystem = None


