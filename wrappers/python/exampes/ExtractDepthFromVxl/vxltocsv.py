###
# PointCloud Python Sample : ShowDepthNoGUI.
#
# Copyright (c) 2018 PointCloud.AI Inc.
#
# Author : Luke.Han
#
# Functional description:
#    Show simple usage with python to extract depth information from vxl file which saved by PointCloudTool
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
      print('path',path)
    elif sys.platform == 'darwin':
      path = sys.path[0] + os.sep + "PointCloudSDK"+os.sep +"macos"

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
import os

def vxltocsv(filename):
    print(" on vxl to csv ")
    """Converts VXL file to csv file"""
    camsys = PointCloud.CameraSystem()
    r = PointCloud.FrameStreamReader(filename, camsys)
    bool1, cols = r.getStreamParamu("frameWidth")
    bool2, rows = r.getStreamParamu("frameHeight")
    if not bool1 or not bool2:
        print("Cannot read the stream")
    if not r.isStreamGood():
        print("Stream is not good: " + filename)
    numFrames = r.size()
    print(" numFrames:",numFrames)
    mAmplitudes = np.zeros(( rows, cols), dtype='float')
    mPhase = np.zeros(( rows, cols), dtype='float')
    pointX = 118
    pointY = 142
    outFileName = os.path.splitext(filename)[0] + '.csv' 
    if outFileName:
        print("outFileName: ",outFileName)
        with open(outFileName, 'w') as f:
            for i in (range(numFrames)):
                if not r.readNext():
                    print("Failed to read frame %d" %i)
                    break
                # Extract depth information
                depthFrame = PointCloud.DepthFrame.typeCast(r.frames[PointCloud.DepthCamera.FRAME_DEPTH_FRAME])
                mDepths = np.array(depthFrame.depth, copy=True).reshape((rows, cols))
                depth = mDepths[pointX][pointY]
               
                # Extract amplitude and phase information
                tofFrame = PointCloud.ToF1608Frame.typeCast(r.frames[PointCloud.DepthCamera.FRAME_RAW_FRAME_PROCESSED])
                mAmplitudes = np.array(tofFrame._amplitude, copy=True).reshape((rows, cols))
                mPhase = np.array(tofFrame._phase, copy=True).reshape((rows, cols))
                amp = mAmplitudes[pointX][pointY]
                phase = mPhase[pointX][pointY]
                # print 
                print("tofFrame.id is %s amp : %s  phase : %s  depth : %.4f" %(tofFrame.id,amp,phase,depth) )
                content  = "%5d,%5d,%.4f"%(amp,phase,depth)
                f.write(content+"\n")
            r.close()
        f.close()

    
if __name__ == "__main__":
    vxltocsv("test.vxl")

