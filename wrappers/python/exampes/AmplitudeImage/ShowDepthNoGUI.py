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
import cv2
import numpy as np

# height: 480
# width: 640
user_camera_height = 480
user_camera_width = 640
user_image = np.ones((user_camera_height, user_camera_width), dtype=np.uint8)  # random.random()方法后面不能加数据类型
user_depth = np.ones((user_camera_height, user_camera_width), dtype=np.float)  # random.random()方法后面不能加数据类型


def user_height_width_to_pixel(height, width):
    return width + height * user_camera_width

def getDefaultSdkPath():
    path = ''
        
    if sys.platform == 'win32':
      import win32api
      path = win32api.GetLongPathName(os.path.dirname(os.path.realpath(sys.argv[0]))+"\\..\\..\\..\\..")
      print('path',path)
      path = path + os.sep + "libs"+os.sep +"windows"
      print('path',path)
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
# import numpy as np



def insertToPathVariable(variable, value):
  if variable not in os.environ:
    os.environ[variable] = value
  elif value not in os.environ.get(variable).split(os.pathsep):
    os.environ[variable] = value + os.pathsep + os.environ[variable]


cameraSystem = None
devices = None

def createWindow():
    global window
    if window == None:
        window = MainWindow(cameraSystem)
    return

class MainWindow():
    def __init__(self, temp_cameraSystem, temp_device):
        self.pic_num = 0
        self.raw_data_func_callback_flag = 0
        self.depth_data_func_callback_flag = 0
        global cameraSystem
        global devices
        cameraSystem = temp_cameraSystem
        devices = temp_device
        print("MainWindow init")
        PointCloud.cvar.logger.setDefaultLogLevel(4)
        self.depthCamera = cameraSystem.connect(devices[0])
        self.data = {}
        if self.depthCamera:
            self.depthCamera.clearAllCallbacks()
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_RAW_FRAME_PROCESSED, self.processToFFrame)
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_DEPTH_FRAME, self.processDepthFrame)
            self.depthCamera.registerCallback(PointCloud.DepthCamera.FRAME_XYZI_POINT_CLOUD_FRAME, self.processPointCloudFrame)
            if not self.depthCamera.start():
                print(" start fail")
            else:
                print(" start ok")
    def processToFFrame(self, depthCamera, frame, type):

        if frame is None:
            return
        print("on new processToFFrame ")

        tofFrame = PointCloud.ToF1608Frame.typeCast(frame)
        if not tofFrame:
            return

        size = tofFrame.size
        self.frameSize = [size.width, size.height]
        self.phaseArray = np.array(tofFrame._phase, copy=True)
        self.amplitude = np.array(tofFrame._amplitude, copy=True)

        if self.user_get_raw_data_ok_flag() == 0:
            print("raw data get")
            for height in range(0, user_camera_height):
                for width in range(0, user_camera_width):
                    user_image[height, width] = self.amplitude[user_height_width_to_pixel(height, width)] / (4096/256)
                    # print(self.phaseArray[user_height_width_to_pixel(height, width)])
            self.user_write_raw_data_ok_flag(1)
        else:
            pass
        # cv2.imshow('img', user_image)
        # cv2.imwrite('test_' + self.pic_num.__str__() + '.png', user_image)  # 写入图片
        # self.pic_num += 1
        # print("num: " + self.pic_num.__str__())
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # amb = np.array(tofFrame._ambient)
        # print("point(x:40,y:30)'s amplitude is %s  ." + amb[40][30])



    def processDepthFrame(self, depthCamera, frame, type):
        #frame = self.depthQueue.get(timeout=0.25)


        if frame is None:
            return

        depthFrame = PointCloud.DepthFrame.typeCast(frame)

        if not depthFrame:
            return

        d = np.array(depthFrame.depth)
        #d1 = np.transpose(d.reshape((depthFrame.size.height, depthFrame.size.width)))
        d1 = d.reshape((depthFrame.size.height, depthFrame.size.width))
        if self.user_get_depth_data_ok_flag() == 0:
            print("depth data get")
            for height in range(0, user_camera_height):
                for width in range(0, user_camera_width):
                    user_depth[height, width] = float(d1[height][width])
                    pass
            self.user_write_depth_data_ok_flag(1)
            pass
        else:
            pass
        # print("point(x:320,y:240)'s distance is %s  meter." %d1[500][600])

    def processPointCloudFrame(self, depthCamera, frame, type):

        light = 0

        if frame is None:
            return

        pointCloudFrame = PointCloud.XYZIPointCloudFrame.typeCast(frame)

        if not pointCloudFrame:
            return

        pcf = np.array(pointCloudFrame, copy=True)
        # for height in range(0, user_camera_height):
        #     for width in range(0, user_camera_width):
        #         point = pointCloudFrame.points[user_height_width_to_pixel(height, width)]
        #         if "nan" in point.i.__str__():
        #             light = 0
        #         else:
        #             light = float(point.i.__str__()) * 1000
        #         if light > 255:
        #             light = 255
                # user_image[height, width] = light

        # Set integration time
        # r0, intg_time = depthCamera.setu("intg_time", 14)
        # print(" ||| Set integration time result: %s" % r0)
        #
        # r0, intg_time = depthCamera.getu("intg_time")
        # print(" ||| integration time : %s %% " % intg_time)
        # cv2.imshow('img', user_image)
        # cv2.waitKey(0)
        # print("pointCloudFrame.size:  %s" %pointCloudFrame.size())

        ###  point.i is intensity, which come from amplitude
        ###  point.z is distance, which come from depth

        # for index in range(pointCloudFrame.size()):
        #     point = pointCloudFrame.points[index]
        #     print("current point : index %s  [ x : %s , y: %s ,z : %s ,i : %s]" %(index, point.x,point.y,point.z,point.i))

    def user_get_raw_data_ok_flag(self):
        return self.raw_data_func_callback_flag

    def user_write_raw_data_ok_flag(self, flag):
        self.raw_data_func_callback_flag = flag

    def user_get_depth_data_ok_flag(self):
        return self.depth_data_func_callback_flag

    def user_write_depth_data_ok_flag(self, flag):
        self.depth_data_func_callback_flag = flag

    def user_get_height_pixle(self):
        return user_camera_height

    def user_get_width_pixle(self):
        return user_camera_width

    def stop(self):
        if self.depthCamera:
            print("depthCamera getrefcount 1:  " ,sys.getrefcount(self.depthCamera))
            self.depthCamera.stop()
            self.depthCamera.clearAllCallbacks()
            print("depthCamera getrefcount 2: " ,sys.getrefcount(self.depthCamera))
            print(" before   cameraSystem. disconnect")
            self.cameraSystem.disconnect(self.depthCamera, True)
            print(" after   cameraSystem. disconnect")
            print("depthCamera getrefcount 3: " ,sys.getrefcount(self.depthCamera))
            del self.depthCamera
            print(" after   del self.depthCamera")
            # print("depthCamera getrefcount 4: " ,sys.getrefcount(self.depthCamera))
            
            self.depthCamera = None
            print(" after   self.depthCamera is None")
            # print("depthCamera getrefcount 5: " ,sys.getrefcount(self.depthCamera))



