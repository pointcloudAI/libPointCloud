# Below are the examples for your reference:
* DepthAscii
* DepthMeasure
* ExtractRawData
* ParseVXL

## DepthAscii

This C++ example shows how to capture image data from a camera while does not depend on any GUI toolkit, therefore it only displays textual information and low-resolution ascii-art of the captured images.
这个例子通过Ascii来展示深度数据，可以在不依赖GUI库的情况下看到一个可视化的图像。同时也展示了如何获得深度相机的常见参数

## DepthMeasure
This C++ example shows how to  query information about the camera and get the distance information.
这个例子展示了如何获得常见的深度相机参数，并且展示了如果通过回调函数得到 某个点的距离

## ExtractRawData

This C++ example shows how to capture the raw A-B data， you can use it to calculate the phase and amplitude and
do your calibration in same cases.

这个例子展示了如何获得原始的四相位数据，这在有些情况下是有用的。例如您修改了光源的调制频率。
您可以重新计算得到phase和amplitude ，可以重新做自己的校准。

## ParseVXL

This C++ example shows how to extract frame information from VXL files.

这个例子展示了从保存的VXL 文件中解析 出 深度信息和点云信息

