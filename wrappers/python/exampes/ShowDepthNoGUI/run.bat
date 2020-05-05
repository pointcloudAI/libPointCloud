
echo curent folder : "%~dp0"
echo current path : "%~f0"
set POINTCLOUD_SDK_PATH="%~dp0..\..\..\..\libs\windows"
echo POINTCLOUD_SDK_PATH : %POINTCLOUD_SDK_PATH%
set DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
set LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
set PYTHONPATH="E:\workspace\libPointCloud-master\libs\windows\lib\python3
echo "%POINTCLOUD_SDK_PATH%"
echo "%PYTHONPATH%"
set PATH= %~dp0..\..\..\..\libs\windows\lib;%~dp0..\..\..\..\libs\windows\lib\python3;%PATH%
echo %PATH%

python ShowDepthNoGUI.py
