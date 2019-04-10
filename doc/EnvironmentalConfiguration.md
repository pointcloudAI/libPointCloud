# Environmental configuration for libPointCloud SDK
## Ubuntu 
### Path  Setting
Modify .bashrc to set environment variables：
```
# vim ~/.bashrc
```
add below source code to the end of bashrc file：
```
export POINTCLOUD_SDK_PATH="/yourspath/libs/ubuntu"
export DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
export PYTHONPATH=$POINTCLOUD_SDK_PATH/lib/python2.7
```
We need to make above changes come into effect：
```
# source ~/.bashrc
```
Finally, echo the constant to verify ：
```
# echo $POINTCLOUD_SDK_PATH
```
### USB Driver Setting 

```
sudo cp ./config/72-CalculusCDK.rules /etc/udev/rules.d/
sudo cp ./config/60-SonyCDK.rules /etc/udev/rules.d/

sudo chmod a+x /etc/udev/rules.d/72-CalculusCDK.rules
sudo chmod a+x /etc/udev/rules.d/60-SonyCDK.rules.rules

sudo udevadm control --reload
```