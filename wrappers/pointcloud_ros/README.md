# ROS Wrapper for PointCloud.AI&reg; DepthEye&trade; Devices
These are packages for using PointCloud.AI depth cameras with ROS.

## Installation Instructions

The following instructions support ROS Indigo, on **Ubuntu 14.04**, and ROS Kinetic, on **Ubuntu 16.04**.

PS. This instructions had been confirmed that work well on "ubuntu 18.04" with ROS Melodic.
For ROS Melodic, please exebute below commands:
```bash
sudo apt-get install ros-melodic-catkin
sudo apt install ros-melodic-ddynamic-reconfigure
```

### Step 1: Install the latest PointCloud.AI  PointCloudSDK 1.0
- #### Downlowd from [Ubuntu Package](https://github.com/pointcloudAI/libPointCloud/tree/master/libs/ubuntu) to your disk.

#### OR

- #### Download from [MacOS Package](https://github.com/pointcloudAI/libPointCloud/tree/master/libs/ubuntu) to your disk.

Add PointCloud_DIR to ~/.bashrc to make CMake FindPackake can find PointCloud :
```bash
vim ~/.bashrc
export PointCloud_DIR='/home/your_ubuntu_login_account/dev/libPointCloud/libs/ubuntu/lib/cmake/PointCloud'
```
BTW. Please remember to amend above PointCloud_DIR to real saved folder of PointCloud SDK.

### Step 2: Install the ROS distribution
- #### Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), on Ubuntu 16.04

### Step 3: Install PointCloud ROS from Sources

- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```
- Clone the latest Poincloud ROS from [here](https://github.com/pointcloudAI/libPointCloud/tree/master/wrappers/) into 'catkin_ws/src/'

The final foler structure is as follows:
```bash
~/catkin_ws/src/pointcloud_camera
```
And then execute below commands under ~/catkin_ws/src/:
```bash
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS:

```bash
roslaunch pointcloud_camera pointcloud_camera.launch
```
OR
```bash
roslaunch pointcloud_camera demo_pointcloud.launch
```
Then open rviz to watch the pointcloud:
<p align="center"><img src="https://raw.githubusercontent.com/pointcloudAI/libPointCloud/master/doc/image/ros_imx556_sample.gif" /></p>

Other stream resolutions and frame rates can optionally be provided as parameters to the 'pointcloud_camera.launch' file.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with camera attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/depth/camera_info
- /camera/depth/image_raw
- /camera/depth/points

### Launch parameters
- **rosbag_filename**: Will publish topics from rosbag file.
- **depth_width**, **depth_height**, **depth_fps**:  If the specified combination of parameters is not available by the device, the stream will not be published. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined). Note: for gyro accel and pose, only _fps option is meaningful.
- **enable_depth**: Choose whether to enable a specified stream or not. Default is true. 
- **base_frame_id**: defines the frame_id all static transformations refers to.
- **All the rest of the frame_ids can be found in the template launch file: [nodelet.launch.xml](./pointcloud_camera/launch/includes/nodelet.launch.xml)**

## Known Issues
* This ROS node does not currently support [ROS Lunar Loggerhead](http://wiki.ros.org/lunar).
* This ROS node does not currently work with [ROS 2](https://github.com/ros2/ros2/wiki).
* This ROS node currently does not provide the unit-tests which ensure the proper operation of the camera.  Future versions of the node will provide ROS compatible unit-tests.
