SDK configuration files are of three types -

1. Main configuration file
2. Camera profile configuration file
3. DML file

All the configuration files are with an extension of ".conf". The format of a configuration file is same as the INI file format, with notable exceptions being

Comments begin with '#' and not ';'
Double-quoted values are not supported. However, spaces in values are supported without double quotes.

The conf file directory varies according to the system platform:
```
libPointCloud/libs/macos/share/pointcloud-1.0.0/conf
libPointCloud/libs/ubuntu/share/pointcloud-1.0.0/conf
libPointCloud/libs/windows/share/pointcloud-1.0.0/conf
```
## DML file
DML file which follows XML syntax, is a way by which all parameters valid for a depth camera are specified.

## Main Configuration File
This consists of top level information for starting up a board such as DML file name, firmware file name (if any), etc. The name of this file is expected to exactly match the name assigned by the C++ class which handles this board. For instance, SonyCDK board has "SonyCDKCamera" as the assigned name in its C++ class, and its main configuration file is named as "SonyCDKCamera.conf". We provide a typical configuration here for reference.

```
[core]
dml = SONY556IMX.dml
camera_profiles = SonyCDKCameraNormal.conf
default_profile = 130

[observed_params]
tsensor = refresh:5
tillum = refresh:5
```

Here, the section 'core' consists of important information about

* Camera profiles supported by this depth camera. This is via the key 'camera_profiles'.
* Default camera profile is specified via ID of the required camera profile.
* DML file name via 'dml' key.

The main configuration may consist of other keys and sections too which are board specific.

The 'observed_params' section is primarily for the benefit for PointCloudTool, which uses this information to periodically monitor some of the key parameters and checks for their validity. The format of this section is as follows.

Key name is to match exactly the name of the parameter to be monitor.
Value is a semi-colon separated list of settings for monitoring. Each setting is a colon separated key-value pair. Supported settings are as follows. Note that all these settings are optional.
'refresh' with value being number of seconds. This is used to set the refresh rate for the parameter. Default value is voxel-viewer dependent.
'valid' with value being two hyphen separated numbers. This represents valid range of numbers. Typically this is used to notify user when parameter value falls outside this range. If not specified, all values of the parameter are treated to be valid.

## Camera Profile Configuration File

A camera profile defines various calibrations and parameter values to be used for running a depth camera. This is a handy of choosing various modes of operation for a depth camera. For instance, a camera profile could be defined for short range of operation of a depth camera, while another one for long range. Similarly, indoor or outdoor, etc.

A camera profile is active and usable only when its file name is given in 'camera_profiles' key of 'core' section in the main configuration file.

The format of this flexible to support variety of possible profiles. A typical configuration is presented below.


```
[global]
id = 130
name = Normal

[defining_params]
#make sure to send this register at first after power-on = inck frequency[mhz]
unambiguous_range = 4.96 #7.45
intg_time =  100 #100	
mod_freq1 = 60
sub_rseq_lnum =  40 #40 #3 

[params]
rngchken = 0
dutof_mode = 0 #4
hmax = 1800 #1900 #1388 #1700 #2304
micr_lnum = 10000 #10000 #2500 
dpth_su_num = 30
binning_mode = 0 #3
phases_num = 8

dlycnt =  5
anadlycnt = 14 #40 
dlycntsf = 0 # 10

[calib]
fy = 478 
fx = 478 
cx = 319 
cy = 239 
k1 = -0.0
k2 = -0.0
k3 = 0.00
p1 = 0.00
p2 = 0.00
calib_disable = 14 #78
actual_distance = 0.7
tsensor_calib = 39 #60
tillum_calib = 0
phase_corr_2 = 0
phase_corr_1 = -131 #69 #-194
disable_offset_corr = 0 #0
phasecorrection = file:SonyCDKCamera70cm8phase60mhzPhaseOffset.bin 

disable_temp_corr = 0
coeff_illum = 0
coeff_sensor = 5
```

If you want to know more about the settings of this file, please contact us by mail dev@pointcloud.ai .