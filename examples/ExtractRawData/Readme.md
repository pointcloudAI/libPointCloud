## Supported modulation frequency:
The unit is Mhz and the available values are integers as below :
```
4,8,16,20,30,35,36,40,50,60,70,80,90,100
```


## Modulation frequency easy setting :

For easy setting ,please modify the conf file which named SonyCDKCameraStandard.conf in below path:
```
libPointCloud/libs/xxx/share/pointcloud-1.0.0/conf
```

You can add the frequency you need. 
For "measure_mode=1" , means using both mod_freq1 only.
For "measure_mode=2" , means using both mod_freq2 only.
For "measure_mode=3" , means using both frequency.
```
[defining_params]
mod_freq1 = 100
mod_freq2 = 20
measure_mode = 3
```

Please noted that if the frequency had been change.

The original calibration data will be invalid, and the calculated distance will not be available and need to be calibrated again.

## Frame Structure and data format of RAW12

![Frame Structure](https://github.com/pointcloudAI/libPointCloud/blob/master/examples/ExtractRawData/Frame_Structure.jpg)

![Data Format of RAW12](https://github.com/pointcloudAI/libPointCloud/blob/master/examples/ExtractRawData/Data%20Format%20RAW12.jpg)

