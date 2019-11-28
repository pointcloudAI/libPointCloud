# 常见问题

## 购买咨询常见问题

* TOF开发板比TOF模组多提供什么？

> 1. 开发板比模组多提供了硬件原理图，您可以参考这个原理修改定制自己的模组
> 2. 开发板可以提供更深入的技术支持

* 是否能提供四相位的原始数据？

> 1. 通过SDK的回调函数提供四相位的原始数据
> 2. 提供源代码例子解析四相位原始数据

* 是否有MIPI信号测试点用于逻辑分析仪分析？

>1. 当前硬件没有提供MIPI信号测试点
>2. 购买开发板后，可以获得原理图，根据原理图的pin脚定义，您可以做一片简单的FPC转接板，在该转接板上可以留出MIPI信号测试点

* MIPI信号是几条Lane？传输速率是多少？
  
>1. TOF芯片支持配置 2lane 或者4lane 输出数据，每条lane的传输速率 960 Mbps/lane
>2. 我们的模组是 4lane mipi 输出，有需要可定制为2Lane输出。

---

## 购买后使用常见问题

* Windows USB驱动安装失败怎么办？
  
>1. 先确认是连接到USB3.0的接口上，接到USB2.0上面是无法安装驱动成功的
>2. 然后确认Type C数据线光滑的一面朝向镜头方向。因为部分机器存在TypeC正反插兼容问题
>3. 最后需要按照Cypress驱动安装指南按照USB驱动
>4. 如果还有问题，请更换一台其他电脑尝试
>5. Windows USB驱动安装请参考 [DephEyeTurbo_USB_Driver_Installation_For_Win10.pdf](https://github.com/pointcloudAI/tools_and_resources/blob/master/windows_viewer/DephEyeTurbo_USB_Driver_Installation_For_Win10%20.pdf) 
>6. 安装驱动过程中，选择设备时 注意应该选【通用串行总线控制器】 ，而不是选【通用串行总线设备】，如果选错了，卸载后重新安装驱动即可。



