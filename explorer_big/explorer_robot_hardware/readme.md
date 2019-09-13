# explorer_robot_hardware
## _explorer_ 机器人的主要控制中心 _robot_hardware_
这个类中的代码主要用于控制 _explorer_ 机器人,会发送信息到 _explorer_driver_ 中以实现对机器人(下位机)的控制

数据获取依赖于 [_ros_control_](https://github.com/ros-controls/ros_control/tree/indigo-devel "ros_control-indigo 版本的github网址") 的 kinetic 版本,待迁移至melodic

现版本基于kinetic,注意ros-kinetic-joint-state-controller         
ros-kinetic-joint-state-publisher ros-kinetic-joint-limits-interface 的安装，注意小车与大车协议不一致，最后修改于2019年

### 潘学谦做出的修改

1. 原版本将一些在不同包内的数据分别写在相应的源代码中间,导致如果需要更改往往需要重新编译,而且容易导致此处的修改没有运用于彼处的毛病,现在将所有参数通过 _lauch_ 文件的命令导入到参数服务器上,并通过 _ros_ 的参数服务器机制导入这些全局数据.目的还是减少编译的次数...
2. 修改 _launch_ 文件,使得程序使用我自己写的 _explorer_arm_controller_ 具体见包 _explorer_arm_controller_

## _explorer_ 的主程序( _main_ 函数)

由 _controller_manager_ 实现对 

> explorer_arm_controller 
> explorer_vice_wheel_controller
> explorer_joint_state_controller
> explorer_drive_controller

四个 _controller_ 的加载
并由 _controller_manager_ 实现以上类与 _robot_hardware_ 的数据交换

## 当前协议内容

| ID   | 01              | 02            |
| ---- | --------------- | ------------- |
| 1    | 左轮         | 右轮 |
| 2    | 底盘复位指令1 | 右轮发下去 大车不接收 |
| 3    | 左前副履带角度        | 右前副履带角度 |
| 4    | 左后副履带角度 | 右后副履带角度      |
| 5    | 机械臂底座左右旋转角度       |               |
| 6    | 机械臂大臂整体上下旋转角度 | 小臂上下旋转角度 |
| 7    | 末端执行器(相机)轴向旋转 | 末端执行器(相机)上下摆动 |
| 8    | 爪子开合        | 爪子轴向旋转 |
| 9    | 回传co2传感器      | 请求指令1 |
| 10 | 回传横滚角Roll | 回传俯仰角Pitch/请求指令1 |
| 11    | 机械臂扭动请求指令1 |  |

8+ID+224+01中数据+02中数据

以上所述的*末端执行器*和*爪子*在当前是指三指机械爪和其后端自由度的摄像头

注意对机械臂限位的设置

## _ros_control_ 大致介绍

_ros_control_ 通过  _controller_manager_ 将机器人底层控制需要的代码(相应的 _robotHW_ 的子类)与机器人功能实现分离,提高代码的复用性
通过功能实现和实际硬件控制的分离,大大提高了机器人的可扩展性同时,关闭一个功能可以通过直接修改相应的launch文件实现,功能增加和修改不需要重新编译(然并卵...)
很大程度上实现了 实现接口 --> 实现功能 的最方便转换
使用的时候注意最好不要将功能实现写在这个包中间,将物理状况和与底盘的通信数据传送写在这个包中,所用功能实现写在对应(或者新写的)插件中放在 _explorer_controller_ 文件夹中

向底盘发送数据和更改底盘状态最好写在一起



## 电子协议大致介绍

CAN ID：0x01e0、0x02e0、0x03e0：设定底盘6个电机的速度
与主控协议：
第一个数据包：前四位用于说明发送数据类型（标准帧还是扩展帧、数据帧还是遥控帧（遥控帧用来请求发送数据）），后四位用于说明发送数据长度（应小于等于8）。数值为8
第二、三个数据包：CANID高八位（数值对应上方ID）、CAN ID低八位（数值为224）。（CAN为标准帧16位情况下）
第四—第十一个数据包：具体数据。四到七个数据包为01数据，八到十一个为02数据。

十六位CAN ID：

高八位：协议ID 

低八位：224（0XE0）

（设计目的：方便看出协议ID）

前三位：

0x08  0x01  0xe0  

