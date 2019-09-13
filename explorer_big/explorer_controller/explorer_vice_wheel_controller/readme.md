# *explorer_vice_wheel_controller*

## 概述

这个包用于对副履带的控制,模仿自机械臂的controller节点,用于添加副履带的软限位

### 注意

除了软限位外,注意电子添加硬限位.否则可能出现倒转或者转过头的情况.

更改时注意修改robot_hardware节点的关于副履带的pos的handle,本版本代码发送副履带的位姿,发送弧度还是角度自行与电子商量.

### 控制机理

这个控制器仅仅完成位置接受和速度控制两个功能，具体通过封装在 _joint_controller.h_ 文件中

如果出现当前无法实现的关节，可以重载上述文件中的 _joint_ 类实现


## 控制数据来源

来自于参数服务器 */explorer_vice_wheel_controller* 的数据

参数设置于 *$(find explorer_robot_hardware)/config/explorer_vice_wheel_controller.yaml* 



