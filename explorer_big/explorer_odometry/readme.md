# explorer_odometry

本节点通过发送id为10,前一位为0,后一位float为1的数据包发送陀螺仪的回传请求,回传的Row,Pitch数据包的id仍然是9,然后调用ROS的里程计接口做姿态融合,由于没有里程计,初始速度,速度增量的值均为零,调用tf函数从欧拉角计算四元数,并且发布pose到odom显示



参考:<http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup/Odom>