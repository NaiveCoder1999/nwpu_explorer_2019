# urdf命名

#### urdf包导出时的命名：explorer_arm_description

命名的开头不能是大写字母，中间除了使用下划线其他的符号可能非法

这个类中的代码主要用于控制 _explorer_ 机器人,会发送信息到 _explorer_driver_ 中以实现对机器人(下位机)的控制

所有的link和joint都必须设置limits限位，否则moveit_setup_assistant在导入时报错，建议在SolidWorks中设置好

如果限位出现小的差错可以直接编辑urdf文件进行修改。

## 当前urdf命名参考

| joint关节       | 说明          |
| --------------- | ------------- |
| base_link | 确定方位 |
| joint_front_back | 指向整车前，确定方位和控制车的俯仰和旋转 |
| left_up_wheel_base_joint | 四个轮子关节 |
| right_up_wheel_base_joint |  |
| left_down_wheel_base_joint |       |
| right_down_wheel_base_joint |               |
| robot_left_right | 指示整车的左右 |
| left_up_fin_base_joint | 四个副履带关节 |
| right_up_fin_base_joint |  |
| left_down_fin_base_joint |  |
| arm1_bearing_joint | 底部左右旋转 |
| arm2_arm1_joint | 大臂上下 |
| arm3_arm2_joint | 小臂上下 |
| pt1_arm_joint | 摄像头轴向旋转 |
| pt2_pt1_joint | 摄像头上下摆动 |
| rotate_joint | 爪子轴向旋转 |
| finger1_joint |  |
| finger2_joint |  |
| finger3_joint |  |
| gripper_joint | 爪子开合 |
