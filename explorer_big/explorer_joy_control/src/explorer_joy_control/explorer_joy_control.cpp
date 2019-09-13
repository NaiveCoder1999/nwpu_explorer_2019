#include "explorer_joy_control.h"
#include <sstream>
using namespace std;

ExplorerTeleop::ExplorerTeleop() : ph_("~"),

    basic_mode_button(joy_msg->R1),
    // 速度切换按钮，可切换三挡
    full_speed_button(joy_msg->button3), mid_speed_button(joy_msg->button2), slow_speed_button(joy_msg->button1),
    speed_switch_button(joy_msg->R2), 
    //speed_front_back(joy_msg->up_down), speed_left_right(joy_msg->left_right),
    // 副履带简易控制指令
    //front_vice_wheel_up_down(joy_msg->up_down), back_vice_wheel_up(joy_msg->button1),back_vice_wheel_down(joy_msg->button3),
    left_front_vice_wheel_up_down(joy_msg->up_down),
    left_back_vice_wheel_up_down(joy_msg->left_right),
    right_front_vice_wheel_up(joy_msg->button1),
    right_front_vice_wheel_down(joy_msg->button3),
    right_back_vice_wheel_up(joy_msg->button2),
    right_back_vice_wheel_down(joy_msg->button4),
    //right_back_vice_wheel_up_down(joy_msg->right_axes_up_down),
    //right_front_vice_wheel_up_down(joy_msg->right_axes_up_down),

    // 副履带平行指令
    /*front_vice_wheel_parallel(joy_msg->button2),  back_vice_wheel_parallel(joy_msg->button4),
    vice_wheel_mode_button(joy_msg->L2),*/

    // 用于重新开启底盘
    //relive(joy_msg->button4),
    // 车辆前进后退指令
    linear_front_back(joy_msg->left_axes_up_down),
    //左右旋转，因更换轮子，左右平移已废弃
    angular_left_right(joy_msg->left_axes_left_right),

    // 机械臂相关
    //机械臂moveit解算与控制，待实现
    arm_moveit_up_down(joy_msg->up_down),
    arm_moveit_left_right(joy_msg->left_right),
    //arm_moveit_back(joy_msg->right_axes_up_down),
    arm_moveit_forward(joy_msg->left_axes_up_down),
    arm_paw_moveit_left_right(joy_msg->left_axes_left_right),
    arm_paw_moveit_up_down(joy_msg->right_axes_up_down),
    //arm_paw_rotateleft(joy_msg->right_axes_left_right),
    //arm_paw_rotateright(joy_msg ->button2),

    // 对应机械臂控制按钮
    arm_control_button(joy_msg->L1),
    // 机械臂直接控制
    arm_moveit_control_button(joy_msg->L2),
    // 机械臂位置移动指令
    //机械臂整体
    arm_control_forward_back(joy_msg->left_axes_up_down), //小臂的移动
    arm_control_up_down(joy_msg->up_down),                //大臂上下
    arm_control_left_right(joy_msg->left_right),          //大臂左右
    // 机械臂视角转动指令
    camera_control_up_down(-joy_msg->right_axes_up_down),     //第一个摆动
    camera_control_rotate(joy_msg->left_axes_left_right), //第一个旋转，摄像头旋转
    // 机械臂复位指令(依照设定,包括视角复位)
    arm_reset_button(joy_msg->right_axes_button),
    // 视角复位指令
    camera_reset_button(joy_msg->left_axes_button),
    // 爪子开合指令,button2与button4
    gripper_control_open_close(joy_msg->right_button_left_right),
    // 爪子旋转指令
    gripper_control_rotate(joy_msg->right_axes_left_right),
    gripper_reset_left(joy_msg->button1),gripper_reset_right(joy_msg->button3),
    // 机械臂移动速度参数，更换舵机电机时要根据实际效果调整此处及yaml文件
    arm_scale(2.5),
    arm_scale_direct(2.5), //单纯调大数值对增加转动流畅度影响不大
    // 机械臂视角转动速度参数
    arm_camera_scale(0.07),
    arm_scale_moveit(0.004), //TODO:测试时待调整
    arm_scale_moveit1(0.03),
    vice_whell_(2.5)
    
{
    joy_msg = new autoJoyExplainer();
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 2, &ExplorerTeleop::joyCallback, this);

    vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    vice_change = nh_.subscribe<std_msgs::Float32>("vice_speed_change", 2, &ExplorerTeleop::vice_wheel_speed_change, this);
    vice_wheel_reset_pub = nh_.advertise<explorer_msgs::explorer_vice_reset>("explorer_vice_wheel_reset", 1);//副履带位置重置信号
    vice_wheel_pub = nh_.advertise<explorer_msgs::explorer_vice_wheel>("explorer_vice_wheel", 1);

    arm_pub = nh_.advertise<explorer_msgs::explorer_moveit_gripper>("explorer_moveit_gripper", 1);
    arm_direct_pub = nh_.advertise<geometry_msgs::TwistStamped>("explorer_arm_direct", 1); 

    reset_pub = nh_.advertise<explorer_msgs::explorer_reset>("explorer_reset", 1); 
    gripper_pub = nh_.advertise<std_msgs::Float32>("explorer_gripper", 1);

    //此处调整消息发布的频率，当前为10Hz
    timer = ph_.createTimer(ros::Duration(0.1), boost::bind(&ExplorerTeleop::publishControl, this));
    //此处副履带系数已经弃用
    ph_.param("vice_scale_init", vice_scale, 2.4);
    ph_.param("vice_scale_full", vice_scale_full, 3.2);
    ph_.param("vice_scale_mid", vice_scale_mid, 2.4);
    ph_.param("vice_scale_slow", vice_scale_slow, 1.6);

    ph_.param("l_scale_init", l_scale, 7.0); //0.3 0.6 1.6
    ph_.param("l_scale_full", l_scale_full, 9.0);
    ph_.param("l_scale_mid", l_scale_mid, 7.0);
    ph_.param("l_scale_slow", l_scale_slow, 5.0);

    //此处数值为下位机发送值的3倍
    ph_.param("a_scale_init", a_scale, 18.0); // 0.1 0.4 0.6
    ph_.param("a_scale_full", a_scale_full, 24.0);
    ph_.param("a_scale_mid", a_scale_mid, 18.0);
    ph_.param("a_scale_slow", a_scale_slow, 12.0);
}

void ExplorerTeleop::vice_wheel_speed_change(std_msgs::Float32 data) {
    vice_scale = data.data;
}

void ExplorerTeleop::front_speed_change(std_msgs::Float32 data)
{
    l_scale = data.data;
}

void ExplorerTeleop::rotation_speed_change(std_msgs::Float32 data)
{
    a_scale = data.data;
}

ExplorerTeleop::~ExplorerTeleop()
{
    delete joy_msg;
}

void ExplorerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    joy_msg->getMessage(joy);
    messageClean();

    if (joy_msg->askForButton(basic_mode_button)){//运动模式
            if (joy_msg->askForButton(speed_switch_button)){
                    if (joy_msg->askForButton(full_speed_button)){ //速度调节按钮
                            l_scale = l_scale_full;
                            a_scale = a_scale_full;
                            vice_scale  =    vice_scale_full;
                            ROS_INFO("set full speed");
                        }   

                    if (joy_msg->askForButton(mid_speed_button)){ 
                            l_scale = l_scale_mid;
                            a_scale = a_scale_mid;
                            vice_scale  =    vice_scale_mid;
                            ROS_INFO("set default speed");
                        }

                    if (joy_msg->askForButton(slow_speed_button)){  
                            l_scale = l_scale_slow;
                            a_scale = a_scale_slow;
                            vice_scale  =  vice_scale_mid;
                            ROS_INFO("set low speed");
                        }
                }
            else {
            //整车前后
            if (std::fabs(joy_msg->askForAxes(linear_front_back)) > 10e-6){
                    vel_publisher.linear.x = joy_msg->askForAxes(linear_front_back) * l_scale;
                }

            //整车旋转，无麦轮无法左右平移
            if (std::fabs(joy_msg->askForAxes(angular_left_right)) > 10e-6){
                    vel_publisher.angular.z = joy_msg->askForAxes(angular_left_right) * a_scale;
                }   
            
            //左前副履带单独移动,axes组按钮
            if(std::fabs(joy_msg->askForAxes(left_front_vice_wheel_up_down))> 10e-6) {
                        vice_wheel_publisher.front_left_wheel_angular = -(joy_msg->askForAxes(left_front_vice_wheel_up_down)) * vice_whell_;
                }

            //左后副履带单独移动
            if(std::fabs(joy_msg->askForAxes(left_back_vice_wheel_up_down))> 10e-6) {
                    vice_wheel_publisher.back_left_wheel_angular = joy_msg->askForAxes(left_back_vice_wheel_up_down) * vice_whell_;
                }

            //右前副履带上移
            if(std::fabs(joy_msg->askForButton(right_front_vice_wheel_up)) > 10e-6) {
                    vice_wheel_publisher.front_right_wheel_angular = -(joy_msg->askForButton(right_front_vice_wheel_up)) * vice_whell_;
                }

            //右前副履带下移
            if(std::fabs(joy_msg->askForButton(right_front_vice_wheel_down))> 10e-6) {
                    vice_wheel_publisher.front_right_wheel_angular = joy_msg->askForButton(right_front_vice_wheel_down) * vice_whell_;
                }

            //右后副履带上移
            if(std::fabs(joy_msg->askForButton(right_back_vice_wheel_up))> 10e-6) {
                    vice_wheel_publisher.back_right_wheel_angular = -(joy_msg->askForButton(right_back_vice_wheel_up)) * vice_whell_;
                }
            //右后副履带下移
            if(std::fabs(joy_msg->askForButton(right_back_vice_wheel_down) )> 10e-6) {
                    vice_wheel_publisher.back_right_wheel_angular = joy_msg->askForButton(right_back_vice_wheel_down) * vice_whell_;
                }
            

            //重启底盘副履带，手柄数字4,TODO
            if (joy_msg->askForButton(arm_reset_button)){
                    ROS_INFO("vice wheel reset");
                    //explorer_msgs::explorer_vice_reset reset_msg;
                    vel_publisher.linear.z = joy_msg->askForButton(arm_reset_button);
                    vice_reset_publisher.front_vice_wheel_reset = true;
                    vice_reset_publisher.back_vice_wheel_reset = true;
                    //vice_wheel_reset_pub.publish(vice_reset_publisher);
                }
            /**
            if (std::fabs(joy_msg->askForButton(joy_msg->button1))> 10e-6
                    && std::fabs(joy_msg->askForButton(joy_msg->button2))> 10e-6
                        && std::fabs(joy_msg->askForButton(joy_msg->button3))> 10e-6
                            && std::fabs(joy_msg->askForButton(joy_msg->button4))> 10e-6){
                    vel_publisher.linear.z = joy_msg->askForButton(relive);
                }
             * */

            }

            
        }

    if (joy_msg->askForButton(arm_control_button))
    { //机械臂控制


        if (joy_msg->askForButton(arm_moveit_control_button))
        { //需要具体在rvi中根据方位在测试 现阶段 button前正、左旋为正
            if (std::fabs(joy_msg->askForAxes(arm_moveit_up_down)) > 10e-6)
            {
                arm_moveit_publisher.z = arm_scale_moveit * joy_msg->askForAxes(arm_moveit_up_down); //按键上下 整体上下
            }

            if (std::fabs(joy_msg->askForAxes(arm_moveit_left_right)) > 10e-6)
            {
                arm_moveit_publisher.y = -arm_scale_moveit * joy_msg->askForAxes(arm_moveit_left_right); //按键左右 整体左右
            }

            if (std::fabs(joy_msg->askForAxes(arm_moveit_forward)) > 10e-6)
            { 
                arm_moveit_publisher.x = arm_scale_moveit * (joy_msg->askForAxes(arm_moveit_forward));//左轴上下 整体前进后退
            }

            if (std::fabs(joy_msg->askForAxes(arm_paw_moveit_up_down)) > 10e-6)
            {
                arm_moveit_publisher.up_down = -arm_scale_moveit1 * joy_msg->askForAxes(arm_paw_moveit_up_down); //右轴上下  上下摆动
            }

            if (std::fabs(joy_msg->askForAxes(arm_paw_moveit_left_right)) > 10e-6)
            {
                arm_moveit_publisher.left_right = arm_scale_moveit1 * joy_msg->askForAxes(arm_paw_moveit_left_right); //左轴左右  左右摆动
            }

            if (std::fabs(joy_msg->askForAxes(gripper_control_rotate)) > 10e-6)
            { //右轴左右 左右旋转
                arm_moveit_publisher.rotate = -arm_scale_moveit1 * (joy_msg->askForAxes(gripper_control_rotate));
            }

            if (std::fabs(joy_msg->askForAxes(gripper_control_open_close)) > 10e-6)
            { //爪子开合
                gripper_move_msg.data = arm_camera_scale* joy_msg->askForAxes(gripper_control_open_close) ;
            }
        }
        else
        { //直接控制
            if (std::fabs(joy_msg->askForAxes(arm_control_forward_back)) > 10e-6)
            {
                arm_direct_publisher.twist.linear.z = -arm_scale_direct * joy_msg->askForAxes(arm_control_forward_back); //小臂上下arm3_arm2_joint
            }

            if (std::fabs(joy_msg->askForAxes(arm_control_left_right)) > 10e-6)
            {
                arm_direct_publisher.twist.linear.y = arm_scale_direct * joy_msg->askForAxes(arm_control_left_right); //整体大臂左右arm1_bearing_joint
            }

            if (std::fabs(joy_msg->askForAxes(arm_control_up_down)) > 10e-6)
            {
                arm_direct_publisher.twist.linear.x = -arm_scale_direct * joy_msg->askForAxes(arm_control_up_down); //整体大臂上下arm2_arm1_joint
            }

            if (std::fabs(joy_msg->askForAxes(camera_control_up_down)) > 10e-6)
            {
                arm_direct_publisher.twist.angular.x = arm_camera_scale * joy_msg->askForAxes(camera_control_up_down); //爪子上下摆动pt2_pt1_joint
            }

            if (std::fabs(joy_msg->askForAxes(camera_control_rotate)) > 10e-6)
            {
                arm_direct_publisher.twist.angular.y = -arm_camera_scale * joy_msg->askForAxes(camera_control_rotate); //爪子后端旋转pt1_arm_joint
            }

            if (std::fabs(joy_msg->askForAxes(gripper_control_rotate)) > 10e-6)
            {
                arm_direct_publisher.twist.angular.z = -arm_camera_scale * joy_msg->askForAxes(gripper_control_rotate); //爪子旋转rotate_joint
            }
            if (std::fabs(joy_msg->askForAxes(gripper_control_open_close)) > 10e-6)
            {
                gripper_move_msg.data = -arm_camera_scale * joy_msg->askForAxes(gripper_control_open_close) ;//爪子开合gripper_joint
            }
        }

        if (joy_msg->askForButton(arm_reset_button))
        {
            ROS_INFO("arm reset");
            explorer_msgs::explorer_reset reset_msg;
            reset_msg.reset_arm = true;
            reset_pub.publish(reset_msg);
        }

        else if (joy_msg->askForButton(gripper_reset_left))
        {
            ROS_INFO("gripper set to 0");
            explorer_msgs::explorer_reset reset_msg;
            reset_msg.gripper_left = true;
            reset_pub.publish(reset_msg);
        }

        else if (joy_msg->askForButton(gripper_reset_right))
        {
            ROS_INFO("gripper set to 3.14");
            explorer_msgs::explorer_reset reset_msg;
            reset_msg.gripper_right = true;
            reset_pub.publish(reset_msg);
        }

        
        else if (joy_msg->askForButton(camera_reset_button))
        {
            if (joy_msg->askForButton(basic_mode_button))
            {
                ROS_INFO("end effector back to left");
                explorer_msgs::explorer_reset reset_msg;
                reset_msg.arm_back = true;
                reset_pub.publish(reset_msg);
            }
            else 
            {
                ROS_INFO("camera reset");
                explorer_msgs::explorer_reset reset_msg;
                reset_msg.reset_camera = true;
                reset_pub.publish(reset_msg);
            }

        }

    }

}

/*
 * 消息发布函数
 * 由createTimer函数生成的计时器保证发送
 * 速率 10hz
 */
void ExplorerTeleop::publishControl()
{

    geometry_msgs::Twist vel_empty;
    geometry_msgs::Twist vel_empty1;
    vel_empty1.linear.x = vel_empty1.linear.y = vel_empty1.linear.z = 0;
    vel_empty1.angular.x = vel_empty1.angular.y = vel_empty1.angular.z = 0;
    if (vel_publisher != vel_empty || last_vel_published != vel_empty)
    {
        vel_pub.publish(vel_publisher);

        if (vel_publisher == vel_empty)
        {
            vel_pub.publish(vel_empty1);
            ROS_INFO("publish vel zero");
        }
        last_vel_published = vel_publisher;
    }

    explorer_msgs::explorer_vice_wheel vice_empty;
    if (vice_wheel_publisher != vice_empty || last_vice_wheel_published != vice_empty)
    {
        vice_wheel_pub.publish(vice_wheel_publisher);

        if (vice_wheel_publisher == vice_empty)
        {
            ROS_INFO("publish vice wheel zero");
        }

        last_vice_wheel_published = vice_wheel_publisher;
    }

    explorer_msgs::explorer_moveit_gripper arm_empty;
    geometry_msgs::TwistStamped arm_empty1;
    /* if (arm_moveit_publisher.x != arm_empty.x || arm_moveit_publisher.y != arm_empty.y||
        arm_moveit_publisher.up_down != arm_empty.up_down || arm_moveit_publisher.left_right != arm_empty.left_right||
        arm_moveit_publisher.rotate != arm_empty.rotate) {*/
    if (arm_moveit_publisher != arm_empty || last_arm_moveit_published != arm_empty)
    {
        arm_pub.publish(arm_moveit_publisher);

        if (arm_moveit_publisher == arm_empty)
        {
            ROS_INFO("publish arm zero 0.0.0.0.");
        }

        last_arm_moveit_published = arm_moveit_publisher;
    }

    if (arm_direct_publisher.twist != arm_empty1.twist || last_arm_direct_published.twist != arm_empty1.twist)
    {
        arm_direct_publisher.header.stamp = ros::Time::now();
        arm_direct_pub.publish(arm_direct_publisher);

        if (arm_direct_publisher == arm_empty1)
        {
            ROS_INFO("publish direct arm zero");
        }
        ROS_INFO("publish direct arm zero");
        last_arm_direct_published = arm_direct_publisher;
    }

    std_msgs::Float32 float_empty;
    if (gripper_move_msg != float_empty)
    {
        gripper_pub.publish(gripper_move_msg);
    }

    explorer_msgs::explorer_vice_reset reset_empty;
    if (vice_reset_publisher != reset_empty || last_vice_reset_published != reset_empty)
    {
        vice_wheel_reset_pub.publish(vice_reset_publisher);

        if (vice_reset_publisher == reset_empty)
        {
            ROS_INFO("vice wheel reset");
        }

        last_vice_reset_published = vice_reset_publisher;
    }
}

/*
 * 用于清空所有的数据
 * 用于收到新的指令之时
 */
void ExplorerTeleop::messageClean()
{
    geometry_msgs::Twist vel_empty;
    this->vel_publisher = vel_empty;
    explorer_msgs::explorer_vice_wheel vice_empty;
    this->vice_wheel_publisher = vice_empty;
    explorer_msgs::explorer_moveit_gripper arm_empty;
    geometry_msgs::TwistStamped arm_empty1;
    this->arm_moveit_publisher = arm_empty;
    this->arm_direct_publisher = arm_empty1;
    std_msgs::Float32 float_empty;
    this->gripper_move_msg = float_empty;
    explorer_msgs::explorer_vice_reset reset_empty;
    this->vice_reset_publisher = reset_empty;
}
