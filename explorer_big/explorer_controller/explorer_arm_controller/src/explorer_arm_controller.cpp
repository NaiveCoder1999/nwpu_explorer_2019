#include "explorer_arm_controller.h"
#include <cmath>

using namespace explorer_arm_controller;
ArmController::ArmController() {}
ArmController::~ArmController() {}
/*
 *机械臂数据的初始化
 */
bool ArmController::init(hardware_interface::PositionJointInterface *joint_handle_,
                         ros::NodeHandle &root_nh,
                         ros::NodeHandle &controller_nh)
{

    // 获取机械臂名称列表
    if (!getNameList(controller_nh, "joints", this->arm_name))
    {
        ROS_ERROR("can not get name list!");
        return false;
    }

    // 建立机械臂描述对象
    for (auto it : arm_name)
    {
        joint_map.insert(std::make_pair(it, new joint_with_limit(it, controller_nh, *joint_handle_)));
    }

    this->state_sub_ = controller_nh.subscribe("/explorer_arm_direct", 10, &ArmController::jointStateSub, this);
    this->reset_sub_ = controller_nh.subscribe("/explorer_reset", 10, &ArmController::resetStateSub, this);
    this->gripper_sub_ = controller_nh.subscribe("/explorer_gripper", 10, &ArmController::gripperStateSub, this);
    this->moveit_sub_ = controller_nh.subscribe("/explorer_moveit_listener/explorer_moveit_joint", 50, &ArmController::moveitSub, this);
    this->yuntai_sub_ = controller_nh.subscribe("/explorer_serial_data/19", 10, &ArmController::yuntaiSub, this);
    //this->imu_pub_ = controller_nh.advertise<sensor_msgs::Imu>("imu", 1);
    ROS_ERROR("arm init successed");
    return true;
}
/*
 *机械臂数据的数据刷新
 */
void ArmController::update(const ros::Time &time, const ros::Duration &period)
{
    ros::Time now_time = time;
    // 计算与上一次的刷新的时间差
    double timep = fabs((front_time.toSec() - now_time.toSec()));
    front_time = now_time;

    // 显示所有机械臂的位置,如果机械臂的位置达到限制,标红
    for (auto name : arm_name)
    {
        if (joint_map[name]->moveWillDanger())
        {
            ROS_ERROR_STREAM("arm " << name << " :\t" << joint_map[name]->getNowPose());
        }
        else
        {
            ROS_INFO_STREAM("arm " << name << " :\t" << joint_map[name]->getNowPose());
        }
    }

    // 检测是否需要复位
    if (!reset_queue.empty())
    {
        std::vector<std::pair<std::string, double>> line;
        // 取出当前的目标
        line = reset_queue.front();
        int get_goal = 0;

        for (auto name : line)
        {
            joint_map[name.first]->setAim(name.second);

            if (joint_map[name.first]->moveToAim(timep))
            {
                // 记录达到目标的数量
                ++get_goal;
            }
        }

        // 全部达到目标,转入下一个位置
        if (get_goal == line.size())
        {
            reset_queue.pop();
        }

        return;
    }

    // 建议对map的遍历使用iterator保证效率
    for (auto it : joint_map)
    {
        it.second->moveToAim(timep);
    }
}

//机械臂移动控制
void ArmController::jointStateSub(const geometry_msgs::TwistStamped &msg)
{
    if (msg.twist.linear.x != 0 || msg.twist.linear.y != 0 || msg.twist.linear.z != 0 ||
        msg.twist.angular.x != 0 || msg.twist.angular.y != 0 || msg.twist.angular.z != 0)
    {
        while (reset_queue.size())
        {
            reset_queue.pop();
        }
    }

    joint_map["arm1_bearing_joint"]->setAim(joint_map["arm1_bearing_joint"]->getNowPose() //整体左右
                                            + msg.twist.linear.y);
    joint_map["arm2_arm1_joint"]->setAim(joint_map["arm2_arm1_joint"]->getNowPose() //整体上下
                                         + msg.twist.linear.x);
    joint_map["arm3_arm2_joint"]->setAim(joint_map["arm3_arm2_joint"]->getNowPose() //小臂上下
                                         - msg.twist.linear.z);

    joint_map["pt1_arm_joint"]->setAim(joint_map["pt1_arm_joint"]->getNowPose() //第一轴向
                                       - msg.twist.angular.y);
    joint_map["pt2_pt1_joint"]->setAim(joint_map["pt2_pt1_joint"]->getNowPose() //摆动
                                       - msg.twist.angular.x);
    joint_map["rotate_joint"]->setAim(joint_map["rotate_joint"]->getNowPose() //转动，转爪子
                                      + msg.twist.angular.z);
}


//爪子开合控制
void ArmController::gripperStateSub(const std_msgs::Float32 &ptr)
{
    double static const cnt = 0.07;
    joint_map["gripper_joint"]->setAim(joint_map["gripper_joint"]->getNowPose() + ptr.data);
    //手动拟合爪子...
    joint_map["finger1_joint"]->setAim(-0.68 * (joint_map["gripper_joint"]->getNowPose() + ptr.data) + 1.18);
    joint_map["finger2_joint"]->setAim(-0.68 * (joint_map["gripper_joint"]->getNowPose() + ptr.data) + 1.18);
    joint_map["finger3_joint"]->setAim(0.68 * (joint_map["gripper_joint"]->getNowPose() + ptr.data) - 1.18);
}

//机械臂位姿重置
void ArmController::resetStateSub(const explorer_msgs::explorer_reset &ptr)
{
    ROS_INFO("reset pub");
    now_moveit_pose = -1;

    while (!reset_queue.empty())
    {
        reset_queue.pop();
    }

    if (ptr.reset_arm)
    {

       std::vector<std::pair<std::string, double>> queue;
       
        if (fabs(joint_map["arm1_bearing_joint"]->getResetPose() - joint_map["arm1_bearing_joint"]->getNowPose()) < PI / 4)
        {
            // 当机械臂偏移角度不大时(小于45度),先自下到上复位小臂
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), joint_map["arm1_bearing_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("gripper_joint"), joint_map["gripper_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger1_joint"), joint_map["finger1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger2_joint"), joint_map["finger2_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger3_joint"), joint_map["finger3_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("rotate_joint"), joint_map["rotate_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("pt2_pt1_joint"), joint_map["pt2_pt1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("pt1_arm_joint"), joint_map["pt1_arm_joint"]->getResetPose()));
            reset_queue.push(queue);
            queue.clear();
            for (auto it = joint_map.begin(); it != joint_map.end(); ++it)
            {
                it->second->readyForResetPose(); //目标值aim_pose被赋值为重置的位姿数值
            }
        }
        else
        {
            // 当机械臂偏移角度过大时,分步复位
            //第一步,将大臂立起来,底座复位
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), 0.0));           
            queue.push_back(std::make_pair(std::string("arm3_arm2_joint"), 0.85));
            queue.push_back(std::make_pair(std::string("arm2_arm1_joint"), -0.44));
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), 0.0));
            reset_queue.push(queue);
            queue.clear();
            

            //第二步,大臂转正,复位小臂
            
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), joint_map["arm1_bearing_joint"]->getResetPose()));

            queue.push_back(std::make_pair(std::string("gripper_joint"), joint_map["gripper_joint"]->getResetPose()));

            queue.push_back(std::make_pair(std::string("finger1_joint"), joint_map["finger1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger2_joint"), joint_map["finger2_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger3_joint"), joint_map["finger3_joint"]->getResetPose()));

            queue.push_back(std::make_pair(std::string("rotate_joint"), joint_map["rotate_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("pt2_pt1_joint"), joint_map["pt2_pt1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("pt1_arm_joint"), joint_map["pt1_arm_joint"]->getResetPose()));
            reset_queue.push(queue);
            queue.clear();

            //第三步,将大臂复位
            queue.push_back(std::make_pair(std::string("arm2_arm1_joint"), joint_map["arm2_arm1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("arm3_arm2_joint"), joint_map["arm3_arm2_joint"]->getResetPose()));
            reset_queue.push(queue);
        }
        
            /**
            if (joint_map["arm3_arm2_joint"]->getNowPose() == 0.0);
            {
                joint_map["arm3_arm2_joint"]->setAim(PI/4);
            }
            joint_map["arm1_bearing_joint"]->readyForResetPose();
            joint_map["arm2_arm1_joint"]->readyForResetPose();
            joint_map["gripper_joint"]->readyForResetPose();
            joint_map["finger1_joint"]->readyForResetPose();
            joint_map["finger2_joint"]->readyForResetPose();
            joint_map["finger3_joint"]->readyForResetPose();
            joint_map["rotate_joint"]->readyForResetPose();
            joint_map["pt2_pt1_joint"]->readyForResetPose();
            joint_map["pt1_arm_joint"]->readyForResetPose();
            joint_map["arm3_arm2_joint"]->readyForResetPose();
            **/
            /**
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), joint_map["arm1_bearing_joint"]->getResetPose()));//左右归位
            queue.push_back(std::make_pair(std::string("arm2_arm1_joint"), joint_map["arm2_arm1_joint"]->getResetPose()));//大臂归位
            queue.push_back(std::make_pair(std::string("gripper_joint"), joint_map["gripper_joint"]->getResetPose()));//瓜子合上
            queue.push_back(std::make_pair(std::string("finger1_joint"), joint_map["finger1_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger2_joint"), joint_map["finger2_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("finger3_joint"), joint_map["finger3_joint"]->getResetPose()));
            queue.push_back(std::make_pair(std::string("rotate_joint"), joint_map["rotate_joint"]->getResetPose()));//爪子旋转归位
            queue.push_back(std::make_pair(std::string("pt2_pt1_joint"), joint_map["pt2_pt1_joint"]->getResetPose()));//摄像头归位
            queue.push_back(std::make_pair(std::string("pt1_arm_joint"), joint_map["pt1_arm_joint"]->getResetPose()));//摄像头上下归位
            queue.push_back(std::make_pair(std::string("arm3_arm2_joint"), joint_map["arm3_arm2_joint"]->getResetPose()));//小臂归位
            reset_queue.push(queue);
            queue.clear();
            **/
            /**
            for (auto it = joint_map.begin(); it != joint_map.end(); ++it)
            {
                it->second->readyForResetPose(); //目标值aim_pose被赋值为重置的位姿数值
            }
            **/
    }

    else if (ptr.reset_camera)
    {  /**
        for (int i = 3; i < arm_name.size(); ++i)
        {
            joint_map[arm_name[i]]->readyForResetPose();
        }
        **/
       std::vector<std::pair<std::string, double>> queue;
       queue.push_back(std::make_pair(std::string("gripper_joint"), joint_map["gripper_joint"]->getResetPose()));
       queue.push_back(std::make_pair(std::string("rotate_joint"), joint_map["rotate_joint"]->getResetPose()));
       queue.push_back(std::make_pair(std::string("pt2_pt1_joint"), joint_map["pt2_pt1_joint"]->getResetPose()));
       queue.push_back(std::make_pair(std::string("pt1_arm_joint"), joint_map["pt1_arm_joint"]->getResetPose()));
       reset_queue.push(queue);
       queue.clear();
    }

    //NOT USED YET
    else if (ptr.reset_gripper)
    {
        joint_map["rotate_joint"]->readyForResetPose();
        joint_map["gripper_joint"]->setAim(1.57);
    }

    else if (ptr.gripper_left)
    {
        //joint_map["rotate_joint"]->handle.setCommand(-1.57);
        joint_map["rotate_joint"]->setAim(-1.57);
        //joint_map["rotate_joint"]->setNowPose(-1.57);

    }

    else if (ptr.gripper_right)
    {
        //joint_map["rotate_joint"]->handle.setCommand(1.57);
        joint_map["rotate_joint"]->setAim(1.57);
        //joint_map["rotate_joint"]->setNowPose(1.57);
    }

    /**
     * 此处重置用于收起机械臂头部扭到侧面保护，用于导航和遥控越障
     * 
     **/
    else if (ptr.arm_back)
    {
        joint_map["arm3_arm2_joint"]->setAim(PI/4);
        joint_map["pt1_arm_joint"]->setAim(1.42);
        joint_map["pt2_pt1_joint"]->setAim(-1.31);
        
        if (joint_map["arm3_arm2_joint"]->getNowPose() == PI/4)
        {
            joint_map["arm3_arm2_joint"]->readyForResetPose();
        }
    }
}

void ArmController::moveitSub(const explorer_msgs::explorer_moveit_values &ptr)
{
    /* if (ptr.values[0] != 0 || ptr.values[1] != 0 || ptr.values[2] != 0 ||
        ptr.values[3] != 0 || ptr.values[4] != 0 || ptr.values[5] != 0) {
        while (reset_queue.size()) {
            reset_queue.pop();
        }
    }*/
    joint_map["arm1_bearing_joint"]->setAim(joint_map["arm1_bearing_joint"]->getNowPose() + ptr.values[0]);
    joint_map["arm2_arm1_joint"]->setAim(joint_map["arm2_arm1_joint"]->getNowPose() + ptr.values[1]);
    joint_map["arm3_arm2_joint"]->setAim(joint_map["arm3_arm2_joint"]->getNowPose() + ptr.values[2]);

    joint_map["pt1_arm_joint"]->setAim(joint_map["pt1_arm_joint"]->getNowPose() + ptr.values[3]);
    joint_map["pt2_pt1_joint"]->setAim(joint_map["pt2_pt1_joint"]->getNowPose() + ptr.values[4]);
    joint_map["rotate_joint"]->setAim(joint_map["rotate_joint"]->getNowPose() + ptr.values[5]);
}

void ArmController::yuntaiSub(explorer_msgs::explorer_low_level_data msg)
{
    joint_map["joint_front_back"]->setAim(-msg.can_serial_data_2 * 3.1415926 / 180);
    joint_map["robot_left_right"]->setAim(-msg.can_serial_data_1 * 3.1415926 / 180);
    //向上传入的四元数
}

bool ArmController::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names)
{
    XmlRpc::XmlRpcValue name_list;

    if (!controller_nh.getParam(name_param, name_list))
    {
        ROS_ERROR("can not get the joint list!");
        return false;
    }

    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if (name_list.size() == 0)
        {
            ROS_ERROR("did not get the joint name!");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++)
        {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("get the error name list!");
                return false;
            }
        }

        names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++)
        {
            names[i] = static_cast<std::string>(name_list[i]);
        }
    }
    else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        names.push_back(name_list);
    }
    else
    {
        ROS_ERROR("the wheel param get error");
        return false;
    }

    return true;
}
