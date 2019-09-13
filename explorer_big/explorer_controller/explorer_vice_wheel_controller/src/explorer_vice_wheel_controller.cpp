#include "explorer_vice_wheel_controller.h"
#include <cmath>

using namespace explorer_vice_wheel_controller;

ViceWheelController::ViceWheelController() {}
ViceWheelController::~ViceWheelController() {}

bool ViceWheelController::init(hardware_interface::PositionJointInterface *joint_handle_,
                         ros::NodeHandle &root_nh,
                         ros::NodeHandle &controller_nh)
{

    // 获取副履带名称列表
    if (!getNameList(controller_nh, "joints", this->vice_wheel_names))
    {
        ROS_ERROR("can not get name list!");
        return false;
    }

    // 建立副履带描述对象
    for (auto it : vice_wheel_names)
    {
        joint_map.insert(std::make_pair(it, new joint_with_limit(it, controller_nh, *joint_handle_)));
    }

    vice_wheel_sub_ = controller_nh.subscribe("explorer_vice_wheel", 10, &ViceWheelController::viceWheelCallBack, this);
    vice_reset_sub_ = controller_nh.subscribe("explorer_vice_wheel_reset", 10, &ViceWheelController::resetCommandCallBack, this);
    //imu_pub = controller_nh.subscribe("explorer_vice_wheel_reset", 10, &ViceWheelController::resetCommandCallBack, this);
    //来自explorer_joy_control
    ROS_ERROR("vice wheel init successed");
    return true;
}
/*
 *副履带数据的数据刷新
 */
void ViceWheelController::update(const ros::Time &time, const ros::Duration &period)
{
    ros::Time now_time = time;
    // 计算与上一次的刷新的时间差
    double time_diff = fabs((front_time.toSec() - now_time.toSec()));
    front_time = now_time;

    // 显示所有副履带的位置,如果副履带的位置达到限制,标红
    for (auto name : vice_wheel_names)
    {
        if (joint_map[name]->moveWillDanger())
        {
            ROS_ERROR_STREAM("wheel " << name << " :\t" << joint_map[name]->getNowPose());
        }
        else
        {
            ROS_INFO_STREAM("wheel " << name << " :\t" << joint_map[name]->getNowPose());
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

            if (joint_map[name.first]->moveToAim(time_diff))
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
        it.second->moveToAim(time_diff);
    }
}

//副履带移动控制
void ViceWheelController::viceWheelCallBack(const explorer_msgs::explorer_vice_wheelConstPtr &ptr)
{
    if (ptr->front_left_wheel_angular != 0 || ptr->front_right_wheel_angular != 0 
    || ptr->back_left_wheel_angular != 0 || ptr->back_right_wheel_angular != 0)
    {
        while (reset_queue.size())
        {
            reset_queue.pop();
        }
    }

    joint_map["left_up_fin_base_joint"]->setAim(joint_map["left_up_fin_base_joint"]->getNowPose() 
                                            + ptr->front_left_wheel_angular);
    joint_map["right_up_fin_base_joint"]->setAim(joint_map["right_up_fin_base_joint"]->getNowPose()
                                         + ptr->front_right_wheel_angular);
    joint_map["left_down_fin_base_joint"]->setAim(joint_map["left_down_fin_base_joint"]->getNowPose()
                                         - ptr->back_left_wheel_angular);
    joint_map["right_down_fin_base_joint"]->setAim(joint_map["right_down_fin_base_joint"]->getNowPose()
                                       - ptr->back_right_wheel_angular);
}

//副履带位姿重置
void ViceWheelController::resetCommandCallBack(const explorer_msgs::explorer_vice_reset &msg)
{
    reset_msg = msg;
    if (reset_msg.front_vice_wheel_reset && reset_msg.back_vice_wheel_reset)
    {
        for (auto name : vice_wheel_names)
        {
            joint_map[name]->setAim(0.0);
        }

    }
}

bool ViceWheelController::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names)
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
