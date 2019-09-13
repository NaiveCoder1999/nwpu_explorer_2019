#ifndef  EXPLORER_VICE_WHEEL_CONTROLLER_H
#define EXPLORER_VICE_WHEEL_CONTROLLER_H
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <explorer_msgs/explorer_vice_wheel.h>
#include <explorer_msgs/explorer_vice_reset.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <cmath>
#include "vice_wheel_joint_controller.hpp"

namespace explorer_vice_wheel_controller
{
struct position
{
    std::vector<double> Position;
    ros::Time time;
    position() : time(0.0) {}
};
class ViceWheelController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
  public:
    ViceWheelController();
    ~ViceWheelController();
    // 初始化函数 是 controller 的构造要求
    // 返回值为是否构造成功
    bool init(hardware_interface::PositionJointInterface *joint_handle_, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    // start 与 stop 函数采用 Controller 默认的函数,即置空

    // 更新函数,每次机械臂的位置数据更新都在这个函数里进行
    void update(const ros::Time &time, const ros::Duration &period);

  private:
    // 获取机械臂名称列表
    bool getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names);
    void viceWheelCallBack(const explorer_msgs::explorer_vice_wheelConstPtr &ptr) ;
    void resetCommandCallBack(const explorer_msgs::explorer_vice_reset &msg);

  private:
    std::vector<std::string> vice_wheel_names; //副履带名称列表
    std::map<std::string, joint *> joint_map;

    //ros::NodeHandle nh_;

    ros::Subscriber vice_wheel_sub_;
    ros::Subscriber vice_reset_sub_;

    ros::Time front_time; // 上一次更新时间
    explorer_msgs::explorer_vice_reset reset_msg;

    std::queue<std::vector<std::pair<std::string, double>>> reset_queue; // 复位任务队列
};
//这句话将这个类设置为插件
PLUGINLIB_EXPORT_CLASS(explorer_vice_wheel_controller::ViceWheelController, controller_interface::ControllerBase);
}

#endif // EXPLORER_VICE_WHEEL_CONTROLLER_H