#ifndef EXPLORER_ODOMETRY_H
#define EXPLORER_ODOMETRY_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <explorer_msgs/explorer_message.h>
#include <explorer_msgs/explorer_low_level_data.h>

class ExplorerOdometry
{
 public:
    ExplorerOdometry(ros::NodeHandle node) ;
    ~ExplorerOdometry() ;
    void start() ;
protected:
private:

    void sendiMURequest() ;
    void sendCancelRequest();

    void iMUMsgSub(const explorer_msgs::explorer_low_level_dataConstPtr &ptr) ;

private:
    ros::NodeHandle nh_ ;
    ros::Publisher imu_pub_ ;
    ros::Publisher odom_pub_;
    ros::Subscriber imu_sub_ ;
    nav_msgs::Odometry odom; //里程计消息
    geometry_msgs::Quaternion odom_quat;//四元数
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;//里程计信息转换器
    ros::Time current_time, last_time;
    int odom_id_ ;
    double x, y, th;//位置的坐标，th未知？？
    double vx, vy, vth;//各个方向上的速度，目前全设为零
    double dt;//时间差
    double delta_x, delta_y, delta_th;//坐标差
    


};

#endif // EXPLORER_ODOMETRY_H
