#include "explorer_odometry.h"
#include <sstream>

using namespace std ;

ExplorerOdometry::ExplorerOdometry(ros::NodeHandle node)
    :nh_(node),odom_id_(10)
{
    stringstream sub_name_ ;
    sub_name_<<"/explorer_serial_data/"<<odom_id_ ;
    imu_pub_ = nh_.advertise<explorer_msgs::explorer_message>("/explorer_driver",10) ;
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    imu_sub_ = nh_.subscribe(sub_name_.str() ,10, &ExplorerOdometry::iMUMsgSub,this);
    
    x = y = th = 0.0;//初始位置全设为0
    vx = vy = vth = 0.0;//无里程计，初始速度及后续速度均为0
    current_time = ros::Time::now();
    last_time = ros::Time::now();

}

void ExplorerOdometry::iMUMsgSub(const explorer_msgs::explorer_low_level_dataConstPtr &ptr)
{
    float roll = ptr->can_serial_data_1;
    float pitch = ptr->can_serial_data_2;
    float yaw = 0.0;
    //ROS_INFO_STREAM("roll  = "<<roll) ;
    ROS_INFO_STREAM("roll = " << roll << " "<< "pitch = " << pitch);

    dt = (current_time - last_time).toSec();
    //位置增量的解算，目前无用
    delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //由欧拉角创建四元数
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    //发布坐标变换
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
    //发布里程计消息，传姿态
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //设置位置与姿态
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //设置速度，暂时没用
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub_.publish(odom);
    last_time = current_time;
    
}

void ExplorerOdometry::sendiMURequest()
{
    explorer_msgs::explorer_message request_message ;
    request_message.high_level_id = odom_id_ ;
    request_message.low_level_id = odom_id_ ;
    request_message.data.push_back(0);
    request_message.data.push_back(1);
    imu_pub_.publish(request_message) ;
}

void ExplorerOdometry::sendCancelRequest()
{
    explorer_msgs::explorer_message request_message ;
    request_message.high_level_id = odom_id_ ;
    request_message.low_level_id = odom_id_ ;
    request_message.data.push_back(0);
    request_message.data.push_back(0);
    imu_pub_.publish(request_message) ;
}

ExplorerOdometry::~ExplorerOdometry()
{
    sendCancelRequest();
    nh_.shutdown();
}

void ExplorerOdometry::start() //此处写增加数据处理
{
    ros::Rate r(50) ;
    sendiMURequest();
    while (ros::ok()) {
        sendiMURequest();
        ros::spinOnce() ; 
        r.sleep() ;
    }
}

int main(int argc ,char **argv)
{
    ros::init(argc , argv,"explorer_odom_pub_lisher") ;
    ros::NodeHandle node;
    ExplorerOdometry odom_(node);
    odom_.start();
    return 0 ;
}
