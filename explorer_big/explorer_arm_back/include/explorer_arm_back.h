#ifndef EXPLORER_ARM_BACK_H
#define EXPLORER_ARM_BACK_H

#include <ros/ros.h>
#include <explorer_msgs/explorer_message.h>
#include <explorer_msgs/explorer_low_level_data.h>

class ExplorerArmBack
{
 public:
    ExplorerArmBack(ros::NodeHandle node) ;
    ~ExplorerArmBack() ;
    void start() ;
protected:

private:
    void sendArmRequest() ;
    void co2MsgSub(const explorer_msgs::explorer_low_level_dataConstPtr &ptr) ;
    
    ros::NodeHandle nh_ ;
    ros::Publisher co2_pub_ ;
    int arm_back_id_ ;
};

#endif // EXPLORER_ARM_BACK_H
