#include "explorer_arm_back.h"
#include <sstream>

using namespace std ;

ExplorerArmBack::ExplorerArmBack(ros::NodeHandle node)
    :nh_(node),
    arm_back_id_(11)//11
{
    //stringstream sub_name_ ;
    //sub_name_<<"/explorer_serial_data/"<<arm_back_id_ ;
    co2_pub_ = nh_.advertise<explorer_msgs::explorer_message>("/explorer_driver",10) ;
}

void ExplorerArmBack::sendArmRequest()
{
    explorer_msgs::explorer_message drive_message ;
    drive_message.high_level_id = arm_back_id_ ;
    drive_message.low_level_id = arm_back_id_ ;
    drive_message.data.push_back(0);
    drive_message.data.push_back(1);
    co2_pub_.publish(drive_message) ;
}

ExplorerArmBack::~ExplorerArmBack()
{
    nh_.shutdown();
}


void ExplorerArmBack::start()
{
    ros::Rate r(10) ;
    while (ros::ok()) 
    {
        sendArmRequest();
        ros::spinOnce() ; 
        r.sleep() ;
    }
}

int main(int argc ,char **argv)
{
    ros::init(argc , argv,"explorer_arm_back") ;
    ros::NodeHandle node ;
    ExplorerArmBack arm_back(node) ;
    arm_back.start();
    return 0 ;
}
