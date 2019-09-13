#include "explorer_moveit.h"
#include <sstream>

explorer_moveit::explorer_moveit(ros::NodeHandle node):
  nh_(node),PLANNING_GROUP("explorer_arm"),move_group(PLANNING_GROUP)

{
    //reference_frame = "frame_link";
    //move_group.setPoseReferenceFrame(reference_frame);
    //current_posi = move_group.getCurrentPose("pt2_link");
    //current_posi = move_group.getCurrentPose("gripper_link");
     //current_RPY = move_group.getCurrentRPY("gripper_link");

    //此处应该使用直接从moveit中读取的数据进行末端执行器的目标状态,
    //但是在获取状态的时候,
    //姿态无法获得,一但访问current_RPT 就会die
    //而且提示Failed to fetch current robot state

    
    //此为直接从rviz读取
    /*
     target_pose.position.x =  current_posi.pose.position.x;
     target_pose.position.y = current_posi.pose.position.y;
     target_pose.position.z = current_posi.pose.position.z;
     target_pose.orientation.x = current_posi.pose.orientation.x;
     target_pose.orientation.y = current_posi.pose.orientation.y;
     target_pose.orientation.z = current_posi.pose.orientation.z;
     target_pose.orientation.w = current_posi.pose.orientation.w;
     rotate = current_RPY.at(2);
     up_down = current_RPY.at(1);
     left_right = current_RPY.at(0);
     */
    
    //此为直接从commender看的数据
    
    
    target_pose.position.x = -0.0694495166048;
    target_pose.position.y = -0.0210640971723;
    target_pose.position.z = 0.0895376533598;
    target_pose.orientation.x = 6.81122190439e-09;
    target_pose.orientation.y = 0.00186164533487;
    target_pose.orientation.z = -0.99999826713;
    target_pose.orientation.w = 3.67319878841e-06;
    rotate = -0.003723292820314199; //左右旋转, Roll横滚？？
    up_down = 2.729880697982673e-08;  //上下摆动，Pitch俯仰？？
    left_right = -3.141585307230307; //左右摆动，Yaw偏航？

    
    
    ROS_ERROR_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

   // move_group.setPlannerId("RRTConfigDefault");
    paw_sub = nh_.subscribe<explorer_msgs::explorer_moveit_gripper>("explorer_moveit_gripper", 2, &explorer_moveit::arm_callback, this);
    paw_reset = nh_.subscribe<explorer_msgs::explorer_reset>("explorer_reset",2,&explorer_moveit::arm_reset,this);
  }
explorer_moveit::~explorer_moveit(){}
//绕x为旋转
//绕y为上下
//绕z为左右?
void explorer_moveit::arm_callback(explorer_msgs::explorer_moveit_gripper gripperptr){
    //先进行初始位置的再次标定，将两个模式统一
    
     /*target_pose.position.x =  current_posi.pose.position.x + gripperptr.x;
     target_pose.position.y = current_posi.pose.position.y + gripperptr.y;
     target_pose.position.z = current_posi.pose.position.z + gripperptr.z;*/
    // left_right = current_RPY[2] + gripperptr.left_right;
    // up_down = current_RPY[1] + gripperptr.up_down;
    // rotate = current_RPY[0] + gripperptr.rotate;


    target_pose.position.x += gripperptr.x; 
    target_pose.position.y += gripperptr.y;
    target_pose.position.z += gripperptr.z;
    left_right += gripperptr.left_right;
    up_down += gripperptr.up_down;
    rotate += gripperptr.rotate;

    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rotate,up_down,left_right);
    ROS_ERROR_STREAM("  w:"<<odom_quat.w << "  x:"<<odom_quat.x<<"  y:"<<odom_quat.y << "  z:"<<odom_quat.z);
    target_pose.orientation.w = (double)odom_quat.w;
    target_pose.orientation.x = (double)odom_quat.x;
    target_pose.orientation.y = (double)odom_quat.y;
    target_pose.orientation.z = (double)odom_quat.z;
    move_group.setPoseTarget(target_pose);
    
    /*moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
      move_group.move();    
      ROS_ERROR("SSSSSSSSSS");
    }*/
    move_group.asyncMove();
} 
void explorer_moveit::arm_reset(explorer_msgs::explorer_reset ptr){

    target_pose.position.x = -0.0694495166048;
    target_pose.position.y = -0.0210640971723;
    target_pose.position.z = 0.0895376533598;
    target_pose.orientation.x = 6.81122190439e-09;
    target_pose.orientation.y = 0.00186164533487;
    target_pose.orientation.z = -0.99999826713;
    target_pose.orientation.w = 3.67319878841e-06;
    rotate = -0.003723292820314199; //左右旋转, Roll横滚？？
    up_down = 2.729880697982673e-08;  //上下摆动，Pitch俯仰？？
    left_right = -3.141585307230307; //左右摆动，Yaw偏航？
  
}
int main (int argc,char **argv){
  ros::init(argc, argv, "explorer_moveit");
  ros::NodeHandle nodehandle;
  ros::AsyncSpinner spinner(1);
  explorer_moveit explorer_moveit(nodehandle);  
  spinner.start();  
}
