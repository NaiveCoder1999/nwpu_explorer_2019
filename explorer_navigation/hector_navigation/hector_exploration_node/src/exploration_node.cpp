//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <ros/ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

class SimpleExplorationPlanner
{
public:
  SimpleExplorationPlanner()
  {
    ros::NodeHandle nh;

    costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfl_); //调用Costmap获取代价地图并存储以便接下来使用

    planner_ = new hector_exploration_planner::HectorExplorationPlanner(); //开始调用exploration_planner
    planner_->initialize("hector_exploration_planner", costmap_2d_ros_);   //初始化

    exploration_plan_service_server_ = nh.advertiseService("get_exploration_path", &SimpleExplorationPlanner::explorationServiceCallback, this);

    exploration_plan_pub_ = nh.advertise<nav_msgs::Path>("exploration_path", 2); //发布路径
  }
  bool explorationServiceCallback(hector_nav_msgs::GetRobotTrajectory::Request &req,
                                  hector_nav_msgs::GetRobotTrajectory::Response &res)
  {
    ROS_INFO("Exploration Service called");

    tf::Stamped<tf::Pose> robot_pose_tf;
    costmap_2d_ros_->getRobotPose(robot_pose_tf);

    geometry_msgs::PoseStamped pose;
    tf::poseStampedTFToMsg(robot_pose_tf, pose);

    if (planner_->a == 0)
    {
      ROS_ERROR("Choose: enter 1 to autonomous navigation, enter 2 to fixed point navigation");
      std::cin >> planner_->exp;
      if (planner_->exp == 1 || planner_->exp == 2) //进入某个模式后禁用进入模式的入口
        planner_->a = 1;
    }

    if (planner_->exp == 2) //定点导航模式
    {

      if (planner_->i == 1) //判断当前是否需要输入目标点
      {
        int i;
        ROS_ERROR("Input the number of point");
        std::cin >> i;
        double point[100];
        for (int j = 0; j < 2 * i; j++)
        {
          ROS_ERROR("Input point coordinates");
          std::cin >> point[j];
        }
        for (int j = 0; j < 2 * i; j++)
        {
          p.push(point[j]);
        }

        if (p.size() != 0)
        {
          planner_->i = 0;
          planner_->b = 1;
        }
      }

      if (planner_->b == 1) //目标点推出容器
      {
        planner_->ax = p.front();
        std::cout << planner_->ax << std::endl;
        p.pop();
        planner_->bx = p.front();
        std::cout << planner_->bx << std::endl;
        p.pop();
        planner_->b = 0;
      }
      geometry_msgs::PoseStamped goalMsg; // 转换为目标点
      goalMsg.pose.position.x = planner_->ax;
      goalMsg.pose.position.y = planner_->bx;
      goalMsg.pose.orientation.w = 1.0;

      planner_->makePlan(pose, goalMsg, res.trajectory.poses); //计算路径

      if (res.trajectory.poses.size() <= 1) //路径太小忽略
      {
        res.trajectory.poses.clear();
      }

      if (res.trajectory.poses.size() == 0)
      {
        planner_->b = 1;
      }

      ROS_ERROR("p.size() = %u", (unsigned int)p.size());

      if (res.trajectory.poses.size() == 0 && p.size() == 0) //没有路径且没有坐标点了则下次调用需要重新输入坐标点
      {
        ROS_ERROR("Completion of task");
        planner_->i = 1;
      }
    }
    else if (planner_->exp == 1)                           //自主导航模式
      planner_->doExploration(pose, res.trajectory.poses); //计算路径

    res.trajectory.header.frame_id = "map";
    res.trajectory.header.stamp = ros::Time::now();
    if (exploration_plan_pub_.getNumSubscribers() > 0)
    {
      exploration_plan_pub_.publish(res.trajectory); //存入路径
    }

    return true;
  }

protected:
  hector_exploration_planner::HectorExplorationPlanner *planner_;
  ros::ServiceServer exploration_plan_service_server_;
  ros::Publisher exploration_plan_pub_;
  costmap_2d::Costmap2DROS *costmap_2d_ros_;
  tf::TransformListener tfl_;
  std::queue<double> p;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationPlanner ep;

  ros::spin();

  return 0;
}
