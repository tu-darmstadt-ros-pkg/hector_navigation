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
#include <hector_path_follower/hector_path_follower.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

class SimpleExplorationController
{
public:
  SimpleExplorationController()
  {
    ros::NodeHandle nh;

    exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");

    path_follower_.initialize(&tfl_);

    exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(15.0), &SimpleExplorationController::timerPlanExploration, this, false );
    cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration, this, false );

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  }

  void timerPlanExploration(const ros::TimerEvent& e)
  {
    hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;

    if (exploration_plan_service_client_.call(srv_exploration_plan)){
      ROS_INFO("Generated exploration path with %u poses", (unsigned int)srv_exploration_plan.response.trajectory.poses.size());
      path_follower_.setPlan(srv_exploration_plan.response.trajectory.poses);
    }else{
      ROS_WARN("Service call for exploration service failed");
    }
  }

  void timerCmdVelGeneration(const ros::TimerEvent& e)
  {
    geometry_msgs::Twist twist;
    path_follower_.computeVelocityCommands(twist);
    vel_pub_.publish(twist);
  }


protected:
  ros::ServiceClient exploration_plan_service_client_;
  ros::Publisher vel_pub_;

  tf::TransformListener tfl_;

  pose_follower::HectorPathFollower path_follower_;

  ros::Timer exploration_plan_generation_timer_;
  ros::Timer cmd_vel_generator_timer_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationController ec;

  ros::spin();

  return 0;
}
