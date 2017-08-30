//=================================================================================================
// Copyright (c) 2017, Kevin Daun, TU Darmstadt
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

#ifndef HECTOR_GPS_CALIBRATION_H_
#define HECTOR_GPS_CALIBRATION_H_
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "Eigen/Core"



class GPSCalibration
{
public:
  GPSCalibration(ros::NodeHandle &nh);

private:

  void navSatCallback(nav_msgs::Odometry msg);
  void optimizeCallback(std_msgs::Empty msg);
  void optimize();
  void publishTF(const ros::WallTimerEvent &unused_timer_event);


  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  std::vector< Eigen::Matrix<double, 2, 1> > gps_poses_;
  std::vector< Eigen::Matrix<double, 2, 1> > world_poses_;
  std::vector< double > covariances_;

  ros::Subscriber nav_sat_sub_;
  ros::Subscriber optimize_sub_;

  ros::Publisher nav_sat_fix_pub_;

  std::vector<ros::WallTimer> wall_timers_;

  std::array<double, 2> translation_;
  double rotation_;

  bool write_debug_file_;
  double max_covariance_;
  double min_pose_distance_;

};

#endif //HECTOR_GPS_CALIBRATION_H_
