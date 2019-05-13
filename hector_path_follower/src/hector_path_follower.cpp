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

/*********************************************************************
* Based heavily on the pose_follower package
*********************************************************************/
#include <hector_path_follower/hector_path_follower.h>


namespace pose_follower {
  HectorPathFollower::HectorPathFollower(): tf_(NULL) {}

  void HectorPathFollower::initialize(tf::TransformListener* tf){
    tf_ = tf;
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    ros::NodeHandle node_private("~/");

    node_private.param("k_trans", K_trans_, 2.0);
    node_private.param("k_rot", K_rot_, 2.0);

    node_private.param("tolerance_trans", tolerance_trans_, 0.1);
    node_private.param("tolerance_rot", tolerance_rot_, 0.2);
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

    node_private.param("holonomic", holonomic_, false);

    node_private.param("samples", samples_, 10);

    node_private.param("max_vel_lin", max_vel_lin_, 0.9);
    node_private.param("max_vel_th", max_vel_th_, 1.4);

    node_private.param("min_vel_lin", min_vel_lin_, 0.1);
    node_private.param("min_vel_th", min_vel_th_, 0.0);
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    node_private.param("robot_base_frame", p_robot_base_frame_, std::string("base_link"));
    node_private.param("global_frame", p_global_frame_, std::string("map"));

    //ros::NodeHandle node;
    //vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ROS_DEBUG("Initialized");
  }

  /*
  void HectorPathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }
  */

  double HectorPathFollower::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

  /*
  bool HectorPathFollower::stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
  }
  */

  bool HectorPathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

    if (global_plan_.empty())
      return false;

    //get the current pose of the robot in the fixed frame
    tf::Stamped<tf::Pose> robot_pose;
    if(!this->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }



    //we want to compute a velocity command based on our current waypoint
    tf::Stamped<tf::Pose> target_pose;
    tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

    ROS_DEBUG("HectorPathFollower: current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
    ROS_DEBUG("HectorPathFollower: target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));

    //get the difference between the two poses
    geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
    ROS_DEBUG("HectorPathFollower: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

    geometry_msgs::Twist limit_vel = limitTwist(diff);

    geometry_msgs::Twist test_vel = limit_vel;
    bool legal_traj = true; //collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);

    double scaling_factor = 1.0;
    double ds = scaling_factor / samples_;

    //let's make sure that the velocity command is legal... and if not, scale down
    if(!legal_traj){
      for(int i = 0; i < samples_; ++i){
        test_vel.linear.x = limit_vel.linear.x * scaling_factor;
        test_vel.linear.y = limit_vel.linear.y * scaling_factor;
        test_vel.angular.z = limit_vel.angular.z * scaling_factor;
        test_vel = limitTwist(test_vel);
        //if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
          legal_traj = true;
          break;
        //}
        scaling_factor -= ds;
      }
    }

    if(!legal_traj){
      ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    //if it is legal... we'll pass it on
    cmd_vel = test_vel;

    bool in_goal_position = false;
    while(fabs(diff.linear.x) <= tolerance_trans_ &&
          fabs(diff.linear.y) <= tolerance_trans_ &&
    fabs(diff.angular.z) <= tolerance_rot_)
    {
      if(current_waypoint_ < global_plan_.size() - 1)
      {
  current_waypoint_++;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
        diff = diff2D(target_pose, robot_pose);
      }
      else
      {
        ROS_INFO("Reached goal: %d", current_waypoint_);
        in_goal_position = true;
        break;
      }
    }

    //if we're not in the goal position, we need to update time
    if(!in_goal_position)
      goal_reached_time_ = ros::Time::now();

    //check if we've reached our goal for long enough to succeed
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
    }

    return true;
  }

  bool HectorPathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    if(!transformGlobalPlan(*tf_, global_plan, p_global_frame_, global_plan_)){
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }
    return true;
  }

  bool HectorPathFollower::isGoalReached(){
    /*
    //@TODO: Do something reasonable here
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped()){
      return true;
    }
    */
    return false;
  }

  geometry_msgs::Twist HectorPathFollower::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
  {
    geometry_msgs::Twist res;
    tf::Pose diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());

    if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
      return res;

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading

    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

    //we'll also check if we can move more effectively backwards
    double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
        pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

    //check if its faster to just back up
    if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
      ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
      yaw_diff = neg_yaw_diff;
    }

    //compute the desired quaterion
    tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
    tf::Quaternion rot = pose2.getRotation() * rot_diff;
    tf::Pose new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    return res;
  }


  geometry_msgs::Twist HectorPathFollower::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    if(!holonomic_)
      res.linear.y = 0.0;
    else
      res.linear.y *= K_trans_;
    res.angular.z *= K_rot_;

    //make sure to bound things by our velocity limits
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
    double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
    if (lin_overshoot > 1.0)
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }

    //we only want to enforce a minimum velocity if we're not rotating in place
    if(lin_undershoot > 1.0)
    {
      res.linear.x *= lin_undershoot;
      res.linear.y *= lin_undershoot;
    }

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

    //we want to check for whether or not we're desired to rotate in place
    if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      res.linear.x = 0.0;
      res.linear.y = 0.0;
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

  bool HectorPathFollower::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
      if (global_plan_.empty())
      {
        ROS_ERROR("Received plan with zero length");
        return false;
      }

      tf::StampedTransform transform;
      tf.lookupTransform(global_frame, ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp,
          plan_pose.header.frame_id, transform);

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      for(unsigned int i = 0; i < global_plan.size(); ++i){
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }


  bool HectorPathFollower::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const {

    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = p_robot_base_frame_;
    robot_pose.stamp_ = ros::Time(0);
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    //get the global pose of the robot
    try{
      tf_->transformPose(p_global_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    // check global_pose timeout

    /*
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_WARN_THROTTLE(1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
          current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
      return false;
    }
    */


    return true;
  }
};
