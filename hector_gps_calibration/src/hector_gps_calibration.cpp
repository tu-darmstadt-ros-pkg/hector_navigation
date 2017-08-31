#include "hector_gps_calibration/hector_gps_calibration.h"
#include "hector_gps_calibration/transform_delta_cost_functor.h"
#include "hector_gps_calibration/angle_local_parameterization.h"

#include <ceres/ceres.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <fstream>

GPSCalibration::GPSCalibration(ros::NodeHandle &nh)
  : tf_listener(tf_buffer),
    translation_{{0,0}},
    rotation_(0.0)
{
  nh.param<double>("translation_x", translation_[0], 0.0);
  nh.param<double>("translation_y", translation_[1], 0.0);

  nh.param<double>("orientation", rotation_, 0.0);

  nh.param<bool>("write_debug_file", write_debug_file_, false);
  nh.param<double>("max_covariance", max_covariance_, 10.0);
  nh.param<double>("min_pose_distance", min_pose_distance_, 0.2);

  ROS_INFO("Initial GPS transformation: \n t: %f %f \n r: %f", translation_[0], translation_[1], rotation_);

  nav_sat_sub_ = nh.subscribe("/odom_gps", 1, &GPSCalibration::navSatCallback, this);
  optimize_sub_ = nh.subscribe("gps/run_optimization", 1, &GPSCalibration::navSatCallback, this);

  nav_sat_fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps_calibration/gps/fix", 5);

  /*TEST
    Eigen::Matrix<double, 3, 1> pos_gps(0, 0, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 1, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 2, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 3, 10);
    gps_poses_.emplace_back(pos_gps);

    Eigen::Matrix<double, 3, 1> pos_world(0.1, 0, 0);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 1, 0.1);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 2, 0);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 3, 0);
    world_poses_.emplace_back(pos_world);
    optimize();*/

  wall_timers_.push_back(nh.createWallTimer(ros::WallDuration(0.1), &GPSCalibration::publishTF, this));
}

void GPSCalibration::navSatCallback(nav_msgs::Odometry msg)
{
  msg.header.stamp = ros::Time::now(); //Ugly hack to work around timestamp issues specific to our robot

  if(msg.pose.covariance[0] > max_covariance_ ) {
    //ROS_WARN("Dropping GPS data. Covariance limit exceeded. Covariance: %f > %f", msg.pose.covariance[0], max_covariance_);
    return;
  }

  Eigen::Matrix<double, 2, 1> pos_gps(msg.pose.pose.position.x,
                                      msg.pose.pose.position.y);

  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tf_buffer.lookupTransform("world", "navsat_link",
                                                 msg.header.stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());

    return;
  }

  Eigen::Matrix<double, 2, 1> pos_world(transformStamped.transform.translation.x,
                                        transformStamped.transform.translation.y);
  bool redundant_data = false;
  if(gps_poses_.size() > 0)
  {
    Eigen::Matrix<double, 2, 1> delta_pose = world_poses_[gps_poses_.size() - 1] - pos_world;
    double pose_distance = std::sqrt(delta_pose.transpose() * delta_pose);
    if(pose_distance < min_pose_distance_)
      redundant_data = true;
  }
  if(!redundant_data)
  {
    gps_poses_.emplace_back(pos_gps);
    world_poses_.emplace_back(pos_world);
    covariances_.emplace_back(msg.pose.covariance[0]);
  }

  if((world_poses_.size() % 10 == 0) && world_poses_.size() > 0)
    optimize();

}

void GPSCalibration::optimizeCallback(std_msgs::Empty msg)
{
  optimize();
}


void GPSCalibration::optimize()
{
  int i = 0;

  ceres::Problem problem;

  for(i = 0; i < world_poses_.size(); ++i)
  {
    problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<TransformDeltaCostFunctor,
          2, 2, 1>(
            new TransformDeltaCostFunctor(world_poses_[i],
                                          gps_poses_[i],
                                          covariances_[i])),
          nullptr, translation_.data(), &rotation_);
  }

  ceres::LocalParameterization* angle_local_parameterization =
      ceres::examples::AngleLocalParameterization::Create();

  problem.SetParameterization(&rotation_, angle_local_parameterization);


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  //  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::TerminationType::CONVERGENCE)
    ROS_WARN("%s", summary.FullReport().c_str());
  ROS_INFO("Translation %f %f", translation_[0], translation_[1]);
  ROS_INFO("Rotation %f", rotation_);

  if(write_debug_file_)
  {
    std::ofstream myfile;
    myfile.open ("gps_alignment_solution.csv");
    const Rigid3<double> transform = Rigid3<double>(
          Eigen::Matrix<double, 3, 1>(translation_[0], translation_[1], 0.0),
        Eigen::Quaternion<double>(std::cos(rotation_/2.0), 0.0, 0.0,
                                  std::sin(rotation_/2.0)));

    myfile <<"gps_x"<<","<<"gps_y"<<","
          <<"world_x"<<","<<"world_y"<<","<<"covariance"<<"\n";
    for(size_t i = 0; i<gps_poses_.size(); ++i)
    {
      const Eigen::Matrix<double, 3, 1> pos_world_gps = transform * Eigen::Matrix<double, 3, 1>(world_poses_[i][0], world_poses_[i][1], 0.0);
      myfile << std::setprecision (15)<< gps_poses_[i][0]<<","<<gps_poses_[i][1]<<","
             <<pos_world_gps[0]<<","<<pos_world_gps[1]<<","<<covariances_[i]<<"\n";
    }
    myfile.close();
  }
}

void GPSCalibration::publishTF(const ::ros::WallTimerEvent& unused_timer_event)
{
  ros::Time publish_time = ros::Time::now();

  geometry_msgs::TransformStamped utm_world_transform;
  utm_world_transform.header.stamp = publish_time;
  utm_world_transform.header.frame_id = "utm";
  utm_world_transform.child_frame_id = "world";
  utm_world_transform.transform.translation.x = translation_[0];
  utm_world_transform.transform.translation.y = translation_[1];
  utm_world_transform.transform.translation.z = 0;
  utm_world_transform.transform.rotation.w = std::cos(rotation_/2.0);
  utm_world_transform.transform.rotation.x = 0.0;
  utm_world_transform.transform.rotation.y = 0.0;
  utm_world_transform.transform.rotation.z = std::sin(rotation_/2.0);
  //tf_broadcaster.sendTransform(utm_world_transform);


  geometry_msgs::TransformStamped worldenu_world_transform;
  worldenu_world_transform.header.stamp = publish_time;
  worldenu_world_transform.header.frame_id = "world_enu";
  worldenu_world_transform.child_frame_id = "world";
  worldenu_world_transform.transform.translation.x = 0;
  worldenu_world_transform.transform.translation.y = 0;
  worldenu_world_transform.transform.translation.z = 0;
  worldenu_world_transform.transform.rotation.w = std::cos(rotation_/2.0);
  worldenu_world_transform.transform.rotation.x = 0.0;
  worldenu_world_transform.transform.rotation.y = 0.0;
  worldenu_world_transform.transform.rotation.z = std::sin(rotation_/2.0);
  tf_broadcaster.sendTransform(worldenu_world_transform);

  geometry_msgs::TransformStamped world_navsat_transform;
  try{
    world_navsat_transform = tf_buffer.lookupTransform("world", "base_link",
                                                 publish_time, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }


  geometry_msgs::TransformStamped utm_navsat_transform;
  tf2::doTransform(world_navsat_transform, utm_navsat_transform, utm_world_transform);

  geodesy::UTMPoint utm_point;
  utm_point.band = 'U';
  utm_point.zone = 32;
  utm_point.easting = utm_navsat_transform.transform.translation.x;
  utm_point.northing = utm_navsat_transform.transform.translation.y;
  utm_point.altitude = 0.0;
  geographic_msgs::GeoPoint geo_point = geodesy::toMsg(utm_point);

  sensor_msgs::NavSatFix nav_sat_fix;
  nav_sat_fix.header.stamp = publish_time;
  nav_sat_fix.header.frame_id = "base_link";
  nav_sat_fix.latitude = geo_point.latitude;
  nav_sat_fix.longitude = geo_point.longitude;
  nav_sat_fix.altitude = 0.0;
  nav_sat_fix_pub_.publish(nav_sat_fix);

}
