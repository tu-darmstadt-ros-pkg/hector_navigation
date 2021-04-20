#include "hector_gps_calibration/hector_gps_calibration.h"
#include "hector_gps_calibration/transform_delta_cost_functor.h"
#include "hector_gps_calibration/angle_local_parameterization.h"

#include <ceres/ceres.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>

GPSCalibration::GPSCalibration(ros::NodeHandle &nh)
  : tf_listener(tf_buffer),
    world_to_utm_translation_{{0, 0}},
    initial_translation_{{0,0}},
    world_to_utm_rotation_(0.0),
    initial_rotation_(0.0)
{
  nh.param<double>("translation_x", initial_translation_[0], 0.0);
  nh.param<double>("translation_y", initial_translation_[1], 0.0);
  nh.param<double>("orientation", initial_rotation_, 0.0);
  nh.param<bool>("write_debug_file", write_debug_file_, false);
  nh.param<double>("max_covariance", max_covariance_, 10.0);
  nh.param<double>("min_pose_distance", min_pose_distance_, 0.2);
  nh.param<std::string>("gnss_sensor_frame", gnss_sensor_frame_, "base_link");
  world_to_utm_translation_ = initial_translation_;
  world_to_utm_rotation_ = initial_rotation_;
  nav_sat_sub_ = nh.subscribe("/odom_gps", 1, &GPSCalibration::navSatCallback, this);
  optimize_sub_ = nh.subscribe("gps/run_optimization", 10, &GPSCalibration::navSatCallback, this);
  syscommand_sub_ = nh.subscribe(
      "syscommand", 10, &GPSCalibration::sysCommandCallback, this);
  initialpose_sub_ = nh.subscribe(
      "initialpose", 10, &GPSCalibration::initialPoseCallback, this);
  nav_sat_fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps_calibration/gps/fix", 5);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/gps_calibration/marker", 5);

  wall_timers_.push_back(nh.createWallTimer(ros::WallDuration(0.1), &GPSCalibration::publishTF, this));
}


void GPSCalibration::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  geometry_msgs::TransformStamped transform_world_enu_world;
  geometry_msgs::TransformStamped transform_world_msg;
  geometry_msgs::TransformStamped transform_sensor_world;

  try{
    transform_world_enu_world = tf_buffer.lookupTransform("world_enu", "world", ros::Time(0));
    transform_world_msg = tf_buffer.lookupTransform("world", msg->header.frame_id, ros::Time(0));
    transform_sensor_world = tf_buffer.lookupTransform(gnss_sensor_frame_, "world", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }
  geometry_msgs::TransformStamped transform_msg_correction;
  transform_msg_correction.header = msg->header;
  transform_msg_correction.transform.translation.x = msg->pose.pose.position.x;
  transform_msg_correction.transform.translation.y = msg->pose.pose.position.y;
  transform_msg_correction.transform.translation.z = msg->pose.pose.position.z;
  transform_msg_correction.transform.rotation.w = msg->pose.pose.orientation.w;
  transform_msg_correction.transform.rotation.x = msg->pose.pose.orientation.x;
  transform_msg_correction.transform.rotation.y = msg->pose.pose.orientation.y;
  transform_msg_correction.transform.rotation.z = msg->pose.pose.orientation.z;

  geometry_msgs::TransformStamped transform_utm_world_enu;
  transform_utm_world_enu.header.stamp = msg->header.stamp;
  transform_utm_world_enu.header.frame_id = "utm";
  transform_utm_world_enu.child_frame_id = "world";
  transform_utm_world_enu.transform.translation.x = world_to_utm_translation_[0];
  transform_utm_world_enu.transform.translation.y = world_to_utm_translation_[1];
  transform_utm_world_enu.transform.translation.z = 0.0;
  transform_utm_world_enu.transform.rotation.w = 1.0;
  transform_utm_world_enu.transform.rotation.x = 0.0;
  transform_utm_world_enu.transform.rotation.y = 0.0;
  transform_utm_world_enu.transform.rotation.z = 0.0;

  tf2::Transform t_utm_world_enu;
  fromMsg(transform_utm_world_enu.transform, t_utm_world_enu);

  geometry_msgs::TransformStamped transform_utm_world_corrected;

  tf2::doTransform(transform_world_enu_world, transform_utm_world_corrected, transform_utm_world_enu);
  tf2::doTransform(transform_world_msg, transform_utm_world_corrected, transform_utm_world_corrected);
  tf2::doTransform(transform_msg_correction, transform_utm_world_corrected, transform_utm_world_corrected);
  tf2::doTransform(transform_sensor_world, transform_utm_world_corrected, transform_utm_world_corrected);

//  tf_buffer.clear();
  gps_poses_.clear();
  world_poses_.clear();
  covariances_.clear();
  world_to_utm_translation_ = {transform_utm_world_corrected.transform.translation.x, transform_utm_world_corrected.transform.translation.y};
  world_to_utm_rotation_ = std::acos(transform_utm_world_corrected.transform.rotation.w) * 2.0;
//  gps_poses_.push_back(w)

}
void GPSCalibration::navSatCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(msg->pose.covariance[0] > max_covariance_ ) {
    ROS_DEBUG("Dropping GPS data. Covariance limit exceeded. Covariance: %f > %f", msg->pose.covariance[0], max_covariance_);
    return;
  }
  Eigen::Matrix<double, 2, 1> pos_gps(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y);
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tf_buffer.lookupTransform("world", msg->header.frame_id,
                                                 msg->header.stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Dropping GPS data. %s",ex.what());
    return;
  }

  Eigen::Matrix<double, 2, 1> pos_world(transformStamped.transform.translation.x,
                                        transformStamped.transform.translation.y);
  bool redundant_data = false;
  if(!gps_poses_.empty())
  {
    Eigen::Matrix<double, 2, 1> delta_pose = world_poses_[gps_poses_.size() - 1] - pos_world;
    double pose_distance = std::sqrt(delta_pose.transpose() * delta_pose);
    if(pose_distance < min_pose_distance_) {        
      ROS_DEBUG("Dropping GPS data. Motion limit is not reached. Distance: %f < %f", pose_distance, min_pose_distance_);
      return;
    }
  }

  gps_poses_.emplace_back(pos_gps);
  world_poses_.emplace_back(pos_world);
  covariances_.emplace_back(msg->pose.covariance[0]);
  ROS_DEBUG("Added observation.");
  

  if(world_poses_.size() % 5 == 0) {
    optimize();
  }

}

void GPSCalibration::optimizeCallback(std_msgs::Empty msg)
{
//  optimize();
}


void GPSCalibration::sysCommandCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "reset_cartographer") {
    ROS_INFO("Resetting now due to syscommand.");
    tf_buffer.clear();
    gps_poses_.clear();
    world_poses_.clear();
    covariances_.clear();
    world_to_utm_translation_ = initial_translation_;
    world_to_utm_rotation_ = initial_rotation_;
    ROS_INFO("Finished reset.");
  }
}


void GPSCalibration::optimize()
{
  ceres::Problem problem;
  for(int i = 0; i < world_poses_.size(); ++i)
  {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<TransformDeltaCostFunctor,
          2, 2, 1>(
            new TransformDeltaCostFunctor(world_poses_[i],
                                          gps_poses_[i],
                                          covariances_[i])),
        nullptr, world_to_utm_translation_.data(), &world_to_utm_rotation_);
  }

  ceres::LocalParameterization* angle_local_parameterization =
      ceres::examples::AngleLocalParameterization::Create();

  problem.SetParameterization(&world_to_utm_rotation_, angle_local_parameterization);


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  //  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::TerminationType::CONVERGENCE) {
    ROS_WARN("%s", summary.FullReport().c_str());
    ROS_INFO("Translation %f %f", world_to_utm_translation_[0], world_to_utm_translation_[1]);
    ROS_INFO("Rotation %f", world_to_utm_rotation_);
  }

  if(write_debug_file_) {
    std::ofstream myfile;
    myfile.open ("gps_alignment_solution.csv");
    const Rigid3<double> transform = Rigid3<double>(
          Eigen::Matrix<double, 3, 1>(world_to_utm_translation_[0], world_to_utm_translation_[1], 0.0),
        Eigen::Quaternion<double>(std::cos(world_to_utm_rotation_/2.0), 0.0, 0.0,
                                  std::sin(world_to_utm_rotation_/2.0)));

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

  geometry_msgs::TransformStamped transform_worldenu_world;
  transform_worldenu_world.header.stamp = publish_time;
  transform_worldenu_world.header.frame_id = "world_enu";
  transform_worldenu_world.child_frame_id = "world";
  transform_worldenu_world.transform.translation.x = 0;
  transform_worldenu_world.transform.translation.y = 0;
  transform_worldenu_world.transform.translation.z = 0;
  transform_worldenu_world.transform.rotation.w = std::cos(world_to_utm_rotation_/2.0);
  transform_worldenu_world.transform.rotation.x = 0.0;
  transform_worldenu_world.transform.rotation.y = 0.0;
  transform_worldenu_world.transform.rotation.z = std::sin(world_to_utm_rotation_/2.0);
  tf_broadcaster.sendTransform(transform_worldenu_world);

  geometry_msgs::TransformStamped transform_world_sensor;
  try{
    transform_world_sensor = tf_buffer.lookupTransform("world", gnss_sensor_frame_,
                                                       publish_time, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }

  geometry_msgs::TransformStamped transform_utm_world;
  transform_utm_world.header.stamp = publish_time;
  transform_utm_world.header.frame_id = "utm";
  transform_utm_world.child_frame_id = "world";
  transform_utm_world.transform.translation.x = world_to_utm_translation_[0];
  transform_utm_world.transform.translation.y = world_to_utm_translation_[1];
  transform_utm_world.transform.translation.z = 0;
  transform_utm_world.transform.rotation.w = std::cos(world_to_utm_rotation_/2.0);
  transform_utm_world.transform.rotation.x = 0.0;
  transform_utm_world.transform.rotation.y = 0.0;
  transform_utm_world.transform.rotation.z = std::sin(world_to_utm_rotation_/2.0);
  geometry_msgs::TransformStamped transform_utm_sensor;
  tf2::doTransform(transform_world_sensor, transform_utm_sensor, transform_utm_world);

  geodesy::UTMPoint utm_point_sensor;
  utm_point_sensor.band = 'U';
  utm_point_sensor.zone = 32;
  utm_point_sensor.easting = transform_utm_sensor.transform.translation.x;
  utm_point_sensor.northing = transform_utm_sensor.transform.translation.y;
  utm_point_sensor.altitude = 0.0;
  geographic_msgs::GeoPoint geo_point_sensor = geodesy::toMsg(utm_point_sensor);

  sensor_msgs::NavSatFix nav_sat_fix_sensor;
  nav_sat_fix_sensor.header.stamp = publish_time;
  nav_sat_fix_sensor.header.frame_id = gnss_sensor_frame_;
  nav_sat_fix_sensor.latitude = geo_point_sensor.latitude;
  nav_sat_fix_sensor.longitude = geo_point_sensor.longitude;
  nav_sat_fix_sensor.altitude = 0.0;
  nav_sat_fix_pub_.publish(nav_sat_fix_sensor);



  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "residuals";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.frame_locked = true;

  const Rigid3d transform(
      Eigen::Matrix<double, 3, 1>(world_to_utm_translation_[0], world_to_utm_translation_[1], (0.0)),
      Eigen::Quaterniond(ceres::cos(world_to_utm_rotation_/(2.0)), (0.0), (0.0),
                         ceres::sin(world_to_utm_rotation_/(2.0))));
  const Rigid3d transform_inv = transform.inverse();

  for(int pose_idx = 0; pose_idx < gps_poses_.size(); ++pose_idx) {
    geometry_msgs::Point p1, p2;
    Eigen::Vector3d p1_gps = {gps_poses_[pose_idx][0], gps_poses_[pose_idx][1], 0.0};
    Eigen::Vector3d p1_world = transform_inv * p1_gps;
    p1.x = p1_world[0];
    p1.y = p1_world[1];
    p1.z = 0;
    p2.x = world_poses_[pose_idx][0];
    p2.y = world_poses_[pose_idx][1];
    p2.z = 0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);

    std_msgs::ColorRGBA color;
    const float relative_covariance = covariances_[pose_idx]/max_covariance_;
    const float color_ratio = relative_covariance > 0.f ? std::sqrt(covariances_[pose_idx]/max_covariance_) : 1.0;
    if(color_ratio < 0.5f) {
      color.r = 2.0 * color_ratio;
      color.g = 1.0;
      color.b = 0.0;
      color.a = 1.0;
    }
    else {
      color.r = 1.0;
      color.g = 2.0  - 2.0 * color_ratio;
      color.b = 0.0;
      color.a = 1.0;
    }
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  }
  marker_pub_.publish(marker);

}
