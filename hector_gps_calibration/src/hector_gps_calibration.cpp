#include "hector_gps_calibration/hector_gps_calibration.h"
#include "hector_gps_calibration/rotation_cost_functor.h"

GPSCalibration::GPSCalibration(ros::NodeHandle &nh)
    : tfListener(tfBuffer)
{
    ros::NodeHandle pnh("~/");
    nav_sat_sub_ = nh.subscribe("/odom_gps", 10, &GPSCalibration::navSatCallback, this);
    optimize_sub_ = nh.subscribe("gps/run_optimization", 10, &GPSCalibration::navSatCallback, this);
}

void GPSCalibration::navSatCallback(nav_msgs::Odometry msg)
{
    Eigen::Matrix<double, 3, 1> pos_gps(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    msg.pose.pose.position.z);
    gps_poses_.emplace_back(pos_gps);

    ros::Duration(0.15).sleep(); //todo(kdaun) remove this temp hack

    geometry_msgs::TransformStamped transformStamped;
    try{
     transformStamped = tfBuffer.lookupTransform("world", msg.header.frame_id,
                              msg.header.stamp);
    }
    catch (tf2::TransformException &ex) {
     ROS_WARN("%s",ex.what());
    }

    Eigen::Matrix<double, 3, 1> pos_world(transformStamped.transform.translation.x,
                                    transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.z);

    world_poses_.emplace_back(pos_world);

}

void GPSCalibration::optimizeCallback(std_msgs::Empty msg)
{

}


void GPSCalibration::optimize()
{

}
