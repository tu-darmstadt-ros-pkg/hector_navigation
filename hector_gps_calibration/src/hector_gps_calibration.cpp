#include "hector_gps_calibration/hector_gps_calibration.h"
#include "hector_gps_calibration/rotation_cost_functor.h"

#include <ceres/ceres.h>

#include <iostream>
#include <fstream>

GPSCalibration::GPSCalibration(ros::NodeHandle &nh)
    : tf_listener(tf_buffer),
      translation_{{0,0,0}},
      rotation_{{1,0,0,0}}
{
    nh.param<double>("translation_x", translation_[0], 0.0);
    nh.param<double>("translation_y", translation_[1], 0.0);
    nh.param<double>("translation_z", translation_[2], 0.0);

    nh.param<double>("quaternion_w", rotation_[0], 1.0);
    nh.param<double>("quaternion_x", rotation_[1], 0.0);
    nh.param<double>("quaternion_y", rotation_[2], 0.0);
    nh.param<double>("quaternion_z", rotation_[3], 0.0);

    nh.param<bool>("write_debug_file",write_debug_file_, false);

    ROS_INFO("Initial GPS transformation: \n t: %f %f %f \n r: %f %f %f %f", translation_[0], translation_[1], translation_[2], rotation_[0], rotation_[1], rotation_[2], rotation_[3]);

    nav_sat_sub_ = nh.subscribe("/odom_gps", 10, &GPSCalibration::navSatCallback, this);
    optimize_sub_ = nh.subscribe("gps/run_optimization", 10, &GPSCalibration::navSatCallback, this);
/*
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
    Eigen::Matrix<double, 3, 1> pos_gps(msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    0);

    geometry_msgs::TransformStamped transformStamped;
    try{
     transformStamped = tf_buffer.lookupTransform("world", "navsat_link" /*msg.header.frame_id*/,
                              msg.header.stamp, ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
     ROS_WARN("%s",ex.what());
     return;
    }

    Eigen::Matrix<double, 3, 1> pos_world(transformStamped.transform.translation.x,
                                    transformStamped.transform.translation.y,
                                    0);

    gps_poses_.emplace_back(pos_gps);
    world_poses_.emplace_back(pos_world);

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
        new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor,
                                        3, 3, 4>(
            new TranslationDeltaCostFunctor(world_poses_[i],
                gps_poses_[i])),
        nullptr, translation_.data(), rotation_.data());
    }
    problem.SetParameterization(rotation_.data(), new ceres::QuaternionParameterization());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //  options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if(summary.termination_type != ceres::TerminationType::CONVERGENCE)
        ROS_WARN("%s", summary.FullReport().c_str());
    ROS_INFO("Translation %f %f %f", translation_[0], translation_[1], translation_[2]);
    ROS_INFO("Rotation %f %f %f %f", rotation_[0], rotation_[1], rotation_[2], rotation_[3]);

    if(write_debug_file_)
    {
        std::ofstream myfile;
        myfile.open ("gps_alignment_solution.csv");
        const Rigid3<double> transform = Rigid3<double>(
            Eigen::Matrix<double, 3, 1>(translation_[0], translation_[1], translation_[2]),
            Eigen::Quaternion<double>(rotation_[0], rotation_[1], rotation_[2],
                                 rotation_[3]));

        myfile <<"gps_x"<<","<<"gps_y"<<","
               <<"gps_z"<<","<<"world_x"<<","
               <<"world_y"<<","<<"world_z"<<"\n";
        for(size_t i = 0; i<gps_poses_.size(); ++i)
        {
            const Eigen::Matrix<double, 3, 1> pos_world_gps = transform * world_poses_[i];
            myfile << std::setprecision (15)<< gps_poses_[i][0]<<","<<gps_poses_[i][1]<<","
                   <<gps_poses_[i][2]<<","<<pos_world_gps[0]<<","
                   <<pos_world_gps[1]<<","<<pos_world_gps[2]<<"\n";
        }
        myfile.close();
    }
}

void GPSCalibration::publishTF(const ::ros::WallTimerEvent& unused_timer_event)
{

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "utm";
    transformStamped.child_frame_id = "world";
    transformStamped.transform.translation.x = translation_[0];
    transformStamped.transform.translation.y = translation_[1];
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.w = rotation_[0];
    transformStamped.transform.rotation.x = rotation_[1];
    transformStamped.transform.rotation.y = rotation_[2];
    transformStamped.transform.rotation.z = rotation_[3];

    tf_broadcaster.sendTransform(transformStamped);

}
