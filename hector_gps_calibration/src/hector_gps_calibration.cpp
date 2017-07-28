#include "hector_gps_calibration/hector_gps_calibration.h"
#include "hector_gps_calibration/rotation_cost_functor.h"

#include <ceres/ceres.h>

GPSCalibration::GPSCalibration(ros::NodeHandle &nh)
    : tfListener(tfBuffer)
{
    ros::NodeHandle pnh("~/");
    nav_sat_sub_ = nh.subscribe("/odom_gps", 10, &GPSCalibration::navSatCallback, this);
    optimize_sub_ = nh.subscribe("gps/run_optimization", 10, &GPSCalibration::navSatCallback, this);
    Eigen::Matrix<double, 3, 1> pos_gps(0, 0, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 1, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 2, 10);
    gps_poses_.emplace_back(pos_gps);
    pos_gps = Eigen::Matrix<double, 3, 1>(0, 3, 10);
    gps_poses_.emplace_back(pos_gps);

    Eigen::Matrix<double, 3, 1> pos_world(0, 0, 0);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 1, 0);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 2, 0);
    world_poses_.emplace_back(pos_world);
    pos_world = Eigen::Matrix<double, 3, 1>(0, 3, 0);
    world_poses_.emplace_back(pos_world);


}

void GPSCalibration::navSatCallback(nav_msgs::Odometry msg)
{
    /*
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

    world_poses_.emplace_back(pos_world);*/

}

void GPSCalibration::optimizeCallback(std_msgs::Empty msg)
{

    int i = 0;


    std::array<double, 3> translation{{0,0,0}};
    std::array<double, 4> rotation{{1,0,0,0}};

    ceres::Problem problem;
    problem.SetParameterization(rotation.data(), new ceres::QuaternionParameterization());

    for(i = 0; i < world_poses_.size(); ++i)
    {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor,
                                        3, 4, 3>(
            new TranslationDeltaCostFunctor(world_poses_[i],
                gps_poses_[i])),
        nullptr, translation.data(), rotation.data());
    }


      // Run the solver!
    ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout<<summary.FullReport()<<std::endl;

}


void GPSCalibration::optimize()
{

}
