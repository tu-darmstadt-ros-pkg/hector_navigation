//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, Florian Berz TU Darmstadt
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

#include <hector_exploration_planner/hector_exploration_planner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <Eigen/Geometry>

#include <hector_exploration_planner/ExplorationPlannerConfig.h>

#define STRAIGHT_COST 100
#define DIAGONAL_COST 141

//#define STRAIGHT_COST 3
//#define DIAGONAL_COST 4

using namespace hector_exploration_planner;

HectorExplorationPlanner::HectorExplorationPlanner()
: costmap_ros_(0)
, costmap_(0)
, initialized_(false)
, map_width_(0)
, map_height_(0)
, num_map_cells_(0)
{}

HectorExplorationPlanner::~HectorExplorationPlanner(){
  this->deleteMapData();
}

HectorExplorationPlanner::HectorExplorationPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in) :
costmap_ros_(NULL), initialized_(false) {
  HectorExplorationPlanner::initialize(name, costmap_ros_in);
}

void HectorExplorationPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in){

  last_mode_ = FRONTIER_EXPLORE;
  // unknown: 255, obstacle 254, inflated: 253, free: 0

  if(initialized_){
    ROS_ERROR("[hector_exploration_planner] HectorExplorationPlanner is already initialized_! Please check why initialize() got called twice.");
    return;
  }

  ROS_INFO("[hector_exploration_planner] Initializing HectorExplorationPlanner");

  // initialize costmaps
  this->costmap_ros_ = costmap_ros_in;
  this->setupMapData();

  // initialize parameters
  ros::NodeHandle private_nh_("~/" + name);
  ros::NodeHandle nh;
  visualization_pub_ = private_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  observation_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("observation_pose", 1, true);
  goal_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);

  dyn_rec_server_.reset(new dynamic_reconfigure::Server<hector_exploration_planner::ExplorationPlannerConfig>(ros::NodeHandle("~/hector_exploration_planner")));

  dyn_rec_server_->setCallback(boost::bind(&HectorExplorationPlanner::dynRecParamCallback, this, _1, _2));

  path_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");

  ROS_INFO("[hector_exploration_planner] Parameter set. security_const: %f, min_obstacle_dist: %d, plan_in_unknown: %d, use_inflated_obstacle: %d, p_goal_angle_penalty_:%d , min_frontier_size: %d, p_dist_for_goal_reached_: %f, same_frontier: %f", p_alpha_, p_min_obstacle_dist_, p_plan_in_unknown_, p_use_inflated_obs_, p_goal_angle_penalty_, p_min_frontier_size_,p_dist_for_goal_reached_,p_same_frontier_dist_);
  //p_min_obstacle_dist_ = p_min_obstacle_dist_ * STRAIGHT_COST;

  this->name = name;
  this->initialized_ = true;
  this->previous_goal_ = -1;

  vis_.reset(new ExplorationTransformVis("exploration_transform"));
  close_path_vis_.reset(new ExplorationTransformVis("close_path_exploration_transform"));
  inner_vis_.reset(new ExplorationTransformVis("inner_exploration_transform"));
  obstacle_vis_.reset(new ExplorationTransformVis("obstacle_transform"));
}

void HectorExplorationPlanner::dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level)
{
  p_plan_in_unknown_ = config.plan_in_unknown;
  p_explore_close_to_path_ = config.explore_close_to_path;
  p_use_inflated_obs_ = config.use_inflated_obstacles;
  p_goal_angle_penalty_ = config.goal_angle_penalty;
  p_alpha_ = config.security_constant;
  p_dist_for_goal_reached_ = config.dist_for_goal_reached;
  p_same_frontier_dist_ = config.same_frontier_distance;
  p_min_frontier_size_ = config.min_frontier_size;
  p_min_obstacle_dist_ = config.min_obstacle_dist * STRAIGHT_COST;
  p_obstacle_cutoff_dist_ = config.obstacle_cutoff_distance;

  p_use_observation_pose_calculation_ = config.use_observation_pose_calculation;
  double angle_rad = config.observation_pose_allowed_angle * (M_PI / 180.0);
  p_cos_of_allowed_observation_pose_angle_ = cos(angle_rad);
  p_close_to_path_target_distance_ = config.close_to_path_target_distance;
}

bool HectorExplorationPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &original_goal, std::vector<geometry_msgs::PoseStamped> &plan){

  this->setupMapData();

  // do exploration? (not used anymore? -> call doExploration())

  if ((original_goal.pose.orientation.w == 0.0) && (original_goal.pose.orientation.x == 0.0) &&
  (original_goal.pose.orientation.y == 0.0) && (original_goal.pose.orientation.z == 0.0)){
      ROS_ERROR("Trying to plan with invalid quaternion, this shouldn't be done anymore, but we'll start exploration for now.");
      return doExploration(start,plan);
  }


  // planning
  ROS_INFO("[hector_exploration_planner] planning: starting to make a plan to given goal point");

  // setup maps and goals
  resetMaps();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  //buildobstacle_trans_array_(p_use_inflated_obs_);

  goal_pose_pub_.publish(original_goal);

  geometry_msgs::PoseStamped adjusted_goal;

  if (p_use_observation_pose_calculation_){
    ROS_INFO("Using observation pose calc.");
    if (!this->getObservationPose(original_goal, 0.5, adjusted_goal)){
      ROS_ERROR("getObservationPose returned false, no area around target point available to drive to!");
      return false;
    }
  }else{
    ROS_INFO("Not using observation pose calc.");
    this->buildobstacle_trans_array_(true);
    adjusted_goal = original_goal;
  }

  observation_pose_pub_.publish(adjusted_goal);

  // plan to given goal
  goals.push_back(adjusted_goal);

  // make plan
  if(!buildexploration_trans_array_(start,goals,true)){
    return false;
  }
  if(!getTrajectory(start,goals,plan)){
    return false;
  }

  // save and add last point
  plan.push_back(adjusted_goal);
  unsigned int mx,my;
  costmap_->worldToMap(adjusted_goal.pose.position.x,adjusted_goal.pose.position.y,mx,my);
  previous_goal_ = costmap_->getIndex(mx,my);

  ROS_INFO("[hector_exploration_planner] planning: plan has been found! plansize: %u ", (unsigned int)plan.size());
  return true;
}

bool HectorExplorationPlanner::doExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){

  ROS_INFO("[hector_exploration_planner] exploration: starting exploration");

  this->setupMapData();

  // setup maps and goals

  resetMaps();
  clearFrontiers();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);


  bool frontiers_found = false;

  if (p_explore_close_to_path_){

    frontiers_found = findFrontiersCloseToPath(goals);

    if (!frontiers_found){
      ROS_WARN("Close Exploration desired, but no frontiers found. Falling back to normal exploration!");
      frontiers_found = findFrontiers(goals);
    }

  }else{
    frontiers_found = findFrontiers(goals);
  }

  // search for frontiers
  if(frontiers_found){

    last_mode_ = FRONTIER_EXPLORE;
    ROS_INFO("[hector_exploration_planner] exploration: found %u frontiers!", (unsigned int)goals.size());
  } else {
    ROS_INFO("[hector_exploration_planner] exploration: no frontiers have been found! starting inner-exploration");
    return doInnerExploration(start,plan);
  }

  // make plan
  if(!buildexploration_trans_array_(start,goals,true)){
    return false;
  }

  if(!getTrajectory(start,goals,plan)){
    ROS_INFO("[hector_exploration_planner] exploration: could not plan to frontier, starting inner-exploration");
    return doInnerExploration(start,plan);
  }

  // update previous goal
  if(!plan.empty()){
    geometry_msgs::PoseStamped thisgoal = plan.back();
    unsigned int mx,my;
    costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
    previous_goal_ = costmap_->getIndex(mx,my);
  }


  ROS_INFO("[hector_exploration_planner] exploration: plan to a frontier has been found! plansize: %u", (unsigned int)plan.size());
  return true;
}

bool HectorExplorationPlanner::doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){
  ROS_INFO("[hector_exploration_planner] inner-exploration: starting exploration");

  // setup maps and goals

  resetMaps();
  clearFrontiers();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);

  // If we have been in inner explore before, check if we have reached the previous inner explore goal
  if (last_mode_ == INNER_EXPLORE){

    tf::Stamped<tf::Pose> robotPose;
    if(!costmap_ros_->getRobotPose(robotPose)){
      ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
    }

    unsigned int xm, ym;
    costmap_->indexToCells(previous_goal_, xm, ym);

    double xw, yw;
    costmap_->mapToWorld(xm, ym, xw, yw);

    double dx = xw - robotPose.getOrigin().getX();
    double dy = yw - robotPose.getOrigin().getY();

    //If we have not  reached the previous goal, try planning and moving toward it.
    //If planning fails, we just continue below this block and try to find another inner frontier
    if ( (dx*dx + dy*dy) > 0.5*0.5){

      geometry_msgs::PoseStamped robotPoseMsg;
      tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

      geometry_msgs::PoseStamped goalMsg;
      goalMsg.pose.position.x = xw;
      goalMsg.pose.position.y = yw;
      goalMsg.pose.orientation.w = 1.0;

      if(makePlan(robotPoseMsg, goalMsg, plan)){
        //Successfully generated plan to (previous) inner explore goal
        ROS_INFO("[hector_exploration_planner] inner-exploration: Planning to previous inner frontier");
        last_mode_ = INNER_EXPLORE;
        return true;
      }
    }
  }

  // search for frontiers
  if(findInnerFrontier(goals)){
    ROS_INFO("[hector_exploration_planner] inner-exploration: found %u inner-frontiers!", (unsigned int)goals.size());
  } else {
    ROS_WARN("[hector_exploration_planner] inner-exploration: no inner-frontiers have been found! exploration failed!");
    return false;
  }

  // make plan
  if(!buildexploration_trans_array_(start,goals,false)){
    ROS_WARN("[hector_exploration_planner] inner-exploration: Creating exploration transform failed!");
    return false;
  }
  if(!getTrajectory(start,goals,plan)){
    ROS_WARN("[hector_exploration_planner] inner-exploration: could not plan to inner-frontier. exploration failed!");
    return false;
  }

  // cutoff last points of plan due to sbpl error when planning close to walls

  int plansize = plan.size() - 5;
  if(plansize > 0 ){
    plan.resize(plansize);
  }

  // update previous goal
  if(!plan.empty()){
    const geometry_msgs::PoseStamped& thisgoal = plan.back();
    unsigned int mx,my;
    costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
    previous_goal_ = costmap_->getIndex(mx,my);
    last_mode_ = INNER_EXPLORE;
  }

  ROS_INFO("[hector_exploration_planner] inner-exploration: plan to an inner-frontier has been found! plansize: %u", (unsigned int)plan.size());
  return true;
}

bool HectorExplorationPlanner::getObservationPose(const geometry_msgs::PoseStamped& observation_pose, const double desired_distance, geometry_msgs::PoseStamped& new_observation_pose)
{
  // We call this from inside the planner, so map data setup and reset already happened
  //this->setupMapData();
  //resetMaps();

  if (!p_use_observation_pose_calculation_){
    ROS_WARN("getObservationPose was called although use_observation_pose_calculation param is set to false. Returning original pose!");
    new_observation_pose = observation_pose;
    this->buildobstacle_trans_array_(true);
    return true;
  }

  unsigned int mxs,mys;
  costmap_->worldToMap(observation_pose.pose.position.x, observation_pose.pose.position.y, mxs, mys);

  double pose_yaw = tf::getYaw(observation_pose.pose.orientation);

  Eigen::Vector2f obs_pose_dir_vec (cos(pose_yaw), sin(pose_yaw));

  this->buildobstacle_trans_array_(true);

  int searchSize = 2.0 / costmap_->getResolution();

  int min_x = mxs - searchSize/2;
  int min_y = mys - searchSize/2;

  if (min_x < 0){
    min_x = 0;
  }

  if (min_y < 0){
    min_y = 0;
  }

  int max_x = mxs + searchSize/2;
  int max_y = mys + searchSize/2;

  if (max_x > static_cast<int>(costmap_->getSizeInCellsX())){
    max_x = static_cast<int>(costmap_->getSizeInCellsX()-1);
  }

  if (max_y > static_cast<int>(costmap_->getSizeInCellsY())){
    max_y = static_cast<int>(costmap_->getSizeInCellsY()-1);
  }

  int closest_x = -1;
  int closest_y = -1;

  unsigned int closest_sqr_dist = UINT_MAX;

  bool no_information = true;

  for (int x = min_x; x < max_x; ++x){
    for (int y = min_y; y < max_y; ++y){

      unsigned int point = costmap_->getIndex(x,y);

      unsigned int obstacle_trans_val = obstacle_trans_array_[point];

      if ( (obstacle_trans_val != UINT_MAX) && (obstacle_trans_val != 0) && (occupancy_grid_array_[point] != costmap_2d::NO_INFORMATION)){

        no_information = false;

        int diff_x = x - (int)mxs;
        int diff_y = y - (int)mys;

        unsigned int sqr_dist = diff_x*diff_x + diff_y*diff_y;

        //std::cout << "diff: " << diff_x << " , " << diff_y << " sqr_dist: " << sqr_dist << " pos: " << x << " , " << y << " closest sqr dist: " << closest_sqr_dist << " obstrans " << obstacle_trans_array_[costmap_->getIndex(x,y)] << "\n";

        if (sqr_dist < closest_sqr_dist){

          Eigen::Vector2f curr_dir_vec(static_cast<float>(diff_x), static_cast<float>(diff_y));
          curr_dir_vec.normalize();

          if (curr_dir_vec.dot(obs_pose_dir_vec) <  p_cos_of_allowed_observation_pose_angle_){

            closest_x = (unsigned int)x;
            closest_y = (unsigned int)y;
            closest_sqr_dist = sqr_dist;
          }
        }
      }
    }
  }

  if (no_information){
    new_observation_pose.pose = observation_pose.pose;
    new_observation_pose.pose.position.z = 0.0;
    ROS_INFO("Observation pose unchanged as no information available around goal area");
    return true;
  }

  //std::cout << "start: " << mxs << " , " << mys << " min: " << min_x << " , " << min_y << " max: " <<  max_x << " , " << max_y << "\n";
  //std::cout << "pos: " << closest_x << " , " << closest_y << "\n";

  // Found valid pose if both coords are larger than -1
  if ((closest_x > -1) && (closest_y > -1)){

    Eigen::Vector2d closest_point_world;
    costmap_->mapToWorld(closest_x, closest_y, closest_point_world.x(),  closest_point_world.y());

    Eigen::Vector2d original_goal_pose(observation_pose.pose.position.x, observation_pose.pose.position.y);

    //geometry_msgs::PoseStamped pose;
    new_observation_pose.header.frame_id = "map";
    new_observation_pose.header.stamp = observation_pose.header.stamp;

    Eigen::Vector2d dir_vec(original_goal_pose - closest_point_world);

    double distance = dir_vec.norm();

    //If we get back the original observation pose (or another one very close to it), return that
    if (distance < (costmap_->getResolution() * 1.5)){
      new_observation_pose.pose = observation_pose.pose;
      new_observation_pose.pose.position.z = 0.0;
      ROS_INFO("Observation pose unchanged");
    }else{

      if (desired_distance < distance){
        new_observation_pose.pose.position.x = closest_point_world.x();
        new_observation_pose.pose.position.y = closest_point_world.y();
        new_observation_pose.pose.position.z = 0.0;
      }else{

        double test_distance = distance;

        Eigen::Vector2d last_valid_pos(closest_point_world);

        do{
          test_distance += 0.1;

          double distance_factor = test_distance / distance;

          Eigen::Vector2d new_pos(original_goal_pose - dir_vec*distance_factor);

          unsigned int x, y;
          costmap_->worldToMap(new_pos[0], new_pos[1], x, y);
          unsigned int idx = costmap_->getIndex(x,y);

          if(!this->isFree(idx)){
            break;
          }

          last_valid_pos = new_pos;

        }while (test_distance < desired_distance);

        new_observation_pose.pose.position.x = last_valid_pos.x();
        new_observation_pose.pose.position.y = last_valid_pos.y();
        new_observation_pose.pose.position.z = 0.0;
      }

      double angle = std::atan2(dir_vec.y(), dir_vec.x());

      new_observation_pose.pose.orientation.w = cos(angle*0.5);
      new_observation_pose.pose.orientation.x = 0.0;
      new_observation_pose.pose.orientation.y = 0.0;
      new_observation_pose.pose.orientation.z = sin(angle*0.5);
      ROS_INFO("Observation pose moved away from wall");
    }

    return true;
  }else{
    // If closest vals are still -1, we didn't find a position
    ROS_ERROR("Couldn't find observation pose for given point.");
    return false;
  }
}

bool HectorExplorationPlanner::doAlternativeExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan){
  ROS_INFO("[hector_exploration_planner] alternative exploration: starting alternative exploration");

  // setup maps and goals
  resetMaps();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  std::vector<geometry_msgs::PoseStamped> old_frontier;
  old_frontier.push_back(oldplan.back());

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);

  // search for frontiers
  if(findFrontiers(goals,old_frontier)){
    ROS_INFO("[hector_exploration_planner] alternative exploration: found %u frontiers!", (unsigned int) goals.size());
  } else {
    ROS_WARN("[hector_exploration_planner] alternative exploration: no frontiers have been found!");
    return false;
  }

  // make plan
  if(!buildexploration_trans_array_(start,goals,true)){
    return false;
  }
  if(!getTrajectory(start,goals,plan)){
    return false;
  }

  const geometry_msgs::PoseStamped& this_goal = plan.back();
  unsigned int mx,my;
  costmap_->worldToMap(this_goal.pose.position.x,this_goal.pose.position.y,mx,my);
  previous_goal_ = costmap_->getIndex(mx,my);

  ROS_INFO("[hector_exploration_planner] alternative exploration: plan to a frontier has been found! plansize: %u ", (unsigned int)plan.size());
  return true;
}

float HectorExplorationPlanner::angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle){
    both_angle = start_angle - goal_angle;
  } else {
    both_angle = goal_angle - start_angle;
  }

  return both_angle;
}

bool HectorExplorationPlanner::exploreWalls(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){

  //@TODO: Properly set this parameters
  int startExploreWall = 1;

  ROS_DEBUG("[hector_exploration_planner] wall-follow: exploreWalls");
  unsigned int mx,my;
  if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)){
    ROS_WARN("[hector_exploration_planner] wall-follow: The start coordinates are outside the costmap!");
    return false;
  }
  int currentPoint=costmap_->getIndex(mx, my);
  int nextPoint;
  int oldDirection = -1;
  int k=0;
  int loop=0;

  while(k<50){
    int adjacentPoints [8];
    getAdjacentPoints(currentPoint, adjacentPoints);
    int dirPoints [3];

    unsigned int minDelta = UINT_MAX;
    unsigned int maxDelta = 0;
    unsigned int thisDelta;
    float minAngle=3.1415; //Rad -> 180°

    geometry_msgs::PoseStamped trajPoint;
    unsigned int gx,gy;

    if(oldDirection==-1){
      // find robot orientation
      for ( int i=0; i<8; i++){
        costmap_->indexToCells((unsigned int)adjacentPoints[i],gx,gy);
        double wx,wy;
        costmap_->mapToWorld(gx,gy,wx,wy);
        std::string global_frame = costmap_ros_->getGlobalFrameID();
        trajPoint.header.frame_id = global_frame;
        trajPoint.pose.position.x = wx;
        trajPoint.pose.position.y = wy;
        trajPoint.pose.position.z = 0.0;
        float yaw = angleDifferenceWall(start, trajPoint);
        if(yaw < minAngle){
          minAngle=yaw;
          oldDirection=i;
        }
      }
    }

    //search possible orientation

    if (oldDirection == 0){
      dirPoints[0]=oldDirection+4; //right-forward
      dirPoints[1]=oldDirection;   //forward
      dirPoints[2]=oldDirection+7; //left-forward
    }
    else if (oldDirection < 3){
      dirPoints[0]=oldDirection+4;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection+3;
    }
    else if (oldDirection == 3){
      dirPoints[0]=oldDirection+4;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection+3;
    }
    else if (oldDirection == 4){
      dirPoints[0]=oldDirection-3;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }
    else if (oldDirection < 7){
      dirPoints[0]=oldDirection-3;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }
    else if (oldDirection == 7){
      dirPoints[0]=oldDirection-7;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }

    // decide LHR or RHR
    if(startExploreWall == -1){
      if(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] <= obstacle_trans_array_[adjacentPoints[dirPoints[2]]]){
        startExploreWall = 0;
        ROS_INFO("[hector_exploration_planner] wall-follow: RHR");//mirror inverted??
      }
      else {
        startExploreWall = 1;
        ROS_INFO("[hector_exploration_planner] wall-follow: LHR");//mirror inverted??
      }
    }



    //switch left and right, LHR
    if(startExploreWall == 1){
      int temp=dirPoints[0];
      dirPoints[0]=dirPoints[2];
      dirPoints[2]=temp;
    }


    // find next point
    int t=0;
    for(int i=0; i<3; i++){
      thisDelta = obstacle_trans_array_[adjacentPoints[dirPoints[i]]];

      if (thisDelta > 3000 || loop > 7) // point is unknown or robot drive loop
      {
        int plansize = plan.size() - 4;
        if(plansize > 0 ){
          plan.resize(plansize);
        }
        ROS_DEBUG("[hector_exploration_planner] wall-follow: END: exploreWalls. Plansize %d", (int)plan.size());
        return !plan.empty();
      }

      if(thisDelta >= (unsigned int) p_min_obstacle_dist_){
        if(obstacle_trans_array_[currentPoint] >= (unsigned int) p_min_obstacle_dist_){
          if(abs(thisDelta - p_min_obstacle_dist_) < minDelta){
            minDelta = abs(thisDelta - p_min_obstacle_dist_);
            nextPoint = adjacentPoints[dirPoints[i]];
            oldDirection = dirPoints[i];
          }
        }
        if(obstacle_trans_array_[currentPoint] < (unsigned int) p_min_obstacle_dist_){
          if(thisDelta > maxDelta){
            maxDelta = thisDelta;
            nextPoint = adjacentPoints[dirPoints[i]];
            oldDirection = dirPoints[i];
          }
        }
      }
      else {
        if(thisDelta < obstacle_trans_array_[currentPoint]){
          t++;
        }
        if(thisDelta > maxDelta){
          maxDelta = thisDelta;
          nextPoint = adjacentPoints[dirPoints[i]];
          oldDirection = dirPoints[i];

        }
      }
    }

    if(t==3 && abs(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] - obstacle_trans_array_[adjacentPoints[dirPoints[1]]]) < STRAIGHT_COST
    && abs(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] - obstacle_trans_array_[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST
    && abs(obstacle_trans_array_[adjacentPoints[dirPoints[1]]] - obstacle_trans_array_[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST){
      nextPoint=adjacentPoints[dirPoints[2]];
      oldDirection=dirPoints[2];
    }

    if(oldDirection==dirPoints[2])
      loop++;
    else
      loop=0;

    // add point
    unsigned int sx,sy;
    costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
    costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
    double wx,wy;
    costmap_->mapToWorld(sx,sy,wx,wy);
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    trajPoint.header.frame_id = global_frame;
    trajPoint.pose.position.x = wx;
    trajPoint.pose.position.y = wy;
    trajPoint.pose.position.z = 0.0;
    // assign orientation
    int dx = gx-sx;
    int dy = gy-sy;
    double yaw_path = std::atan2(dy,dx);
    trajPoint.pose.orientation.x = 0.0;
    trajPoint.pose.orientation.y = 0.0;
    trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
    trajPoint.pose.orientation.w = cos(yaw_path*0.5f);
    plan.push_back(trajPoint);

    currentPoint=nextPoint;
    k++;
  }
  ROS_DEBUG("[hector_exploration_planner] wall-follow: END: exploreWalls. Plansize %d", (int)plan.size());
  return !plan.empty();
}

void HectorExplorationPlanner::setupMapData()
{

#ifdef COSTMAP_2D_LAYERED_COSTMAP_H_
  costmap_ = costmap_ros_->getCostmap();
#else
  if (costmap_) delete costmap_;
  costmap_ = new costmap_2d::Costmap2D;
  costmap_ros_->getCostmapCopy(*costmap_);
#endif

  //Below code can be used to guarantee start pose is cleared. Somewhat risky.
  //@TODO: Make available through dynamic reconfigure
  /*
  std::vector<geometry_msgs::Point> points;
  costmap_ros_->getOrientedFootprint(points);

  bool filled = costmap_->setConvexPolygonCost(points, costmap_2d::FREE_SPACE);

  //std::vector<geometry_msgs::Point> points = costmap_ros_->getRobotFootprint();
  for (size_t i = 0; i < points.size(); ++i)
    std::cout << points[i];
  if (filled)
    ROS_INFO("Set costmap to free");
  else
    ROS_INFO("Failed to set costmap free");
  */

  if ((this->map_width_ != costmap_->getSizeInCellsX()) || (this->map_height_ != costmap_->getSizeInCellsY())){
    map_width_ = costmap_->getSizeInCellsX();
    map_height_ = costmap_->getSizeInCellsY();
    num_map_cells_ = map_width_ * map_height_;

    // initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
    exploration_trans_array_.reset(new unsigned int[num_map_cells_]);
    obstacle_trans_array_.reset(new unsigned int[num_map_cells_]);
    is_goal_array_.reset(new bool[num_map_cells_]);
    frontier_map_array_.reset(new int[num_map_cells_]);
    clearFrontiers();
    resetMaps();
  }

  occupancy_grid_array_ = costmap_->getCharMap();
}

void HectorExplorationPlanner::deleteMapData()
{
  exploration_trans_array_.reset();
  obstacle_trans_array_.reset();
  is_goal_array_.reset();
  frontier_map_array_.reset();
}


bool HectorExplorationPlanner::buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty, bool use_cell_danger){

  ROS_DEBUG("[hector_exploration_planner] buildexploration_trans_array_");

  // reset exploration transform
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);

  std::queue<int> myqueue;

  size_t num_free_goals = 0;

  // initialize goals
  for(unsigned int i = 0; i < goals.size(); ++i){
    // setup goal positions
    unsigned int mx,my;

    if(!costmap_->worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my)){
      //ROS_WARN("[hector_exploration_planner] The goal coordinates are outside the costmap!");
      continue;
    }

    int goal_point = costmap_->getIndex(mx,my);

    // Ignore free goal for the moment, check after iterating over all goals if there is not valid one at all
    if(!isFree(goal_point)){
      continue;
    }else{
      ++num_free_goals;
    }

    unsigned int init_cost = 0;
    if(false){
      init_cost = angleDanger(angleDifference(start,goals[i])) * getDistanceWeight(start,goals[i]);
    }

    exploration_trans_array_[goal_point] = init_cost;

    // do not punish previous frontiers (oscillation)
    if(false && isValid(previous_goal_)){
      if(isSameFrontier(goal_point, previous_goal_)){
        ROS_DEBUG("[hector_exploration_planner] same frontier: init with 0");
        exploration_trans_array_[goal_point] = 0;
      }
    }

    ROS_DEBUG("[hector_exploration_planner] Goal init cost: %d, point: %d", exploration_trans_array_[goal_point], goal_point);
    is_goal_array_[goal_point] = true;
    myqueue.push(goal_point);
  }

  if (num_free_goals == 0){
    ROS_WARN("[hector_exploration_planner] All goal coordinates for exploration transform invalid (occupied or out of bounds), aborting.");
    return false;
  }

  // exploration transform algorithm
  if (use_cell_danger){
    while(myqueue.size()){
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) {
        if (isFree(straightPoints[i])) {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST + cellDanger(straightPoints[i]);

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
            myqueue.push(straightPoints[i]);
          }
        }

        if (isFree(diagonalPoints[i])) {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST + cellDanger(diagonalPoints[i]);

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            myqueue.push(diagonalPoints[i]);
          }
        }
      }
    }
  }else{
    while(myqueue.size()){
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) {
        if (isFree(straightPoints[i])) {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST;

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
            myqueue.push(straightPoints[i]);
          }
        }

        if (isFree(diagonalPoints[i])) {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST;

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            myqueue.push(diagonalPoints[i]);
          }
        }
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildexploration_trans_array_");

  vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());
  return true;
}

bool HectorExplorationPlanner::buildobstacle_trans_array_(bool use_inflated_obstacles){
  ROS_DEBUG("[hector_exploration_planner] buildobstacle_trans_array_");
  std::queue<int> myqueue;

  // init obstacles
  for(unsigned int i=0; i < num_map_cells_; ++i){
    if(occupancy_grid_array_[i] == costmap_2d::LETHAL_OBSTACLE){
      myqueue.push(i);
      obstacle_trans_array_[i] = 0;
    } else if(use_inflated_obstacles){
      if(occupancy_grid_array_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        myqueue.push(i);
        obstacle_trans_array_[i] = 0;
      }
    }
  }

  unsigned int obstacle_cutoff_value = static_cast<unsigned int>((p_obstacle_cutoff_dist_ / costmap_->getResolution()) * STRAIGHT_COST + 0.5);

  // obstacle transform algorithm
  while(myqueue.size()){
    int point = myqueue.front();
    myqueue.pop();

    unsigned int minimum = obstacle_trans_array_[point];
    if (minimum > obstacle_cutoff_value) continue;

    int straightPoints[4];
    getStraightPoints(point,straightPoints);
    int diagonalPoints[4];
    getDiagonalPoints(point,diagonalPoints);

    // check all 8 directions
    for(int i = 0; i < 4; ++i){
      if (isValid(straightPoints[i]) && (obstacle_trans_array_[straightPoints[i]] > minimum + STRAIGHT_COST)) {
        obstacle_trans_array_[straightPoints[i]] = minimum + STRAIGHT_COST;
        myqueue.push(straightPoints[i]);
      }
      if (isValid(diagonalPoints[i]) && (obstacle_trans_array_[diagonalPoints[i]] > minimum + DIAGONAL_COST)) {
        obstacle_trans_array_[diagonalPoints[i]] = minimum + DIAGONAL_COST;
        myqueue.push(diagonalPoints[i]);
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildobstacle_trans_array_");

  obstacle_vis_->publishVisOnDemand(*costmap_, obstacle_trans_array_.get());
  return true;
}

bool HectorExplorationPlanner::getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan){

  ROS_DEBUG("[hector_exploration_planner] getTrajectory");

  // setup start positions
  unsigned int mx,my;

  if(!costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my)){
    ROS_WARN("[hector_exploration_planner] The start coordinates are outside the costmap!");
    return false;
  }

  int currentPoint = costmap_->getIndex(mx,my);
  int nextPoint = currentPoint;



  geometry_msgs::PoseStamped trajPoint;
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  trajPoint.header.frame_id = global_frame;

  if (is_goal_array_[currentPoint]){
    ROS_INFO("Already at goal point position. No pose vector generated.");
    return true;
  }

  while(!is_goal_array_[currentPoint]){
    int thisDelta;
    int adjacentPoints[8];
    getAdjacentPoints(currentPoint,adjacentPoints);

    int maxDelta = 0;

    for(int i = 0; i < 8; ++i){
      if(isFree(adjacentPoints[i])){
        thisDelta = exploration_trans_array_[currentPoint] - exploration_trans_array_[adjacentPoints[i]];
        if(thisDelta > maxDelta){
          maxDelta = thisDelta;
          nextPoint = adjacentPoints[i];
        }
      }
    }

    // This happens when there is no valid exploration transform data at the start point for example
    if(maxDelta == 0){
      ROS_WARN("[hector_exploration_planner] No path to the goal could be found by following gradient!");
      return false;
    }


    // make trajectory point
    unsigned int sx,sy,gx,gy;
    costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
    costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
    double wx,wy;
    costmap_->mapToWorld(sx,sy,wx,wy);

    trajPoint.pose.position.x = wx;
    trajPoint.pose.position.y = wy;
    trajPoint.pose.position.z = 0.0;

    // assign orientation
    int dx = gx-sx;
    int dy = gy-sy;
    double yaw_path = std::atan2(dy,dx);
    trajPoint.pose.orientation.x = 0.0;
    trajPoint.pose.orientation.y = 0.0;
    trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
    trajPoint.pose.orientation.w = cos(yaw_path*0.5f);

    plan.push_back(trajPoint);

    currentPoint = nextPoint;
    maxDelta = 0;
  }

  ROS_DEBUG("[hector_exploration_planner] END: getTrajectory. Plansize %u", (unsigned int)plan.size());
  return !plan.empty();
}

bool HectorExplorationPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers){
  std::vector<geometry_msgs::PoseStamped> empty_vec;
  return findFrontiers(frontiers,empty_vec);
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
bool HectorExplorationPlanner::findFrontiersCloseToPath(std::vector<geometry_msgs::PoseStamped> &frontiers){

  clearFrontiers();
  frontiers.clear();

  // get the trajectory as seeds for the exploration transform
  hector_nav_msgs::GetRobotTrajectory srv_path;
  if (path_service_client_.call(srv_path)){

    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

    // We push poses of the travelled trajectory to the goals vector for building the exploration transform
    std::vector<geometry_msgs::PoseStamped> goals;

    size_t size = traj_vector.size();
    ROS_INFO("[hector_exploration_planner] Size of trajectory vector for close exploration %u", (unsigned int)size);

    if(size > 0){
      geometry_msgs::PoseStamped lastPose = traj_vector[size-1];
      goals.push_back(lastPose);

      if (size > 1){

        for(int i = static_cast<int>(size-2); i >= 0; --i){
          const geometry_msgs::PoseStamped& pose = traj_vector[i];
          unsigned int x,y;
          costmap_->worldToMap(pose.pose.position.x,pose.pose.position.y,x,y);
          unsigned int m_point = costmap_->getIndex(x,y);

          double dx = lastPose.pose.position.x - pose.pose.position.x;
          double dy = lastPose.pose.position.y - pose.pose.position.y;

          if((dx*dx) + (dy*dy) > (0.25*0.25)){
            goals.push_back(pose);
            lastPose = pose;
          }
        }

        ROS_INFO("[hector_exploration_planner] pushed %u goals (trajectory) for close to robot frontier search", (unsigned int)goals.size());

        // make exploration transform
        tf::Stamped<tf::Pose> robotPose;
        if(!costmap_ros_->getRobotPose(robotPose)){
          ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
        }
        geometry_msgs::PoseStamped robotPoseMsg;
        tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

        if (!buildexploration_trans_array_(robotPoseMsg, goals, false, false)){
          ROS_WARN("[hector_exploration_planner]: Creating exploration transform array in find inner frontier failed, aborting.");
          return false;
        }

        close_path_vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());

        unsigned int explore_threshold = static_cast<unsigned int> (static_cast<double>(STRAIGHT_COST) * (1.0/costmap_->getResolution()) * p_close_to_path_target_distance_);

        //std::vector<geometry_msgs::PoseStamped> close_frontiers;

        for(unsigned int i = 0; i < num_map_cells_; ++i){

          unsigned int current_val = exploration_trans_array_[i];

          if(current_val < UINT_MAX){

            if (current_val >= explore_threshold){ //&& current_val <= explore_threshold+ DIAGONAL_COST){
              geometry_msgs::PoseStamped finalFrontier;
              double wx,wy;
              unsigned int mx,my;
              costmap_->indexToCells(i, mx, my);
              costmap_->mapToWorld(mx,my,wx,wy);
              std::string global_frame = costmap_ros_->getGlobalFrameID();
              finalFrontier.header.frame_id = global_frame;
              finalFrontier.pose.position.x = wx;
              finalFrontier.pose.position.y = wy;
              finalFrontier.pose.position.z = 0.0;

              double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

              //if(frontier_is_valid){

              finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

              frontiers.push_back(finalFrontier);

            }

          }
        }

        return frontiers.size() > 0;





      }
    }
  }





  // list of all frontiers in the occupancy grid
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < num_map_cells_; ++i){
    if(isFrontier(i)){
      allFrontiers.push_back(i);
    }
  }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i){
    if(!isFrontierReached(allFrontiers[i])){
      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      unsigned int mx,my;
      costmap_->indexToCells(allFrontiers[i], mx, my);
      costmap_->mapToWorld(mx,my,wx,wy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

      //if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      frontiers.push_back(finalFrontier);
    }
    //}
  }

  return (frontiers.size() > 0);
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
bool HectorExplorationPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers){

  // get latest costmap
  clearFrontiers();

  // list of all frontiers in the occupancy grid
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < num_map_cells_; ++i){
    if(isFrontier(i)){
      allFrontiers.push_back(i);
    }
  }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i){
    if(!isFrontierReached(allFrontiers[i])){
      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      unsigned int mx,my;
      costmap_->indexToCells(allFrontiers[i], mx, my);
      costmap_->mapToWorld(mx,my,wx,wy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

      //if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      frontiers.push_back(finalFrontier);
    }
    //}
  }

  return (frontiers.size() > 0);

  //@TODO: Review and possibly remove unused code below

  // value of the next blob
  int nextBlobValue = 1;
  std::list<int> usedBlobs;

  for(unsigned int i = 0; i < allFrontiers.size(); ++i){

    // get all adjacent blobs to the current frontier point
    int currentPoint = allFrontiers[i];
    int adjacentPoints[8];
    getAdjacentPoints(currentPoint,adjacentPoints);

    std::list<int> blobs;

    for(int j = 0; j < 8; j++){
      if(isValid(adjacentPoints[j]) && (frontier_map_array_[adjacentPoints[j]] > 0)){
        blobs.push_back(frontier_map_array_[adjacentPoints[j]]);
      }
    }
    blobs.unique();

    if(blobs.empty()){
      // create new blob
      frontier_map_array_[currentPoint] = nextBlobValue;
      usedBlobs.push_back(nextBlobValue);
      nextBlobValue++;
    } else {
      // merge all found blobs
      int blobMergeVal = 0;

      for(std::list<int>::iterator adjBlob = blobs.begin(); adjBlob != blobs.end(); ++adjBlob){
        if(adjBlob == blobs.begin()){
          blobMergeVal = *adjBlob;
          frontier_map_array_[currentPoint] = blobMergeVal;
        } else {

          for(unsigned int k = 0; k < allFrontiers.size(); k++){
            if(frontier_map_array_[allFrontiers[k]] == *adjBlob){
              usedBlobs.remove(*adjBlob);
              frontier_map_array_[allFrontiers[k]] = blobMergeVal;
            }
          }
        }
      }
    }
  }

  int id = 1;

  bool visualization_requested = (visualization_pub_.getNumSubscribers() > 0);

  // summarize every blob into a single point (maximum obstacle_trans_array_ value)
  for(std::list<int>::iterator currentBlob = usedBlobs.begin(); currentBlob != usedBlobs.end(); ++currentBlob){
    int current_frontier_size = 0;
    int max_obs_idx = 0;

    for(unsigned int i = 0; i < allFrontiers.size(); ++i){
      int point = allFrontiers[i];

      if(frontier_map_array_[point] == *currentBlob){
        current_frontier_size++;
        if(obstacle_trans_array_[point] > obstacle_trans_array_[allFrontiers[max_obs_idx]]){
          max_obs_idx = i;
        }
      }
    }

    if(current_frontier_size < p_min_frontier_size_){
      continue;
    }

    int frontier_point = allFrontiers[max_obs_idx];
    unsigned int x,y;
    costmap_->indexToCells(frontier_point,x,y);

    // check if frontier is valid (not to close to robot and not in noFrontiers vector
    bool frontier_is_valid = true;

    if(isFrontierReached(frontier_point)){
      frontier_is_valid = false;
    }

    for(size_t i = 0; i < noFrontiers.size(); ++i){
      const geometry_msgs::PoseStamped& noFrontier = noFrontiers[i];
      unsigned int mx,my;
      costmap_->worldToMap(noFrontier.pose.position.x,noFrontier.pose.position.y,mx,my);
      int no_frontier_point = costmap_->getIndex(x,y);
      if(isSameFrontier(frontier_point,no_frontier_point)){
        frontier_is_valid = false;
      }
    }

    geometry_msgs::PoseStamped finalFrontier;
    double wx,wy;
    costmap_->mapToWorld(x,y,wx,wy);
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    finalFrontier.header.frame_id = global_frame;
    finalFrontier.pose.position.x = wx;
    finalFrontier.pose.position.y = wy;
    finalFrontier.pose.position.z = 0.0;

    double yaw = getYawToUnknown(costmap_->getIndex(x,y));

    if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      frontiers.push_back(finalFrontier);
    }

    // visualization (export to method?)
    if(visualization_requested){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "hector_exploration_planner";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.position.z = 0.0;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;

      if(frontier_is_valid){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
      }else{
        marker.color.r = 1.0;
        marker.color.g = 0.0;
      }

      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(5,0);
      visualization_pub_.publish(marker);
    }

  }
  return !frontiers.empty();
}

bool HectorExplorationPlanner::findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier){
  clearFrontiers();

  // get the trajectory as seeds for the exploration transform
  hector_nav_msgs::GetRobotTrajectory srv_path;
  if (path_service_client_.call(srv_path)){

    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

    // We push poses of the travelled trajectory to the goals vector for building the exploration transform
    std::vector<geometry_msgs::PoseStamped> goals;

    size_t size = traj_vector.size();
    ROS_DEBUG("[hector_exploration_planner] size of trajectory vector %u", (unsigned int)size);

    if(size > 0){
      geometry_msgs::PoseStamped lastPose = traj_vector[size-1];
      goals.push_back(lastPose);

      if (size > 1){

        // Allow collision at start in case vehicle is (very) close to wall
        bool collision_allowed = true;

        for(int i = static_cast<int>(size-2); i >= 0; --i){
          const geometry_msgs::PoseStamped& pose = traj_vector[i];
          unsigned int x,y;
          costmap_->worldToMap(pose.pose.position.x,pose.pose.position.y,x,y);
          unsigned int m_point = costmap_->getIndex(x,y);

          double dx = lastPose.pose.position.x - pose.pose.position.x;
          double dy = lastPose.pose.position.y - pose.pose.position.y;

          bool point_in_free_space = isFreeFrontiers(m_point);

          // extract points with 0.5m distance (if free)
          if(point_in_free_space){
            if((dx*dx) + (dy*dy) > (0.25*0.25)){
              goals.push_back(pose);
              lastPose = pose;
              collision_allowed = false;
            }
          }

          if (!point_in_free_space && !collision_allowed){
            break;
          }
        }
      }


      ROS_DEBUG("[hector_exploration_planner] pushed %u goals (trajectory) for inner frontier-search", (unsigned int)goals.size());

      // make exploration transform
      tf::Stamped<tf::Pose> robotPose;
      if(!costmap_ros_->getRobotPose(robotPose)){
        ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
      }
      geometry_msgs::PoseStamped robotPoseMsg;
      tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

      if (!buildexploration_trans_array_(robotPoseMsg, goals, false)){
        ROS_WARN("[hector_exploration_planner]: Creating exploration transform array in find inner frontier failed, aborting.");
        return false;
      }

      inner_vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());

      unsigned int x,y;
      costmap_->worldToMap(robotPoseMsg.pose.position.x,robotPoseMsg.pose.position.y,x,y);




      // get point with maximal distance to trajectory
      int max_exploration_trans_point = -1;
      unsigned int max_exploration_trans_val = 0;

      for(unsigned int i = 0; i < num_map_cells_; ++i){

        if(exploration_trans_array_[i] < UINT_MAX){
          if(exploration_trans_array_[i] > max_exploration_trans_val){
            if(!isFrontierReached(i)){
              max_exploration_trans_point = i;
              max_exploration_trans_val = exploration_trans_array_[i];
            }
          }
        }
      }

      if (max_exploration_trans_point == 0){
        ROS_WARN("[hector_exploration_planner]: Couldn't find max exploration transform point for inner exploration, aborting.");
        return false;
      }

      geometry_msgs::PoseStamped finalFrontier;
      unsigned int fx,fy;
      double wfx,wfy;
      costmap_->indexToCells(max_exploration_trans_point,fx,fy);
      costmap_->mapToWorld(fx,fy,wfx,wfy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wfx;
      finalFrontier.pose.position.y = wfy;
      finalFrontier.pose.position.z = 0.0;

      // assign orientation
      int dx = fx-x;
      int dy = fy-y;
      double yaw_path = std::atan2(dy,dx);
      finalFrontier.pose.orientation.x = 0.0;
      finalFrontier.pose.orientation.y = 0.0;
      finalFrontier.pose.orientation.z = sin(yaw_path*0.5f);
      finalFrontier.pose.orientation.w = cos(yaw_path*0.5f);

      innerFrontier.push_back(finalFrontier);

      if(visualization_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "hector_exploration_planner";
        marker.id = 100;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = wfx;
        marker.pose.position.y = wfy;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_path);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(5,0);
        visualization_pub_.publish(marker);
      }
      return true;
    }
  }
  return false;
}

/*
 * checks if a given point is a frontier cell. a frontier cell is a cell in the occupancy grid
 * that seperates known from unknown space. Therefore the cell has to be free but at least three
 * of its neighbours need to be unknown
 */
bool HectorExplorationPlanner::isFrontier(int point){
  if(isFreeFrontiers(point)){

    int adjacentPoints[8];
    getAdjacentPoints(point,adjacentPoints);

    for(int i = 0; i < 8; ++i){
      if(isValid(adjacentPoints[i])){
        if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){

          int no_inf_count = 0;
          int noInfPoints[8];
          getAdjacentPoints(adjacentPoints[i],noInfPoints);
          for(int j = 0; j < 8; j++){


            if( isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == costmap_2d::NO_INFORMATION){
              ++no_inf_count;

              if(no_inf_count > 2){
                return true;
              }
            }
          }
        }
      }
    }
  }

  return false;
}


void HectorExplorationPlanner::resetMaps(){
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
}

void HectorExplorationPlanner::clearFrontiers(){
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
}

inline bool HectorExplorationPlanner::isValid(int point){
  return (point>=0);
}

bool HectorExplorationPlanner::isFree(int point){

  if(isValid(point)){
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free


    if(p_use_inflated_obs_){
      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    } else {
      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    }

    if(p_plan_in_unknown_){
      if(occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION){
        return true;
      }
    }
  }
  return false;
}

bool HectorExplorationPlanner::isFreeFrontiers(int point){

  if(isValid(point)){
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free


    if(p_use_inflated_obs_){
      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    } else {
      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    }
  }
  return false;
}

bool HectorExplorationPlanner::isFrontierReached(int point){

  tf::Stamped<tf::Pose> robotPose;
  if(!costmap_ros_->getRobotPose(robotPose)){
    ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  unsigned int fx,fy;
  double wfx,wfy;
  costmap_->indexToCells(point,fx,fy);
  costmap_->mapToWorld(fx,fy,wfx,wfy);


  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached_*p_dist_for_goal_reached_)) {
    ROS_DEBUG("[hector_exploration_planner]: frontier is within the squared range of: %f", p_dist_for_goal_reached_);
    return true;
  }
  return false;

}

bool HectorExplorationPlanner::isSameFrontier(int frontier_point1, int frontier_point2){
  unsigned int fx1,fy1;
  unsigned int fx2,fy2;
  double wfx1,wfy1;
  double wfx2,wfy2;
  costmap_->indexToCells(frontier_point1,fx1,fy1);
  costmap_->indexToCells(frontier_point2,fx2,fy2);
  costmap_->mapToWorld(fx1,fy1,wfx1,wfy1);
  costmap_->mapToWorld(fx2,fy2,wfx2,wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if((dx*dx) + (dy*dy) < (p_same_frontier_dist_*p_same_frontier_dist_)){
    return true;
  }
  return false;
}

inline unsigned int HectorExplorationPlanner::cellDanger(int point){

  if ((int)obstacle_trans_array_[point] <= p_min_obstacle_dist_){
    return static_cast<unsigned int>(p_alpha_ * std::pow(p_min_obstacle_dist_ - obstacle_trans_array_[point], 2) + .5);
  }
  //ROS_INFO("%d", (int)obstacle_trans_array_[point] );
  //return 80000u - std::min(80000u, obstacle_trans_array_[point]*40);

  //return (2000u - std::min(2000u, obstacle_trans_array_[point])) / 500u;
  //std::cout << obstacle_trans_array_[point] << "\n";

  return 0;
}

float HectorExplorationPlanner::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle){
    both_angle = start_angle - goal_angle;
  } else {
    both_angle = goal_angle - start_angle;
  }

  if(both_angle > M_PI){
    both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
  }

  return both_angle;
}

// Used to generate direction for frontiers
double HectorExplorationPlanner::getYawToUnknown(int point){
  int adjacentPoints[8];
  getAdjacentPoints(point,adjacentPoints);

  int max_obs_idx = 0;
  unsigned int max_obs_dist = 0;

  for(int i = 0; i < 8; ++i){
    if(isValid(adjacentPoints[i])){
      if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){
        if(obstacle_trans_array_[adjacentPoints[i]] > max_obs_dist){
          max_obs_idx = i;
          max_obs_dist = obstacle_trans_array_[adjacentPoints[i]];
        }
      }
    }
  }

  int orientationPoint = adjacentPoints[max_obs_idx];
  unsigned int sx,sy,gx,gy;
  costmap_->indexToCells((unsigned int)point,sx,sy);
  costmap_->indexToCells((unsigned int)orientationPoint,gx,gy);
  int x = gx-sx;
  int y = gy-sy;
  double yaw = std::atan2(y,x);

  return yaw;

}

unsigned int HectorExplorationPlanner::angleDanger(float angle){
  float angle_fraction = std::pow(angle,3);///M_PI;
  unsigned int result = static_cast<unsigned int>(p_goal_angle_penalty_ * angle_fraction);
  return result;
}

float HectorExplorationPlanner::getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2){
  float distance = std::sqrt(std::pow(point1.pose.position.x - point2.pose.position.x,2) + std::pow(point1.pose.position.y - point2.pose.position.y,2));
  if(distance < 0.5){
    return 5.0;
  } else {
    return 1;
  }
}

/*
 These functions calculate the index of an adjacent point (left,upleft,up,upright,right,downright,down,downleft) to the
 given point. If there is no such point (meaning the point would cause the index to be out of bounds), -1 is returned.
 */
inline void HectorExplorationPlanner::getStraightPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);

}

inline void HectorExplorationPlanner::getDiagonalPoints(int point, int points[]){

  points[0] = upleft(point);
  points[1] = upright(point);
  points[2] = downright(point);
  points[3] = downleft(point);

}

/*
inline void HectorExplorationPlanner::getStraightAndDiagonalPoints(int point, int straight_points[], int diag_points[]){
  /
  // Can go up if index is larger than width
  bool up = (point >= (int)map_width_);

  // Can go down if
  bool down = ((point/map_width_) < (map_width_-1));


  bool right = ((point + 1) % map_width_ != 0);
  bool left = ((point % map_width_ != 0));

}
*/

inline void HectorExplorationPlanner::getAdjacentPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);

}

inline int HectorExplorationPlanner::left(int point){
  // only go left if no index error and if current point is not already on the left boundary
  if((point % map_width_ != 0)){
    return point-1;
  }
  return -1;
}
inline int HectorExplorationPlanner::upleft(int point){
  if((point % map_width_ != 0) && (point >= (int)map_width_)){
    return point-1-map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::up(int point){
  if(point >= (int)map_width_){
    return point-map_width_;
  }
  return -1;
}
inline int HectorExplorationPlanner::upright(int point){
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0)){
    return point-map_width_+1;
  }
  return -1;
}
inline int HectorExplorationPlanner::right(int point){
  if((point + 1) % map_width_ != 0){
    return point+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::downright(int point){
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1))){
    return point+map_width_+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::down(int point){
  if((point/map_width_) < (map_height_-1)){
    return point+map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::downleft(int point){
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0)){
    return point+map_width_-1;
  }
  return -1;

}

//        // visualization (export to another method?)
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = "map";
//        marker.header.stamp = ros::Time();
//        marker.ns = "hector_exploration_planner";
//        marker.id = i + 500;
//        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//        marker.action = visualization_msgs::Marker::ADD;
//        marker.pose.position = goals[i].pose.position;
//        marker.scale.x = 0.2;
//        marker.scale.y = 0.2;
//        marker.scale.z = 0.2;
//        marker.color.a = 1.0;
//        marker.color.r = 0.0;
//        marker.color.g = 0.0;
//        marker.color.b = 1.0;
//        marker.lifetime = ros::Duration(5,0);
//        marker.text = boost::lexical_cast<std::string>((int)init_cost) + " - " + boost::lexical_cast<std::string>(getDistanceWeight(start,goals[i]));
//        visualization_pub_.publish(marker);

//void HectorExplorationPlanner::saveMaps(std::string path){

//    char costmapPath[1000];
//    sprintf(costmapPath,"%s.map",path.data());
//    char explorationPath[1000];
//    sprintf(explorationPath,"%s.expl",path.data());
//    char obstaclePath[1000];
//    sprintf(obstaclePath,"%s.obs",path.data());
//    char frontierPath[1000];
//    sprintf(frontierPath,"%s.front",path.data());


//    costmap.saveMap(costmapPath);
//    FILE *fp_expl = fopen(explorationPath,"w");
//    FILE *fp_obs = fopen(obstaclePath,"w");
//    FILE *fp_front = fopen(frontierPath,"w");

//    if (!fp_expl || !fp_obs || !fp_front)
//    {
//        ROS_WARN("[hector_exploration_planner] Cannot save maps");
//        return;
//    }

//    for(unsigned int y = 0; y < map_height_; ++y){
//        for(unsigned int x = 0;x < map_width_; ++x){
//            unsigned int expl = exploration_trans_array_[costmap.getIndex(x,y)];
//            unsigned int obs = obstacle_trans_array_[costmap.getIndex(x,y)];
//            int blobVal = frontier_map_array_[costmap.getIndex(x,y)];
//            fprintf(fp_front,"%d\t", blobVal);
//            fprintf(fp_expl,"%d\t", expl);
//            fprintf(fp_obs, "%d\t", obs);
//        }
//        fprintf(fp_expl,"\n");
//        fprintf(fp_obs,"\n");
//        fprintf(fp_front,"\n");
//    }

//    fclose(fp_expl);
//    fclose(fp_obs);
//    fclose(fp_front);
//    ROS_INFO("[hector_exploration_planner] Maps have been saved!");
//    return;

//}

//    // add last point to path (goal point)
//    for(unsigned int i = 0; i < goals.size(); ++i){
//        unsigned int mx,my;
//        costmap.worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my);

//        if(currentPoint == (int)costmap.getIndex(mx,my)){
//            plan.push_back(goals[i]);
//            previous_goal_ = currentPoint;
//        }

//    }

