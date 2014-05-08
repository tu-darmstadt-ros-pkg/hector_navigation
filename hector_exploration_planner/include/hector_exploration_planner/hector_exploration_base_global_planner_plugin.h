//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef HECTOR_EXPLORATION_BASE_GLOBAL_PLANNER_PLUGIN_H___
#define HECTOR_EXPLORATION_BASE_GLOBAL_PLANNER_PLUGIN_H___

#include "hector_exploration_planner.h"

#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_DECLARE_CLASS(hector_global_planner, HectorGlobalPlanner, hector_global_planner::HectorGlobalPlanner, nav_core::BaseGlobalPlanner);

namespace hector_exploration_planner{

class HectorExplorationBaseGlobalPlannerPlugin : public nav_core::BaseGlobalPlanner
{
public:
  HectorExplorationBaseGlobalPlannerPlugin();
  virtual ~HectorExplorationBaseGlobalPlannerPlugin();

  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

protected:
  HectorExplorationPlanner* exploration_planner;
};


}

#endif
