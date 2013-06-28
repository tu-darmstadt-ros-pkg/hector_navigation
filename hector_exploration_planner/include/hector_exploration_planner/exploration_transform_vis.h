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

#ifndef EXPLORATION_TRANSFORM_VIS_H___
#define EXPLORATION_TRANSFORM_VIS_H___

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>


namespace hector_exploration_planner{

  class ExplorationTransformVis
  {
  public:
    ExplorationTransformVis(const std::string& topic_name)
    {
      ros::NodeHandle pnh("~");
      exploration_transform_pointcloud_pub_ = pnh.advertise<sensor_msgs::PointCloud>(topic_name, 2, false);
    }

    virtual ~ExplorationTransformVis()
    {}

    void publishVisOnDemand(const costmap_2d::Costmap2D& costmap, const unsigned int* exploration_array)
    {
      if (exploration_transform_pointcloud_pub_.getNumSubscribers() > 0){
        unsigned int size_x = costmap.getSizeInCellsX();
        unsigned int size_y = costmap.getSizeInCellsY();
        unsigned int size = size_x * size_y;

        unsigned int max = 0;

        for (size_t i = 0; i < size; ++i){
          if ((exploration_array[i] < INT_MAX) && (exploration_array[i] > max)){
            max = exploration_array[i];
          }
        }

        float max_f = static_cast<float>(max);

        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "/map";
        cloud.header.stamp = ros::Time::now();

        double x_world, y_world;

        geometry_msgs::Point32 point;

        for (size_t x = 0; x < size_x; ++x){
          for (size_t y = 0; y < size_y; ++y){

            unsigned int index = costmap.getIndex(x,y);

            if (exploration_array[index] < INT_MAX){

              costmap.mapToWorld(x,y, x_world, y_world);
              point.x = x_world;
              point.y = y_world;
              point.z = static_cast<float>(exploration_array[index])/max_f;

              cloud.points.push_back(point);
            }

          }
        }
        exploration_transform_pointcloud_pub_.publish(cloud);
      }
    }

  protected:

    ros::Publisher exploration_transform_pointcloud_pub_;
  };

}

#endif
