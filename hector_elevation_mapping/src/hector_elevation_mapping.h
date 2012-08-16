#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h> 

#include <std_msgs/String.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <hector_elevation_msgs/ElevationMapMetaData.h>
#include <hector_elevation_msgs/ElevationGrid.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <hector_map_tools/HectorMapTools.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>

namespace hector_elevation_mapping
{

class ElevationMapping : public nodelet::Nodelet{

public:
    virtual void onInit();
    ~ElevationMapping();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_sensor_msg);
    void sysMessageCallback(const std_msgs::String& string);
    void updateParamsCallback(const nav_msgs::MapMetaData& grid_map_meta_data);
    void timerCallback(const ros::TimerEvent& event);

private:
    HectorMapTools::CoordinateTransformer<float> world_map_transform;

    tf::TransformListener listener;
    ros::NodeHandle nHandle;
    ros::NodeHandle nPrivateHandle;

    ros::Subscriber sub_pointCloud;
    ros::Subscriber sub_sysMessage;
    ros::Subscriber sub_grid_map_info;

    ros::Publisher pub_local_map;
    ros::Publisher pub_local_map_meta;
    ros::Publisher pub_global_map;
    ros::Publisher pub_global_map_meta;

    ros::Publisher pub_height_update;

    ros::Timer timer;

    hector_elevation_msgs::ElevationMapMetaData elevation_map_meta;
    hector_elevation_msgs::ElevationGrid local_elevation_map;
    hector_elevation_msgs::ElevationGrid global_elevation_map;
    std::vector<double> cell_variance;

    nav_msgs::MapMetaData grid_map_meta;

    double sensor_variance;
    double max_observable_distance;

    bool publish_poseupdate;
    int max_height_levels, max_height;

    double poseupdate_pub_period;
    double poseupdate_height_covariance;
    int poseupdate_used_pattern_size;

    std::string sensor_frame_id, map_frame_id, local_map_frame_id;
    std::string grid_map_topic, local_elevation_map_topic, global_elevation_map_topic, point_cloud_topic, paramSysMsgTopic, pose_update_topic;

};
}
