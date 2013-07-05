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
    /// Default plugin constructor
    virtual void onInit();

    /// Default deconstructor
    ~ElevationMapping();

    /// cloudCallback get called if a new 3D point cloud is avaible
    /**
    * \param [in] pointcloud2_sensor_msg contains the 3D point cloud
    */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_sensor_msg);

    /// sysMessageCallback This function listen to system messages
    /**
    * \param [in] string parameter contains system messages, like "reset"
    */
    void sysMessageCallback(const std_msgs::String& string);


    /// updateMapParamsCallback updates the map meta information if someone has changed it
    /**
    * \param [in] map_meta_data map meta information like grid resolution or origin
    */
    void updateParamsCallback(const nav_msgs::MapMetaData& map_meta_data);


    /// timerCallback publishes periodically a height pose update
    /**
    * \param [in] event is not used
    */
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

    nav_msgs::MapMetaData map_meta;

    double sensor_variance;
    double max_observable_distance;

    bool publish_poseupdate;
    int max_height_levels, max_height;

    double poseupdate_pub_period;
    double poseupdate_height_covariance;
    int poseupdate_used_pattern_size;

    std::string sensor_frame_id, map_frame_id, local_map_frame_id;
    std::string grid_map_topic, local_elevation_map_topic, global_elevation_map_topic, point_cloud_topic, sys_msg_topic, pose_update_topic;

};
}
