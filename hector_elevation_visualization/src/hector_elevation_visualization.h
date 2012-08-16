#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>

#include <hector_elevation_msgs/ElevationMapMetaData.h>
#include <hector_elevation_msgs/ElevationGrid.h>

#include <dynamic_reconfigure/server.h>
#include <hector_elevation_visualization/ElevationVisualizationConfig.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <hector_map_tools/HectorMapTools.h>


class ElevationVisualization{

public:
    ElevationVisualization();
    ~ElevationVisualization();

    void dynRecParamCallback(hector_elevation_visualization::ElevationVisualizationConfig &config, uint32_t level);
    void sys_message_callback(const std_msgs::String& string);
    void map_callback(const hector_elevation_msgs::ElevationGrid& elevation_map);

private:
    ros::NodeHandle nHandle;

    ros::Subscriber sub_elevation_map;
    ros::Subscriber sub_sys_message_callback;
    ros::Publisher map_marker_array_publisher;

    dynamic_reconfigure::Server<hector_elevation_visualization::ElevationVisualizationConfig> dyn_rec_server_;

    visualization_msgs::MarkerArray map_marker_array_msg;

    HectorMapTools::CoordinateTransformer<float> world_map_transform;

    std::string elevation_map_frame_id,sys_msg_topic;

    int max_height_levels;

    double max_height;

    void visualize_map(const hector_elevation_msgs::ElevationGrid& elevation_map, tf::StampedTransform local_map_transform);
};
