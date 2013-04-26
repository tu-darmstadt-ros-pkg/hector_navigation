#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "hector_elevation_mapping.h" 

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace hector_elevation_mapping
{
PLUGINLIB_DECLARE_CLASS(hector_elevation_mapping, ElevationMapping, hector_elevation_mapping::ElevationMapping, nodelet::Nodelet)

void ElevationMapping::onInit()
{
    int width, height, zero_elevation;

    nPrivateHandle = getPrivateNodeHandle();
    nHandle = getNodeHandle();

    nPrivateHandle.param("elevation_resolution", elevation_map_meta.resolution_z, 0.01); //[m]
    nPrivateHandle.param("elevation_zero_level", zero_elevation, 16384); //[cell]
    elevation_map_meta.zero_elevation = zero_elevation;
    nPrivateHandle.param("min_observable_elevation", elevation_map_meta.min_elevation, -1.00); //[m]
    nPrivateHandle.param("max_observable_elevation", elevation_map_meta.max_elevation, 0.30); //[m]
    nPrivateHandle.param("max_observable_distance", max_observable_distance, 4.00); //[m]

    nPrivateHandle.param("map_resolution", elevation_map_meta.resolution_xy, 0.05); //[m]
    nPrivateHandle.param("max_grid_size_x", width, 1024); //[cell]
    elevation_map_meta.width = width;
    nPrivateHandle.param("max_grid_size_y", height, 1024); //[cell]
    elevation_map_meta.height = height;

    nPrivateHandle.param("publish_poseupdate", publish_poseupdate, true); //[]

    nPrivateHandle.param("sensor_variance", sensor_variance, 0.001); //[m^2]

    nPrivateHandle.param("origin_x",elevation_map_meta.origin.position.x, -(double)elevation_map_meta.width*(double)elevation_map_meta.resolution_xy/2.0f); //[m]
    nPrivateHandle.param("origin_y",elevation_map_meta.origin.position.y, -(double)elevation_map_meta.height*(double)elevation_map_meta.resolution_xy/2.0f); //[m]
    nPrivateHandle.param("origin_z",elevation_map_meta.origin.position.z, 0.0); //[m]
    nPrivateHandle.param("orientation_x",elevation_map_meta.origin.orientation.x, 0.0); //[]
    nPrivateHandle.param("orientation_y",elevation_map_meta.origin.orientation.y, 0.0); //[]
    nPrivateHandle.param("orientation_z",elevation_map_meta.origin.orientation.z, 0.0); //[]
    nPrivateHandle.param("orientation_w",elevation_map_meta.origin.orientation.w, 1.0); //[]

    local_elevation_map.info = elevation_map_meta;
    global_elevation_map.info = elevation_map_meta;

    map_meta.origin.position.x = elevation_map_meta.origin.position.x;
    map_meta.origin.position.y = elevation_map_meta.origin.position.y;
    map_meta.origin.position.z = elevation_map_meta.origin.position.z;

    map_meta.origin.orientation.x = elevation_map_meta.origin.orientation.x;
    map_meta.origin.orientation.y = elevation_map_meta.origin.orientation.y;
    map_meta.origin.orientation.z = elevation_map_meta.origin.orientation.z;
    map_meta.origin.orientation.w = elevation_map_meta.origin.orientation.w;

    map_meta.resolution = elevation_map_meta.resolution_xy;
    world_map_transform.setTransforms(map_meta);

    nPrivateHandle.param("map_frame_id", map_frame_id,std::string("map"));
    nPrivateHandle.param("local_map_frame_id", local_map_frame_id,std::string("base_stabilized"));
    nPrivateHandle.param("local_elevation_map_topic", local_elevation_map_topic, std::string("elevation_map_local"));
    nPrivateHandle.param("global_elevation_map_topic", global_elevation_map_topic, std::string("elevation_map_global"));
    nPrivateHandle.param("point_cloud_topic", point_cloud_topic, std::string("openni/depth/points"));
    nPrivateHandle.param("grid_map_topic", grid_map_topic, std::string("scanmatcher_map"));
    nPrivateHandle.param("pose_update_topic",pose_update_topic,std::string("poseupdate"));

    nPrivateHandle.param("sysMsgTopic", paramSysMsgTopic, std::string("syscommand"));

    nPrivateHandle.param("poseupdate_pub_period",poseupdate_pub_period,1.0); //[s]
    nPrivateHandle.param("poseupdate_height_covariance",poseupdate_height_covariance,0.25); //[m²]
    nPrivateHandle.param("poseupdate_used_pattern_size",poseupdate_used_pattern_size,3); //[]

    // allocate memory and set default values
    local_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
    global_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
    cell_variance = std::vector<double>(elevation_map_meta.width * elevation_map_meta.height,std::numeric_limits<double>::max());

    sub_pointCloud = nHandle.subscribe(point_cloud_topic,1,&ElevationMapping::cloudCallback,this);
    sub_sysMessage = nHandle.subscribe(paramSysMsgTopic, 10, &ElevationMapping::sysMessageCallback, this);
    sub_grid_map_info = nHandle.subscribe(grid_map_topic + "_metadata",1,&ElevationMapping::updateParamsCallback,this);

    pub_local_map = nHandle.advertise<hector_elevation_msgs::ElevationGrid>(local_elevation_map_topic, 1, true);
    pub_local_map_meta = nHandle.advertise<hector_elevation_msgs::ElevationMapMetaData>(local_elevation_map_topic + "_metadata", 1, true);
    pub_global_map = nHandle.advertise<hector_elevation_msgs::ElevationGrid>(global_elevation_map_topic, 1, true);
    pub_global_map_meta = nHandle.advertise<hector_elevation_msgs::ElevationMapMetaData>(global_elevation_map_topic + "_metadata", 1, true);

    pub_local_map_meta.publish(local_elevation_map.info);
    pub_global_map_meta.publish(global_elevation_map.info);

    pub_height_update = nHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_update_topic, 1, true);

    if(publish_poseupdate)
        timer = nHandle.createTimer(ros::Duration(poseupdate_pub_period), &ElevationMapping::timerCallback,this);

    ROS_INFO("HectorEM: is up and running.");   
}

ElevationMapping::~ElevationMapping()
{
    ROS_INFO("HectorEM: Shutting down!");
}

void ElevationMapping::timerCallback(const ros::TimerEvent& event)
{
    tf::StampedTransform local_map_transform;

    // get local map transform
    try
    {
        listener.waitForTransform(map_frame_id, local_map_frame_id,ros::Time(0),ros::Duration(1.0));
        listener.lookupTransform(map_frame_id, local_map_frame_id,ros::Time(0), local_map_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_DEBUG("HectorEM: localmap transform in timer callback failed");
        ROS_ERROR("%s",ex.what());
        return;
    }

    // allign grid points
    Eigen::Vector2f index_world(local_map_transform.getOrigin().x(), local_map_transform.getOrigin().y());
    Eigen::Vector2f index_map = world_map_transform.getC2Coords(index_world);

    // check if elevation of current robot position is known, otherwise cancel pose update
    if(global_elevation_map.data[MAP_IDX(elevation_map_meta.width, (int)index_map(0), (int)index_map(1))] != (int16_t)-elevation_map_meta.zero_elevation)
    {
        geometry_msgs::PoseWithCovarianceStamped cell_height_average;

        int error_level  = 0;
        int pattern_cell_quantity = 4*poseupdate_used_pattern_size*poseupdate_used_pattern_size;

        /// todo check if min/max index is inside map
        // include neighbours
        for(int x=index_map(0)-poseupdate_used_pattern_size;x<index_map(0)+poseupdate_used_pattern_size;x++)
        {
            for(int y=index_map(1)-poseupdate_used_pattern_size;y<index_map(1)+poseupdate_used_pattern_size;y++)
            {
                int cell_index = MAP_IDX(elevation_map_meta.width, x, y);

                if(global_elevation_map.data[cell_index] == (int16_t)-elevation_map_meta.zero_elevation)
                {
                    // unknown cell
                    error_level++;
                }
                else
                {
                    // cell is knwon, accumulate cell hight
                    cell_height_average.pose.pose.position.z += (global_elevation_map.data[cell_index]-elevation_map_meta.zero_elevation)*elevation_map_meta.resolution_z;
                }
            }
        }

        // only publish pose update, if more than 1/2 of pattern cells are known
        if(error_level < pattern_cell_quantity/2)
        {
            pattern_cell_quantity -= error_level;

            cell_height_average.pose.pose.position.z = cell_height_average.pose.pose.position.z/(double)pattern_cell_quantity;

            cell_height_average.header.frame_id = map_frame_id;
            cell_height_average.header.stamp = ros::Time::now();

            cell_height_average.pose.covariance.at(0) = 0.0; //no x-position information
            cell_height_average.pose.covariance.at(7) = 0.0; //no y-position information
            cell_height_average.pose.covariance.at(14) = 1.0/poseupdate_height_covariance;

            pub_height_update.publish(cell_height_average);

            ROS_DEBUG("HectorEM: published height update %f",cell_height_average.pose.pose.position.z);

        }
    }
}

void ElevationMapping::sysMessageCallback(const std_msgs::String& string)
{
    ROS_DEBUG("HectorEM: sysMsgCallback, msg contents: %s", string.data.c_str());

    if (string.data == "reset")
    {
        ROS_INFO("HectorEM: reset");

        // allocate memory and set default values
        local_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
        global_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
        cell_variance = std::vector<double>(elevation_map_meta.width * elevation_map_meta.height,std::numeric_limits<double>::max());

    }
}

void ElevationMapping::updateParamsCallback(const nav_msgs::MapMetaData& map_meta_data)
{
    ROS_DEBUG("HectorEM: received new grid map meta data -> overwrite old parameters");

    // set new parameters
    elevation_map_meta.width = map_meta_data.width;
    elevation_map_meta.height = map_meta_data.height;
    elevation_map_meta.resolution_xy = map_meta_data.resolution;

    elevation_map_meta.origin.position.x = map_meta_data.origin.position.x;
    elevation_map_meta.origin.position.y = map_meta_data.origin.position.y;
    elevation_map_meta.origin.position.z = map_meta_data.origin.position.z;

    elevation_map_meta.origin.orientation.x = map_meta_data.origin.orientation.x;
    elevation_map_meta.origin.orientation.y = map_meta_data.origin.orientation.y;
    elevation_map_meta.origin.orientation.z = map_meta_data.origin.orientation.z;
    elevation_map_meta.origin.orientation.w = map_meta_data.origin.orientation.w;

    local_elevation_map.info = elevation_map_meta;
    global_elevation_map.info = elevation_map_meta;

    map_meta = map_meta_data;
    world_map_transform.setTransforms(map_meta);

    // allocate memory and set default values
    local_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
    global_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);
    cell_variance = std::vector<double>(elevation_map_meta.width * elevation_map_meta.height,std::numeric_limits<double>::max());

    pub_local_map_meta.publish(local_elevation_map.info);
    pub_global_map_meta.publish(global_elevation_map.info);
}

void ElevationMapping::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_sensor_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_map_pcl (new pcl::PointCloud<pcl::PointXYZ> ()),
    pointcloud2_sensor_pcl (new pcl::PointCloud<pcl::PointXYZ> ());
    tf::StampedTransform local_map_transform;

    ROS_DEBUG("HectorEM received a point cloud.");

    // converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
    pcl::fromROSMsg(*pointcloud2_sensor_msg, *pointcloud2_sensor_pcl);


    // transform cloud to /map frame
    try
    {
        listener.waitForTransform(map_frame_id, pointcloud2_sensor_msg->header.frame_id,pointcloud2_sensor_pcl->header.stamp,ros::Duration(1.0));
        pcl_ros::transformPointCloud(map_frame_id,*pointcloud2_sensor_pcl,*pointcloud2_map_pcl,listener);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ROS_DEBUG("HectorEM: pointcloud transform failed");
        return;
    }

    // get local map transform
    try
    {
        listener.waitForTransform(map_frame_id, local_map_frame_id,pointcloud2_sensor_msg->header.stamp,ros::Duration(1.0));
        listener.lookupTransform(map_frame_id, local_map_frame_id, pointcloud2_sensor_msg->header.stamp, local_map_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ROS_DEBUG("HectorEM: localmap transform in cloud callback failed");
        return;
    }

    bool local_map_is_subscribed = (pub_local_map.getNumSubscribers () > 0);
    bool global_map_is_subscribed = (pub_global_map.getNumSubscribers () > 0);

    if(local_map_is_subscribed)
        local_elevation_map.data = std::vector<int16_t>(elevation_map_meta.width * elevation_map_meta.height,(int16_t)-elevation_map_meta.zero_elevation);

    unsigned int size = (unsigned int)pointcloud2_map_pcl->points.size();

    // iterate trough all points
    for (unsigned int k = 0; k < size; ++k)
    {
        const pcl::PointXYZ& pt_cloud = pointcloud2_map_pcl->points[k];

        double measurement_distance = pointcloud2_sensor_pcl->points[k].z;

        // check for invalid measurements
        if (isnan(pt_cloud.x) || isnan(pt_cloud.y) || isnan(pt_cloud.z))
            continue;

        // check max distance (manhatten norm)
        if(max_observable_distance < measurement_distance)
            continue;

        // check min/max height
        if(elevation_map_meta.min_elevation+local_map_transform.getOrigin().z() > pt_cloud.z || elevation_map_meta.max_elevation+local_map_transform.getOrigin().z() < pt_cloud.z)
            continue;

        // allign grid points
        Eigen::Vector2f index_world(pt_cloud.x, pt_cloud.y);
        Eigen::Vector2f index_map (world_map_transform.getC2Coords(index_world));

        unsigned int cell_index = MAP_IDX(elevation_map_meta.width, (int)round(index_map(0)), (int)round(index_map(1)));

        int16_t* pt_local_map = &local_elevation_map.data[cell_index];
        int16_t* pt_global_map = &global_elevation_map.data[cell_index];
        double*  pt_var = &cell_variance[cell_index];


        if(local_map_is_subscribed)
        {
            // elevation in current cell in meter
            double cell_elevation = elevation_map_meta.resolution_z*(*pt_local_map-elevation_map_meta.zero_elevation);

            // store maximum of each cell
            if(pt_cloud.z > cell_elevation)
                *pt_local_map = (int16_t)(round(pt_cloud.z/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);

            // filter each cell localy
//            double measurement_variance = sensor_variance*(measurement_distance*measurement_distance);
//            if(*pt_local_map == (int16_t)-elevation_map_meta.zero_elevation)
//            {
//                // unknown cell -> use current measurement
//                *pt_local_map = (int16_t)(round(pt_cloud.z/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);
//                *pt_var = measurement_variance;
//            }
//            else
//            {
//                // fuse cell_elevation with measurement
//                *pt_local_map = (int16_t) (round(((measurement_variance * cell_elevation + *pt_var * pt_cloud.z)/(*pt_var + measurement_variance))/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);
//                *pt_var = (measurement_variance * *pt_var)/(measurement_variance + *pt_var);
//            }
        }

        if(publish_poseupdate || global_map_is_subscribed)
        {
            // fuse new measurements with existing map

            // elevation in current cell in meter
            double cell_elevation = elevation_map_meta.resolution_z*(*pt_global_map-elevation_map_meta.zero_elevation);

            // measurement variance
            double measurement_variance = sensor_variance*(measurement_distance*measurement_distance);

            // mahalanobis distance
            double mahalanobis_distance = sqrt((pt_cloud.z - cell_elevation)*(pt_cloud.z - cell_elevation)/(measurement_variance*measurement_variance));

            if(pt_cloud.z > cell_elevation && (mahalanobis_distance > 5.0))
            {
                *pt_global_map = (int16_t)(round(pt_cloud.z/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);
                *pt_var = measurement_variance;
                continue;
            }

            if((pt_cloud.z < cell_elevation) && (mahalanobis_distance > 5.0))
            {
                *pt_global_map = (int16_t) (round(((measurement_variance * cell_elevation + *pt_var * pt_cloud.z)/(*pt_var + measurement_variance))/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);
                //*pt_var = (measurement_variance * *pt_var)/(measurement_variance + *pt_var);
                *pt_var = measurement_variance;
                continue;
            }

            *pt_global_map = (int16_t) (round(((measurement_variance * cell_elevation + *pt_var * pt_cloud.z)/(*pt_var + measurement_variance))/elevation_map_meta.resolution_z) + (int16_t)elevation_map_meta.zero_elevation);
            *pt_var = (measurement_variance * *pt_var)/(measurement_variance + *pt_var);
        }
    }


    if(local_map_is_subscribed)
    {
        // set the header information on the map
        local_elevation_map.header.stamp = pointcloud2_sensor_msg->header.stamp;
        local_elevation_map.header.frame_id = map_frame_id;

        pub_local_map.publish(local_elevation_map);
    }

    if(global_map_is_subscribed)
    {
        // set the header information on the map
        global_elevation_map.header.stamp = pointcloud2_sensor_msg->header.stamp;
        global_elevation_map.header.frame_id = map_frame_id;

        pub_global_map.publish(global_elevation_map);
    }

}

}
