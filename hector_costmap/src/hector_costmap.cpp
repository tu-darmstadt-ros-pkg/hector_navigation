#include "hector_costmap.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define OCCUPIED_CELL 100
#define FREE_CELL 0
#define UNKNOWN_CELL -1

#define USE_GRID_MAP_ONLY 0
#define USE_ELEVATION_MAP_ONLY 1
#define USE_GRID_AND_ELEVATION_MAP 2

CostMapCalculation::CostMapCalculation() : nHandle("~")
{
    int width, height;
    double grid_res_xy;

    nHandle.param("elevation_resolution", grid_res_z, 0.01); //[m]

    nHandle.param("max_grid_size_x", width, 1024); //[cell]
    cost_map.info.width = width;
    nHandle.param("max_grid_size_y", height, 1024); //[cell]
    cost_map.info.height = height;
    nHandle.param("map_resolution", grid_res_xy, 0.05); //[m]
    cost_map.info.resolution = grid_res_xy;
    nHandle.param("origin_x",cost_map.info.origin.position.x, -(double)cost_map.info.width*(double)cost_map.info.resolution/2.0); //[m]
    nHandle.param("origin_y",cost_map.info.origin.position.y, -(double)cost_map.info.height*(double)cost_map.info.resolution/2.0); //[m]
    nHandle.param("origin_z",cost_map.info.origin.position.z, 0.0); //[m]
    nHandle.param("orientation_x",cost_map.info.origin.orientation.x, 0.0); //[]
    nHandle.param("orientation_y",cost_map.info.origin.orientation.y, 0.0); //[]
    nHandle.param("orientation_z",cost_map.info.origin.orientation.z, 0.0); //[]
    nHandle.param("orientation_w",cost_map.info.origin.orientation.w, 1.0); //[]

    nHandle.param("initial_free_cells_radius", initial_free_cells_radius, 0.30); //[m]
    nHandle.param("update_radius", update_radius_world, 4.0); //[m]
    nHandle.param("max_delta_elevation", max_delta_elevation, 0.07); //[m]

    nHandle.param("map_frame_id", map_frame_id,std::string("/map"));
    nHandle.param("local_map_frame_id", local_map_frame_id,std::string("/base_footprint"));
    nHandle.param("cost_map_topic", cost_map_topic, std::string("/cost_map"));
    nHandle.param("elevation_map_topic", elevation_map_topic, std::string("/elevation_map_local"));
    nHandle.param("grid_map_topic", grid_map_topic, std::string("/scanmatcher_map"));

    nHandle.param("use_elevation_map", use_elevation_map, true);
    nHandle.param("use_grid_map", use_grid_map, true);
    nHandle.param("allow_kinect_to_clear_occupied_cells", allow_kinect_to_clear_occupied_cells, true);
    nHandle.param("max_clear_size", max_clear_size, 2);

    nHandle.param("costmap_pub_freq", costmap_pub_freq, 4.0); //[Hz]

    nHandle.param("sysMsgTopic", paramSysMsgTopic, std::string("/syscommand"));

    cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);
    elevation_cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);

    world_map_transform.setTransforms(cost_map.info);

    // loop through starting area
    min_index(0) = cost_map.info.width/2-floor(initial_free_cells_radius/cost_map.info.resolution);
    min_index(1) = cost_map.info.height/2-floor(initial_free_cells_radius/cost_map.info.resolution);
    max_index(0) = cost_map.info.width/2+floor(initial_free_cells_radius/cost_map.info.resolution);
    max_index(1) = cost_map.info.height/2+floor(initial_free_cells_radius/cost_map.info.resolution);
    for (int v = min_index(1); v < max_index(1); ++v)
        for (int u = min_index(0); u < max_index(0); ++u)
            cost_map.data[MAP_IDX(cost_map.info.width, u, v)] = FREE_CELL;

    pub_cost_map = nHandle.advertise<nav_msgs::OccupancyGrid>(cost_map_topic, 1, true);

    received_elevation_map = false;
    received_grid_map = false;

    sub_elevation_map = nHandle.subscribe(elevation_map_topic,10,&CostMapCalculation::callbackElevationMap,this);
    sub_grid_map = nHandle.subscribe(grid_map_topic,10,&CostMapCalculation::callbackGridMap,this);

    sub_map_info = nHandle.subscribe("/map_metadata",1,&CostMapCalculation::updateMapParamsCallback,this);

    sub_sysMessage = nHandle.subscribe(paramSysMsgTopic, 10, &CostMapCalculation::sysMessageCallback, this);
    dyn_rec_server_.setCallback(boost::bind(&CostMapCalculation::dynRecParamCallback, this, _1, _2));

    timer = nHandle.createTimer(ros::Duration(1.0/costmap_pub_freq), &CostMapCalculation::timerCallback,this);

    ROS_INFO("HectorCM: is up and running.");

    ros::spin();
}


CostMapCalculation::~CostMapCalculation()
{
    ROS_INFO("HectorCM: Shutting down!");
}


void CostMapCalculation::sysMessageCallback(const std_msgs::String& string)
{
    ROS_DEBUG("HectorCM: sysMsgCallback, msg contents: %s", string.data.c_str());

    if (string.data == "reset")
    {
        ROS_INFO("HectorCM: reset");

        // set default values
        cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);
        elevation_cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);

        // loop through starting area
        min_index(0) = cost_map.info.width/2-floor(initial_free_cells_radius/cost_map.info.resolution);
        min_index(1) = cost_map.info.height/2-floor(initial_free_cells_radius/cost_map.info.resolution);
        max_index(0) = cost_map.info.width/2+floor(initial_free_cells_radius/cost_map.info.resolution);
        max_index(1) = cost_map.info.height/2+floor(initial_free_cells_radius/cost_map.info.resolution);
        for (int v = min_index(1); v < max_index(1); ++v)
            for (int u = min_index(0); u < max_index(0); ++u)
                cost_map.data[MAP_IDX(cost_map.info.width, u, v)] = FREE_CELL;

        received_elevation_map = false;
        received_grid_map = false;
    }
}


void CostMapCalculation::dynRecParamCallback(hector_costmap::CostMapCalculationConfig &config, uint32_t level)
{

    max_delta_elevation = config.max_delta_elevation;
    use_elevation_map = config.use_elevation_map;
    use_grid_map = config.use_grid_map;
    allow_kinect_to_clear_occupied_cells = config.allow_kinect_to_clear_occupied_cells;
    max_clear_size = config.max_clear_size;
    costmap_pub_freq = config.costmap_pub_freq;

    if(use_elevation_map)
        sub_elevation_map = nHandle.subscribe(elevation_map_topic,10,&CostMapCalculation::callbackElevationMap,this);
    else
        sub_elevation_map.shutdown();

    if(use_grid_map)
        sub_grid_map = nHandle.subscribe(grid_map_topic,10,&CostMapCalculation::callbackGridMap,this);
    else
        sub_grid_map.shutdown();
}


void CostMapCalculation::updateMapParamsCallback(const nav_msgs::MapMetaData& map_meta_data)
{
    ROS_DEBUG("HectorCM: received new map meta data -> overwrite old parameters");

    // set new parameters
    cost_map.info.width = map_meta_data.width;
    cost_map.info.height = map_meta_data.height;
    cost_map.info.resolution = map_meta_data.resolution;

    cost_map.info.origin.position.x = map_meta_data.origin.position.x;
    cost_map.info.origin.position.y = map_meta_data.origin.position.y;
    cost_map.info.origin.position.z = map_meta_data.origin.position.z;

    cost_map.info.origin.orientation.x = map_meta_data.origin.orientation.x;
    cost_map.info.origin.orientation.y = map_meta_data.origin.orientation.y;
    cost_map.info.origin.orientation.z = map_meta_data.origin.orientation.z;
    cost_map.info.origin.orientation.w = map_meta_data.origin.orientation.w;

    world_map_transform.setTransforms(cost_map.info);

    // allocate memory
    cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);
    elevation_cost_map.data.assign(cost_map.info.width * cost_map.info.height,UNKNOWN_CELL);

    // loop through starting area
    min_index(0) = cost_map.info.width/2-floor(initial_free_cells_radius/cost_map.info.resolution);
    min_index(1) = cost_map.info.height/2-floor(initial_free_cells_radius/cost_map.info.resolution);
    max_index(0) = cost_map.info.width/2+floor(initial_free_cells_radius/cost_map.info.resolution);
    max_index(1) = cost_map.info.height/2+floor(initial_free_cells_radius/cost_map.info.resolution);
    for (int v = min_index(1); v < max_index(1); ++v)
        for (int u = min_index(0); u < max_index(0); ++u)
            cost_map.data[MAP_IDX(cost_map.info.width, u, v)] = FREE_CELL;

    // update flags
    received_elevation_map = false;
    received_grid_map = false;
}

void CostMapCalculation::timerCallback(const ros::TimerEvent& event)
{
    if(received_elevation_map || received_grid_map)

    ROS_DEBUG("HectorCM: published a new costmap");

    // set the header information on the map
    cost_map.header.stamp = ros::Time::now();
    cost_map.header.frame_id = map_frame_id;

    pub_cost_map.publish(cost_map);

}


void CostMapCalculation::callbackElevationMap(const hector_elevation_msgs::ElevationGridConstPtr& elevation_map_msg)
{
    ROS_DEBUG("HectorCM: received new elevation map");

    // check header
    if((int)(1000*elevation_map_msg->info.resolution_xy) != (int)(1000*cost_map.info.resolution) &&  // Magic number 1000 -> min grid size should not be less than 1mm
            elevation_map_msg->info.height != cost_map.info.height &&
            elevation_map_msg->info.width != cost_map.info.width)
    {
        ROS_ERROR("HectorCM: elevation map resolution and or size incorrect!");
        return;
    }

    // store elevation_map_msg in local variable
    elevation_map_ = cv::Mat(elevation_map_msg->info.height, elevation_map_msg->info.width, CV_16S, const_cast<int16_t*>(&elevation_map_msg->data[0]), 2*(size_t)elevation_map_msg->info.width);

    // store elevation map zero level
    elevation_zero_level = elevation_map_msg->info.zero_elevation;

    // compute region of intereset
    if(!computeWindowIndices(elevation_map_msg->header.stamp, update_radius_world))
        return;

    // loop through each element
    int filtered_cell, filtered_cell_x, filtered_cell_y;
    for (int v = min_index(1); v < max_index(1); ++v)
    {
        for (int u = min_index(0); u < max_index(0); ++u)
        {
            // compute cost_map_index
            unsigned int cost_map_index = MAP_IDX(cost_map.info.width, u, v);

            // check if neighbouring cells are known
            if(elevation_map_.at<int16_t>(v+1,u) == (-elevation_zero_level) ||
                    elevation_map_.at<int16_t>(v-1,u) == (-elevation_zero_level) ||
                    elevation_map_.at<int16_t>(v,u+1) == (-elevation_zero_level) ||
                    elevation_map_.at<int16_t>(v,u-1) == (-elevation_zero_level))
                continue;

            // edge filter
            filtered_cell_y = abs(elevation_map_.at<int16_t>(v,u-1) - elevation_map_.at<int16_t>(v,u+1));
            filtered_cell_x = abs(elevation_map_.at<int16_t>(v-1,u) - elevation_map_.at<int16_t>(v+1,u));


            if(filtered_cell_x > filtered_cell_y)
                filtered_cell = filtered_cell_x;
            else
                filtered_cell =  filtered_cell_y;

            // check if cell is traversable
            if(filtered_cell > max_delta_elevation/grid_res_z)
            {
                // cell is not traversable -> mark it as occupied
                elevation_cost_map.data[cost_map_index] = OCCUPIED_CELL;
            }
            else
            {
                // cell is traversable -> mark it as free
                elevation_cost_map.data[cost_map_index] = FREE_CELL;
            }
        }
    }

    // set elevation map received flag
    received_elevation_map = true;

    if(use_grid_map)
    {
        // calculate cost map
        if(received_grid_map)
        {
            calculateCostMap(USE_GRID_AND_ELEVATION_MAP);
        }
        else
        {
            ROS_WARN("HectorCM received no grid map, use only elevation map to compute costmap");
            calculateCostMap(USE_ELEVATION_MAP_ONLY);
        }
    }
    else
    {
        calculateCostMap(USE_ELEVATION_MAP_ONLY);
    }
}

void CostMapCalculation::callbackGridMap(const nav_msgs::OccupancyGridConstPtr& grid_map_msg)
{
    ROS_DEBUG("HectorCM: received new grid map");

    // check header
    if((int)(1000*grid_map_msg->info.resolution) != (int)(1000*cost_map.info.resolution) &&  // Magic number 1000 -> min grid size should not be less than 1mm
            grid_map_msg->info.height != cost_map.info.height &&
            grid_map_msg->info.width != cost_map.info.width)
    {
        ROS_ERROR("HectorCM: grid map resolution and or size incorrect!");
        return;
    }

    grid_map_msg_ = grid_map_msg;

    // copy grid_map_msg to local variable
    grid_map_ = cv::Mat(grid_map_msg_->info.height, grid_map_msg_->info.width, CV_8S, const_cast<int8_t*>(&grid_map_msg_->data[0]), (size_t)grid_map_msg_->info.width);

    // set grid map received flag
    received_grid_map = true;

    // compute region of intereset
    if(!computeWindowIndices(grid_map_msg->header.stamp, update_radius_world))
        return;

    // calculate cost map
    if(use_elevation_map)
    {
        if(received_elevation_map)
        {
            calculateCostMap(USE_GRID_AND_ELEVATION_MAP);
        }
        else
        {
            ROS_DEBUG("HectorCM: received no elevation map, use only grid map to compute costmap");
            calculateCostMap(USE_GRID_MAP_ONLY);
        }
    }
    else
    {
        calculateCostMap(USE_GRID_MAP_ONLY);
    }
}

bool CostMapCalculation::calculateCostMap(char map_level)
{
    switch(map_level)
    {
    case USE_GRID_MAP_ONLY:
    {
        // cost map based on grid map only

        ROS_DEBUG("HectorCM: compute costmap based on grid map");

        // loop through each element
        for (int v = min_index(1); v < max_index(1); ++v)
        {
            for (int u = min_index(0); u < max_index(0); ++u)
            {
                unsigned int index = MAP_IDX(cost_map.info.width, u, v);

                // check if cell is known
                if((char)grid_map_.data[index] != UNKNOWN_CELL)
                {
                    if(grid_map_.data[index] == OCCUPIED_CELL)
                    {
                        // cell is occupied
                        cost_map.data[index] = OCCUPIED_CELL;
                    }
                    else
                    {
                        // cell is not occupied
                        cost_map.data[index] = FREE_CELL;
                    }
                }
            }
        }

        break;
    }
    case USE_ELEVATION_MAP_ONLY:
    {
        // cost map based on elevation map only

        ROS_DEBUG("HectorCM: compute costmap based on elevation map");


        // loop through each element
        for (int v = min_index(1); v < max_index(1); ++v)
        {
            for (int u = min_index(0); u < max_index(0); ++u)
            {
                unsigned int index = MAP_IDX(cost_map.info.width, u, v);

                // check if cell is known
                if(elevation_cost_map.data[index] != UNKNOWN_CELL)
                {
                    if(elevation_cost_map.data[index] == OCCUPIED_CELL)
                    {
                        // cell is occupied
                        cost_map.data[index] = OCCUPIED_CELL;
                    }
                    else
                    {
                        // cell is not occupied
                        cost_map.data[index] = FREE_CELL;
                    }
                }
            }
        }

        break;
    }
    case USE_GRID_AND_ELEVATION_MAP:
    {
        // cost map based on elevation and grid map

        ROS_DEBUG("HectorCM: compute costmap based on grid and elevation map");

        int checksum_grid_map;

        // loop through each element
        for (int v = min_index(1); v < max_index(1); ++v)
        {
            for (int u = min_index(0); u < max_index(0); ++u)
            {
                unsigned int index = MAP_IDX(cost_map.info.width, u, v);

                // check if cell is known
                if(grid_map_.at<int8_t>(v,u) != UNKNOWN_CELL)
                {
                    if(grid_map_.at<int8_t>(v,u) == OCCUPIED_CELL || elevation_cost_map.data[index] == OCCUPIED_CELL)
                    {
                        checksum_grid_map = 0;

                        checksum_grid_map += grid_map_.at<int8_t>(v-1, u);
                        checksum_grid_map += grid_map_.at<int8_t>(v+1, u);
                        checksum_grid_map += grid_map_.at<int8_t>(v,   u-1);
                        checksum_grid_map += grid_map_.at<int8_t>(v,   u+1);
                        checksum_grid_map += grid_map_.at<int8_t>(v+1, u+1);
                        checksum_grid_map += grid_map_.at<int8_t>(v-1, u-1);
                        checksum_grid_map += grid_map_.at<int8_t>(v+1, u-1);
                        checksum_grid_map += grid_map_.at<int8_t>(v-1, u+1);

                        if(elevation_cost_map.data[index] == FREE_CELL && allow_kinect_to_clear_occupied_cells)
                        {
                            if(checksum_grid_map <= max_clear_size*OCCUPIED_CELL)
                            {
                                // cell is free
                                cost_map.data[index] = FREE_CELL;
                            }
                            else
                            {
                                // cell is occupied
                                cost_map.data[index] = OCCUPIED_CELL;
                            }
                        }
                        else
                        {
                            // cell is occupied
                            cost_map.data[index] = OCCUPIED_CELL;
                        }
                    }
                    else
                    {
                        cost_map.data[index] = FREE_CELL;
                    }
                }
            }
        }
    }
        break;
    }

    ROS_DEBUG("HectorCM: computed a new costmap");

    return true;
}

bool CostMapCalculation::computeWindowIndices(ros::Time time,double update_radius)
{
    int update_radius_map;
    Eigen::Vector2f index_world, index_map;

    // window update region
    // only update cells within the max a curtain radius of current robot position
    tf::StampedTransform local_map_transform;

    // get local map transform
    try
    {
        listener.waitForTransform(map_frame_id, local_map_frame_id, time, ros::Duration(5));
        listener.lookupTransform(map_frame_id, local_map_frame_id, time, local_map_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    index_world(0) = local_map_transform.getOrigin().x();
    index_world(1) = local_map_transform.getOrigin().y();
    index_map = world_map_transform.getC2Coords(index_world);
    update_radius_map = floor(((double)update_radius/(double)cost_map.info.resolution));


    // compute window min/max index
    if(index_map(1) < update_radius_map)
        min_index(1) = 0;
    else
        min_index(1) = index_map(1) - update_radius_map;

    if(index_map(1) + update_radius_map > cost_map.info.height)
        max_index(1) = cost_map.info.height;
    else
        max_index(1) = index_map(1) + update_radius_map;

    if(index_map(0) < update_radius_map)
        min_index(0) = 0;
    else
        min_index(0) = index_map(0) - update_radius_map;

    if(index_map(0) + update_radius_map > cost_map.info.width)
        max_index(0) = cost_map.info.width;
    else
        max_index(0) = index_map(0) + update_radius_map;

    return true;
}

