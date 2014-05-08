#include <hector_elevation_visualization/hector_elevation_visualization.h>


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


ElevationVisualization::ElevationVisualization(): nHandle("~")
{
    nHandle.param("max_height_levels",max_height_levels,10); //[cell]
    nHandle.param("min_height",min_height,-1.0); //[m]
    nHandle.param("max_height",max_height,1.5); //[m]
    nHandle.param("color_factor",color_factor,0.8); //[]
    double r, g, b, a;
    nHandle.param("color/r", r, 0.0);
    nHandle.param("color/g", g, 0.0);
    nHandle.param("color/b", b, 1.0);
    nHandle.param("color/a", a, 1.0);
    marker_color.r = r;
    marker_color.g = g;
    marker_color.b = b;
    marker_color.a = a;
    nHandle.param("use_color_map", use_color_map, true); //[]

    nHandle.param("elevation_map_frame_id", elevation_map_frame_id,std::string("/elevation_map_local"));
    nHandle.param("paramSysMsgTopic", sys_msg_topic, std::string("/syscommand"));

    map_marker_array_publisher = nHandle.advertise<visualization_msgs::MarkerArray>("elevation_map_marker_array", 1,true);

    sub_elevation_map = nHandle.subscribe(elevation_map_frame_id,1,&ElevationVisualization::map_callback,this);
    sub_sys_message_callback = nHandle.subscribe(sys_msg_topic,1,&ElevationVisualization::sys_message_callback,this);

    dyn_rec_server_.setCallback(boost::bind(&ElevationVisualization::dynRecParamCallback, this, _1, _2));

    ros::spin();
}

ElevationVisualization::~ElevationVisualization()
{

}

void ElevationVisualization::dynRecParamCallback(hector_elevation_visualization::ElevationVisualizationConfig &config, uint32_t level)
{
    max_height_levels = config.max_height_levels;
    min_height = config.min_height;
    max_height = config.max_height;
    color_factor = config.color_factor;
    use_color_map = config.use_color_map;

    if(std::strcmp(elevation_map_frame_id.c_str(),(config.elevation_map_frame_id).c_str()))
    {
        elevation_map_frame_id = config.elevation_map_frame_id;

        sub_elevation_map = nHandle.subscribe(elevation_map_frame_id,1,&ElevationVisualization::map_callback,this);
    }
}


void ElevationVisualization::visualize_map(const hector_elevation_msgs::ElevationGrid& elevation_map, tf::StampedTransform local_map_transform)
{
    int current_height_level;

    for (unsigned i = 0; i < map_marker_array_msg.markers.size(); ++i)
    {
        map_marker_array_msg.markers[i].points.clear();
    }
    map_marker_array_msg.markers.clear();


    // each array stores all cubes of one height level:
    map_marker_array_msg.markers.resize(max_height_levels+1);

    for (int index_y = 0; index_y < (int)elevation_map.info.height; ++index_y)
    {
        for (int index_x = 0; index_x < (int)elevation_map.info.width; ++index_x)
        {
            // visualize only known cells
            if(elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)] != (int16_t)-elevation_map.info.zero_elevation)
            {
                geometry_msgs::Point cube_center;
                Eigen::Vector2f index_map(index_x, index_y);
                Eigen::Vector2f index_world = world_map_transform.getC1Coords(index_map);

                cube_center.x = index_world(0);//+elevation_map.info.resolution_xy/2.0;
                cube_center.y = index_world(1);//+elevation_map.info.resolution_xy/2.0;
                cube_center.z = (elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)]-elevation_map.info.zero_elevation)*elevation_map.info.resolution_z;
                current_height_level = max_height_levels/2+(int)round(std::min(std::max((double)cube_center.z+local_map_transform.getOrigin().z(), -(double)max_height), (double)max_height)*(double)max_height_levels/((double)max_height*2.0f));
                map_marker_array_msg.markers[current_height_level].points.push_back(cube_center);

                if(use_color_map)
                {
                    double h = (1.0 - std::min(std::max((cube_center.z-min_height)/ (max_height - min_height), 0.0), 1.0)) *color_factor;
                    map_marker_array_msg.markers[current_height_level].colors.push_back(heightMapColor(h));
                }
            }
        }
    }

    for (unsigned i = 0; i < map_marker_array_msg.markers.size(); ++i)
    {
        std::stringstream ss;
        ss <<"Level "<<i;
        map_marker_array_msg.markers[i].ns = ss.str();
        map_marker_array_msg.markers[i].id = i;
        map_marker_array_msg.markers[i].header.frame_id = "/map";
        map_marker_array_msg.markers[i].header.stamp =  elevation_map.header.stamp;
        map_marker_array_msg.markers[i].lifetime = ros::Duration();
        map_marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        map_marker_array_msg.markers[i].scale.x = elevation_map.info.resolution_xy;
        map_marker_array_msg.markers[i].scale.y = elevation_map.info.resolution_xy;
        map_marker_array_msg.markers[i].scale.z = elevation_map.info.resolution_z;
        map_marker_array_msg.markers[i].color = marker_color;

        if (map_marker_array_msg.markers[i].points.size() > 0)
            map_marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        else
            map_marker_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;
    }

}


void ElevationVisualization::map_callback(const hector_elevation_msgs::ElevationGrid& elevation_map)
{
    // Visualization of Elevation Map
    if(map_marker_array_publisher.getNumSubscribers () > 0)
    {
        tf::TransformListener listener;
        tf::StampedTransform local_map_transform;

        // get local map transform
        try
        {
            listener.waitForTransform("/base_footprint", "/map",ros::Time(0),ros::Duration(5.0));
            listener.lookupTransform("/base_footprint", "/map", ros::Time(0), local_map_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        // init transform
        nav_msgs::MapMetaData map_meta;
        map_meta.resolution = elevation_map.info.resolution_xy;
        map_meta.height = elevation_map.info.height;
        map_meta.width = elevation_map.info.width;
        map_meta.origin = elevation_map.info.origin;

        world_map_transform.setTransforms(map_meta);

        // visualize map
        visualize_map(elevation_map, local_map_transform);


        // publish map
        map_marker_array_publisher.publish(map_marker_array_msg);
    }
}

void ElevationVisualization::sys_message_callback(const std_msgs::String& string)
{
    ROS_DEBUG("sysMsgCallback, msg contents: %s", string.data.c_str());

    if (string.data == "reset")
    {
        ROS_INFO("reset");

        for (unsigned i = 0; i < map_marker_array_msg.markers.size(); ++i)
        {
            map_marker_array_msg.markers[i].points.clear();
        }
        map_marker_array_msg.markers.clear();
    }
}

std_msgs::ColorRGBA ElevationVisualization::heightMapColor(double h)
{

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
        color.r = v; color.g = n; color.b = m;
        break;
    case 1:
        color.r = n; color.g = v; color.b = m;
        break;
    case 2:
        color.r = m; color.g = v; color.b = n;
        break;
    case 3:
        color.r = m; color.g = n; color.b = v;
        break;
    case 4:
        color.r = n; color.g = m; color.b = v;
        break;
    case 5:
        color.r = v; color.g = m; color.b = n;
        break;
    default:
        color.r = 1; color.g = 0.5; color.b = 0.5;
        break;
    }

    return color;
}



