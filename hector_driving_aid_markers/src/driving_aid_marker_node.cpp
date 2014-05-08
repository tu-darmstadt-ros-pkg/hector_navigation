#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class DrivingAidMarker
{
public:
  DrivingAidMarker()
  {

    ros::NodeHandle pnh("~");

    pnh.param("left_side_y_outer", p_left_side_y_outer_, 0.0);
    pnh.param("left_side_y_inner", p_left_side_y_inner_, 0.0);
    pnh.param("right_side_y_outer", p_right_side_y_outer_, 0.0);
    pnh.param("right_side_y_inner", p_right_side_y_inner_, 0.0);
    pnh.param("offset_z", p_offset_z_, -0.112-0.07);

    pub_timer_ = pnh.createTimer(ros::Duration(0.1), &DrivingAidMarker::pubTimerCallback, this, false);
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("driving_aid", 1,false);

    visualization_msgs::Marker marker;
    //marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = "/base_link";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r= 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.ns ="wheel_footprint";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    std::vector<geometry_msgs::Point> point_vector;

    geometry_msgs::Point tmp;
    tmp.x = 0.0;
    tmp.z = p_offset_z_;
    tmp.y = p_left_side_y_outer_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_left_side_y_inner_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_right_side_y_outer_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_right_side_y_inner_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);


    marker.points = point_vector;

    marker_array_.markers.push_back(marker);
  }
  
  ~DrivingAidMarker()
  {
    
  }

  void pubTimerCallback(const ros::TimerEvent& event)
  {
    if (marker_pub_.getNumSubscribers() > 0){
      marker_array_.markers[0].header.stamp = ros::Time::now();
      marker_pub_.publish(marker_array_);
    }
  }

protected:
  visualization_msgs::MarkerArray marker_array_;

  double p_left_side_y_outer_;
  double p_left_side_y_inner_;
  double p_right_side_y_outer_;
  double p_right_side_y_inner_;
  double p_offset_z_;

  ros::Publisher marker_pub_;
  ros::Timer pub_timer_;
};

int main (int argc, char** argv)
{
  ros::init(argc,argv,"hector_driving_aid_marker_node");

  DrivingAidMarker driving_aid_marker;

  ros::spin();

  return (-1);
}
