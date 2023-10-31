#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class My_Filter {
 public:
  My_Filter(ros::NodeHandle nh);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
 private:
  
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;
  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;
  std::string scan_topic_, cloud_topic_, cloud_frame_;
};

My_Filter::My_Filter(ros::NodeHandle nh){
  nh.getParam("scan_topic", scan_topic_);
  scan_topic_ = "/" + scan_topic_;
  nh.getParam("cloud_topic", cloud_topic_);
  cloud_topic_ = "/" + cloud_topic_;
  nh.getParam("cloud_frame", cloud_frame_);
  cloud_frame_ = "/" + cloud_frame_;
  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> (scan_topic_, 1, &My_Filter::scanCallback, this);
  point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud> (cloud_topic_, 1, false);
}


void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  sensor_msgs::PointCloud cloud;
  //ROS_ERROR("Received laser scan: %d", scan->ranges.size());

  if(!tfListener_.waitForTransform(
      scan->header.frame_id,
      cloud_frame_,
      scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
      ros::Duration(0.1))){
    return;
  }

  try
  {
    projector_.transformLaserScanToPointCloud(cloud_frame_, *scan, cloud, tfListener_);
    //ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Laser2Point: %zd, max range: %f", cloud.points.size(), scan->range_max);
    point_cloud_publisher_.publish(cloud);

  }
  catch (const tf::TransformException &exception) {
    ROS_ERROR("%s",exception.what());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser2point");
  ros::NodeHandle node("~");

  ROS_WARN("STARTING laser2point");
  My_Filter filter(node);
  ROS_WARN("Initialized My_Filter");
  ros::spin();

  return 0;
}

