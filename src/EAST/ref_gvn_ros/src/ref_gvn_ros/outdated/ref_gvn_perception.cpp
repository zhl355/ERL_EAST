
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

class RefGvnPerception {
 public:
  /*!
   * Constructor.
   */
  RefGvnPerception(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RefGvnPerception();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;

  ros::Publisher dist_pub_;

  // parameters

  std_msgs::Float32 dQgO_;  // dist from governor to obstacle space in Q norm

  // scan sub
  std::string scan_topic_;
  /************************ init functions *****************************/
  // init parameters, external and internal

  // parameter reading function
  bool readParameters();

  /************************ callback functions *****************************/
  // scanCallback
  void scanCallback(const sensor_msgs::LaserScan& msg);

  /************************ Visualization functions *****************************/
};

bool RefGvnPerception::readParameters() {
  // read parameters for scan subscriber
  // for scan topic name
  if (!nh_.getParam("scan_topic", scan_topic_)) {
    ROS_ERROR("Could not find [scan_topic] parameter");
    return false;
  } else {
    ROS_INFO_STREAM("Read scan_topic_:" << scan_topic_);
  }
  return true;
}

void RefGvnPerception::RefGvnPerception(ros::NodeHandle nh) : nh_(nh), scan_topic_("/scan") {
  // Load parameters
  if (!readParameters()) {
    ROS_ERROR("RefGvnPerception parameter loading failed!");
    ros::requestShutdown();
  } else {
    ROS_INFO("RefGvnPerception parameter loading successfully");
  }

  // init subscriber
  scan_sub_ = nh_.subscribe(scan_topic_, 1, &RefGvnPerception::scanCallback, this);

  // init publishers
  dist_pub_ = nh_.advertise<std_msgs::Float32>("~dist_pub", 0);
}

void RefGvnPerception::scanCallback(const sensor_msgs::LaserScan& msg) {
  // Compute d_Q(g, O)
  // TODOZHL frame change
  // we need to change frame from lidar_base to time-varying governor frame
  // need to get point cloud (lidar endpoints) and compute quadratic distances
  // a. get scan to pointcloud
  // b. get governor state and pointcloud in same frame (odom)
  // c. XgF = pt_arr - gvn_loc2d, dgO_vec = np.sqrt(np.sum(np.matmul(XgF, Q) * XgF, axis=1))
  dgO_ = *std::min_element(msg.ranges.begin(), msg.ranges.end());
  dist_pub_.pub(dgO)
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ref_gvn_perception");
  ros::NodeHandle node("~");
  ros::spin();
  return 0;
}
