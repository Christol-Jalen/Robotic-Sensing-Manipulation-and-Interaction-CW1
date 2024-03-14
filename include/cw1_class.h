/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
///////////////////////////////
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <sensor_msgs/PointCloud2.h>  // camera message type
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/passthrough.h>

#include <map>
#include <tuple>


// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_x/example.h"

class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  void 
  pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); // Subscribe point cloud msg from /r200 camera

  ////////////////////////////////
  void
  pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask);

  bool
  moveArm(geometry_msgs::Pose target_pose);

  bool
  moveGripper(float width);

  void
  detectColor(); // used in task 2

  void
  applyPT(); // filter function

  geometry_msgs::Pose
  transformPoint(pcl::PointXYZRGB cloud_point); // trans point cloud from camera coordinate to world coordinate

  pcl::RGB 
  extractRGBValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud); // used in task 3, compute the average RGB of a cluster
  
  std::pair<std::map<std::tuple<float, float, float>, pcl::RGB>, std::map<std::tuple<float, float, float>, pcl::RGB>>
  clustering();

  std::string 
  determineColor(const pcl::RGB& rgbValue); // used in task 3

  std::string 
  removeLastCharacter(const std::string& str);

  geometry_msgs::Pose
  setPose(float x, float y, float z);

  ///////////////////////////////

  sensor_msgs::PointCloud2ConstPtr cloud_msg_; // point cloud read from r200 camera Subscriber
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered_; // used in task3, filter floor point cloud
  pcl::PassThrough<pcl::PointXYZRGB> g_pt_; // used for filter
  
  // ros::Publisher filtered_cloud_publisher_;

  /* ----- class member variables ----- */

  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  double z_offset_ = 0.125;
  double angle_offset_ = 3.14159 / 4.0;
  double approach_distance_ = 0.2;

  float scan_pos_x_ = 0.38;
  float scan_pos_y_ = 0;
  float scan_pos_z_ = 0.875;

  // a vector list consist with string
  std::vector<std::string> basket_colors_;
  std::string resultString_;
  
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  ros::Subscriber sub_;

  ////////////////////////////////

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  ////////////////////////////////

};

#endif // end of include guard for CW1_CLASS_H_
