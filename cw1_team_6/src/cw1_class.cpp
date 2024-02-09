/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh)
{
  /* class constructor */
  
  nh_ = nh;
  // initialise the subscriber to subscribe the point cloud camera.
  sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 10, &cw1::pointCloudCallback, this);

  // initialise the publisher to publish the filtered point cloud.
  // filtered_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1, true);

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);
  
  // initialise the global pointer
  pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_cloud_filtered_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // receive object and goal position from the request
  geometry_msgs::PoseStamped object_loc = request.object_loc;
  geometry_msgs::PointStamped goal_loc = request.goal_loc;
  // call function to perform task 1
  pickAndPlace(object_loc.pose.position, goal_loc.point);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  
  for (const geometry_msgs::PointStamped& basket_loc : request.basket_locs)
  {
    // set the desired check pose
    geometry_msgs::Pose check_pose = setPose(basket_loc.point.x, basket_loc.point.y, basket_loc.point.z + z_offset_ + approach_distance_);

    /* Now perform the scanning of each basket */
    bool success = true;

    // move the arm above the object
    success *= moveArm(check_pose);
    if (not success) 
    {
      ROS_ERROR("Moving arm to pick approach pose failed");
      return false;
    }

    // call function that detect the color of basket
    detectColor();
  }

  // return the string to response
  for (size_t i = 0; i < basket_colors_.size(); ++ i) 
  {
    response.basket_colours.push_back(basket_colors_[i]);
  }
  ROS_INFO("Task 2 done");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */
  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  
  // set the scanning pose
  geometry_msgs::Pose scanning_pose = setPose(scan_pos_x_, scan_pos_y_, scan_pos_z_);

  /* Now perform the scanning */
  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the camera to the highest the position
  success *= moveArm(scanning_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // remove the floor elements in the point cloud
  ROS_INFO("Applying pass through filter");
  applyPT();

  // cluster the point cloud 
  ROS_INFO("Applying clustering");
  auto centroids = clustering();

  // create variables to store the results returned from the clustering function
  std::map<std::tuple<float, float, float>, pcl::RGB> mapCube = centroids.first;
  std::map<std::tuple<float, float, float>, pcl::RGB> mapBasket = centroids.second;

  // create variables to store the color and location map of the Cubes and Basksts
  std::map<std::string, geometry_msgs::Pose> mapCubeTrans;
  std::map<std::string, geometry_msgs::Pose> mapBasketTrans;

  // Transform the position in mapCube from camera frame to world frame
  // Determing the color in mapCube from rgb values
  int i = 1;
  for (const auto & pair : mapCube) 
  {
    // Accessing the key and value of the map
    std::tuple<float, float, float> key = pair.first;
    pcl::RGB rgb_value = pair.second;
    std::string colorCube = determineColor(rgb_value);
    colorCube = colorCube + std::to_string(i);
    ROS_INFO("The color of the cube is: %s\n",colorCube.c_str());

    // Extracting coordinates from the key
    float x = std::get<0>(key);
    float y = std::get<1>(key);
    float z = std::get<2>(key);
    pcl::PointXYZRGB centre_point;
    centre_point.x = x;
    centre_point.y = y;
    centre_point.z = z;
    geometry_msgs::Pose poseCube = transformPoint(centre_point);
    ROS_INFO("The position of the cube is: %f, %f, %f\n", poseCube.position.x, poseCube.position.y, poseCube.position.z);
    mapCubeTrans[colorCube] = poseCube;
    i ++;
  }

  // Transform the position in mapBasket from camera frame to world frame
  // Determing the color in mapBasket from rgb values
  for (const auto & pair : mapBasket) 
  {
    // Accessing the key and value of the map
    std::tuple<float, float, float> key = pair.first;
    pcl::RGB rgb_value = pair.second;
    std::string colorBask = determineColor(rgb_value);
    ROS_INFO("The color of the basket is: %s\n", colorBask.c_str());
    
    // Extracting coordinates from the key
    float x = std::get<0>(key);
    float y = std::get<1>(key);
    float z = std::get<2>(key);
    pcl::PointXYZRGB centre_point;
    centre_point.x = x;
    centre_point.y = y;
    centre_point.z = z;
    geometry_msgs::Pose poseBask = transformPoint(centre_point);
    ROS_INFO("The position of the basket is: %f, %f, %f\n", poseBask.position.x, poseBask.position.y, poseBask.position.z);
    mapBasketTrans[colorBask] = poseBask;
  }

  // Match the key in the two maps: mapBasketTrans and mapCubeTrans to perform classifying cubes
  for (auto const& pair : mapBasketTrans) 
  {
    ROS_INFO("String Key: %s, Pose (x: %f, y: %f, z: %f)",
             pair.first.c_str(),
             pair.second.position.x, pair.second.position.y, pair.second.position.z);
    
    // Search for matching keys in mapCubeTrans
    for (auto const& cubePair : mapCubeTrans) 
    {
      std::string basketKey = pair.first;
      std::string cubeKey = cubePair.first;
      // Remove the last character from the cubeKey
      cubeKey = removeLastCharacter(cubeKey);
      if (cubeKey == basketKey) 
      {   
        ROS_INFO("Color Match!!!");
        ROS_INFO("Corresponding Cube Key: %s, Pose (x: %f, y: %f, z: %f)",
                  cubePair.first.c_str(),
                  cubePair.second.position.x, cubePair.second.position.y, cubePair.second.position.z);

        geometry_msgs::Point pointCube; // cube location
        geometry_msgs::Point pointBask; // basket location
        pointCube.x = cubePair.second.position.x;
        pointCube.y = cubePair.second.position.y;
        pointCube.z = cubePair.second.position.z;
        pointBask.x = pair.second.position.x;
        pointBask.y = pair.second.position.y;
        pointBask.z = pair.second.position.z;

        // call function to perform pick and place
        pickAndPlace(pointCube, pointBask);  
      }
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////

std::string 
cw1::removeLastCharacter(const std::string& str) 
{
  // This function removes the last charecter of a string

  if (str.empty()) 
  {
    return str;
  }
  return str.substr(0, str.size() - 1);
}

///////////////////////////////////////////////////////////////////////////////

std::string 
cw1::determineColor(const pcl::RGB& rgbValue)
{
  // This function determine the color from rgb values

  if (rgbValue.b > rgbValue.r && rgbValue.b > rgbValue.g)
  {
    return "blue";
  } 
  else if (rgbValue.r > rgbValue.b && rgbValue.r > rgbValue.g)
  {
    return "red";
  } 
  else
  {
    return "purple";
  }
}

///////////////////////////////////////////////////////////////////////////////

std::pair<std::map<std::tuple<float, float, float>, pcl::RGB>, std::map<std::tuple<float, float, float>, pcl::RGB>>
cw1::clustering()
{
  // This function will cluster the point cloud and return results in two maps

  // Vector to store the clusters
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_vector;

  // Create a KDTree for searching nearest neighbors
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(pcl_cloud_filtered_);

  // cluster indices in extraction
  std::vector<pcl::PointIndices> cluster_indices;

  // Define parameters for clustering
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; // extracting clusters
  ec.setClusterTolerance(0.02); // Adjust as per your point cloud's characteristics
  ec.setMinClusterSize(100);    // Minimum points in a cluster
  ec.setMaxClusterSize(10000);   // Maximum points in a cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_cloud_filtered_); // point cloud data to be clustered
  ec.extract(cluster_indices); // store the cluster result

  // Get the number of clusters
  size_t num_clusters = cluster_indices.size();

  // Print or use the number of clusters
  ROS_INFO("Number of clusters found: %lu", num_clusters);

  // Map to store centroids and RGB values of cubes and baskets
  std::map<std::tuple<float, float, float>, pcl::RGB> cube_centroids;
  std::map<std::tuple<float, float, float>, pcl::RGB> basket_centroids;

  // Process each cluster
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto & idx : cluster.indices) 
    {
      cluster_cloud->push_back((*pcl_cloud_filtered_)[idx]);
    }
    cluster_cloud->width = cluster_cloud->size();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = true;

    // Get the centroid of the cluster
    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(*cluster_cloud, centroid);

    // Create a key for the map using the coordinates of the centroid
    std::tuple<float, float, float> centroid_key(centroid.x, centroid.y, centroid.z);

    // Check if the cluster is a cube or a basket based on its size
    if (cluster_cloud->size() < 2000) // Cube
    {
      cube_centroids[centroid_key] = extractRGBValue(cluster_cloud);
    }
    else // Basket
    {
      basket_centroids[centroid_key] = extractRGBValue(cluster_cloud);
    }
  }

  return std::make_pair(cube_centroids, basket_centroids);
}

///////////////////////////////////////////////////////////////////////////////

pcl::RGB
cw1::extractRGBValue(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // This function calculate the average RGB value for a point cloud

  // Calculate the total number of points in the cloud
  int num_points = cloud->size();

  // Initialize variables to store the sum of RGB values
  int sum_r = 0, sum_g = 0, sum_b = 0;

  // Iterate over all points in the cloud and accumulate RGB values
  for (int i = 0; i < num_points; ++i) 
  {
    sum_r += (*cloud)[i].r;
    sum_g += (*cloud)[i].g;
    sum_b += (*cloud)[i].b;
  }

  // Calculate the average RGB values
  pcl::RGB avg_rgb;
  avg_rgb.r = sum_r / num_points;
  avg_rgb.g = sum_g / num_points;
  avg_rgb.b = sum_b / num_points;

  return avg_rgb;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw1::transformPoint(pcl::PointXYZRGB cloud_point)
{     
  // This function transform a point in the camera frame to a pose in the world frame

  // Define the orientation
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  geometry_msgs::Point original_point;
  original_point.x = cloud_point.x;
  original_point.y = cloud_point.y;
  original_point.z = cloud_point.z;

  // Transform the position from camera frame to world frame
  geometry_msgs::Point transformed_point;
  transformed_point.x = scan_pos_x_ - original_point.y + 0.04; // add camera offset
  transformed_point.y = scan_pos_y_ - original_point.x;
  transformed_point.z = 0.02; // fixed pickup hight

  // Store the final result
  geometry_msgs::Pose pose;
  pose.orientation = grasp_orientation;
  pose.position.x = transformed_point.x;
  pose.position.y = transformed_point.y;
  pose.position.z = transformed_point.z;
  ROS_INFO("Transformed position is: x=%f, y=%f, z=%f", pose.position.x, pose.position.y, pose.position.z);

  return pose;
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::applyPT()
{
  // This function applys a pass through filter to remove the floor

  g_pt_.setInputCloud (pcl_cloud_);
  g_pt_.setFilterFieldName ("z");
  g_pt_.setFilterLimits (0, 0.785);
  g_pt_.filter (*pcl_cloud_filtered_);
}

///////////////////////////////////////////////////////////////////////////////

void 
cw1::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_) 
{
  // This is the subscriber callback function

  // Convert PointCloud2 message to a PointCloud<PointXYZRGB>
  pcl::fromROSMsg(*cloud_msg_, *pcl_cloud_);

  // // you can umcomment this code and 110 line of "cw1_class.h" to see the filtered_cloud_msg in rviz.
  // // Publish the filtered point cloud
  // sensor_msgs::PointCloud2 filtered_cloud_msg;
  // pcl::toROSMsg(*pcl_cloud_filtered_, filtered_cloud_msg);
  // filtered_cloud_msg.header = cloud_msg_->header;  // Copy the header to maintain the timestamp and frame_id
  // filtered_cloud_publisher_.publish(filtered_cloud_msg);
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::detectColor()
{
  // This function determines the color of a object by looking at the rgb value at the centre point of a point cloud

  pcl::PointXYZRGB filtered_point = pcl_cloud_->points[230720]; // The contre point of the unfiltered point cloud
  uint32_t rgb = *reinterpret_cast<int*>(&filtered_point.rgb);
  uint8_t r = (rgb >> 16) & 0x0000ff;
  uint8_t g = (rgb >> 8) & 0x0000ff;
  uint8_t b = (rgb) & 0x0000ff;
  ROS_INFO("RGB: %u, %u, %u", r, g, b);

  // Define thresholds for color detection
  const uint8_t lowThreshold = 40; // Represents the 0.1 threshold
  const uint8_t highThreshold = 210; // Represents the 0.8 threshold

  // Identify colors based on RGB values, adding result to the end
  if (r <= lowThreshold && g <= lowThreshold && b >= highThreshold) 
  {
    basket_colors_.push_back("blue");
    ROS_INFO("Get blue");
  } 
  else if (r >= highThreshold && g <= lowThreshold && b <= lowThreshold) 
  {
    basket_colors_.push_back("red");
    ROS_INFO("Get red");
  } 
  else if (r >= highThreshold && g <= lowThreshold && b >= highThreshold) 
  {
    basket_colors_.push_back("purple");
    ROS_INFO("Get purple");
  } 
  else 
  {
    basket_colors_.push_back("none");
    ROS_INFO("Get none");
  }

  // print all result into a string
  resultString_ = "[";
  for (size_t i = 0; i < basket_colors_.size(); ++i) 
  {
    resultString_ += "\"" + basket_colors_[i] + "\"";
    if (i < basket_colors_.size() - 1) 
    {
      resultString_ += ", ";
    }
  }
  resultString_ += "]";
  ROS_INFO_STREAM("The Basket colors: " << resultString_);
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2 */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////

void
cw1::pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask)
{
  // This function pick a cube from given position and drop it at another given position

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose = setPose(pointCube.x, pointCube.y, pointCube.z + z_offset_);
  ROS_INFO("grasp pose %f, %f, %f\n", grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
  }

  // open the gripper
  success *= moveGripper(gripper_open_);
  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }

  // grasp!
  success *= moveGripper(gripper_closed_);
  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
  }

  // retreat with object
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
  }

  // move over the basket
  approach_pose.position.x = pointBask.x; // set up position that over the basket
  approach_pose.position.y = pointBask.y;

  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving over the basket failed");
  }

  // place the cube
  success *= moveGripper(gripper_open_);
  if (not success)
  {
    ROS_ERROR("Placing the cube failed");
  }

  ROS_INFO("Placing successful");
}

/////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw1::setPose(float x, float y, float z)
{
  // This function return a gripper pose for given xyz and oriention straight down to the floor

  // determine the grasping orientation
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = grasp_orientation;

  return pose;
}