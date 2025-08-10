#include "rovorMove.hpp"
#include <iostream>

double toRad(double degree)
{
  return degree * 3.14159265359 / 180;
}
double round_to_precision(double value, int precision)
{
  double factor = value * std::pow(10.0, precision);

  int x = factor;

  double y = x * std::pow(10.0, -precision);
  // std::cout << x << " " << y << std::endl;
  return y;
}

rovorMove::rovorMove()
    : Node("rovorMove"),
      move{false},
      InitialrotaionCompleted{false},
      obstacleIs{false},
      minDistanceObstacle{10000},
      minDistanceObstacleAngle{180}
{

  gps = {0, 0, 0};
  yaw = {0, 0, 0};
  acceleration = {0, 0, 1};
  destinationGiven = {0, 0, 0};
  toAngle = 0;
  intialDiffrence = 0;
  distance = 0;

  VelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  GpsSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_plugin/out", 10, std::bind(&rovorMove::gps_callback, this, std::placeholders::_1));
  yawDataSubscriber = this->create_subscription<custom_messsages_interface::msg::Yaw>("/yaw", 10, std::bind(&rovorMove::YawDataCallback, this, std::placeholders::_1));
  destDataSubscriber = this->create_subscription<custom_messsages_interface::msg::Vec>("/destination", 10, std::bind(&rovorMove::getDestination, this, std::placeholders::_1));
  // angularVelocitySubscriber = this->create_subscription<custom_messsages_interface::msg::Vec>("/angular_velocity", 10, std::bind(&rovorMove::getAngularVelocity, this, std::placeholders::_1));

  ObstaclePclSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/obstacles", 10,
      std::bind(&rovorMove::getObstacles, this, std::placeholders::_1));

  timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&rovorMove::clockCycle, this));
}

void rovorMove::clockCycle()
{
  minDistanceObstacle = 10000;
  minDistanceObstacleAngle = 180;
  for (int i = 0; i < obstacleDataType1.size(); i++)
  {

    if (minDistanceObstacle > obstacleDataType1[i][0])
    {
      minDistanceObstacle = obstacleDataType1[i][0];
      minDistanceObstacleAngle = obstacleDataType1[i][1];
    }
    // RCLCPP_INFO(this->get_logger(),
    //             "NoOfObstacle %d:  distance=%.2f m, angle=%.2f°",
    //             i + 1, obstacleDataType1[i][0], obstacleDataType1[i][1]);
  }
  // RCLCPP_INFO(this->get_logger(),
  //             "NoOfObstacle =%d ,distance=%.2f m, angle=%.2f°asffa",
  //             obstacleDataType1.size(), minDistanceObstacle, minDistanceObstacleAngle);
  //   robot_Rotate();
  //   robot_goBrrr();

  if (move)
  {
    if (!InitialrotaionCompleted)
    {

      robot_Rotate();
      InitialRotateDone();
    }

    else
    {

      robot_Rotate();
      robot_goBrrr();
    }
    geometry_msgs::msg::Twist msg;

    msg.linear.x = acceleration.x;  // speed
    msg.angular.z = acceleration.y; // rotation

    VelPublisher->publish(msg);
  }

  // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
  //  std::cout<<"hi hi"<<std::endl;
  // RCLCPP_INFO(this->get_logger(), "\n\nYaw rad: %.6f Yaw Deg:%.6f\ndes yaw: %.6f\ngps  lat:%.6f long:%.6f\ndest lat:%.6f long:%.6f\ndistance:%.6f",
  //           yaw.y, yaw.x, toAngle, gps.x, gps.y, destinationGiven.x, destinationGiven.y, distance);
}

void rovorMove::getObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  obstacleDataType1.clear();
  latest_pcl = msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.5); // meters between points to be in the same cluster
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  RCLCPP_INFO(this->get_logger(), "Found %zu clusters", cluster_indices.size());
  if (cluster_indices.size())
  {
    obstacleIs = true;
  }
  else
  {
    obstacleIs = false;
  }

  int cluster_id = 0;
  for (const auto &indices : cluster_indices)
  {
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (int idx : indices.indices)
    {
      sum_x += cloud->points[idx].x;
      sum_y += cloud->points[idx].y;
      sum_z += cloud->points[idx].z;
    }
    double cx = sum_x / indices.indices.size();
    double cy = sum_y / indices.indices.size();
    double cz = sum_z / indices.indices.size();

    // Distance
    double pcl_distance = sqrt(cx * cx + cy * cy + cz * cz);

    // Yaw (left/right)
    double yaw_rad = atan2(cy, cx);
    double yaw_deg = yaw_rad * 180.0 / M_PI;

    // Pitch (up/down)
    double pitch_rad = atan2(cz, sqrt(cx * cx + cy * cy));
    double pitch_deg = pitch_rad * 180.0 / M_PI;

    // Full 3D angle
    double dot = cx / pcl_distance; // forward vector (1,0,0) dot target normalized
    double theta_3D = acos(dot) * 180.0 / M_PI;

    std::vector<double> l1;
    l1.push_back(pcl_distance);

    l1.push_back(theta_3D - 90);
    obstacleDataType1.push_back(l1);
  }
}

void rovorMove::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gps = {msg->latitude, msg->longitude, msg->altitude};
}

void rovorMove::YawDataCallback(const custom_messsages_interface::msg::Yaw::SharedPtr msg)
{
  yaw = {msg->yaw_degree, msg->yaw_radian, 0};
}

void rovorMove::getDestination(const custom_messsages_interface::msg::Vec::SharedPtr msg)
{
  move = true;
  InitialrotaionCompleted = false;
  destinationGiven = {msg->x, msg->y, 0};
}

/*
void getAngularVelocity(const custom_messsages_interface::msg::Vec::SharedPtr msg)
{
}*/

void rovorMove::robot_Rotate()
{
  double radPerSec = 0;
  if (!(minDistanceObstacleAngle < 45 && minDistanceObstacleAngle > -45))
  {
    double lat1 = toRad(gps.x);
    double lon1 = toRad(gps.y);
    double lat2 = toRad(destinationGiven.x);
    double lon2 = toRad(destinationGiven.y);

    double dLon = lon2 - lon1;

    double x = sin(dLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double initialBearing = atan2(x, y);

    double initialBearingDeg = initialBearing * (180.0 / 3.14159265359);

    double bearing = fmod((initialBearingDeg + 90.0), 360.0);
    toAngle = -bearing;

    if (toAngle > 180)
    {
      toAngle = toAngle - 360;
    }
    if (toAngle < -180)
    {
      toAngle = toAngle + 360;
    }

    std::cout << "toangle" << toAngle << std::endl;
    radPerSec = round_to_precision(toAngle - yaw.x, 2) / 60;
  }
  else
  {
    if (minDistanceObstacleAngle > 0)
    {
      radPerSec = round_to_precision(50 - minDistanceObstacleAngle, 2) / 60;
      radPerSec = -100 / 60;
    }
    else
    {
      radPerSec = round_to_precision(-50 - minDistanceObstacleAngle, 2) / 60;
      radPerSec = +100 / 60;
    }
  }

  // double acc=

  if (radPerSec > 3)
  {
    radPerSec = 3;
  }

  if (round_to_precision(yaw.x, 2) == toAngle)
  {
    acceleration.y = 0;
    // RCLCPP_INFO(this->get_logger(), "Hellooo ----------------------------------");
  }
  else
  {
    acceleration.y = radPerSec;
  }
}

void rovorMove::InitialRotateDone()
{
  if (toAngle - yaw.x < 20)
  { // RCLCPP_INFO(this->get_logger(), "Hellooo ----------------------------------");
    InitialrotaionCompleted = true;
  }
}

void rovorMove::robot_goBrrr()
{

  double metPerSec = 0;
  double R = 6371000;
  double lat1 = toRad(gps.x);
  double lon1 = toRad(gps.y);
  double lat2 = toRad(destinationGiven.x);
  double lon2 = toRad(destinationGiven.y);

  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;

  double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
             std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
  double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

  distance = R * c;
  if (!(minDistanceObstacleAngle < 45 && minDistanceObstacleAngle > -45))
  {

    // RCLCPP_INFO(this->get_logger(), "distance :");

    metPerSec = round_to_precision(distance, 6) / 10;
  }
  else
  {
    double distanceavg = 0;
    int i = 0;

    for (i = 0; i < obstacleDataType1.size(); i++)
    {

      // RCLCPP_INFO(this->get_logger(),
      //             "NoOfObstacle %d:  distance=%.2f m, angle=%.2f°",
      //             i + 1, obstacleDataType1[i][0], obstacleDataType1[i][1]);
      distanceavg += obstacleDataType1[i][0];
    }
    distanceavg = distanceavg / i;
    metPerSec = round_to_precision(distanceavg, 6) / 10;
  }
  // double acc=

  if (metPerSec > 15)
  {
    metPerSec = 15;
  }

  if (distance < 0.25)
  {
    acceleration.x = 0;
    acceleration.y = 0;
    move = false;
    InitialrotaionCompleted = false;
    // RCLCPP_INFO(this->get_logger(), "Hellooo ----------------------------------");
  }
  else
  {
    acceleration.x = metPerSec;
  }
}