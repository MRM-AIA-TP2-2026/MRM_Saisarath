#include <memory>
#include <math.h>
// #include <vector.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "custom_messsages_interface/msg/yaw.hpp"
#include "custom_messsages_interface/msg/vec.hpp"
#include "3Vec.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

using std::placeholders::_1;

class rovorMove : public rclcpp::Node
{
public:
  rovorMove();

private:
  void clockCycle();
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void YawDataCallback(const custom_messsages_interface::msg::Yaw::SharedPtr msg);
  void getDestination(const custom_messsages_interface::msg::Vec::SharedPtr msg);
  // void getAngularVelocity(const custom_messsages_interface::msg::Vec::SharedPtr msg);
  void robot_Rotate();
  void robot_goBrrr();
  void InitialRotateDone();
  void getObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelPublisher;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GpsSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Yaw>::SharedPtr yawDataSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Vec>::SharedPtr destDataSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Vec>::SharedPtr angularVelocitySubscriber;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ObstaclePclSub;

private:
  vec3 gps;              //.x lat , .y long .z alti
  vec3 yaw;              //.x is deg .y is rad
  vec3 destinationGiven; //.x lat .y long
  vec3 acceleration;     //.x linear .y angular

  sensor_msgs::msg::PointCloud2::SharedPtr latest_pcl;
  std::vector<std::vector<double>> obstacleDataType1;

public:
  double toAngle;
  double distance;
  double intialDiffrence;

  bool InitialrotaionCompleted;
  bool move;
  bool obstacleIs;

  double minDistanceObstacle;
  double minDistanceObstacleAngle;
};