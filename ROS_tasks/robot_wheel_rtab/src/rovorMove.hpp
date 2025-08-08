#include <memory>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "custom_messsages_interface/msg/yaw.hpp"
#include "custom_messsages_interface/msg/vec.hpp"
#include "3Vec.h"

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

public:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelPublisher;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr GpsSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Yaw>::SharedPtr yawDataSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Vec>::SharedPtr destDataSubscriber;
  rclcpp::Subscription<custom_messsages_interface::msg::Vec>::SharedPtr angularVelocitySubscriber;

private:
  vec3 gps;              //.x lat , .y long .z alti
  vec3 yaw;              //.x is deg .y is rad
  vec3 destinationGiven; //.x lat .y long
  vec3 acceleration;     //.x linear .y angular
public:
  double toAngle;
  double distance;
  double intialDiffrence;

  bool InitialrotaionCompleted;
  bool move;
};