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
      InitialrotaionCompleted{false}
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

  timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&rovorMove::clockCycle, this));
}

void rovorMove::clockCycle()
{
  //  robot_Rotate();
  //  robot_goBrrr();
  if (move)
  {
    if (!InitialrotaionCompleted)
    {

      // robot_Rotate();
      // InitialRotateDone();
    }

    else
    {

      // robot_Rotate();
      // robot_goBrrr();
    }
    geometry_msgs::msg::Twist msg;

    msg.linear.x = acceleration.x;  // speed
    msg.angular.z = acceleration.y; // rotation

    VelPublisher->publish(msg);
  }

  // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
  //  std::cout<<"hi hi"<<std::endl;
  RCLCPP_INFO(this->get_logger(), "\n\nYaw rad: %.6f Yaw Deg:%.6f\ndes yaw: %.6f\ngps  lat:%.6f long:%.6f\ndest lat:%.6f long:%.6f\ndistance:%.6f",
              yaw.y, yaw.x, toAngle, gps.x, gps.y, destinationGiven.x, destinationGiven.y, distance);
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

  double radPerSec = round_to_precision(toAngle - yaw.x, 2) / 60;

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

  // RCLCPP_INFO(this->get_logger(), "distance :");

  double metPerSec = round_to_precision(distance, 6) / 10;

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