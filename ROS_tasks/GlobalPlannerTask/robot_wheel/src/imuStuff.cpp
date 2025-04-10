#include "imuStuff.hpp"

/*
double findYaw(double *quater){
  double x,y,z,w;
  x=quater[0];
  y=quater[1];
  z=quater[2];
  w=quater[3];

  double yaw=atan2((2*(w*z+x*y)),1-2*(y*y-z*z));
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
*/
double findYaw(double *quater)
{
  double x, y, z, w;
  x = quater[0];
  y = quater[1];
  z = quater[2];
  w = quater[3];
  tf2::Quaternion tf_quat(x, y, z, w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

imu::imu() : Node("imuDat")
{
  subscriber = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&imu::imu_callback, this, std::placeholders::_1));
  publisher = this->create_publisher<custom_messsages_interface::msg::Yaw>("yaw", 10);
  angularVelocityPublisher = this->create_publisher<custom_messsages_interface::msg::Vec>("angular_velocity", 10);
  // publisher = this->create_publisher<robot_wheel::msg::YawData>("yaw_data", 10);
}

void imu::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

  double quarter[] = {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
  double yaw = findYaw(quarter);
  // RCLCPP_INFO(this->get_logger(), "\nimu x:%.6f,y:%.6f,z:%.6f,w:%.6f, angle=%.6f,angledegree=%.6f", quarter[0], quarter[1], quarter[2], quarter[3], yaw, yaw * 180 / 3.14159265359);
  // RCLCPP_INFO(this->get_logger(), "\nAcceleration linear:%.6f, angular acceleration:%.6f", msg->.x, msg->angular_velocity.z);

  custom_messsages_interface::msg::Yaw yaw_msg;
  yaw_msg.yaw_degree = yaw * 180 / 3.14159265359;
  yaw_msg.yaw_radian = yaw;

  custom_messsages_interface::msg::Vec acc_msg;
  acc_msg.z = msg->angular_velocity.z;
  acc_msg.x = 0;
  acc_msg.y = 0;

  publisher->publish(yaw_msg);
  angularVelocityPublisher->publish(acc_msg);
}