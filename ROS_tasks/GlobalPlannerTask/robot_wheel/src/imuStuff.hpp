#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "custom_messsages_interface/msg/yaw.hpp"
#include "custom_messsages_interface/msg/vec.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <math.h>

class imu : public rclcpp::Node
{

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;
    rclcpp::Publisher<custom_messsages_interface::msg::Yaw>::SharedPtr publisher;
    rclcpp::Publisher<custom_messsages_interface::msg::Vec>::SharedPtr angularVelocityPublisher;
    // rclcpp::TimerBase::SharedPtr timer;
public:
    imu();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};