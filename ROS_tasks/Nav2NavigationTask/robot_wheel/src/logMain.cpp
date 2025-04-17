#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class logData : public rclcpp::Node
{
public:
    logData()
        : Node("logData")
    {

        RCLCPP_INFO(this->get_logger(), "logging\n ");
        lidarSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidarScan", 10, std::bind(&logData::lidarCallback, this, std::placeholders::_1));
        timer = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&logData::logIt, this));

        logIt();
    }

    void logIt()
    {
        // std::cout << "hi" << std::endl;
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan msg)
    {
        for (int i = 0; i < msg.ranges.size(); i++)
        {
            std::cout << msg.ranges[i] << std::endl;
        }
        std::cout << "hohohoho" << std::endl;
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSubscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<logData>());
    rclcpp::shutdown();
    return 0;
}