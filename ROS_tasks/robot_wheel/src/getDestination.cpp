#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "custom_messsages_interface/msg/vec.hpp"
#include <iostream>

using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("getDestination")
    {

        publisher = this->create_publisher<custom_messsages_interface::msg::Vec>("destination", 10);

        double x, y, z;
        x = 0;
        y = 0;
        z = 0;

        std::cout << "Enter lat value: ";
        std::cin >> x;
        std::cout << "Enter long value: ";
        std::cin >> y;

        auto msg = custom_messsages_interface::msg::Vec();
        msg.x = x;
        msg.y = y;
        msg.z = z;

        publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "entered: lat=%.2f, long=%.2f", msg.x, msg.y);
    }

private:
    rclcpp::Publisher<custom_messsages_interface::msg::Vec>::SharedPtr publisher;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}