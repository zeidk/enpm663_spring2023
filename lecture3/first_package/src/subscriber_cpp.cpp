#include <string>
#include "subscriber_cpp.hpp"
#include <example_interfaces/msg/string.hpp>

using namespace std::chrono_literals;

void SubscriberNode::chatter_callback(const example_interfaces::msg::String::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received data: " << msg->data);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto subscriber_cpp = std::make_shared<SubscriberNode>("subscriber_cpp");
    rclcpp::spin(subscriber_cpp);
    rclcpp::shutdown();
}
