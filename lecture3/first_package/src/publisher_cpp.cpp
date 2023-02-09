#include <rclcpp/rclcpp.hpp>
#include "publisher_cpp.hpp"

void PublisherNode::timer_callback()
{
    msg_.data = "Help me Obi-Wan Kenobi, you are my only hope.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << msg_.data);
    publisher_->publish(msg_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto publisher_cpp = std::make_shared<PublisherNode>("publisher_cpp");
    rclcpp::spin(publisher_cpp);
    rclcpp::shutdown();
}