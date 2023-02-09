#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

// timer
class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(std::string node_name) : Node(node_name)
    {
        auto f = 
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("chatter663", 10, std::bind(&SubscriberNode::chatter_callback, this, std::placeholders::_1));
    }

private:
    // attributes
    example_interfaces::msg::String msg_;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

    // methods
    void chatter_callback(const example_interfaces::msg::String::SharedPtr msg);
};