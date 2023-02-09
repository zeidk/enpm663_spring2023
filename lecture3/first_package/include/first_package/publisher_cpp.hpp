#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

// timer
class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(std::string node_name) : Node(node_name)
    {
        // initialize the message
        msg_ = example_interfaces::msg::String();
        // timer callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0)),
                                         std::bind(&PublisherNode::timer_callback, this));

        // publisher
        publisher_ = this->create_publisher<example_interfaces::msg::String>("chatter663", 10);
    }

private:
    // attributes
    example_interfaces::msg::String msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;

    // methods
    void timer_callback();
};