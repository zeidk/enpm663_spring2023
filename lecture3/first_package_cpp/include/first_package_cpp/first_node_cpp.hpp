#pragma once

#include <rclcpp/rclcpp.hpp>

class FirstNode : public rclcpp::Node
{
public:
    FirstNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "first_node_cpp node running");
    }

private:

};