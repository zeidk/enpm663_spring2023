#include <rclcpp/rclcpp.hpp>
#include "first_node_cpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FirstNode>("first_node_cpp");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
