#include <cstdio>
#include "starlite_ros/cpp_header.hpp"
#include "rclcpp/rclcpp.hpp"

// This file is just a test to launch C++ nodes

int main(int argc, char **argv)
{   printf("hello world from cpp node\n");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node_name");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}