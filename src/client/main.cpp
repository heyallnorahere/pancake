#include <rclcpp/rclcpp.hpp>

#include "pancake/client/client.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pancake::client::Client>());
    rclcpp::shutdown();
    
    return 0;
}