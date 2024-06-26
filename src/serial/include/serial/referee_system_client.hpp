#pragma once

#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "DataType.h"

class RefereeClient : public rclcpp::Node {
public:
    RefereeClient();
    bool connect_server();
    rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedFuture send_request(uint16_t cmd_id);

private:
    rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedPtr client;
};