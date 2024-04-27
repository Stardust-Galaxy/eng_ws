#pragma once

#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "DataType.h"

class RefereeGraphicClient : public rclcpp::Node {
public:
    RefereeGraphicClient();
    bool connect_server();
    rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedFuture send_request(uint16_t cmd_id);

private:
    rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedPtr client;
};
