#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "../include/serial/referee_system_client.hpp"

RefereeClient::RefereeClient() : Node("referee_client") {
    client = this->create_client<my_msg_interface::srv::RefereeMsg>("RefereeService");
}

bool RefereeClient::connect_server() {
    while(!client->wait_for_service(std::chrono::seconds(1))) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    return true;
}

rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedFuture RefereeClient::send_request(uint16_t cmd_id) {
    auto request = std::make_shared<my_msg_interface::srv::RefereeMsg::Request>();
    request->cmd_id = cmd_id;
    return client->async_send_request(request);
}
