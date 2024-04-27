#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "my_msg_interface/srv/referee_msg.hpp"
#include "../include/serial/referee_system_graphic_client.hpp"

RefereeGraphicClient::RefereeGraphicClient() : Node("referee_graphic_client") {
    client = this->create_client<my_msg_interface::srv::RefereeMsg>("RefereeGraphicService");
}

bool RefereeGraphicClient::connect_server() {
    while(!client->wait_for_service(std::chrono::seconds(1))) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    return true;
}

rclcpp::Client<my_msg_interface::srv::RefereeMsg>::SharedFuture RefereeGraphicClient::send_request(uint16_t cmd_id) {
    auto request = std::make_shared<my_msg_interface::srv::RefereeMsg::Request>();
    request->cmd_id = cmd_id;
    return client->async_send_request(request);
}
