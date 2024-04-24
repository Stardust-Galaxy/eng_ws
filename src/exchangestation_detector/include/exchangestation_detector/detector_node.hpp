#pragma once
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


#include <memory>
#include <vector>

#include "exchangestation_detector/exchangestation_detector.hpp"
#include "msg_interfaces/msg/angle.hpp"
#include "msg_interfaces/msg/receive_data.hpp"
class exchangestation_detector_node : public rclcpp::Node {
public:
    exchangestation_detector_node(const rclcpp::NodeOptions & options);
    ~exchangestation_detector_node();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr & msg);

    std::unique_ptr<ExchangeStationDetector> initDetector();
    //Detector
    std::unique_ptr<ExchangeStationDetector> detector;
    //Publisher
    rclcpp::Publisher<msg_interfaces::msg::Angle>::SharedPtr angle_pub;
    //Subscriber
    rclcpp::Subscription<msg_interfaces::msg::ReceiveData>::SharedPtr receive_data_sub;
    //Camera Info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    cv::Point2f cam_center;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info;
    //Image subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    //Receive data subscription
    void setReceiveData(const msg_interfaces::msg::ReceiveData::SharedPtr msg);
    //Receive Data
    msg_interfaces::msg::ReceiveData receive_data;

    float currentPitch = 30.0;
    float currentHeight = 200.0;
    

};