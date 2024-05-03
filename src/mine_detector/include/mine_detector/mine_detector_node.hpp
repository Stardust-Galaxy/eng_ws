#ifndef MINE_DETECTOR_NODE_
#define MINE_DETECTOR_NODE_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"    

#include "mine_detector/mine_detector.hpp"

class MineDetectorNode : public rclcpp::Node {
public:
    MineDetectorNode(const rclcpp::NodeOptions & options);
    ~MineDetectorNode();
private:
    std::unique_ptr<MineDetector> initDetector();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr & msg);
    std::unique_ptr<MineDetector> detector;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};

#endif