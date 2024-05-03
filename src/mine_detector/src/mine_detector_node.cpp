#include "mine_detector/mine_detector_node.hpp"

MineDetectorNode::MineDetectorNode(const rclcpp::NodeOptions & options) : Node("mine_detector_node",options) {
    RCLCPP_INFO(this->get_logger(), "Starting Mine Detection Node");
    detector = initDetector();
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            this->imageCallback(msg);
        });
}

MineDetectorNode::~MineDetectorNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Mine Detection Node");
}

std::unique_ptr<MineDetector> MineDetectorNode::initDetector() {
    return std::make_unique<MineDetector>();
}

void MineDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr & msg) {
    auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    detector->getImage(img);
    detector->show();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    MineDetectorNode
)