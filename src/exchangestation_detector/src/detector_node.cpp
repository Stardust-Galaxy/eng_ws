
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rmw/qos_profiles.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

#include <vector>
#include <memory>
#include <chrono>
#include <functional>

#include "exchangestation_detector/detector_node.hpp"
#include "exchangestation_detector/exchangestation_detector.hpp"
#include "serial/packet.hpp"
using namespace std::chrono_literals;
using namespace cv;
exchangestation_detector_node::exchangestation_detector_node(const rclcpp::NodeOptions & options) :Node("exchangestation_detector", options) {
    // Create a node
    RCLCPP_INFO(this->get_logger(), "Starting Exchange Station Recognition Node");
    // Create a detector
    detector = initDetector();
    // Create a publisher
    angle_pub = this->create_publisher<msg_interfaces::msg::Angle>("angle", rclcpp::SensorDataQoS());
    // Create a camera info subscriber
    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info = msg;
        cam_center = cv::Point2f(msg->width / 2, msg->height / 2);
    });
    // Create an image subscriber
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg);
    });

}

exchangestation_detector_node::~exchangestation_detector_node() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Exchange Station Recognition Node");
}

std::unique_ptr<ExchangeStationDetector> exchangestation_detector_node::initDetector() {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].step = 1;
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = 255;
    this->declare_parameter("red_threshold", 150, descriptor);
    this->declare_parameter("blue_threshold", 80, descriptor);

    descriptor.description = "0-RED,`1-BLUE";
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = 1;
    this->declare_parameter("detect_color", 0, descriptor);
    auto detectColor = this->get_parameter("detect_color").as_int();
    int redThreshold = this->get_parameter("red_threshold").as_int();
    int blueThreshold = this->get_parameter("blue_threshold").as_int();

    return std::make_unique<ExchangeStationDetector>(redThreshold, blueThreshold, detectColor);
}

void exchangestation_detector_node::imageCallback(const sensor_msgs::msg::Image::SharedPtr & msg) {
    auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    detector->getImage(img);
    detector->selectContours();
    detector->getCorners();
    Packet packet = detector->solveAngle();
    msg_interfaces::msg::Angle angle_msg;
    angle_msg.found = packet.found;
    angle_msg.quaternion1 = packet.quaternion1;
    angle_msg.quaternion2 = packet.quaternion2;
    angle_msg.quaternion3 = packet.quaternion3;
    angle_msg.quaternion4 = packet.quaternion4;
    angle_msg.x = packet.x;
    angle_msg.y = packet.y;
    angle_msg.z = packet.z;
    angle_pub->publish(angle_msg);
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  exchangestation_detector_node
)