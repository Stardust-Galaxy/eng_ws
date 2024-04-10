#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;
using namespace sensor_msgs::msg;
using namespace rclcpp;

Mat g_image;
Mat distortionMatrix = Mat::zeros(1, 5, CV_64F);
int k1, k2 , p1 , p2 , k3;
void updateDistortionMatrix(int, void* ) {
    double k_1 = (double)(k1 - 10000) / 10000;
    double k_2 = (double)(k2 - 10000) / 10000;
    double p_1 = (double)(p1 - 10000) / 10000;
    double p_2 = (double)(p2 - 10000)/ 10000;
    double k_3 = (double)(k3 - 10000)/ 10000;
    distortionMatrix.at<double>(0, 0) = k_1;
    distortionMatrix.at<double>(0, 1) = k_2;
    distortionMatrix.at<double>(0, 2) = p_1;
    distortionMatrix.at<double>(0, 3) = p_2;
    distortionMatrix.at<double>(0, 4) = k_3;


    Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1850.906255, 0.0000, 496.601643,
         0.0000, 1846.847240, 212.542900,
         0.0000, 0.0000, 1.0000);
    Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionMatrix,g_image.size(),1);
    Mat map1 = Mat(g_image.size(), CV_32FC1);
    Mat map2 = Mat(g_image.size(), CV_32FC1);
    initUndistortRectifyMap(cameraMatrix, distortionMatrix, Mat(), newCameraMatrix, g_image.size(), CV_16SC2, map1, map2);
    Mat undistortedImage;
    remap(g_image, undistortedImage, map1, map2, INTER_LINEAR);
    imshow("Undistorted Image", undistortedImage);
    cv::waitKey(1);
}

int main(int argc,char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<rclcpp::Node>("camera_calibration");
    auto camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, [](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        auto camera_info = msg;
    });
    auto sub = node->create_subscription<Image>("/image_raw", rclcpp::SensorDataQoS(), [](const Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        g_image = cv_ptr->image;
    });
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("Undistorted Image", WINDOW_AUTOSIZE);
    cv::waitKey(1);
    Mat distortionMatrix = Mat::zeros(1, 5, CV_64F);
    createTrackbar("k1", "Undistorted Image", &k1, 20000, updateDistortionMatrix, nullptr);
    createTrackbar("k2", "Undistorted Image", &k2, 20000, updateDistortionMatrix, nullptr);
    createTrackbar("p1", "Undistorted Image", &p1, 20000, updateDistortionMatrix, nullptr);
    createTrackbar("p2", "Undistorted Image", &p2, 20000, updateDistortionMatrix, nullptr);
    createTrackbar("k3", "Undistorted Image", &k3, 20000, updateDistortionMatrix, nullptr);
    while(rclcpp::ok()) {
        if(!g_image.empty()) {
            updateDistortionMatrix(0, nullptr);
            cv::imshow("Original Image", g_image);
            cv::waitKey(1);
        }
        rclcpp::spin_some(node);
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}