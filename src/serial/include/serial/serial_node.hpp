
#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#pragma once 

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "serial.hpp"
#include "packet.hpp"
#include "msg_interfaces/msg/angle.hpp"
#include "msg_interfaces/msg/receive_data.hpp"
#include "my_msg_interface/srv/referee_graphic_msg.hpp"
#include "crc_check.hpp"
#include "referee_system_client.hpp"

namespace serialport
{
    class SerialPortNode : public rclcpp::Node
    {
        typedef msg_interfaces::msg::Angle AngleMsg;
        typedef msg_interfaces::msg::ReceiveData ReceiveDataMsg;
    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        bool sendData(AngleMsg::SharedPtr msg);
        void angleMsgCallback(AngleMsg::SharedPtr msg);
        void transformData(Packet angle_msg,u_char* data);

        
        void serialWatcher();

    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        //CoordSolver coordsolver_;
        bool print_serial_info_;
        bool print_referee_info_;
        std::unique_ptr<std::thread> receive_thread_;
        
        mutex mutex_;
        bool using_port_;
        bool found_target_;
        atomic<int> mode_;
        atomic<bool> flag_;
        rclcpp::TimerBase::SharedPtr watch_timer_;
        rclcpp::TimerBase::SharedPtr send_timer_;
        
        //tf2
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        
    public:
        rclcpp::Subscription<AngleMsg>::SharedPtr angle_info_sub_;
        rclcpp::Publisher<ReceiveDataMsg>::SharedPtr receive_data_pub_;
        rclcpp::TimerBase::SharedPtr request_timer_;
    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::unique_ptr<SerialPort> initSerialPort();

        //std::unique_ptr<DataTransform> data_transform_;
        //std::unique_ptr<DataTransform> initDataTransform();
    
    private:
        std::map<std::string, int> params_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        void requestDataFromService();
        bool handleServiceResponse(uint16_t cmd_id, const my_msg_interface::srv::RefereeGraphicMsg::Response::SharedPtr response);
        std::array<RM_referee::PacketType, 29> allPacketTypes = {
        RM_referee::PacketType::CustomRobotData,
        RM_referee::PacketType::KeyboardMouseMessage
    };
    }; 
} //namespace serialport

#endif