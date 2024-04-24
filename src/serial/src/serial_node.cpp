/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2023-05-31 17:42:16
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport_node.cpp
 */
#include "../include/serial/serial_node.hpp"


using namespace std::placeholders;
namespace serialport
{
    SerialPortNode::SerialPortNode(const rclcpp::NodeOptions& options)
    : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_WARN(this->get_logger(), "Serialport node...");
        try
        {
            serial_port_ = initSerialPort();
            //data_transform_ = initDataTransform();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.reliable();
        qos.durability();
        qos.deadline();
        // qos.best_effort();
        // qos.durability_volatile();
   
        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 5;
        
        angle_info_sub_ = this->create_subscription<AngleMsg>(
            "/angle", 
            qos,
            std::bind(&SerialPortNode::angleMsgCallback, this, _1)
        );
        
        auto client = std::make_shared<RefereeGraphicClient>();
        bool flag = client->connect_server();
        if (!flag) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Service not available.");
        }
        request_timer_ = this->create_wall_timer(
            1ms, 
            [this, client]() {
                uint16_t cmd_id = 0x0304;
                auto response = client->send_request(cmd_id);
                //RCLCPP_INFO(client->get_logger(),"Sending Request:0x%x",cmd_id);
                //handle response
                if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS) {
                    auto result = response.get();
                    if(handleServiceResponse(cmd_id, result)) {
                        //RCLCPP_INFO(client->get_logger(),"Response received");
                    }
                    else {
                        //RCLCPP_ERROR(client->get_logger(),"Response not received");
                    }
                }
                
            }
        );
        
        if (using_port_)
        { 
            watch_timer_ = rclcpp::create_timer(
                this, 
                this->get_clock(), 
                100ms, 
                std::bind(&SerialPortNode::serialWatcher, this)
            );
        }
        
        receive_data_pub_ = this->create_publisher<ReceiveDataMsg>(
            "/receive_data", 
            qos
        );
        receive_thread_ = std::make_unique<std::thread>(&SerialPortNode::receiveData, this);
    }

    SerialPortNode::~SerialPortNode()
    {
    }

    /**
     * @brief 串口监管线程
     * 
     */
    void SerialPortNode::serialWatcher()
    {
        if (access(serial_port_->serial_data_.device.path.c_str(), F_OK) == -1 || !serial_port_->serial_data_.is_initialized)
        {
            serial_port_->serial_data_.is_initialized = true;
            if (!serial_port_->openPort())
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Port open failed!!!");
            }
        }
    }

    

    bool SerialPortNode::handleServiceResponse(uint16_t cmd_id, const my_msg_interface::srv::RefereeGraphicMsg::Response::SharedPtr response) {
        uint16_t response_cmd_id = response->cmd_id;
        if (response_cmd_id != cmd_id) {
            RCLCPP_ERROR(this->get_logger(), "Response cmd_id does not match request cmd_id!");
            return false;
        }
        uint16_t data_size = response->data_stream.size();
        /*For Test*/
        //RM_referee::KeyboardMouseMessageStruct T;
        //memcpy(&T, response->data_stream.data(), response->data_length);
        //RCLCPP_INFO(this->get_logger(), "mouse_x:%ld", T.mouse_x);
        //RCLCPP_INFO(this->get_logger(), "mouse_y:%ld", T.mouse_y);
        //RCLCPP_INFO(this->get_logger(), "mouse_z:%ld", T.mouse_z);
        //RCLCPP_INFO(this->get_logger(), "left_button_down:%d", T.left_button_down);
        //RCLCPP_INFO(this->get_logger(), "right_button_down:%d", T.right_button_down);
        //RCLCPP_INFO(this->get_logger(), "keyboard_value: %d", T.keyboard_value);
        if(response->data_length != 0) {
            mutex_.lock();
            serial_port_->Tdata[0] = 0xA9;     
            for (size_t i = 0; i < response->data_stream.size(); ++i) {
                serial_port_->Tdata[i + 1] = response->data_stream[i];
            }
            serial_port_->sendData();
            //RCLCPP_INFO(this->get_logger(), "Data sent to serial port.");
            mutex_.unlock();
            return true;
        }
        return false;
    }

    /**
     * @brief 数据发送线程
     * @details 标志位为0xA5的package包含模式位、陀螺仪数据、弹速以及上一发弹丸的发弹延迟
     * 
     */

    
    void SerialPortNode::receiveData()
    {
        vector<float> vehicle_pos_info;
        while (1)
        {   
            // 若串口离线则跳过数据发送
            if (!serial_port_->serial_data_.is_initialized)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "Serial port offline!!!");
                usleep(1000);
                continue;
            }

            // 数据读取不成功进行循环
            bool is_receive_data = false; 
            while (!is_receive_data)
            {
                mutex_.lock();
                is_receive_data = serial_port_->receiveData();
                mutex_.unlock();
                if(!is_receive_data)
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "CHECKSUM FAILED OR NO DATA RECVIED!!!");
                    usleep(1000);
                    continue;
                }
            }
            
            u_char flag = serial_port_->serial_data_.rdata[0];
            u_char mode = serial_port_->serial_data_.rdata[1];
            mode_ = mode;
            //printf("header:%c\n", flag);
            /*
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                100,
                "mode:%d", 
                mode
            );
            */
            
            if(flag == 0xA7) {
                short pitch = 0.0;
                short height = 0.0;
                memcpy(&pitch, &serial_port_->serial_data_.rdata[2], 2);
                memcpy(&height, &serial_port_->serial_data_.rdata[4], 2);
                //RCLCPP_INFO(this->get_logger(),"pitch:%.2f", pitch);
                //RCLCPP_INFO(this->get_logger(),"height:%.2f", height);
                
                msg_interfaces::msg::ReceiveData receive_data;
                receive_data.pitch = (float)pitch;
                receive_data.height = (float)height;
                receive_data_pub_->publish(receive_data);
                
                //RCLCPP_INFO(this->get_logger(), "Receive data success!");
            }
            
            /*
            if (flag == 0xA5)
            {
                std::vector<float> quat;
                std::vector<float> gyro;
                std::vector<float> acc;
                float bullet_speed = 0.0;
                float shoot_delay = 0.0;
                
                // Process IMU Datas.
                data_transform_->getQuatData(&serial_port_->serial_data_.rdata[2], quat);
                data_transform_->getGyroData(&serial_port_->serial_data_.rdata[18], gyro);
                data_transform_->getAccData(&serial_port_->serial_data_.rdata[30], acc);
                data_transform_->getBulletSpeed(&serial_port_->serial_data_.rdata[42], bullet_speed);
                data_transform_->getShootDelay(&serial_port_->serial_data_.rdata[46], shoot_delay);
                
                // Gimbal angle.
                // float yaw_angle = 0.0, pitch_angle = 0.0;
                // data_transform_->getYawAngle(flag, &serial_port_->serial_data_.rdata[55], yaw_angle);
                // data_transform_->getPitchAngle(flag, &serial_port_->serial_data_.rdata[59], pitch_angle);
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "yaw_angle:%.2f", yaw_angle);
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "pitch_angle:%.2f", pitch_angle);
                if (print_serial_info_)
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "quat:[%.3f %.3f %.3f %.3f]", quat[0], quat[1], quat[2], quat[3]);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "gyro:[%.3f %.3f %.3f]", gyro[0], gyro[1], gyro[2]);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "acc:[%.3f %.3f %.3f]", acc[0], acc[1], acc[2]);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "bullet_speed::%.3f", bullet_speed);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "shoot_delay:%.3f", shoot_delay);
                }

                rclcpp::Time now = this->get_clock()->now();
                SerialMsg serial_msg;
                // sensor_msgs::msg::Imu imu_msg;
                serial_msg.header.frame_id = "serial";
                serial_msg.header.stamp = now;
                serial_msg.imu.header.frame_id = "imu_link";
                serial_msg.imu.header.stamp = now;
                // imu_msg.header.frame_id = "imu_link";
                // imu_msg.header.stamp = now;
                serial_msg.mode = mode;
                serial_msg.bullet_speed = bullet_speed;
                serial_msg.shoot_delay = shoot_delay;
                // 下位机乘1000发，会准一点
                serial_msg.imu.orientation.w = quat[0]/1000;
                serial_msg.imu.orientation.x = quat[1]/1000;
                serial_msg.imu.orientation.y = quat[2]/1000;
                serial_msg.imu.orientation.z = quat[3]/1000;
                serial_msg.imu.angular_velocity.x = gyro[0];
                serial_msg.imu.angular_velocity.y = gyro[1];
                serial_msg.imu.angular_velocity.z = gyro[2];
                serial_msg.imu.linear_acceleration.x = acc[0];
                serial_msg.imu.linear_acceleration.y = acc[1];
                serial_msg.imu.linear_acceleration.z = acc[2];
                // imu_msg.orientation.w = quat[0];
                // imu_msg.orientation.x = quat[1];
                // imu_msg.orientation.y = quat[2];
                // imu_msg.orientation.z = quat[3];
                // imu_msg.angular_velocity.x = gyro[0];
                // imu_msg.angular_velocity.y = gyro[1];
                // imu_msg.angular_velocity.z = gyro[2];
                // imu_msg.linear_acceleration.x = acc[0];
                // imu_msg.linear_acceleration.y = acc[1];
                // imu_msg.linear_acceleration.z = acc[2];
                geometry_msgs::msg::TransformStamped t;

                // Read message content and assign it to corresponding tf variables
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "base_link";
                t.child_frame_id = "imu_link";

                // Translation
                t.transform.translation.x = 0.0;
                t.transform.translation.y = 0.0;
                t.transform.translation.z = 0.0;

                // Rotation
                t.transform.rotation.x = serial_msg.imu.orientation.x;
                t.transform.rotation.y = serial_msg.imu.orientation.y;
                t.transform.rotation.z = serial_msg.imu.orientation.z;
                t.transform.rotation.w = serial_msg.imu.orientation.w;

                // Send the transformation
                tf_broadcaster_->sendTransform(t);

                // Pub serial msg
                serial_msg_pub_->publish(std::move(serial_msg));
                // imu_msg_pub_->publish(std::move(imu_msg));
            }
            */
        }
    }
    

    /**
     * @brief 数据发送函数
     * 
     * @param target_info 云台信息（pitch、yaw轴偏转角度等）
     * @return true 
     * @return false 
     */
    bool SerialPortNode::sendData(AngleMsg::SharedPtr angle_info)
    {
        /*
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(),
            100,
            "sending_mode: %d", 
            mode_
        );
        */
        if (this->using_port_)
        {   
            Packet angle_data;
           
            angle_data.found = angle_info->found;
            angle_data.roll= angle_info->roll;
            angle_data.pitch = angle_info->pitch;
            angle_data.yaw= angle_info->yaw;
            angle_data.x = angle_info->x;
            angle_data.y = angle_info->y;
            angle_data.z = angle_info->z;
            serialport::CrcCheck crc_check_ = CrcCheck();
            crc_check_.Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&angle_data), sizeof(angle_data));

            
          

            // 根据不同mode进行对应的数据转换
            transformData(angle_data, serial_port_->Tdata);
            
            // Time of entire loop.
            /*
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time start = angle_info->header.stamp;
            double duration = (now.nanoseconds() - start.nanoseconds()) / 1e6;
            
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                100, 
                "All_delay:%.2fms", 
                duration
            );
            */
            // 数据发送
            mutex_.lock();
            serial_port_->sendData();
            mutex_.unlock();
            return true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Not use port...");
            return false;
        }
    }
    
    void SerialPortNode::transformData(Packet angle_data, u_char* Tdata)
    {
      // 0xA5
      Tdata[0] = 0xA7;
      // mode
      Tdata[1] = mode_;
      Tdata[2] = angle_data.found;
      // roll
      float* roll = reinterpret_cast<float*>(&angle_data.roll);
      u_char* rollBytes = reinterpret_cast<u_char*>(roll);
      for (int i = 0; i < 4; i++)
      {
        Tdata[3 + i] = rollBytes[i];
      }

      // pitch
      float* pitch = reinterpret_cast<float*>(&angle_data.pitch);
      u_char* pitchBytes = reinterpret_cast<u_char*>(pitch);
      for (int i = 0; i < 4; i++)
      {
        Tdata[7 + i] = pitchBytes[i];
      }

      // yaw
      float* yaw = reinterpret_cast<float*>(&angle_data.yaw);
      u_char* yawBytes = reinterpret_cast<u_char*>(yaw);
      for (int i = 0; i < 4; i++)
      {
        Tdata[11 + i] = yawBytes[i];
      }

      // x
      float* x = reinterpret_cast<float*>(&angle_data.x);
      u_char* xBytes = reinterpret_cast<u_char*>(x);
      for (int i = 0; i < 4; i++)
      {
        Tdata[15 + i] = xBytes[i];
      }

      // y
      float* y = reinterpret_cast<float*>(&angle_data.y);
      u_char* yBytes = reinterpret_cast<u_char*>(y);
      for (int i = 0; i < 4; i++)
      {
        Tdata[19 + i] = yBytes[i];
      }

      // z
      float* z = reinterpret_cast<float*>(&angle_data.z);
      u_char* zBytes = reinterpret_cast<u_char*>(z);
      for (int i = 0; i < 4; i++)
      {
        Tdata[23 + i] = zBytes[i];
      }
    
    }

    void SerialPortNode::angleMsgCallback(AngleMsg::SharedPtr msg)
    {
        if(!sendData(msg))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub angle msg...");
        }
    }

    /**
     * @brief 修改参数
     * 
     * @param param 参数服务器变动的参数
     * @return true 
     * @return false 
     */
    bool SerialPortNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->using_port_ = param.as_bool();
            break;
        case 1:
            this->baud_ = param.as_int();
            break;
        case 2:
            this->found_target_ = param.as_bool();
            break;
        case 3:
            this->print_serial_info_ = param.as_bool();
            break;
        case 4:
            this->print_referee_info_= param.as_bool();
            break;
        default:
            break;
        }
        return true;
    }
    
    /**
     * @brief 参数回调函数
     * 
     * @param params 参数服务器变动的参数
     * @return rcl_interfaces::msg::SetParametersResult 
     */
    rcl_interfaces::msg::SetParametersResult SerialPortNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(param);
        }
        return result;
    }

    /**
     * @brief 初始化串口类
     * 
     * @return std::unique_ptr<SerialPort> 
     */
    std::unique_ptr<SerialPort> SerialPortNode::initSerialPort()
    {
        params_map_ =
        {
            {"using_port", 0},
            {"baud", 1},
            {"found_target", 2},
            {"print_serial_info", 3},
            {"print_referee_info", 4}
        };

        this->declare_parameter<std::string>("port_id", "483/5740/200");
        this->get_parameter("port_id", this->id_);

        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("baud", this->baud_);

        this->declare_parameter<bool>("using_port", true);
        this->get_parameter("using_port", this->using_port_);

        this->declare_parameter<bool>("found_target", false);
        this->get_parameter("found_target", this->found_target_);

        this->declare_parameter("print_serial_info", false);
        this->get_parameter("print_serial_info", this->print_serial_info_);

        this->declare_parameter("print_referee_info", false);
        this->get_parameter("print_referee_info", this->print_referee_info_);

        return std::make_unique<SerialPort>(id_, baud_, using_port_);
    }

    /**
     * @brief 初始化数据转换类
     * 
     * @return std::unique_ptr<DataTransform> 
     */
    /*
    std::unique_ptr<DataTransform> SerialPortNode::initDataTransform()
    {
        return std::make_unique<DataTransform>();
    }
    */
} //namespace serialport

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::SerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serialport::SerialPortNode)