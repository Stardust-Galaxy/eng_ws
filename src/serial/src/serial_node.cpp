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
        second_angle_info_sub_ = this->create_subscription<AngleMsg>(
            "/second",
            qos,
            std::bind(&SerialPortNode::angleMsgCallback, this, _1)
        );
        auto graphicClient = std::make_shared<RefereeGraphicClient>();
        auto regularClient = std::make_shared<RefereeClient>();
        bool flag = graphicClient->connect_server();
        if (!flag) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Service not available.");
        }
        bool flag2 = regularClient->connect_server();
        if(!flag2) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Service not available.");
        }
        request_graphic_timer_ = this->create_wall_timer(
            1ms, 
            [this, graphicClient]() {
                uint16_t cmd_id = 0x0304;
                auto response = graphicClient->send_request(cmd_id);
                //RCLCPP_INFO(graphicClient->get_logger(),"Sending Request:0x%x",cmd_id);
                //handle response
                if (rclcpp::spin_until_future_complete(graphicClient,response) == rclcpp::FutureReturnCode::SUCCESS) {
                    auto result = response.get();
                    if(handleGraphicServiceResponse(cmd_id, result)) {
                        //RCLCPP_INFO(graphicClient->get_logger(),"Response:0x%x received",cmd_id);
                    }
                    else {
                        RCLCPP_ERROR(graphicClient->get_logger(),"Response:0x%x not received",cmd_id);
                    }
                }
                
            }
        );
        request_timer = this->create_wall_timer(
            100ms,
            [this,regularClient]() {
                RM_referee::RobotStateStruct RobotStateT;
                RM_referee::RobotRfidStateStruct RobotRfidStateT;
                for(auto packetType : allRegularPacketTypes) {
                    uint cmd_id = (uint16_t)packetType;
                    auto response = regularClient->send_request(cmd_id);
                    if(rclcpp::spin_until_future_complete(regularClient,response) == rclcpp::FutureReturnCode::SUCCESS) {
                        auto result = response.get();
                        if(result->data_stream.size() != 0) {
                            if(result->cmd_id == 0x0202) 
                                memcpy(&RobotStateT,result->data_stream.data(),result->data_length);
                            else if(result->cmd_id == 0x0209)
                                memcpy(&RobotRfidStateT,result->data_stream.data(),result->data_length);
                        }
                    }
                }
                handleRegularServiceResponse(RobotStateT,RobotRfidStateT);
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

    

    bool SerialPortNode::handleGraphicServiceResponse(uint16_t cmd_id, const my_msg_interface::srv::RefereeMsg::Response::SharedPtr response) {
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

    bool SerialPortNode::handleRegularServiceResponse(RM_referee::RobotStateStruct RobotStateT,RM_referee::RobotRfidStateStruct RobotRfidStateT) {
        mutex_.lock();
        serial_port_->Tdata[0] = 0xAB;
        memcpy(&serial_port_->Tdata[1],&RobotRfidStateT,sizeof(RM_referee::RobotRfidStateStruct));
        serial_port_->Tdata[5] |= (RobotStateT.power_management_chassis_output << 0);
        serial_port_->Tdata[5] |= (RobotStateT.power_management_gimbal_output << 1);
        serial_port_->Tdata[5] |= (RobotStateT.power_management_shooter_output << 2);
        serial_port_->sendData();
        mutex_.unlock();
        RCLCPP_INFO(this->get_logger(),"Two packet sent together successfully!");
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
            angle_data.mode = angle_info->mode;
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
      Tdata[1] = angle_data.mode;
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