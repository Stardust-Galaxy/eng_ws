/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#pragma once 
// #include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <mutex>
#include "enums.h"
namespace RM_referee{
    // Packet header structure
    #pragma pack(1)
    struct PacketHeader {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t SequenceNumber;
        uint8_t CRC8;
    } ;
    #pragma pack()
    static_assert(sizeof(PacketHeader) == 5, "PacketHeader must be 5 bytes long with packing");
    static constexpr uint8_t StartOfFrame = 0xa5;

    // Base packet
    class RefereePacket {
        public:
            RefereePacket(){};
            virtual ~RefereePacket(){};
            virtual uint16_t GetID() = 0;
            virtual uint16_t GetDataLength() = 0;
            /**
             * @return 处理的字节数
            */
            virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) {
                (void)cmd_id;
                (void)data;
                (void)data_size;
                std::cout<<"[Warning : This should not appear !]\n";
                return 0 ;
            };

    };
    
    #define GENERATEPACK(TYPE,STRUCT) \
    class TYPE##Packet : public RefereePacket { \
    protected:\
    public:\
        STRUCT m_value;\
        std::mutex m_mutex;\
        TYPE##Packet(){};\
        ~TYPE##Packet(){};\
        void testsuccess(){std::cout<<"\nSuccess!!!\n";};\
        static uint16_t StaticGetID(){return uint16_t(PacketType::TYPE);};\
        uint16_t GetID(){return StaticGetID();};\
        static uint16_t StaticGetDataLength(){return sizeof(STRUCT);};\
        uint16_t GetDataLength() {return StaticGetDataLength();};\
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; \
    };

    /**
    @brief  如何使用：
            How to use GENERATEPACK(TYPE,STRUCT):
            Example：0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            定义数据包结构体并断言数据包大小
            
            struct ExtSupplyProjectileActionStruct { 
                uint8_t reserved; 
                uint8_t supply_robot_id;  
                uint8_t supply_projectile_step; 
                uint8_t supply_projectile_num; 
            };
            static_assert(sizeof(ExtSupplyProjectileActionStruct) == 4, "ExtSupplyProjectileActionStruct must be 4 bytes long with packing");
            GENERATEPACK(ExtSupplyProjectileAction,ExtSupplyProjectileActionStruct)
    @warning  type,struct不要重复；type请查阅enum.h
    @brief 等效于：
            //0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            class ExtSupplyProjectileActionPacket : public RefereePacket { 
            TODO fix it
            };
    */

    //0x0201 RobotStatePacket 6 RobotState
    #pragma pack(push, 1)
    struct RobotStateStruct{ 
        uint8_t robot_id; 
        uint8_t robot_level; 
        uint16_t current_HP;   
        uint16_t maximum_HP; 
        uint16_t shooter_barrel_cooling_value; 
        uint16_t shooter_barrel_heat_limit; 
        uint16_t chassis_power_limit;   
        uint8_t power_management_gimbal_output : 1; 
        uint8_t power_management_chassis_output : 1;   
        uint8_t power_management_shooter_output : 1;  }; 
    #pragma pack(pop)
    static_assert(sizeof(RobotStateStruct) == 13 , "RobotStateStruct must be 3 bytes long with packing");
    GENERATEPACK(RobotState,RobotStateStruct)

    //0x0209 RobotRfidStatePacket 4 RobotRfidState
    struct RobotRfidStateStruct { 
        uint32_t rfid_status; };
    static_assert(sizeof(RobotRfidStateStruct) == 4, "RobotRfidStateStruct must be 16 bytes long with packing");
    GENERATEPACK(RobotRfidState,RobotRfidStateStruct)
}