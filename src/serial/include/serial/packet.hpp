#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <vector>

struct Packet {
    bool found = false;
    int mode = 0;
    float roll = 0.3;
    float pitch = 0.4;
    float yaw = 0.4;
    float x = 0.5;
    float y = 0.7;
    float z = 0.8;
    uint16_t checksum = 0;
}__attribute__((packed));

inline std::vector<uint8_t> toVector(const Packet & data)
{
  std::vector<uint8_t> packet(sizeof(Packet));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(Packet), packet.begin());
  return packet;
}

#endif // PACKET_HPP
