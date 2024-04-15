#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <vector>

struct Packet {
    bool found = false;

    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
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
