#ifndef PACKET_HPP
#define PACKET_HPP

#include <cstdint>
#include <vector>

struct Packet {
    bool found = false;

    float quaternion1 = 1.023;
    float quaternion2 = 2.023;
    float quaternion3 = 3.045;
    float quaternion4 = 4.054;
    float x = 1.078;
    float y = 1.056;
    float z = 1.045;
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
