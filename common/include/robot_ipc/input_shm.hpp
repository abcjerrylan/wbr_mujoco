#pragma once

#include <cstdint>

namespace robot_ipc
{

#pragma pack(push, 1)
struct input_shm_t
{
    std::uint32_t magic = 0x57425249u;
    std::uint8_t w = 0;
    std::uint8_t s = 0;
    std::uint8_t a = 0;
    std::uint8_t d = 0;
    std::uint8_t q = 0;
    std::uint8_t e = 0;
    std::uint8_t space = 0;
    std::uint8_t r = 0;
    std::uint8_t f = 0;
    std::uint8_t _pad[3] = {};
};
#pragma pack(pop)

}  // namespace robot_ipc
