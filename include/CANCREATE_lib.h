#pragma once
#include <cstddef>
#include <cstdint>
struct CANCREATEFrame { uint32_t identifier{0}; uint8_t data_length{0}; uint8_t data[8]{}; bool extended{false}; bool remote{false}; };
