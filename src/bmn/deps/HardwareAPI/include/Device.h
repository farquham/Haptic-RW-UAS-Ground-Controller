// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstddef>
#include <cstdint>
#include <istream>

namespace Haply
{
    namespace HardwareAPI
    {
        static const uint8_t VECTOR_SIZE = 3;
        static const uint8_t QUATERNION_SIZE = 4;

        namespace Devices
        {

            // This is an internal implementation type and is not part of the
            // public interface.
            struct Device
            {
                explicit Device(std::iostream* stream);

             protected:
                static const uint16_t BUFFER_SIZE = 1024;
                std::iostream* stream;
                uint8_t* w_buffer;
                uint8_t* r_buffer;

                void WriteBytes(size_t n);
                uint8_t ReadHeaderCode();
                int ReadBytes(size_t n);
                void SendCommand(const uint8_t header_code, const float* data,
                                 const int& float_count);

#if defined(_DEBUG)
                void LogInt(int i);
                void LogByte(unsigned char b);
                void LogBytes(unsigned char* buffer, int offset, int length);
#endif
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
