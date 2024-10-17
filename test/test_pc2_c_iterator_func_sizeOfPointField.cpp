// BSD 3-Clause License

// Copyright (c) 2024, fofolevrai

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/**
 * @brief Using Google Test to Unit Test C Code web link
 * @link https://distek.com/2014/03/21/using-google-test-unit-test-c-code/ @endlink
 */

/**
 * @brief How to setup vs code to colcon build, test and debug
 * @link https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html @endlink
 */


#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/point_field.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include "point_cloud2_iterator.h"

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_WrongArgs)
{
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::uniform_int_distribution<uint8_t> distr(sensor_msgs__msg__PointField__FLOAT64 + 1, std::numeric_limits<uint8_t>::max());
  // uint8_t rand_num = distr(gen);

  for(uint8_t num_u8 = 255; sensor_msgs__msg__PointField__FLOAT64 < num_u8; num_u8--)
  {
    //  std::cerr << "tries : " << (int)tries << " | ";
    ASSERT_EQ(0, sizeOfPointField(num_u8));
    //  std::cerr << "\r\n";
  }
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs)
{
  for(uint8_t num_u8 = sensor_msgs__msg__PointField__INT8; sensor_msgs__msg__PointField__FLOAT64 < num_u8; num_u8++)
  {
    //  std::cerr << "tries : " << (int)tries << " | ";
    ASSERT_NE(0, sizeOfPointField(num_u8));
    //  std::cerr << "\r\n";
  }
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_INT8)
{
    ASSERT_EQ(sizeof(uint8_t), sizeOfPointField(sensor_msgs__msg__PointField__INT8));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_UINT8)
{
    ASSERT_EQ(sizeof(uint8_t), sizeOfPointField(sensor_msgs__msg__PointField__UINT8));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_INT16)
{
    ASSERT_EQ(sizeof(uint16_t), sizeOfPointField(sensor_msgs__msg__PointField__INT16));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_UINT16)
{
    ASSERT_EQ(sizeof(uint16_t), sizeOfPointField(sensor_msgs__msg__PointField__UINT16));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_INT32)
{
    ASSERT_EQ(sizeof(uint32_t), sizeOfPointField(sensor_msgs__msg__PointField__UINT32));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_UINT32)
{
    ASSERT_EQ(sizeof(uint32_t), sizeOfPointField(sensor_msgs__msg__PointField__UINT32));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_FLOAT32)
{
    ASSERT_EQ(sizeof(uint32_t), sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32));
}

TEST(test_c_pc2_iterator, sizeOfPointFieldFunc_RightArgs_FLOAT64)
{
    ASSERT_EQ(sizeof(uint64_t), sizeOfPointField(sensor_msgs__msg__PointField__FLOAT64));
}