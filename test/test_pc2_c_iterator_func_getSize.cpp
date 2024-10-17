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


TEST(test_c_pc2_iterator, GetSizeFunc_NullArgs)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  
  size_t l_cloud_size_t = GetSize(cloud);
  ASSERT_EQ((int) l_cloud_size_t, 0);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2InstanciatedWithNoPoints)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(0, l_cloud_size_t);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2Instanciated_noPoint_step)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(0, l_cloud_size_t);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2Instanciated_higher_fractionnal_size)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  cloud->data.size = 3;
  cloud->point_step = 2;

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(0, l_cloud_size_t);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2Instanciated_lower_fractionnal_size)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  cloud->data.size = 2;
  cloud->point_step = 3;

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(0, l_cloud_size_t);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2Instanciated_noData)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  cloud->data.size = 2;
  cloud->point_step = 2;

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(0, l_cloud_size_t);
}

TEST(test_c_pc2_iterator, GetSizeFunc_pc2Instanciated_even_args)
{
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
    /* Mock */
  uint32_t height_size = 8;
  uint32_t width_size = 8;
  bool is_bigendian = false;

  bool success = CreatePointCloud2FromDevice(cloud, "xyz", height_size, width_size, is_bigendian);

  size_t l_cloud_size_t = GetSize(cloud);

  ASSERT_EQ(height_size * width_size, l_cloud_size_t);
}
  