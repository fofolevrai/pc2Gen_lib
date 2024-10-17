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


TEST(test_c_pc2_iterator, resize_Null_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  size_t data_alloc = 0;
  
  bool success = dataResize(cloud_ptr, data_alloc);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, resize_Null_cloud_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  size_t data_alloc = 10;
  
  bool success = dataResize(cloud_ptr, data_alloc);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, resize_Null_size_Args)
{
  sensor_msgs__msg__PointCloud2 cloud = (const sensor_msgs__msg__PointCloud2) {0xff};
  size_t data_alloc = 0;
  
  cloud.point_step = 1;

  bool success = dataResize(&cloud, data_alloc);
  ASSERT_TRUE(success);
  EXPECT_EQ(NULL, cloud.data.data);
}

TEST(test_c_pc2_iterator, resize_same_size)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  size_t data_alloc = 4;
  
  cloud->point_step = 10;

  bool success = dataResize(cloud, data_alloc);
  ASSERT_TRUE(success);
  EXPECT_TRUE(cloud->data.data);  //  If size equal, no operation is performed, thus resulting to an unmodified data pointer (here nil as declared in mock)
  EXPECT_EQ(data_alloc, cloud->data.size);
}

TEST(test_c_pc2_iterator, resize_extend_size)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};
  size_t data_alloc = 4;
  cloud.point_step = 10;

  bool success = dataResize(&cloud, data_alloc);
  ASSERT_TRUE(success);
  EXPECT_TRUE(cloud.data.data);
  EXPECT_EQ(data_alloc, cloud.data.size);
}

