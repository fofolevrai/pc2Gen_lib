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


TEST(test_c_pc2_iterator, resizeFromArgs_Null_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  const size_t width = 0, height = 0;
  
  bool success = ResizeFromArgs(cloud_ptr, width, height);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, resizeFromArgs_Null_cloud_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  const size_t width = 10, height = 10;
  
  bool success = ResizeFromArgs(cloud_ptr, width, height);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, resizeFromArgs_Null_size_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = sensor_msgs__msg__PointCloud2__create();
  const size_t width = 0, height = 0;

  cloud_ptr->point_step = 1;
  
  bool success = ResizeFromArgs(cloud_ptr, width, height);
  ASSERT_TRUE(success);
  EXPECT_TRUE(cloud_ptr);
  EXPECT_EQ(NULL, cloud_ptr->data.data);
}

TEST(test_c_pc2_iterator, resizeFromArgs_same_size)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 * cloud_ptr = sensor_msgs__msg__PointCloud2__create();
  const size_t width = 10, height = 3;

  cloud_ptr->width = 10;
  cloud_ptr->height = 3;
  cloud_ptr->point_step = 2;

  bool success = ResizeFromArgs(cloud_ptr, width, height);
  ASSERT_TRUE(success);
  EXPECT_TRUE(cloud_ptr->data.data);
  EXPECT_EQ(height * width, cloud_ptr->data.size);
  EXPECT_EQ(width * cloud_ptr->point_step, cloud_ptr->row_step);
  EXPECT_EQ(width, cloud_ptr->width);
  EXPECT_EQ(height, cloud_ptr->height);
}

TEST(test_c_pc2_iterator, resizeFromArgs_extend_size)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};
  const size_t width = 6, height = 10;

  /* Mock */
  cloud.point_step = 2;

  bool success = ResizeFromArgs(&cloud, width, height);
  ASSERT_TRUE(success);
  EXPECT_TRUE(cloud.data.data);
  EXPECT_EQ(height * width, cloud.data.size);
  EXPECT_EQ(width * cloud.point_step, cloud.row_step);
  EXPECT_EQ(width, cloud.width);
  EXPECT_EQ(height, cloud.height);
}

