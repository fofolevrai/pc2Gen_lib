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


TEST(test_c_pc2_iterator, SetPointFieldsFromDeviceFunc_null_args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;

  /* Mock */
  uint32_t height_size = 8;
  uint32_t width_size = 8;
  bool is_bigendian = false;

  bool success = SetPointFieldsFromDevice(cloud_ptr, "xyz");

  EXPECT_FALSE(success);
}

TEST(test_c_pc2_iterator, SetPointFieldsFromDeviceFunc_unknow_device)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};

  /* Mock */
  uint32_t height_size = 8;
  uint32_t width_size = 8;
  bool is_bigendian = false;

  bool success = SetPointFieldsFromDevice(&cloud, "unknown");

  EXPECT_FALSE(success);
}

TEST(test_c_pc2_iterator, SetPointFieldsFromDeviceFunc_16px_xyz_args)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};

  /* Mock */
  uint32_t height_size = 8;
  uint32_t width_size = 8;
  bool is_bigendian = false;

  bool success = SetPointFieldsFromDevice(&cloud, "xyz");
  
  EXPECT_TRUE(success);
  EXPECT_EQ(is_bigendian, cloud.is_bigendian);
  EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);
  EXPECT_EQ(g_customDevice[0].nbr_pointFields, cloud.fields.size);
  EXPECT_EQ(g_customDevice[0].nbr_pointFields * sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32), cloud.point_step);  //  "xyz" is 'sensor_msgs__msg__PointField__FLOAT32' type
  
  for(uint16_t pf_counter_u16 = 0; pf_counter_u16 < g_customDevice[0].nbr_pointFields; pf_counter_u16++)
  {
    EXPECT_STREQ(g_customDevice[0].pointFieldCapacity[pf_counter_u16], cloud.fields.data[pf_counter_u16].name.data);
  }
}

TEST(test_c_pc2_iterator, SetPointFieldsFromDeviceFunc_HD_rgbd_args)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};

  bool success = SetPointFieldsFromDevice(&cloud, "rgbd_camera");
  
  EXPECT_TRUE(success);
  EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);
  EXPECT_EQ(g_customDevice[2].nbr_pointFields, cloud.fields.size);
  EXPECT_EQ(g_customDevice[2].nbr_pointFields * sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32), cloud.point_step);  //  "rgb" is 'sensor_msgs__msg__PointField__FLOAT32' type
  
  for(uint16_t pf_counter_u16 = 0; pf_counter_u16 < g_customDevice[2].nbr_pointFields; pf_counter_u16++)
  {
    EXPECT_STREQ(g_customDevice[2].pointFieldCapacity[pf_counter_u16], cloud.fields.data[pf_counter_u16].name.data);
  }
}

TEST(test_c_pc2_iterator, SetPointFieldsFromDeviceFunc_4KUHD_rgb_args)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};


  bool success = SetPointFieldsFromDevice(&cloud, "rgb_camera");
  
  EXPECT_TRUE(success);
  EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);
  EXPECT_EQ(g_customDevice[1].nbr_pointFields, cloud.fields.size);
  EXPECT_EQ(g_customDevice[1].nbr_pointFields * sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32), cloud.point_step);  //  "rgb" is 'sensor_msgs__msg__PointField__FLOAT32' type
  EXPECT_EQ(cloud.point_step * cloud.width, cloud.row_step);
  for(uint16_t pf_counter_u16 = 0; pf_counter_u16 < g_customDevice[1].nbr_pointFields; pf_counter_u16++)
  {
    //  std::cout << g_customDevice[1].pointFieldCapacity[pf_counter_u16]<< " : " << cloud.fields.data[pf_counter_u16].name.data;    // debug
    EXPECT_STREQ(g_customDevice[1].pointFieldCapacity[pf_counter_u16], cloud.fields.data[pf_counter_u16].name.data);
  }
}

