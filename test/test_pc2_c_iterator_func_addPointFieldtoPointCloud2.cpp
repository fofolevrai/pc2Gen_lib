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


TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_Null_cloud_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  sensor_msgs__msg__PointField field_to_add = {
    .name = {"test_field", 10, 11},
    .offset = 10,
    .datatype = sensor_msgs__msg__PointField__INT8,
    .count = 1
  };
  
  bool success = addPointFieldtoPointCloud2(field_to_add, cloud_ptr);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_Null_PointField_Capacity_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = sensor_msgs__msg__PointCloud2__create();
  sensor_msgs__msg__PointField field_to_add ={
    .name = {"test_field", 10, 0},
    .offset = 10,
    .datatype = sensor_msgs__msg__PointField__INT8,
    .count = 1
  };
  
  bool success = addPointFieldtoPointCloud2(field_to_add, cloud_ptr);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_PointField_name_field_size_higher_capacity_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = sensor_msgs__msg__PointCloud2__create();
  sensor_msgs__msg__PointField field_to_add ={
    .name = {"test_field", 2, 1},
    .offset = 10,
    .datatype = sensor_msgs__msg__PointField__INT8,
    .count = 1
  };
  
  bool success = addPointFieldtoPointCloud2(field_to_add, cloud_ptr);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_PointField_Zero_Offset)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};
  sensor_msgs__msg__PointField field_to_add ={
    .name = {"test_field", 10, 11},
    .offset = 0,
    .datatype = sensor_msgs__msg__PointField__INT8,
    .count = 1
  };
  
  bool success = addPointFieldtoPointCloud2(field_to_add, &cloud);

  ASSERT_TRUE(success);
  EXPECT_STREQ(field_to_add.name.data, cloud.fields.data[field_to_add.offset].name.data);
  EXPECT_EQ(field_to_add.offset, cloud.fields.data->offset);
    //  EXPECT_EQ(field_to_add.offset + field_to_add.count * sizeOfPointField(field_to_add.datatype), cloud.point_step);  // @bug fofolevrai Add the possibility for the user to force offset
  EXPECT_EQ(field_to_add.datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(field_to_add.count, cloud.fields.data[0].count);
  EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);
  EXPECT_EQ(field_to_add.count, cloud.fields.size);
}


TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_PointField_Offset)
{
  sensor_msgs__msg__PointCloud2 cloud = {0};
  sensor_msgs__msg__PointField field_to_add ={
    .name = {"test_field", 10, 11},
    .offset = 10,
    .datatype = sensor_msgs__msg__PointField__INT8,
    .count = 1
  };
  
  bool success = addPointFieldtoPointCloud2(field_to_add, &cloud);
  ASSERT_TRUE(success);
  EXPECT_STREQ(field_to_add.name.data, cloud.fields.data->name.data);
  EXPECT_EQ(field_to_add.offset, cloud.fields.data[0].offset);
  //  EXPECT_EQ(field_to_add.offset + field_to_add.count * sizeOfPointField(field_to_add.datatype), cloud.point_step);  // @bug fofolevrai Add the possibility for the user to force offset
  EXPECT_EQ(field_to_add.datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(field_to_add.count, cloud.fields.data[0].count);
  EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);
  EXPECT_EQ(field_to_add.count, cloud.fields.size);
}

TEST(test_c_pc2_iterator, addPointFieldtoPointCloud2Func_PointField_Multiple)
{
  uint32_t nbr_fields_u32_size =0, point_step_size_u32 = 0, i = 0;
  bool success = 0;
  sensor_msgs__msg__PointCloud2 cloud = {0};
  size_t fields_to_add_size = 5;
  sensor_msgs__msg__PointField fields_to_add[fields_to_add_size] =
  {
    {
      .name = {"test_field_01", 13, 14},
      .offset = 0,
      .datatype = sizeOfPointField(sensor_msgs__msg__PointField__INT8),
      .count = 1
    },
    {
      .name = {"test_field_02", 13, 14},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8),
      .datatype = sensor_msgs__msg__PointField__INT16,
      .count = 1
    },
    {
      .name = {"test_field_03", 13, 14},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8 + sensor_msgs__msg__PointField__INT16),
      .datatype = sensor_msgs__msg__PointField__FLOAT32,
      .count = 1
    },
    {
      .name = {"test_field_04", 13, 14},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8 + sensor_msgs__msg__PointField__INT16 + sensor_msgs__msg__PointField__FLOAT32),
      .datatype = sensor_msgs__msg__PointField__FLOAT64,
      .count = 1
    },
    {
      .name = {"test_field_05", 13, 14},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8 + sensor_msgs__msg__PointField__INT16 + sensor_msgs__msg__PointField__FLOAT32 + sensor_msgs__msg__PointField__FLOAT64),
      .datatype = sensor_msgs__msg__PointField__INT16,
      .count = 2
    }
  };


  for(sensor_msgs__msg__PointField field_to_add : fields_to_add)
  {
    success = addPointFieldtoPointCloud2(field_to_add, &cloud);
    ASSERT_TRUE(success);
    EXPECT_STREQ(field_to_add.name.data, cloud.fields.data[i].name.data);
    EXPECT_EQ(field_to_add.offset, cloud.fields.data[i].offset);
    EXPECT_EQ(field_to_add.datatype, cloud.fields.data[i].datatype);
    EXPECT_EQ(field_to_add.count, cloud.fields.data[i].count);
    EXPECT_EQ(cloud.fields.size, cloud.fields.capacity);

    point_step_size_u32 += field_to_add.count * sizeOfPointField(field_to_add.datatype);      //  sum of all '.count' field of 'fields_to_add'
    EXPECT_EQ(++i, cloud.fields.size);
    EXPECT_EQ(point_step_size_u32, cloud.point_step);
  }
}