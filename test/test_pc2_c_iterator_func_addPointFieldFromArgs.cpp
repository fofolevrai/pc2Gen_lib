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


TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_Null_cloud_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = NULL;
  
  bool success = addPointFieldFromArgs(cloud_ptr, "test_field", 1, sensor_msgs__msg__PointField__INT8);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_Null_Name_Args)
{
  sensor_msgs__msg__PointCloud2 * cloud_ptr = sensor_msgs__msg__PointCloud2__create();
  
  bool success = addPointFieldFromArgs(cloud_ptr, NULL, 1, sensor_msgs__msg__PointField__INT8);
  ASSERT_FALSE(success);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_name_field_large_count_Args)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 * cloud = sensor_msgs__msg__PointCloud2__create();
  const char * name = "mytest_field";
  const uint8_t datatype = sensor_msgs__msg__PointField__INT8;
  const uint32_t count = 100;

  /* Act */
  bool success = addPointFieldFromArgs(cloud, name, count, datatype);

  /* Test */
  ASSERT_TRUE(success);
  EXPECT_EQ(strlen(name), cloud->fields.data->name.size);
  EXPECT_EQ(strlen(name)+1, cloud->fields.data->name.capacity);
  EXPECT_STREQ(name, cloud->fields.data[0].name.data);
  EXPECT_EQ(0, cloud->fields.data[0].offset);
  EXPECT_EQ(datatype, cloud->fields.data[0].datatype);
  EXPECT_EQ(count, cloud->fields.data[0].count);
  EXPECT_EQ(sizeOfPointField(datatype) * count, cloud->point_step);
  EXPECT_EQ(cloud->width * cloud->point_step, cloud->row_step);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_Nil_Offset)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 cloud = {0};
  const char * name = "test_field";
  const uint8_t datatype = sensor_msgs__msg__PointField__INT16;
  const uint32_t count = 1;
  
  /* Act */
  bool success = addPointFieldFromArgs(&cloud, name, count, datatype);

  /* Test */
  ASSERT_TRUE(success);
  EXPECT_EQ(strlen(name), cloud.fields.data->name.size);
  EXPECT_EQ(strlen(name)+1, cloud.fields.data->name.capacity);
  EXPECT_STREQ(name, cloud.fields.data[0].name.data);
  EXPECT_EQ(0, cloud.fields.data[0].offset);
  EXPECT_EQ(datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(count, cloud.fields.data[0].count);
  EXPECT_EQ(sizeOfPointField(datatype) * count, cloud.point_step);
  EXPECT_EQ(cloud.width * cloud.point_step, cloud.row_step);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_Width_Height_Large_Count)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 cloud = {.height = 8, .width=16};
  const char * name = "myLargeTest_field";
  const uint8_t datatype = sensor_msgs__msg__PointField__FLOAT64;
  const uint32_t count = 40;
  
  /* Act */
  bool success = addPointFieldFromArgs(&cloud, name, count, datatype);

  /* Test */
  ASSERT_TRUE(success);
  EXPECT_EQ(strlen(name), cloud.fields.data->name.size);
  EXPECT_EQ(strlen(name)+1, cloud.fields.data->name.capacity);
  EXPECT_STREQ(name, cloud.fields.data[0].name.data);
  EXPECT_EQ(datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(count, cloud.fields.data[0].count);
  EXPECT_EQ(sizeOfPointField(datatype) * count, cloud.point_step);
  EXPECT_EQ(cloud.width * cloud.point_step, cloud.row_step);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_Zero_Offset)
{
  /* Mock */
  sensor_msgs__msg__PointCloud2 cloud = {0};
  const char * name = "mytestField";
  uint32_t count = 1;
  uint8_t datatype = sensor_msgs__msg__PointField__INT8;

  /* Act */
  bool success = addPointFieldFromArgs(&cloud, name, count, datatype);

  /* Test */
  ASSERT_TRUE(success);
  EXPECT_EQ(strlen(name), cloud.fields.data->name.size);
  EXPECT_EQ(strlen(name)+1, cloud.fields.data->name.capacity);
  EXPECT_STREQ(name, cloud.fields.data[0].name.data);
  EXPECT_EQ(0, cloud.fields.data[0].offset);
  EXPECT_EQ(datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(count, cloud.fields.data[0].count);
  EXPECT_EQ(sizeOfPointField(datatype) * count, cloud.point_step);
  EXPECT_EQ(cloud.width * cloud.point_step, cloud.row_step);
}


TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_Offset)
{
  /* Mock */
  const uint32_t point_step_mock_offset_u32 = 5;
  sensor_msgs__msg__PointCloud2 cloud = {.point_step = point_step_mock_offset_u32};
  const char * name = "test_field";
  uint32_t count = 4;
  uint8_t datatype = sensor_msgs__msg__PointField__INT8;
  
  /* Act */
  bool success = addPointFieldFromArgs(&cloud, name, count, datatype);

  /* Test */
  ASSERT_TRUE(success);
  EXPECT_EQ(strlen(name), cloud.fields.data->name.size);
  EXPECT_EQ(strlen(name)+1, cloud.fields.data->name.capacity);
  EXPECT_STREQ(name, cloud.fields.data[0].name.data);
  EXPECT_EQ(point_step_mock_offset_u32, cloud.fields.data[0].offset);
  EXPECT_EQ(datatype, cloud.fields.data[0].datatype);
  EXPECT_EQ(count, cloud.fields.data[0].count);
  EXPECT_EQ(point_step_mock_offset_u32 + sizeOfPointField(datatype) * count, cloud.point_step);
  EXPECT_EQ(cloud.width * cloud.point_step, cloud.row_step);
}

TEST(test_c_pc2_iterator, addPointFieldFromArgsFunc_PointField_Multiple)
{
  uint32_t nbr_fields_u32_size =0, point_step_size_u32 = 0, i = 0;
  bool success = 0;
  sensor_msgs__msg__PointCloud2 cloud = {0};
  const size_t fields_to_add_size = 6;
  sensor_msgs__msg__PointField fields_to_add[fields_to_add_size] =
  {
    {
      .name = { .data = "test_field_01"},
      .offset = 0,
      .datatype = sizeOfPointField(sensor_msgs__msg__PointField__INT8),
      .count = 1
    },
    {
      .name = { .data = "test_field_02"},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8),
      .datatype = sensor_msgs__msg__PointField__INT16,
      .count = 1
    },
    {
      .name = { .data = "test_field_03"},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8) + sizeOfPointField(sensor_msgs__msg__PointField__INT16),
      .datatype = sensor_msgs__msg__PointField__FLOAT32,
      .count = 1
    },
    {
      .name = { .data = "test_field_04"},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8) + sizeOfPointField(sensor_msgs__msg__PointField__INT16) + sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32),
      .datatype = sensor_msgs__msg__PointField__FLOAT64,
      .count = 1
    },
    {
      .name = { .data = "test_field_05"},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8) + sizeOfPointField(sensor_msgs__msg__PointField__INT16) + sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32) + sizeOfPointField(sensor_msgs__msg__PointField__FLOAT64),
      .datatype = sensor_msgs__msg__PointField__INT16,
      .count = 2
    },
    {
      .name = { .data = "test_field_06"},
      .offset = sizeOfPointField(sensor_msgs__msg__PointField__INT8) + sizeOfPointField(sensor_msgs__msg__PointField__INT16) + sizeOfPointField(sensor_msgs__msg__PointField__FLOAT32) + sizeOfPointField(sensor_msgs__msg__PointField__FLOAT64) + fields_to_add[4].count * sizeOfPointField(sensor_msgs__msg__PointField__INT16),  // 'fields_to_add[4].count' to make sure count number is taken into account by offset calculation 
      .datatype = sensor_msgs__msg__PointField__INT16,
      .count = 6
    }
  };


  for(sensor_msgs__msg__PointField field_to_add : fields_to_add)
  {
    success = addPointFieldFromArgs(&cloud, field_to_add.name.data, field_to_add.count, field_to_add.datatype);
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