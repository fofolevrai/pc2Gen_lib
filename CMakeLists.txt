cmake_minimum_required(VERSION 3.8)
project(c_pc2_iterator)

# Set Default C++17 standard for compiler
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


# Set Default C11 standard for compiler
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_library SHARED
  src/point_cloud2_iterator.c
  )

# Add this compile definition for debug builds, this same logic works for
# target_compile_options, target_link_options, etc.
target_compile_definitions(${PROJECT_NAME}_library PRIVATE
    $<$<CONFIG:Debug>:DEBUG_BUILD>
)

ament_target_dependencies(${PROJECT_NAME}_library std_msgs sensor_msgs ${c_typesupport_target})
target_include_directories(${PROJECT_NAME}_library PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_export_targets(export_${PROJECT_NAME}_library HAS_LIBRARY_TARGET)
ament_export_dependencies(std_msgs sensor_msgs ${c_typesupport_target})

install(DIRECTORY include/
  DESTINATION include
  )

# How to create a ROS2 library
# https://answers.ros.org/question/357633/how-to-create-a-ros2-library/
# ament_export_targets(${PROJECT_NAME}_library HAS_LIBRARY_TARGET)
install(
  TARGETS ${PROJECT_NAME}_library
  EXPORT export_${PROJECT_NAME}_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_pc2_c_iterator.cpp
    test/test_pc2_c_iterator_func_sizeOfPointField.cpp
    test/test_pc2_c_iterator_func_dataResize.cpp
    test/test_pc2_c_iterator_func_ResizeFromArgs.cpp
    test/test_pc2_c_iterator_func_addPointFieldtoPointCloud2.cpp
    test/test_pc2_c_iterator_func_addPointFieldFromArgs.cpp
    test/test_pc2_c_iterator_func_SetPointFieldsFromDevice.cpp
    test/test_pc2_c_iterator_func_createPointCloud2FromDevice.cpp
    test/test_pc2_c_iterator_func_getSize.cpp
    )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    )
  # target_sources(${PROJECT_NAME}_test PRIVATE src/point_cloud2_iterator.c)
  # add_executable(${PROJECT_NAME}_test ${target_sources})# make the tests an executable so you can manually run them if needed
  ament_target_dependencies(${PROJECT_NAME}_test std_msgs sensor_msgs)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_library)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION test/${PROJECT_NAME})
endif()

ament_package()
