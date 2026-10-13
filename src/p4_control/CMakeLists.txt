cmake_minimum_required(VERSION 3.8)
project(p4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(p4_common REQUIRED)


add_library(p4_control SHARED src/ik.cpp src/trajectory.cpp)
target_include_directories(p4_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(p4_control
  orocos-kdl
  kdl_parser::kdl_parser
  Eigen3::Eigen)
ament_target_dependencies(p4_control p4_common)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ik test/test_ik.cpp)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory p4_control)
  target_link_libraries(test_ik p4_control)
  target_compile_definitions(test_ik PRIVATE
    URDF_PATH="${p4_common_DIR}/../../../share/p4_common/urdf/so101_new_calib.urdf")

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS p4_control EXPORT export_p4_control
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_p4_control HAS_LIBRARY_TARGET)
ament_export_dependencies(orocos_kdl kdl_parser p4_common)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
