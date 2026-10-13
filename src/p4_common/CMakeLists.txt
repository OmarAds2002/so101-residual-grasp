cmake_minimum_required(VERSION 3.8)
project(p4_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

add_library(p4_common SHARED src/calibration.cpp)
target_include_directories(p4_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(p4_common yaml-cpp::yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)

  ament_add_gtest(test_joint_order test/test_joint_order.cpp)
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  find_package(yaml-cpp REQUIRED)

  target_include_directories(test_joint_order PRIVATE include)
  target_include_directories(test_calibration PRIVATE include)
  target_link_libraries(test_joint_order tinyxml2::tinyxml2)
  target_link_libraries(test_calibration p4_common tinyxml2)
  target_compile_definitions(test_joint_order PRIVATE
    URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/urdf/so101_new_calib.urdf")
  target_compile_definitions(test_calibration PRIVATE
    CALIBRATION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/calibration.yaml"
    URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/urdf/so101_new_calib.urdf")
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS p4_common EXPORT export_p4_common
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_p4_common HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()
