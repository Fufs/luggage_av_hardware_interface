cmake_minimum_required(VERSION 3.8)
project(luggage_av_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(NANOPB_SRC_ROOT_FOLDER "/usr/local/include/nanopb")
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/extra)

# DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # Nanopb # Weird generator issues
  Protobuf # Weird linker issues
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# include_directories(${NANOPB_INCLUDE_DIRS})


# COMPILE
# nanopb_generate_cpp(TARGET proto src/proto/wheel_commands.proto)

add_library(
  luggage_av_hardware_interface
  SHARED
  src/luggage_av_hardware_interface.cpp
  src/cobs.c
  src/proto/wheel_commands.pb.cc
)
target_compile_features(luggage_av_hardware_interface PUBLIC cxx_std_17)
target_include_directories(luggage_av_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/luggage_av_hardware_interface>
)

ament_target_dependencies(
  luggage_av_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface luggage_av_hardware_interface.xml)


# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/luggage_av_hardware_interface
)

install(TARGETS luggage_av_hardware_interface
  EXPORT export_luggage_av_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# EXPORT
ament_export_targets(export_luggage_av_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
