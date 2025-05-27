cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL REQUIRED libserial)

# Shared Library (hardware interface)
add_library(${PROJECT_NAME} SHARED src/BumperbotHardware.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Executable
add_executable(simple_serial_transmitter src/transmitter.cpp)

target_include_directories(simple_serial_transmitter PUBLIC ${SERIAL_INCLUDE_DIRS})

target_link_libraries(simple_serial_transmitter ${SERIAL_LIBRARIES} )

ament_target_dependencies(simple_serial_transmitter
  rclcpp
  std_msgs
)

# Pluginlib Export
pluginlib_export_plugin_description_file(hardware_interface src/BumperbotHardware.xml) 

# Install Targets
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_serial_transmitter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Plugin XML
install(FILES
  src/BumperbotHardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()