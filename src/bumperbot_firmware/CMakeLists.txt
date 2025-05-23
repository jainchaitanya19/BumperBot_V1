cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

add_executable(simple_serial_transmitter src/transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)

target_include_directories(simple_serial_transmitter PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(simple_serial_transmitter ${SERIAL_LIBRARIES})

install(TARGETS simple_serial_transmitter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()