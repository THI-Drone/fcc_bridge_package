cmake_minimum_required(VERSION 3.8)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(JSON_ImplicitConversions OFF) # deactivate implicit type conversion

project(fcc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)  # JSON library
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common_package REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(include)

add_executable(fcc_bridge
        src/main.cpp
        src/fcc_bridge_node_ros.cpp
        src/fcc_bridge_node_mavsdk.cpp
        src/fcc_bridge_node_conversion.cpp
)

target_link_libraries(fcc_bridge
        MAVSDK::mavsdk
)

ament_target_dependencies(fcc_bridge rclcpp interfaces common_package nlohmann_json)

install (TARGETS
  fcc_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
