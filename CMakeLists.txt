cmake_minimum_required(VERSION 3.8)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(JSON_ImplicitConversions OFF) # deactivate implicit type conversion

project(fcc_bridge)

# Compiler flags courtesy of https://airbus-seclab.github.io/c-compiler-security/
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(
          -Wall
          -Wextra
          -Wpedantic
          -Werror
          -Wdouble-promotion
          -Wformat=2
          -Wswitch-enum
          -Wswitch-default
          -Wformat-overflow=2
          -Wformat-truncation=2
          -Wformat-security
          -Wnull-dereference
          -Wstack-protector
          -Wtrampolines
          -Walloca
          -Wvla
          -Warray-bounds=2
          -Wimplicit-fallthrough=3
          -Wshift-overflow=2
          -Wcast-qual
          -Wstringop-overflow=4
          -Wconversion
          -Warith-conversion
          -Wlogical-op
          -Wduplicated-cond
          -Wduplicated-branches
          -Wformat-signedness
          -Wshadow
          -Wstrict-overflow=5
          -Wundef
          -Wstack-usage=1000000
          -Wcast-align=strict
          -Wlogical-op
          -Wduplicated-cond
          -Wduplicated-branches
          -Wformat-signedness
          -Wcast-align=strict

          -fstack-protector-strong
          -fstack-clash-protection
          -fPIE

          -Wl,-z,relro
          -Wl,-z,now
          -Wl,-z,noexecstack
          -Wl,-z,separate-code
  )

  add_compile_definitions(D_FORTIFY_SOURCE=2)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  message(WARNING "Clang support untested")

  add_compile_definitions(
          -Werror
          -Walloca
          -Wcast-qual
          -Wconversion
          -Wformat=2
          -Wformat-security
          -Wnull-dereference
          -Wstack-protector
          -Wvla -Warray-bounds
          -Warray-bounds-pointer-arithmetic
          -Wassign-enum
          -Wbad-function-cast
          -Wconditional-uninitialized
          -Wconversion
          -Wfloat-equal
          -Wformat-type-confusion
          -Widiomatic-parentheses
          -Wimplicit-fallthrough
          -Wloop-analysis
          -Wpointer-arith
          -Wshift-sign-overflow
          -Wshorten-64-to-32
          -Wswitch-enum
          -Wtautological-constant-in-range-compare
          -Wunreachable-code-aggressive
          -Wthread-safety
          -Wthread-safety-beta
          -Wcomma

          -fstack-protector-strong
          -fsanitize=safe-stack
          -fPIE
          -fstack-clash-protection
          -fsanitize=bounds
          -fsanitize-undefined-trap-on-error
          -fsanitize=cfi

          -Wl,-z,relro
          -Wl,-z,now
          -Wl,-z,noexecstack
          -Wl,-z,separate-code
  )

  add_compile_definitions(D_FORTIFY_SOURCE=3)

else()

message(FATAL_ERROR "Unsupported compiler")

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
        src/fcc_bridge_node_safety.cpp
        src/fcc_bridge_node_commands.cpp
        src/fcc_bridge_node_telemetry.cpp
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
