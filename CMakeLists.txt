cmake_minimum_required(VERSION 3.8)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(fcc_bridge)

set(CXX_COMPILE_OPTIONS
        # Compilation syntax and safety flags
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Walloca
        -Wcast-qual
        -Wconversion
        -Wfloat-equal
        -Wformat-security
        -Wformat=2
        -Wnull-dereference
        -Wstack-protector
        -Wswitch-enum
        -Wvla

        # Flags to support ASLR and stack canaries
        -fstack-protector-strong
        -fstack-clash-protection
        -fPIE

        # Linker flags
        -Wl,-z,relro
        -Wl,-z,now
        -Wl,-z,noexecstack
        -Wl,-z,separate-code
)

# Compiler flags courtesy of https://airbus-seclab.github.io/c-compiler-security/
if(CMAKE_COMPILER_IS_GNUCXX)
  list(APPEND CXX_COMPILE_OPTIONS

          -Wdouble-promotion
          -Wswitch-default
          -Wformat-overflow=2
          -Wformat-truncation=2
          -Wtrampolines
          -Warray-bounds=2
          -Wimplicit-fallthrough=3
          -Wshift-overflow=2
          -Wstringop-overflow=4
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
  )

  SET(CXX_COMPILE_DEFINITIONS D_FORTIFY_SOURCE=2)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  message(WARNING "Clang support untested")

  list(APPEND CXX_COMPILE_OPTIONS
          -Wconversion
          -Warray-bounds
          -Warray-bounds-pointer-arithmetic
          -Wassign-enum
          -Wbad-function-cast
          -Wconditional-uninitialized
          -Wformat-type-confusion
          -Widiomatic-parentheses
          -Wimplicit-fallthrough
          -Wloop-analysis
          -Wpointer-arith
          -Wshift-sign-overflow
          -Wshorten-64-to-32
          -Wtautological-constant-in-range-compare
          -Wunreachable-code-aggressive
          -Wthread-safety
          -Wthread-safety-beta
          -Wcomma

          -fsanitize=safe-stack
          -fsanitize=bounds
          -fsanitize-undefined-trap-on-error
  )

  set(CXX_COMPILE_DEFINITIONS D_FORTIFY_SOURCE=3)

else()

  message(FATAL_ERROR "Unsupported compiler")

endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common_package REQUIRED)
find_package(MAVSDK REQUIRED)

set(FCC_BRIDGE_CXX_FILES
        src/fcc_bridge_node_ros.cpp
        src/fcc_bridge_node_conversion.cpp
        src/fcc_bridge_node_safety.cpp
        src/fcc_bridge_node_commands.cpp
        src/fcc_bridge_node_telemetry.cpp
)

add_executable(fcc_bridge
        src/main.cpp
        ${FCC_BRIDGE_CXX_FILES}
        src/fcc_bridge_node_mavsdk.cpp
)

target_compile_options(fcc_bridge PRIVATE
        ${CXX_COMPILE_OPTIONS}
)

target_compile_definitions(fcc_bridge PRIVATE
        ${CXX_COMPILE_DEFINITIONS}
)

target_include_directories(fcc_bridge PUBLIC
        include
)

target_link_libraries(fcc_bridge PRIVATE
        MAVSDK::mavsdk
)

ament_target_dependencies(fcc_bridge
        PUBLIC
        rclcpp
        interfaces
        common_package
)

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

  find_package(ament_cmake_gtest REQUIRED)

  ament_find_gtest()

  if(NOT GTEST_FOUND)
    message(FATAL_ERROR "Did not find GTest!")
  endif()

  add_executable(fcc_bridge_test
          # FCC Bridge files:
          ${FCC_BRIDGE_CXX_FILES}

          # Mock MAVSDK file:
          test/fcc_bridge_node_mavsdk_mock.cpp

          # Test files:
          test/main.cpp

          # Safety function tests
          test/safety/test_mavsdk_rth_cb.cpp
  )

  target_compile_options(fcc_bridge_test PRIVATE
          ${CXX_COMPILE_OPTIONS}
  )

  target_compile_definitions(fcc_bridge_test PRIVATE
          ${CXX_COMPILE_DEFINITIONS}
  )

  target_include_directories(fcc_bridge_test
          PRIVATE
            $<TARGET_PROPERTY:fcc_bridge,INCLUDE_DIRECTORIES>
            test/include
          SYSTEM PRIVATE
            "${GTEST_INCLUDE_DIRS}"
  )

  target_link_libraries(fcc_bridge_test
          PRIVATE
            MAVSDK::mavsdk
            ${GTEST_MAIN_LIBRARIES}
            ${GTEST_LIBRARIES}
  )

  ament_target_dependencies(fcc_bridge_test
          PUBLIC
            rclcpp
            interfaces
            common_package
  )

  ament_add_gtest_test(fcc_bridge_test)

endif()

ament_package()
