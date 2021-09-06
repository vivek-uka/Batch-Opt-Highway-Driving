cmake_minimum_required(VERSION 3.5)
project(mpc_car_batch)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#  set(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
#endif()
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -mtune=corei7 -fopenmp -g ${CMAKE_CXX_FLAGS} -lstdc++fs ")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(msgs_car REQUIRED)

add_library(mpc_node_lib src/optim_batch.cpp)
target_link_libraries(mpc_node_lib eigen-quadprog)

add_executable(mpc_node src/main_batch.cpp)
target_link_libraries(mpc_node mpc_node_lib eigen-quadprog)
ament_target_dependencies(mpc_node rclcpp std_msgs geometry_msgs nav_msgs msgs_car)


install(TARGETS
	mpc_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
