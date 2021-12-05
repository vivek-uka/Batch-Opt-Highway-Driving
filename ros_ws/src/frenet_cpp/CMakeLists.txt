cmake_minimum_required(VERSION 3.5)
project(frenet_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs_car REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
    include()
  )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_library(cubic_spline_planner src/cubic_spline_planner.cpp)
target_link_libraries(cubic_spline_planner ${OpenCV_LIBS})

add_library(frenet_optimal_trajectory src/frenet_optimal_trajectory.cpp)
target_link_libraries(frenet_optimal_trajectory cubic_spline_planner ${OpenCV_LIBS})

#add_executable(frenet_node src/frenet_main.cpp)
#target_link_libraries(frenet_node frenet_optimal_trajectory cubic_spline_planner)
#ament_target_dependencies(frenet_node rclcpp std_msgs)

add_executable(frenet_car src/frenet_car.cpp)
target_link_libraries(frenet_car frenet_optimal_trajectory cubic_spline_planner yaml-cpp)
ament_target_dependencies(frenet_car rclcpp std_msgs msgs_car)


install(TARGETS
	#frenet_node
  frenet_car
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
