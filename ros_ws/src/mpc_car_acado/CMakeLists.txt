cmake_minimum_required(VERSION 3.5)
project(mpc_car_acado)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ACADO REQUIRED )
find_package(msgs_car REQUIRED)

add_library(mpc_solver
externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/codegen/acado_qpoases_interface.cpp
  model/codegen/acado_integrator.c
  model/codegen/acado_solver.c
  model/codegen/acado_auxiliary_functions.c)


target_include_directories(mpc_solver PUBLIC
model/codegen/
externals/qpoases
externals/qpoases/INCLUDE
externals/qpoases/SRC)

link_directories(${ACADO_ENV_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( . ${ACADO_ENV_INCLUDE_DIRS} )

include_directories(
  include
  ${ACADO_ENV_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()

# add_executable(getting_started src/getting_started.cpp)
# target_link_libraries(getting_started  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
# ament_target_dependencies(getting_started ${ACADO_ENV_SHARED_LIBRARIES})
# set_target_properties(getting_started PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable(mpc_node src/main_acado.cpp)
target_link_libraries(mpc_node mpc_solver eigen-quadprog ${ACADO_ENV_SHARED_LIBRARIES})
ament_target_dependencies(mpc_node rclcpp std_msgs geometry_msgs nav_msgs msgs_car ${ACADO_ENV_SHARED_LIBRARIES})

install(TARGETS
  # getting_started
  mpc_node
	DESTINATION lib/${PROJECT_NAME}
)

