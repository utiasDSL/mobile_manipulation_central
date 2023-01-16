cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulation_central)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  urdf
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

# Python setup
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mobile_manipulation_central
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS Eigen3 pinocchio
)

###########
## Build ##
###########

set(FLAGS ${pinocchio_CFLAGS_OTHER})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${pinocchio_INCLUDE_DIRS}
)
link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# NOTE: bindings are not currently useful, so not built
# pybind11_add_module(bindings SHARED
#   src/pybindings.cpp
# )
# target_link_libraries(bindings PRIVATE
#   ${catkin_LIBRARIES}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mobile_manipulation_central.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ridgeback_vicon_estimator_node src/ridgeback_vicon_estimator_node.cpp)
target_link_libraries(ridgeback_vicon_estimator_node
  ${catkin_LIBRARIES}
)

add_executable(projectile_vicon_estimator_node src/projectile_vicon_estimator_node.cpp)
target_link_libraries(projectile_vicon_estimator_node
  ${catkin_LIBRARIES}
)

add_executable(kinematics_example src/kinematics_example.cpp)
target_link_libraries(kinematics_example
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

#############
## Install ##
#############

# install(TARGETS bindings
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )
# set_target_properties(bindings
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )
