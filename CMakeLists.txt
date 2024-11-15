cmake_minimum_required(VERSION 3.10.2)
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
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS EIGEN3 pinocchio
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
)

add_executable(ridgeback_vicon_estimator_node src/ridgeback_vicon_estimator_node.cpp)
target_link_libraries(ridgeback_vicon_estimator_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(projectile_vicon_estimator_node src/projectile_vicon_estimator_node.cpp)
target_link_libraries(projectile_vicon_estimator_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(kinematics_example src/kinematics_example.cpp)
target_link_libraries(kinematics_example
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  Eigen3::Eigen
)

# Python setup
catkin_python_setup()
