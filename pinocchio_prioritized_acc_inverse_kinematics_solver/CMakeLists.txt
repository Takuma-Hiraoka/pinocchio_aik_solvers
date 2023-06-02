cmake_minimum_required(VERSION 3.0.2)
project(pinocchio_prioritized_acc_inverse_kinematics_solver)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  pinocchio_aik_constraint
  prioritized_qp_osqp
  )

find_package(pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/PrioritizedAccInverseKinematicsSolver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  )
