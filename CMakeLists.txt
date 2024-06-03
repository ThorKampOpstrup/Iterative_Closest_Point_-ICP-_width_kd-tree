cmake_minimum_required(VERSION 3.5)
project(scan_matcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########
include_directories(
  include
)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

add_library(kdtree
  src/kdtree.cpp
)

add_library(icp
  src/icp.cpp
)

##############################
add_executable(align
  apps/align.cpp
)

add_executable(align_rotated_src
  apps/align_rotated_src.cpp
)

add_executable(kdtree_from_pc
  apps/kdtree_from_pc.cpp
)

add_executable(kdtree_test
  apps/kdtree_test.cpp
)

add_executable(icp_test
  apps/icp_test.cpp
)

add_executable(icp_from_pc
  apps/icp_from_pc.cpp
)

add_executable(pc_conversion_test
apps/pc_conversion_test.cpp
)


add_executable(icp_from_pc_convergence
  apps/icp_from_pc_convergence.cpp
)

##############################
add_dependencies(align
  kdtree
  align_rotated_src
  kdtree_from_pc
  kdtree_test
  icp_test
  icp
  icp_from_pc
  ndt_omp
)
##############################
target_link_libraries(icp
  PRIVATE kdtree
)

target_link_libraries(align
  ${PCL_LIBRARIES}
  ndt_omp
)

target_link_libraries(align_rotated_src
  ${PCL_LIBRARIES}
  ndt_omp
)

target_link_libraries(kdtree_from_pc
  ${PCL_LIBRARIES}
  kdtree
  ndt_omp
)

target_link_libraries(kdtree_test
  ${PCL_LIBRARIES}
  kdtree
  ndt_omp
)

target_link_libraries(icp_test
  ${PCL_LIBRARIES}
  icp
)

target_link_libraries(icp_from_pc
  ${PCL_LIBRARIES}
  kdtree
  icp
)

target_link_libraries(pc_conversion_test
  ${PCL_LIBRARIES}
  kdtree
  icp
)

target_link_libraries(icp_from_pc_convergence
  ${PCL_LIBRARIES}
  kdtree
  icp
)

##############################

ament_target_dependencies(align
  rclcpp
  std_msgs
)

ament_target_dependencies(align_rotated_src
  rclcpp
  std_msgs
)

ament_target_dependencies(kdtree_from_pc
  rclcpp
  std_msgs
)

ament_target_dependencies(kdtree_test
  rclcpp
  std_msgs
)

ament_target_dependencies(icp_test
  rclcpp
  std_msgs
)

ament_target_dependencies(icp_from_pc
  rclcpp
  std_msgs
)

ament_target_dependencies(pc_conversion_test
  rclcpp
  std_msgs
)

ament_target_dependencies(icp_from_pc_convergence
  rclcpp
  std_msgs
)

############
## INSTAL ##
############

install(TARGETS 
  align
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  align_rotated_src
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  kdtree_from_pc
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  kdtree_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  icp_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  icp_from_pc
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  pc_conversion_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  icp_from_pc_convergence
  DESTINATION lib/${PROJECT_NAME}
)

##############################

install(
  TARGETS
    align
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    align_rotated_src
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    kdtree_from_pc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    kdtree_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    icp_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    icp_from_pc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    pc_conversion_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  TARGETS
    icp_from_pc_convergence
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

##############################

# install headers
install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

ament_package()
