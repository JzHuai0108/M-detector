cmake_minimum_required(VERSION 3.5)
project(m_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Only set a default build type for single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------------------------ 
# Warnings & optimization flags by compiler + config
# ------------------------------------------------------------------------------
if(MSVC)
  # Common flags
  add_compile_options(/permissive- /EHsc /Zc:__cplusplus)

  # Per-config optimization + debug info
  add_compile_options(
      $<$<CONFIG:Release>:/O2>
      $<$<CONFIG:Release>:/W4>
      $<$<CONFIG:Release>:/DNDEBUG>

      $<$<CONFIG:RelWithDebInfo>:/O2>
      $<$<CONFIG:RelWithDebInfo>:/W4>
      $<$<CONFIG:RelWithDebInfo>:/DNDEBUG>

      $<$<CONFIG:Debug>:/Od>
      $<$<CONFIG:Debug>:/W4>
      $<$<CONFIG:Debug>:/Zi>
  )

  # (Optional) AVX2 + precise FP
  # add_compile_options(
  #     $<$<CONFIG:Release>:/fp:precise>
  #     $<$<CONFIG:Release>:/arch:AVX2>
  #     $<$<CONFIG:RelWithDebInfo>:/fp:precise>
  #     $<$<CONFIG:RelWithDebInfo>:/arch:AVX2>
  # )

else()
  # Avoid -ffast-math and -march=native (can change numerical results)
  # GCC/Clang side (single-config); here CMAKE_BUILD_TYPE matters
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Wno-unused-function -DNDEBUG)
  else()
    add_compile_options(-O1 -g -Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Wno-unused-function)
  endif()

  add_compile_options(-fexceptions)
endif()

# ------------------------------------------------------------------------------
# ROOT_DIR macro (CMake 3.5: use add_definitions)
# ------------------------------------------------------------------------------
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  # if(N GREATER 15)
  #   add_definitions(-DMP_EN)
  #   add_definitions(-DMP_PROC_NUM=14)
  #   message("core for MP:  4")
  # elseif(N GREATER 5)
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# find_package(OpenMP QUIET)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
# find_package(TBB REQUIRED COMPONENTS tbb)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include)

add_message_files(FILES
  CustomPoint.msg
  CustomMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC src/DynObjFilter.cpp src/DynObjCluster.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(dynfilter src/dynfilter_with_odom.cpp)
target_link_libraries(dynfilter ${PROJECT_NAME})
add_dependencies(dynfilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(display_prediction src/display_prediction.cpp )
target_link_libraries(display_prediction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(display_prediction
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(reader src/reader.cpp)
target_link_libraries(reader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(cal_recall src/cal_recall_multi.cpp)
target_link_libraries(cal_recall ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


## --- Install targets (library + nodes) ---
install(TARGETS
  ${PROJECT_NAME}
  dynfilter
  display_prediction
  reader
  cal_recall
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # static libs (.a)
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # shared libs (.so)
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}   # executables (lib/<pkg>/node)
)

## --- Install public headers (if you have any) ---
# Assumes headers live in include/<project_name>/
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## --- Install runtime resources (optional but typical) ---
# Install launch files, YAML configs, RViz configs, etc.
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
    PATTERN "*.xml"
    PATTERN "*.yaml"
    PATTERN "*.rviz"
)
