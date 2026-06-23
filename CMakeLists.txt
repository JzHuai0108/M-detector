cmake_minimum_required(VERSION 3.5)
project(m_detector)

include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(M_DETECTOR_BUILD_ROS "Build ROS messages, wrappers, and tools." ON)

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

include(GNUInstallDirs)

if(M_DETECTOR_BUILD_ROS)
  find_package(catkin QUIET COMPONENTS
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
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(lio_common CONFIG REQUIRED)
if(NOT TARGET lio_common::lio_common)
  message(FATAL_ERROR "lio_common was found, but it did not export lio_common::lio_common.")
endif()

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include)

if(catkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})

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
endif()

add_library(${PROJECT_NAME} STATIC src/DynObjFilter.cpp src/DynObjCluster.cpp)
add_library(m_detector::m_detector ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    lio_common::lio_common
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
	  yaml-cpp::yaml-cpp
)

if(catkin_FOUND)
  add_executable(dynfilter
    src/dynfilter_with_odom.cpp
    src/DynObjFilterRos.cpp
  )
  target_link_libraries(dynfilter ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(dynfilter
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_executable(display_prediction src/display_prediction.cpp )
  target_link_libraries(display_prediction ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
  add_dependencies(display_prediction
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_executable(reader src/reader.cpp)
  target_link_libraries(reader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

  add_executable(cal_recall src/cal_recall_multi.cpp)
  target_link_libraries(cal_recall ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

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
      PATTERN "kuangye_*.yaml"
      PATTERN "xiangyin_*.yaml"
      PATTERN "*.rviz"
  )
else()
  install(TARGETS ${PROJECT_NAME}
    EXPORT m_detectorTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(
    FILES
      include/${PROJECT_NAME}/DynObjCluster.h
      include/${PROJECT_NAME}/DynObjFilter.h
      include/${PROJECT_NAME}/omp_compat.h
      include/${PROJECT_NAME}/parallel_q.h
      include/${PROJECT_NAME}/types.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  )
  install(DIRECTORY include/${PROJECT_NAME}/cluster_predict
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
  install(
    EXPORT m_detectorTargets
    NAMESPACE m_detector::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/m_detector
  )
  install(FILES
    config/kuangye_hesai16_building_mos.yaml
    config/kuangye_hesai16_mos.yaml
    config/kuangye_mid360_mos.yaml
    config/kuangye_vlp16_mos.yaml
    config/xiangyin_hesai32_mos.yaml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
  )
  configure_package_config_file(
    cmake/m_detectorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/m_detectorConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/m_detector
  )
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/m_detectorConfigVersion.cmake
    VERSION 0.1.0
    COMPATIBILITY SameMajorVersion
  )
  install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/m_detectorConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/m_detectorConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/m_detector
  )
endif()
