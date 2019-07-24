cmake_minimum_required(VERSION 2.8.3)
project(cloudwatch_metrics_collector)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# removed -Werror=pedantic until ros.h is fixed
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(dataflow_lite REQUIRED)
find_package(file_management REQUIRED)
find_package(cloudwatch_metrics_common REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cloudwatch_metrics_common
        roscpp
        aws_common
        aws_ros1_common
        rosgraph_msgs
        ros_monitoring_msgs
        std_srvs
        std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_library(${PROJECT_NAME}_lib SHARED src/metrics_collector.cpp src/metrics_collector_parameter_helper.cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${dataflow_lite_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${dataflow_lite_common_LIBRARIES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${file_management_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${file_management_LIBRARIES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${cloudwatch_metrics_common_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${cloudwatch_metrics_common_LIBRARIES})

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS} ${cloudwatch_metrics_common_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${cloudwatch_metrics_common_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  set(CW_METRICS_COLLECTOR_TEST test_cloudwatch_metrics_collector)
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)
  if(GMOCK_FOUND)
    add_rostest_gmock(${CW_METRICS_COLLECTOR_TEST}
      test/test_cloudwatch_metrics_collector.test
      test/cloudwatch_metrics_collector_test.cpp
    )
    target_include_directories(${CW_METRICS_COLLECTOR_TEST} PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_metrics_collector_INCLUDE_DIRS}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(${CW_METRICS_COLLECTOR_TEST}
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)

    add_rostest_gtest(${CW_METRICS_COLLECTOR_TEST}
      test/test_cloudwatch_metrics_collector.test
      test/cloudwatch_metrics_collector_test.cpp
    )
    target_include_directories(${CW_METRICS_COLLECTOR_TEST} PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_metrics_collector_INCLUDE_DIRS}
    )
    target_link_libraries(${CW_METRICS_COLLECTOR_TEST}
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )
  endif()
endif()
