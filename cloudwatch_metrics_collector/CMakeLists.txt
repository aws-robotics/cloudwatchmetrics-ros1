cmake_minimum_required(VERSION 2.8.3)
project(cloudwatch_metrics_collector)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cloudwatch_metrics_common
  ros_monitoring_msgs
  aws_common
  aws_ros1_common
)

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
set(CW_METRICS_COLLECTOR_SRC src/cloudwatch_metrics_collector.cpp)
add_executable(${PROJECT_NAME} ${CW_METRICS_COLLECTOR_SRC})
add_library(${PROJECT_NAME}_lib ${CW_METRICS_COLLECTOR_SRC})
target_include_directories(${PROJECT_NAME}  PRIVATE include ${catkin_INCLUDE_DIRS} ${cloudwatch_metrics_collector_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${cloudwatch_metrics_collector_INCLUDE_DIRS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})


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
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(/usr/include/gmock /usr/src/gmock)
  add_library(libgmock /usr/src/gmock/src/gmock-all.cc)

  SET(CW_METRICS_COLLECTOR_TEST test_cloudwatch_metrics_collector)
  add_rostest_gtest(${CW_METRICS_COLLECTOR_TEST}
    test/test_cloudwatch_metrics_collector.test
    test/cloudwatch_metrics_collector_test.cpp)
  target_include_directories(${CW_METRICS_COLLECTOR_TEST} PRIVATE include ${catkin_INCLUDE_DIRS}
    ${cloudwatch_metrics_collector_INCLUDE_DIRS})
  target_link_libraries(${CW_METRICS_COLLECTOR_TEST} ${PROJECT_NAME}_lib libgmock ${catkin_LIBRARIES})
endif()
