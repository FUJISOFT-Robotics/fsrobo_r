cmake_minimum_required(VERSION 2.8.12)

project(fsrobo_r_trajectory_filters)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS moveit_ros_planning trajectory_msgs)

catkin_package(
    CATKIN_DEPENDS moveit_ros_planning trajectory_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} 
  src/simple_resample_filter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
