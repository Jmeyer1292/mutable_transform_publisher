cmake_minimum_required(VERSION 2.8.3)
project(mutable_transform_publisher)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  tf2
  tf2_ros
)

find_package(yaml-cpp REQUIRED)

add_service_files(
  FILES
  SetTransform.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    ${YAML_CPP_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_yaml
  CATKIN_DEPENDS
    tf2
    tf2_ros
    geometry_msgs
    message_runtime
   DEPENDS
    yaml-cpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


# Core ROS capability
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/publisher.cpp
  src/${PROJECT_NAME}/mutable_transform_publisher.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Yaml Stuff
add_library(${PROJECT_NAME}_yaml
  src/${PROJECT_NAME}/yaml_serialization.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})


# Primary Node
add_executable(${PROJECT_NAME}_node src/mutable_transform_publisher_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME mutable_transform_pub PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${PROJECT_NAME}_yaml)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_yaml ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
)
