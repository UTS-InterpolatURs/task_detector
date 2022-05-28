cmake_minimum_required(VERSION 3.0.2)
project(task_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

# add_message_files(
#   DIRECTORY
#   msg
#   FILES
# )

add_service_files(
  DIRECTORY
  srv
  FILES
  GetFeature.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cv_bridge geometry_msgs image_transport sensor_msgs message_runtime
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

install(
  FILES
  launch/launch.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/image/ImageConverter.cpp
  src/classifier/Classifier.cpp
  src/classifier/TaskFinder.cpp
  src/coordinate/Coordinate.cpp
  src/utils/CoordinateSmoother.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# add_dependencies(${PROJECT_NAME} edge_detector_generate_messages_cpp)